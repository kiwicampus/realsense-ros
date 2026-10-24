// Kiwi added: see nvenc_color.h
#include "nvenc_color.h"

#include <nvjpg_decoder/nvjpg_decoder.h>
#include <pthread.h>
#include <sys/resource.h>
#include <sys/syscall.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <cstring>

namespace realsense2_camera {
namespace {
// nvjpg_decoder C API: profile 0 = baseline, preset 3 = medium, max_perf 1 (per-session NVENC clock: needed
// for 30 fps next to the other encoders), depth 2
constexpr int kProfileBaseline = 0;
constexpr int kPresetMedium = 3;
constexpr int kMaxPerf = 1;
constexpr int kDepth = 2;
constexpr size_t kMaxAccessUnit = 2 << 20;
constexpr int kAnonBlanked = 2;  // encode return: anonymizer failed, the frame was encoded blanked
constexpr auto kAnonLogPeriod = std::chrono::seconds(30);
// Growth of the published boxes around their centre: the encoder's own (faces 1.4 = ellipse covering the box,
// plates 1.15) plus a margin for the low-res frame they are applied to, a frame or so later, as a rectangle
constexpr float kFaceGrow = 1.6f, kPlateGrow = 1.3f;
constexpr int kMaxBoxes = 100;
}  // namespace

NvencColorEncoder::NvencColorEncoder(rclcpp::Node& node, const std::string& topic, const std::string& frame_id,
                                     int bitrate, int gop, const NvencAnonymizer& anon, int thread_nice,
                                     rclcpp::Logger logger)
    : _logger(logger), _frame_id(frame_id), _bitrate(bitrate), _gop(gop), _anon(anon), _thread_nice(thread_nice),
      _out(kMaxAccessUnit)
{
    // Same QoS as ffmpeg_image_transport: reliable, keep last 2 GOPs
    _pub = node.create_publisher<foxglove_msgs::msg::CompressedVideo>(topic, rclcpp::QoS(std::max(10, 2 * gop)));
    _info_pub = node.create_publisher<sensor_msgs::msg::CameraInfo>(topic + "/camera_info", 10);
    if (_anon.enabled && !_anon.detections_topic.empty())
        _det_pub = node.create_publisher<detection_msgs::msg::Detections>(_anon.detections_topic,
                                                                          rclcpp::SensorDataQoS());
    _thread = std::thread(&NvencColorEncoder::run, this);
}

NvencColorEncoder::~NvencColorEncoder()
{
    _stop = true;
    _cv.notify_all();
    if (_thread.joinable()) _thread.join();
    if (_enc) nvjpg_enc_destroy(_enc);  // STREAMOFF + free all buffers; the kernel also frees them on a crash
}

void NvencColorEncoder::push(const rs2::video_frame& frame, int64_t stamp_ns)
{
    {
        std::lock_guard<std::mutex> lock(_mutex);
        if (_queue.size() >= kQueue) _queue.pop_front();  // the encoder fell kQueue frames behind: drop the oldest
        _queue.emplace_back(frame, stamp_ns);
    }
    _cv.notify_one();
}

bool NvencColorEncoder::start(const rs2::video_frame& frame)
{
    char err[256] = "";
    const int fps = frame.get_profile().fps();
    _enc = nvjpg_enc_create(frame.get_width(), frame.get_height(), fps, _bitrate, _gop, kProfileBaseline, kPresetMedium,
                            kMaxPerf, kDepth, err, sizeof(err));
    if (!_enc)
    {
        RCLCPP_ERROR(_logger, "color_nvenc: encoder did not start (%s): no color H.264", err);
        return false;
    }
    if (_anon.enabled)
    {
        // Fail closed: if the detector does not load, the encoder blanks every frame
        if (nvjpg_enc_set_anonymizer(_enc, _anon.lib_path.c_str(), _anon.engine_path.c_str(), _anon.threshold,
                                     _anon.detect_every, err, sizeof(err)) != 0)
            RCLCPP_ERROR(_logger, "color_nvenc: anonymizer did not load (%s): every frame is encoded blanked", err);
        else
            RCLCPP_INFO(_logger, "color_nvenc: anonymizer %s, threshold %.2f, detect every %d frame(s)",
                        _anon.engine_path.c_str(), _anon.threshold, _anon.detect_every);
        _anon_log_time = std::chrono::steady_clock::now();
    }
    // CameraInfo of the full-res frames, filled as the driver fills color/camera_info
    const rs2_intrinsics in = frame.get_profile().as<rs2::video_stream_profile>().get_intrinsics();
    _info.header.frame_id = _frame_id;
    _info.width = in.width;
    _info.height = in.height;
    _info.k = {in.fx, 0, in.ppx, 0, in.fy, in.ppy, 0, 0, 1};
    _info.r = {1, 0, 0, 0, 1, 0, 0, 0, 1};
    _info.p = {in.fx, 0, in.ppx, 0, 0, in.fy, in.ppy, 0, 0, 0, 1, 0};
    const int coeffs = in.model == RS2_DISTORTION_KANNALA_BRANDT4 ? 4 : 5;
    _info.distortion_model = coeffs == 4 ? "equidistant" : "plumb_bob";
    _info.d.assign(in.coeffs, in.coeffs + coeffs);
    RCLCPP_INFO(_logger, "color_nvenc: %dx%d %s @%d fps -> NVENC H.264 %d bit/s GOP %d on %s", frame.get_width(),
                frame.get_height(), rs2_format_to_string(frame.get_profile().format()), fps, _bitrate, _gop,
                _pub->get_topic_name());
    return true;
}

void NvencColorEncoder::publish(size_t len, int64_t stamp_ns)
{
    foxglove_msgs::msg::CompressedVideo msg;
    msg.timestamp.sec = static_cast<int32_t>(stamp_ns / 1000000000);
    msg.timestamp.nanosec = static_cast<uint32_t>(stamp_ns % 1000000000);
    msg.frame_id = _frame_id;
    msg.format = "h264";
    msg.data.assign(_out.begin(), _out.begin() + len);
    _info.header.stamp = msg.timestamp;
    _pub->publish(std::move(msg));
    _info_pub->publish(_info);
}

// The boxes the anonymizer applied to the frame just encoded (none when it was blanked: then nothing is published,
// and the low-res color is not published either, see anonymizer_ok)
void NvencColorEncoder::publish_detections(int64_t stamp_ns, int width, int height)
{
    float boxes[4 * kMaxBoxes], scores[kMaxBoxes];
    int cls[kMaxBoxes];
    const int n = nvjpg_enc_anon_boxes(_enc, boxes, cls, scores, kMaxBoxes);
    if (n < 0) return;
    detection_msgs::msg::Detections msg;
    msg.header.stamp.sec = static_cast<int32_t>(stamp_ns / 1000000000);
    msg.header.stamp.nanosec = static_cast<uint32_t>(stamp_ns % 1000000000);
    msg.header.frame_id = _frame_id;
    msg.camera = "stereo";
    msg.width = width;
    msg.height = height;
    for (int i = 0; i < n; ++i)
    {
        const float* b = boxes + 4 * i;
        const float s = cls[i] == 0 ? kFaceGrow : kPlateGrow;
        const float cx = 0.5f * (b[0] + b[2]), cy = 0.5f * (b[1] + b[3]);
        const float hw = 0.5f * s * (b[2] - b[0]), hh = 0.5f * s * (b[3] - b[1]);
        detection_msgs::msg::Detection d;
        d.box.x = std::max(0.f, cx - hw);
        d.box.y = std::max(0.f, cy - hh);
        d.box.w = std::min(1.f, cx + hw) - d.box.x;
        d.box.h = std::min(1.f, cy + hh) - d.box.y;
        d.prob = scores[i];
        d.class_name = cls[i] == 0 ? "face" : "plate";
        msg.detections.push_back(d);
    }
    _det_pub->publish(std::move(msg));
}

// Every kAnonLogPeriod: detection time and blanked frames
void NvencColorEncoder::log_anon_stats()
{
    const auto now = std::chrono::steady_clock::now();
    if (now - _anon_log_time < kAnonLogPeriod) return;
    _anon_log_time = now;
    uint64_t frames, detections, failures;
    double mean_ms, max_ms;
    if (nvjpg_enc_anon_stats(_enc, &frames, &detections, &failures, &mean_ms, &max_ms) != 0) return;
    RCLCPP_INFO(_logger, "color_nvenc: anonymizer %lu frames, %lu detections (%.1f ms mean, %.1f ms max), %lu blanked",
                static_cast<unsigned long>(frames), static_cast<unsigned long>(detections), mean_ms, max_ms,
                static_cast<unsigned long>(failures));
}

void NvencColorEncoder::run()
{
    // The encoder thread also launches the anonymizer's GPU work: when it waits for a CPU core, the GPU idles
    // between kernels and the frame takes longer. A higher priority keeps it fed under a loaded CPU
    pthread_setname_np(pthread_self(), "nvenc_color");
    if (_thread_nice != 0 && setpriority(PRIO_PROCESS, static_cast<id_t>(syscall(SYS_gettid)), _thread_nice) != 0)
        RCLCPP_WARN(_logger, "color_nvenc: thread priority (nice %d) not set: %s", _thread_nice, strerror(errno));
    while (!_stop)
    {
        rs2::frame f;
        int64_t stamp = 0;
        {
            std::unique_lock<std::mutex> lock(_mutex);
            _cv.wait(lock, [this] { return _stop || !_queue.empty(); });
            if (_stop) break;
            f = std::move(_queue.front().first);
            stamp = _queue.front().second;
            _queue.pop_front();
        }
        const auto v = f.as<rs2::video_frame>();
        const auto format = v.get_profile().format();
        const int fmt = format == RS2_FORMAT_RGB8 ? 0 : format == RS2_FORMAT_BGR8 ? 1 : -1;
        if (fmt < 0)
        {
            RCLCPP_ERROR(_logger, "color_nvenc: color format %s not supported: no color H.264",
                         rs2_format_to_string(format));
            _failed = true;
            break;
        }
        if (!_enc && !start(v))
        {
            _failed = true;
            break;
        }
        size_t len = 0;
        int key = 0;
        int64_t out_stamp = 0;
        // Pipelined: the access unit that comes out is the previous frame's, with its own stamp
        const int rc = nvenc_raw_encode(_enc, static_cast<const uint8_t*>(v.get_data()), v.get_stride_in_bytes(), fmt,
                                        stamp, _out.data(), _out.size(), &len, &key, &out_stamp);
        if (rc == -5)
        {
            // V4L2 timeout / error: a new encoder on the next frame (it starts with an IDR)
            RCLCPP_WARN(_logger, "color_nvenc: %s: restarting the encoder", nvjpg_enc_last_error(_enc));
            nvjpg_enc_destroy(_enc);
            _enc = nullptr;
            if (++_errors > 5)
            {
                RCLCPP_ERROR(_logger, "color_nvenc: too many encoder errors: no color H.264");
                _failed = true;
                break;
            }
            continue;
        }
        if (_anon.enabled && (rc == 0 || rc == kAnonBlanked))
        {
            // The low-res color (image_raw) is not published while this is false (see anonymizer_ok)
            const bool ok = rc == 0;
            if (ok != _anon_ok.exchange(ok))
            {
                if (ok)
                    RCLCPP_INFO(_logger, "color_nvenc: anonymizer working: color image_raw published");
                else
                    RCLCPP_ERROR(_logger, "color_nvenc: anonymizer failed (%s): color image_raw stops until it works",
                                 nvjpg_enc_last_error(_enc));
            }
        }
        if (rc == kAnonBlanked)  // encoded (blanked): the access unit must still go out
            RCLCPP_WARN_THROTTLE(_logger, _steady_clock, 5000, "color_nvenc: %s", nvjpg_enc_last_error(_enc));
        else if (rc != 0)
        {
            RCLCPP_DEBUG(_logger, "color_nvenc: frame skipped (%d): %s", rc, nvjpg_enc_last_error(_enc));
            continue;
        }
        _errors = 0;
        if (len > 0) publish(len, out_stamp);
        if (_det_pub && rc == 0) publish_detections(stamp, v.get_width(), v.get_height());
        if (_anon.enabled) log_anon_stats();
    }
}
}  // namespace realsense2_camera
