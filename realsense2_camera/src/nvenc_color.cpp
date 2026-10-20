// Kiwi added: see nvenc_color.h
#include "nvenc_color.h"

#include <nvjpg_decoder/nvjpg_decoder.h>

namespace realsense2_camera {
namespace {
// nvjpg_decoder C API: profile 0 = baseline, preset 3 = medium, max_perf 1 (per-session NVENC clock: needed
// for 30 fps next to the other encoders), depth 2
constexpr int kProfileBaseline = 0;
constexpr int kPresetMedium = 3;
constexpr int kMaxPerf = 1;
constexpr int kDepth = 2;
constexpr size_t kMaxAccessUnit = 2 << 20;
}  // namespace

NvencColorEncoder::NvencColorEncoder(rclcpp::Node& node, const std::string& topic, const std::string& frame_id,
                                     int bitrate, int gop, rclcpp::Logger logger)
    : _logger(logger), _frame_id(frame_id), _bitrate(bitrate), _gop(gop), _out(kMaxAccessUnit)
{
    // Same QoS as ffmpeg_image_transport: reliable, keep last 2 GOPs
    _pub = node.create_publisher<foxglove_msgs::msg::CompressedVideo>(topic, rclcpp::QoS(std::max(10, 2 * gop)));
    _info_pub = node.create_publisher<sensor_msgs::msg::CameraInfo>(topic + "/camera_info", 10);
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

void NvencColorEncoder::run()
{
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
        if (rc != 0)
        {
            RCLCPP_DEBUG(_logger, "color_nvenc: frame skipped (%d): %s", rc, nvjpg_enc_last_error(_enc));
            continue;
        }
        _errors = 0;
        if (len > 0) publish(len, out_stamp);
    }
}
}  // namespace realsense2_camera
