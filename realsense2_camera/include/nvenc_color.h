// Kiwi added: zero-copy H.264 of the full-res color on the Jetson engines (nvjpg_decoder library:
// RGB8/BGR8 -> VIC -> NVENC), published as foxglove_msgs/CompressedVideo by the driver itself.
#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <detection_msgs/msg/detections.hpp>
#include <foxglove_msgs/msg/compressed_video.hpp>
#include <librealsense2/rs.hpp>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <string>
#include <thread>
#include <vector>

namespace realsense2_camera {
// Face / licence plate pixelation of the encoded frames (nvjpg_decoder anonymizer: libanon.so + TensorRT engine)
struct NvencAnonymizer
{
    bool enabled = false;
    std::string lib_path, engine_path;
    float threshold = 0.2f;
    int detect_every = 1;
    // detection_msgs/Detections of the boxes applied to each encoded frame (class_name face / plate, grown for the
    // low-res frames), e.g. for video_mapping to blur the same detections in the stream; empty: not published
    std::string detections_topic;
};

class NvencColorEncoder
{
   public:
    // topic: CompressedVideo topic (relative to the node), with the CameraInfo of its full-res frames on
    // <topic>/camera_info; the encoder starts on the first frame. With anon.enabled, every frame is anonymized
    // before encoding, or blanked when that fails (never encoded as it is). thread_nice: CPU priority of the
    // encoder thread (named "nvenc_color"; < 0 = higher), 0 leaves it as the node's
    NvencColorEncoder(rclcpp::Node& node, const std::string& topic, const std::string& frame_id, int bitrate, int gop,
                      const NvencAnonymizer& anon, int thread_nice, rclcpp::Logger logger);
    ~NvencColorEncoder();

    // True once the encoder could not start: the caller uses its fallback path
    bool failed() const { return _failed; }
    // With the anonymizer: true while frames are being anonymized; false before the first one, after a frame that
    // could not be anonymized (until the next one that is) and once the encoder stopped. Always true without it
    bool anonymizer_ok() const { return !_anon.enabled || (_anon_ok && !_failed); }
    // Hand over one color frame (no copy: the rs2 frame is kept). The worker encodes the frames in order from a
    // queue of kQueue frames; only when the queue is full is the oldest frame dropped
    void push(const rs2::video_frame& frame, int64_t stamp_ns);
    static constexpr size_t kQueue = 6;  // 3 dropped up to 15 of 900 frames per 30 s with nav2 on the CPU

   private:
    bool start(const rs2::video_frame& frame);
    void run();
    void publish(size_t len, int64_t stamp_ns);
    void log_anon_stats();
    void publish_detections(int64_t stamp_ns, int width, int height);

    rclcpp::Logger _logger;
    std::string _frame_id;
    int _bitrate, _gop;
    NvencAnonymizer _anon;
    int _thread_nice;
    std::chrono::steady_clock::time_point _anon_log_time;
    rclcpp::Clock _steady_clock{RCL_STEADY_TIME};  // log throttling
    rclcpp::Publisher<foxglove_msgs::msg::CompressedVideo>::SharedPtr _pub;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr _info_pub;
    sensor_msgs::msg::CameraInfo _info;
    rclcpp::Publisher<detection_msgs::msg::Detections>::SharedPtr _det_pub;
    void* _enc = nullptr;
    std::vector<uint8_t> _out;
    std::atomic<bool> _failed{false};
    std::atomic<bool> _stop{false};
    std::mutex _mutex;
    std::condition_variable _cv;
    std::deque<std::pair<rs2::frame, int64_t>> _queue;  // oldest first
    uint64_t _errors = 0;
    std::atomic<bool> _anon_ok{false};  // the last encoded frame was anonymized
    std::thread _thread;
};
}  // namespace realsense2_camera
