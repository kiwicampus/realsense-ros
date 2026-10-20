// Kiwi added: zero-copy H.264 of the full-res color on the Jetson engines (nvjpg_decoder library:
// RGB8/BGR8 -> VIC -> NVENC), published as foxglove_msgs/CompressedVideo by the driver itself.
#pragma once

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <foxglove_msgs/msg/compressed_video.hpp>
#include <librealsense2/rs.hpp>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <thread>
#include <vector>

namespace realsense2_camera {
class NvencColorEncoder
{
   public:
    // topic: CompressedVideo topic (relative to the node); the encoder starts on the first frame
    NvencColorEncoder(rclcpp::Node& node, const std::string& topic, const std::string& frame_id, int bitrate, int gop,
                      rclcpp::Logger logger);
    ~NvencColorEncoder();

    // True once the encoder could not start: the caller uses its fallback path
    bool failed() const { return _failed; }
    // Hand over one color frame (no copy: the rs2 frame is kept). The worker encodes the frames in order from a
    // queue of kQueue frames; only when the queue is full is the oldest frame dropped
    void push(const rs2::video_frame& frame, int64_t stamp_ns);
    static constexpr size_t kQueue = 3;

   private:
    bool start(const rs2::video_frame& frame);
    void run();
    void publish(size_t len, int64_t stamp_ns);

    rclcpp::Logger _logger;
    std::string _frame_id;
    int _bitrate, _gop;
    rclcpp::Publisher<foxglove_msgs::msg::CompressedVideo>::SharedPtr _pub;
    void* _enc = nullptr;
    std::vector<uint8_t> _out;
    std::atomic<bool> _failed{false};
    std::atomic<bool> _stop{false};
    std::mutex _mutex;
    std::condition_variable _cv;
    std::deque<std::pair<rs2::frame, int64_t>> _queue;  // oldest first
    uint64_t _errors = 0;
    std::thread _thread;
};
}  // namespace realsense2_camera
