#include <memory>
#include <sstream>
#include <fcntl.h>
#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <iostream>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/opencv.hpp"


class FakeWebcam
{
   public:
    int width;
    int height;
    FakeWebcam(std::string video_device, int width, int height)
    {
        this->width = width;
        this->height = height;
        v4l2lo = open(video_device.c_str(), O_WRONLY | O_SYNC);
        if (v4l2lo < 0)
        {
            std::cout << "Error opening v4l2l device: " << strerror(errno);
            device_opened = false;
        }
        {
            struct v4l2_format v;
            int t;
            v.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
            v.fmt.pix.width = width;
            v.fmt.pix.height = height;
            v.fmt.pix.pixelformat = V4L2_PIX_FMT_RGB24;
            v.fmt.pix.sizeimage = width * height * 3;
            t = ioctl(v4l2lo, VIDIOC_S_FMT, &v);
            if (t < 0)
            {
                std::cout << "Error opening v4l2l device with ioctl: " << strerror(errno);
                device_opened = false;
            }
        }
        device_opened = true;
    }

    void schedule_frame(cv::Mat frame)
    {
        if (width != frame.cols || height != frame.rows)
        {
            std::cout << "Error with current frame, should be: " << height << "x" << width << " Got: " << frame.rows
                      << "x" << frame.cols << std::endl;
            return;
        }
        if (!device_opened)
        {
            std::cout << "Error: Virtual device not opened" << std::endl;
            return;
        }
        int size = frame.total() * frame.elemSize();
        auto written = write(v4l2lo, frame.data, size);
        if (written < 0)
        {
            std::cout << "Error writing v4l2l device";
            close(v4l2lo);
            return;
        }
    }

   private:
    int v4l2lo;
    bool device_opened;
};