#ifndef IO__USBCamera_HPP
#define IO__USBCamera_HPP

#include <chrono>
#include <opencv2/opencv.hpp>
#include <thread>

#include "tools/thread_safe_queue.hpp"

namespace io
{
class USBCamera
{
public:
  USBCamera(const std::string & open_name, const std::string & config_path);
  ~USBCamera();
  void read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp);
  std::string device_name;

private:
  struct CameraData
  {
    cv::Mat img;
    std::chrono::steady_clock::time_point timestamp;
  };

  std::mutex cap_mutex_;
  cv::VideoCapture cap_;
  std::string open_name_;
  int usb_frame_rate_;
  int open_count_;
  double image_width_, image_height_;
  bool quit_, ok_;
  std::thread capture_thread_;
  std::thread daemon_thread_;
  tools::ThreadSafeQueue<CameraData> queue_;

  void try_open();
  void open();
  void close();
};

}  // namespace io

#endif