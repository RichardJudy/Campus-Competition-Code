#include "usbcamera.hpp"

#include <stdexcept>
#include <thread>
#include <chrono>

#include "tools/logger.hpp"
#include "tools/yaml.hpp"

using namespace std::chrono_literals;

namespace io
{
USBCamera::USBCamera(const std::string & open_name, const std::string & config_path)
: open_name_(open_name), quit_(false), ok_(false), queue_(1), open_count_(0)
{
  auto yaml = tools::load(config_path);
  image_width_ = tools::read<double>(yaml, "image_width");
  image_height_ = tools::read<double>(yaml, "image_height");
  usb_frame_rate_ = tools::read<double>(yaml, "usb_frame_rate");
  try_open();

  daemon_thread_ = std::thread{[this] {
    while (!quit_) {
      std::this_thread::sleep_for(100ms);
      if (ok_) continue;

      if (open_count_ > 20) {
        quit_ = true;
        {
          std::lock_guard<std::mutex> lock(cap_mutex_);
          close();
        }
        if (capture_thread_.joinable()) {
          capture_thread_.join();
        }
        break;
      }

      if (capture_thread_.joinable()) capture_thread_.join();
      {
        std::lock_guard<std::mutex> lock(cap_mutex_);
        close();
      }
      try_open();
    }
  }};
}

USBCamera::~USBCamera()
{
  quit_ = true;
  {
    std::lock_guard<std::mutex> lock(cap_mutex_);
    close();
  }
  if (daemon_thread_.joinable()) daemon_thread_.join();
  if (capture_thread_.joinable()) capture_thread_.join();
}

void USBCamera::read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp)
{
  CameraData data;
  queue_.pop(data);

  img = data.img;
  timestamp = data.timestamp;
}

void USBCamera::open()
{
  std::lock_guard<std::mutex> lock(cap_mutex_);
  std::string true_device_name = "/dev/" + open_name_;
  cap_.open(true_device_name, cv::CAP_V4L);
  if (!cap_.isOpened()) {
    tools::logger()->warn("Failed to open USB camera");
    return;
  }
  cap_.set(cv::CAP_PROP_FRAME_WIDTH, image_width_);
  cap_.set(cv::CAP_PROP_FRAME_HEIGHT, image_height_);
  cap_.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
  cap_.set(cv::CAP_PROP_FPS, usb_frame_rate_);

  capture_thread_ = std::thread{[this] {
    ok_ = true;
    std::this_thread::sleep_for(50ms);
    while (!quit_) {
      std::this_thread::sleep_for(1ms);

      cv::Mat img;
      bool success;
      {
        std::lock_guard<std::mutex> lock(cap_mutex_);
        if (!cap_.isOpened()) {
          break;
        }
        success = cap_.read(img);
      }

      if (!success) {
        tools::logger()->warn("Failed to read frame, exiting capture thread");
        break;
      }

      auto timestamp = std::chrono::steady_clock::now();
      queue_.push({img, timestamp});
    }
    ok_ = false;
  }};
}

void USBCamera::try_open()
{
  try {
    open();
    open_count_++;
  } catch (const std::exception & e) {
    tools::logger()->warn("{}", e.what());
  }
}

void USBCamera::close()
{
  if (cap_.isOpened()) {
    cap_.release();
  }
}

}  // namespace io