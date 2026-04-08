"""C++ TensorRT Detection Node - Header File."""

#ifndef WHEELTEC_ROBOT_DETECTION__DETECTION_NODE_HPP_
#define WHEELTEC_ROBOT_DETECTION__DETECTION_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/float32.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <NvInfer.h>
#include <NvOnnxParser.h>
#include <cuda_runtime_api.h>

#include "wheeltec_robot_msg/msg/detection2_d.hpp"
#include "wheeltec_robot_msg/msg/detection2_d_array.hpp"

#include <memory>
#include <vector>
#include <string>
#include <chrono>

namespace wheeltec_robot_detection
{

class Logger : public nvinfer1::ILogger
{
public:
  void log(Severity severity, const char* msg) noexcept override
  {
    if (severity <= Severity::kWARNING) {
      std::cout << msg << std::endl;
    }
  }
};

struct Detection
{
  int class_id;
  std::string class_name;
  float confidence;
  cv::Rect bbox;
};

class DetectionNode : public rclcpp::Node
{
public:
  explicit DetectionNode(const rclcpp::NodeOptions & options);
  ~DetectionNode();

private:
  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
  bool loadEngine(const std::string & engine_path);
  void preprocessImage(const cv::Mat & image, float* gpu_input);
  std::vector<Detection> postprocessOutput(float* gpu_output, int output_size,
                                           const cv::Size & original_size);
  std::vector<Detection> nms(std::vector<Detection> & detections, float nms_threshold);

  // TensorRT members
  Logger logger_;
  nvinfer1::IRuntime* runtime_;
  nvinfer1::ICudaEngine* engine_;
  nvinfer1::IExecutionContext* context_;

  // CUDA buffers
  void* buffers_[2];  // input and output
  cudaStream_t stream_;

  // Model parameters
  int input_h_;
  int input_w_;
  int output_size_;
  float conf_threshold_;
  float nms_threshold_;
  std::vector<std::string> class_names_;

  // ROS2 members
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Publisher<wheeltec_robot_msg::msg::Detection2DArray>::SharedPtr detections_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr fps_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr latency_pub_;

  // Performance tracking
  int frame_count_;
  std::chrono::steady_clock::time_point last_fps_time_;
  float fps_;
};

}  // namespace wheeltec_robot_detection

#endif  // WHEELTEC_ROBOT_DETECTION__DETECTION_NODE_HPP_
