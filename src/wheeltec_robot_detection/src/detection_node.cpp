#include "wheeltec_robot_detection/detection_node.hpp"
#include <fstream>
#include <algorithm>

namespace wheeltec_robot_detection
{

DetectionNode::DetectionNode(const rclcpp::NodeOptions & options)
: Node("detection_node_cpp", options),
  runtime_(nullptr),
  engine_(nullptr),
  context_(nullptr),
  frame_count_(0),
  fps_(0.0f)
{
  // Declare parameters
  this->declare_parameter<std::string>("engine_path", "");
  this->declare_parameter<int>("input_h", 640);
  this->declare_parameter<int>("input_w", 640);
  this->declare_parameter<float>("conf_threshold", 0.5);
  this->declare_parameter<float>("nms_threshold", 0.45);
  this->declare_parameter<std::vector<std::string>>("class_names", std::vector<std::string>());

  // Get parameters
  std::string engine_path = this->get_parameter("engine_path").as_string();
  input_h_ = this->get_parameter("input_h").as_int();
  input_w_ = this->get_parameter("input_w").as_int();
  conf_threshold_ = this->get_parameter("conf_threshold").as_double();
  nms_threshold_ = this->get_parameter("nms_threshold").as_double();
  class_names_ = this->get_parameter("class_names").as_string_array();

  // Load TensorRT engine
  if (!loadEngine(engine_path)) {
    RCLCPP_ERROR(this->get_logger(), "Failed to load TensorRT engine");
    throw std::runtime_error("Failed to load TensorRT engine");
  }

  // Create CUDA stream
  cudaStreamCreate(&stream_);

  // Create subscribers
  auto qos = rclcpp::QoS(rclcpp::KeepLast(1));
  qos.best_effort();

  image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    "/camera/color/image_raw",
    qos,
    std::bind(&DetectionNode::imageCallback, this, std::placeholders::_1)
  );

  // Create publishers
  detections_pub_ = this->create_publisher<wheeltec_robot_msg::msg::Detection2DArray>(
    "/detections", 10
  );

  fps_pub_ = this->create_publisher<std_msgs::msg::Float32>("/ai/fps", 10);
  latency_pub_ = this->create_publisher<std_msgs::msg::Float32>("/ai/latency", 10);

  last_fps_time_ = std::chrono::steady_clock::now();

  RCLCPP_INFO(this->get_logger(), "C++ TensorRT Detection Node initialized");
}

DetectionNode::~DetectionNode()
{
  // Free CUDA resources
  cudaFree(buffers_[0]);
  cudaFree(buffers_[1]);
  cudaStreamDestroy(stream_);

  // Free TensorRT resources
  if (context_) delete context_;
  if (engine_) delete engine_;
  if (runtime_) delete runtime_;
}

bool DetectionNode::loadEngine(const std::string & engine_path)
{
  // Read engine file
  std::ifstream file(engine_path, std::ios::binary);
  if (!file.good()) {
    RCLCPP_ERROR(this->get_logger(), "Engine file not found: %s", engine_path.c_str());
    return false;
  }

  file.seekg(0, std::ios::end);
  size_t size = file.tellg();
  file.seekg(0, std::ios::beg);

  std::vector<char> engine_data(size);
  file.read(engine_data.data(), size);
  file.close();

  // Create runtime and deserialize engine
  runtime_ = nvinfer1::createInferRuntime(logger_);
  engine_ = runtime_->deserializeCudaEngine(engine_data.data(), size);

  if (!engine_) {
    RCLCPP_ERROR(this->get_logger(), "Failed to deserialize engine");
    return false;
  }

  context_ = engine_->createExecutionContext();

  // Allocate CUDA buffers using new API
  int32_t input_index = 0;  // Assuming first binding is input
  int32_t output_index = 1; // Assuming second binding is output

  // Get tensor names and store them
  input_tensor_name_ = engine_->getIOTensorName(input_index);
  output_tensor_name_ = engine_->getIOTensorName(output_index);

  auto input_dims = engine_->getTensorShape(input_tensor_name_.c_str());
  auto output_dims = engine_->getTensorShape(output_tensor_name_.c_str());

  size_t input_size = 1 * 3 * input_h_ * input_w_ * sizeof(float);
  output_size_ = 1;
  for (int i = 0; i < output_dims.nbDims; i++) {
    output_size_ *= output_dims.d[i];
  }
  size_t output_size = output_size_ * sizeof(float);

  cudaMalloc(&buffers_[input_index], input_size);
  cudaMalloc(&buffers_[output_index], output_size);

  RCLCPP_INFO(this->get_logger(), "TensorRT engine loaded successfully");
  RCLCPP_INFO(this->get_logger(), "Input: %dx%d, Output size: %d", input_w_, input_h_, output_size_);

  return true;
}

void DetectionNode::preprocessImage(const cv::Mat & image, float* gpu_input)
{
  // Letterbox resize
  cv::Mat resized;
  float scale = std::min(
    static_cast<float>(input_w_) / image.cols,
    static_cast<float>(input_h_) / image.rows
  );

  int new_w = static_cast<int>(image.cols * scale);
  int new_h = static_cast<int>(image.rows * scale);

  cv::resize(image, resized, cv::Size(new_w, new_h), 0, 0, cv::INTER_LINEAR);

  // Pad to input size
  int pad_w = (input_w_ - new_w) / 2;
  int pad_h = (input_h_ - new_h) / 2;

  cv::Mat padded;
  cv::copyMakeBorder(resized, padded, pad_h, pad_h, pad_w, pad_w,
                     cv::BORDER_CONSTANT, cv::Scalar(114, 114, 114));

  // Convert BGR to RGB
  cv::cvtColor(padded, padded, cv::COLOR_BGR2RGB);

  // Normalize and convert to float
  padded.convertTo(padded, CV_32FC3, 1.0 / 255.0);

  // HWC to CHW format
  std::vector<cv::Mat> channels(3);
  cv::split(padded, channels);

  std::vector<float> input_data;
  input_data.reserve(3 * input_h_ * input_w_);

  for (auto & channel : channels) {
    input_data.insert(input_data.end(), (float*)channel.data,
                     (float*)channel.data + input_h_ * input_w_);
  }

  // Copy to GPU
  cudaMemcpyAsync(gpu_input, input_data.data(),
                  input_data.size() * sizeof(float),
                  cudaMemcpyHostToDevice, stream_);
}

std::vector<Detection> DetectionNode::postprocessOutput(
  float* gpu_output, int output_size, const cv::Size & original_size)
{
  // Copy output from GPU
  std::vector<float> output(output_size);
  cudaMemcpyAsync(output.data(), gpu_output, output_size * sizeof(float),
                  cudaMemcpyDeviceToHost, stream_);
  cudaStreamSynchronize(stream_);

  std::vector<Detection> detections;

  // YOLOv8 output format: [batch, 84, 8400]
  // 84 = 4 (bbox) + 80 (classes)
  int num_classes = 80;
  int num_boxes = output_size / (4 + num_classes);

  for (int i = 0; i < num_boxes; i++) {
    float* box_data = output.data() + i * (4 + num_classes);

    // Get bbox
    float x_center = box_data[0];
    float y_center = box_data[1];
    float width = box_data[2];
    float height = box_data[3];

    // Get class scores
    float max_score = 0.0f;
    int max_class = 0;

    for (int c = 0; c < num_classes; c++) {
      float score = box_data[4 + c];
      if (score > max_score) {
        max_score = score;
        max_class = c;
      }
    }

    if (max_score < conf_threshold_) continue;

    // Convert to corner format and scale to original image
    float scale_x = static_cast<float>(original_size.width) / input_w_;
    float scale_y = static_cast<float>(original_size.height) / input_h_;

    int x_min = static_cast<int>((x_center - width / 2) * scale_x);
    int y_min = static_cast<int>((y_center - height / 2) * scale_y);
    int x_max = static_cast<int>((x_center + width / 2) * scale_x);
    int y_max = static_cast<int>((y_center + height / 2) * scale_y);

    Detection det;
    det.class_id = max_class;
    det.class_name = max_class < class_names_.size() ?
                     class_names_[max_class] : "class_" + std::to_string(max_class);
    det.confidence = max_score;
    det.bbox = cv::Rect(x_min, y_min, x_max - x_min, y_max - y_min);

    detections.push_back(det);
  }

  // Apply NMS
  return nms(detections, nms_threshold_);
}

std::vector<Detection> DetectionNode::nms(
  std::vector<Detection> & detections, float nms_threshold)
{
  // Sort by confidence
  std::sort(detections.begin(), detections.end(),
    [](const Detection & a, const Detection & b) {
      return a.confidence > b.confidence;
    });

  std::vector<Detection> result;
  std::vector<bool> suppressed(detections.size(), false);

  for (size_t i = 0; i < detections.size(); i++) {
    if (suppressed[i]) continue;

    result.push_back(detections[i]);

    for (size_t j = i + 1; j < detections.size(); j++) {
      if (suppressed[j]) continue;

      // Calculate IoU
      cv::Rect inter = detections[i].bbox & detections[j].bbox;
      float inter_area = inter.area();
      float union_area = detections[i].bbox.area() + detections[j].bbox.area() - inter_area;
      float iou = inter_area / union_area;

      if (iou > nms_threshold) {
        suppressed[j] = true;
      }
    }
  }

  return result;
}

void DetectionNode::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  auto start_time = std::chrono::steady_clock::now();

  try {
    // Convert ROS image to OpenCV
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    cv::Mat image = cv_ptr->image;
    cv::Size original_size = image.size();

    // Preprocess on GPU
    preprocessImage(image, static_cast<float*>(buffers_[0]));

    // Run inference using new API
    context_->setTensorAddress(input_tensor_name_.c_str(), buffers_[0]);
    context_->setTensorAddress(output_tensor_name_.c_str(), buffers_[1]);
    context_->enqueueV3(stream_);

    // Postprocess
    std::vector<Detection> detections = postprocessOutput(
      static_cast<float*>(buffers_[1]), output_size_, original_size
    );

    // Create ROS message
    auto detection_array = wheeltec_robot_msg::msg::Detection2DArray();
    detection_array.header = msg->header;

    for (const auto & det : detections) {
      auto detection_msg = wheeltec_robot_msg::msg::Detection2D();
      detection_msg.class_name = det.class_name;
      detection_msg.class_id = det.class_id;
      detection_msg.confidence = det.confidence;

      detection_msg.x_min = det.bbox.x;
      detection_msg.y_min = det.bbox.y;
      detection_msg.x_max = det.bbox.x + det.bbox.width;
      detection_msg.y_max = det.bbox.y + det.bbox.height;

      // Normalized coordinates
      detection_msg.x_center = (det.bbox.x + det.bbox.width / 2.0f) / original_size.width;
      detection_msg.y_center = (det.bbox.y + det.bbox.height / 2.0f) / original_size.height;
      detection_msg.width = det.bbox.width / static_cast<float>(original_size.width);
      detection_msg.height = det.bbox.height / static_cast<float>(original_size.height);

      detection_array.detections.push_back(detection_msg);
    }

    detections_pub_->publish(detection_array);

    // Calculate and publish latency
    auto end_time = std::chrono::steady_clock::now();
    float latency = std::chrono::duration<float, std::milli>(end_time - start_time).count();

    auto latency_msg = std_msgs::msg::Float32();
    latency_msg.data = latency;
    latency_pub_->publish(latency_msg);

    // Update FPS
    frame_count_++;
    auto current_time = std::chrono::steady_clock::now();
    float elapsed = std::chrono::duration<float>(current_time - last_fps_time_).count();

    if (elapsed >= 1.0f) {
      fps_ = frame_count_ / elapsed;
      frame_count_ = 0;
      last_fps_time_ = current_time;

      auto fps_msg = std_msgs::msg::Float32();
      fps_msg.data = fps_;
      fps_pub_->publish(fps_msg);

      RCLCPP_INFO(this->get_logger(),
        "FPS: %.1f, Latency: %.1fms, Detections: %zu",
        fps_, latency, detections.size());
    }

  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Detection error: %s", e.what());
  }
}

}  // namespace wheeltec_robot_detection

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(wheeltec_robot_detection::DetectionNode)
