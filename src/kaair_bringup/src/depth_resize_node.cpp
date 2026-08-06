// depth_resize_node.cpp
//
// /femto/depth/image_raw/compressedDepth 를 구독하여(디코딩은 image_transport 플러그인이 처리)
// 해상도가 정확히 576(h) x 640(w) 일 때만 720(h) x 1280(w) 로 리사이즈한 뒤
// /femto/depth/aligned 를 base topic 으로 재발행한다.
//
// image_transport::create_publisher 를 사용하므로, 시스템에 설치된 transport
// 플러그인에 따라 아래 토픽들이 "자동으로" 함께 생성된다:
//   /femto/depth/aligned                (raw)
//   /femto/depth/aligned/compressedDepth (compressed_depth_image_transport)
//   /femto/depth/aligned/compressed      (compressed_image_transport, 지원되는 인코딩인 경우)
//
// header(stamp, frame_id)는 원본 그대로 복사하여 TF/시간 동기화가 깨지지 않도록 한다.
// depth 값 왜곡을 피하기 위해 기본 보간법은 INTER_NEAREST 사용.

#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

using sensor_msgs::msg::Image;

class DepthResizeNode : public rclcpp::Node
{
public:
  DepthResizeNode()
  : Node("depth_resize_node")
  {
    // ---- Parameters ----
    input_topic_ = this->declare_parameter<std::string>(
      "input_topic", "/femto/depth/image_raw");
    output_topic_ = this->declare_parameter<std::string>(
      "output_topic", "/femto/depth/aligned");
    // "compressedDepth" | "raw" | "compressed" 등 image_transport 플러그인 이름
    input_transport_ = this->declare_parameter<std::string>(
      "input_transport", "compressedDepth");

    expected_h_ = this->declare_parameter<int>("expected_input_height", 576);
    expected_w_ = this->declare_parameter<int>("expected_input_width", 640);
    out_h_ = this->declare_parameter<int>("output_height", 720);
    out_w_ = this->declare_parameter<int>("output_width", 1280);

    const std::string interp_str = this->declare_parameter<std::string>(
      "interpolation", "nearest");
    if (interp_str == "linear") {
      interpolation_ = cv::INTER_LINEAR;
    } else if (interp_str == "area") {
      interpolation_ = cv::INTER_AREA;
    } else {
      interpolation_ = cv::INTER_NEAREST;
    }

    // 센서 데이터 QoS (BEST_EFFORT) - 카메라 드라이버 기본값과 호환
    const auto qos = rclcpp::SensorDataQoS().get_rmw_qos_profile();

    // base topic 에 publisher 를 만들면 raw + 사용 가능한 모든 transport 플러그인이
    // 자동으로 함께 advertise 된다 (compressedDepth 포함).
    pub_ = image_transport::create_publisher(this, output_topic_, qos);

    // 명시적으로 "compressedDepth" transport 로 구독 -> 실제로는
    // <input_topic_>/compressedDepth 토픽을 구독하고, 플러그인이 내부적으로
    // PNG 디코딩 + depth 역양자화(inverse quantization)를 수행해서 raw Image 로 넘겨준다.
    sub_ = image_transport::create_subscription(
      this, input_topic_,
      std::bind(&DepthResizeNode::imageCallback, this, std::placeholders::_1),
      input_transport_, qos);

    RCLCPP_INFO(
      this->get_logger(),
      "depth_resize_node: %s (%s transport, expect %dx%d) -> %s (%dx%d), interp=%s",
      input_topic_.c_str(), input_transport_.c_str(), expected_w_, expected_h_,
      output_topic_.c_str(), out_w_, out_h_, interp_str.c_str());
  }

private:
  void imageCallback(const Image::ConstSharedPtr & msg)
  {
    if (static_cast<int>(msg->height) != expected_h_ ||
      static_cast<int>(msg->width) != expected_w_)
    {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Unexpected depth resolution %ux%u (expected %dx%d) - dropping frame",
        msg->width, msg->height, expected_w_, expected_h_);
      return;
    }

    cv_bridge::CvImageConstPtr cv_ptr;
    try {
      // passthrough: 16UC1 / 32FC1 등 원본 인코딩 그대로 유지
      cv_ptr = cv_bridge::toCvShare(msg, msg->encoding);
    } catch (const cv_bridge::Exception & e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }

    cv::Mat resized;
    try {
      cv::resize(
        cv_ptr->image, resized, cv::Size(out_w_, out_h_),
        0, 0, interpolation_);
    } catch (const cv::Exception & e) {
      RCLCPP_ERROR(this->get_logger(), "cv::resize exception: %s", e.what());
      return;
    }

    cv_bridge::CvImage out_img;
    out_img.header = msg->header;   // stamp/frame_id 원본 유지 -> TF 동기화 유지
    out_img.encoding = msg->encoding;
    out_img.image = resized;

    pub_.publish(*out_img.toImageMsg());
  }

  std::string input_topic_, output_topic_, input_transport_;
  int expected_h_{0}, expected_w_{0}, out_h_{0}, out_w_{0};
  int interpolation_{cv::INTER_NEAREST};

  image_transport::Publisher pub_;
  image_transport::Subscriber sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DepthResizeNode>());
  rclcpp::shutdown();
  return 0;
}
