// depth_resize_node.cpp
//
// /femto/depth/image_raw 를 구독하여(입력 transport 는 파라미터로 설정, 기본
// compressedDepth) 수신된 이미지의 해상도가 정확히 720(h) x 1280(w) 인 경우에만
// 그대로(리사이즈 없이) /femto/depth/aligned 로 재발행한다.
//
// depth_registration 이 켜진 카메라 드라이버가 이미 color 해상도(720x1280)에
// 맞춰 정렬된 depth 를 출력한다고 가정하므로, 이 노드는 리사이즈를 하지 않고
// 해상도 검증 + 재발행만 수행한다.
// 해상도가 기대값과 다르면 (카메라가 튀는 경우) 재발행하지 않고 throttle 경고만 남긴다.
// 메시지를 그대로 전달하므로 header(stamp, frame_id)를 포함한 모든 필드가
// 원본과 동일하게 유지되어 TF/시간 동기화가 깨지지 않는다.
//
// image_transport::create_publisher 를 사용하므로, 시스템에 설치된 transport
// 플러그인에 따라 아래 토픽들이 "자동으로" 함께 생성된다:
//   /femto/depth/aligned                (raw)
//   /femto/depth/aligned/compressedDepth (compressed_depth_image_transport)
//   /femto/depth/aligned/compressed      (compressed_image_transport, 지원되는 인코딩인 경우)

#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>

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

    // 재발행을 허용할 유일한 해상도 (depth_registration 후 기대되는 해상도)
    expected_h_ = this->declare_parameter<int>("expected_height", 720);
    expected_w_ = this->declare_parameter<int>("expected_width", 1280);

    // RELIABLE QoS로 고정 (구독/발행 모두). 상대측(카메라 드라이버/구독자)의
    // QoS 도 RELIABLE 이어야 실제로 연결된다.
    auto qos = rclcpp::QoS(rclcpp::KeepLast(5)).reliable().get_rmw_qos_profile();

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
      "depth_resize_node: %s (%s transport) -> %s, expect %dx%d (해상도 불일치 시 drop)",
      input_topic_.c_str(), input_transport_.c_str(), output_topic_.c_str(),
      expected_w_, expected_h_);
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

    // 리사이즈 없이 그대로 재발행 (해상도 검증 통과 시에만 도달)
    pub_.publish(msg);
  }

  std::string input_topic_, output_topic_, input_transport_;
  int expected_h_{0}, expected_w_{0};

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
