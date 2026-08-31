// depth_resize_node.cpp
//
// depth_registration 이 켜진 카메라 드라이버가 이미 color 해상도(720x1280)에
// 맞춰 정렬된 depth 를 출력한다고 가정하고, 해상도 검증 + 재발행만 수행한다.
// 해상도가 기대값과 다르면(카메라가 튀는 경우) 재발행하지 않고 throttle 경고만
// 남긴다. 메시지를 그대로 전달하므로 header(stamp, frame_id) 등 모든 필드가
// 원본과 동일하게 유지되어 TF/시간 동기화가 깨지지 않는다.
//
// raw 경로와 compressedDepth 경로를 서로 다른 콜백 그룹(=별도 executor 스레드)
// 에서 독립적으로 처리한다. 두 경로 중 하나가 느려져도(예: 다른 노드가 compressed
// 를 구독해서 부하가 걸리는 경우) 나머지 경로의 처리가 밀리지 않게 하기 위함이다.
//
//   /femto/depth/image_raw                -> /femto/depth/aligned
//   /femto/depth/image_raw/compressedDepth -> /femto/depth/aligned/compressedDepth
//
// compressedDepth 경로는 image_transport 플러그인을 거치지 않고
// sensor_msgs/CompressedImage 토픽을 직접 구독/발행한다. 즉 PNG 디코드/재인코드를
// 전혀 하지 않는다(byte-for-byte passthrough). 해상도 판별도 압축을 풀지 않고,
// compressed_depth_image_transport 가 채워 넣는 [ConfigHeader][PNG 파일] 구조에서
// PNG의 첫 번째 청크인 IHDR(항상 고정 오프셋)만 읽어서 width/height 를 얻는다.

#include <algorithm>
#include <array>
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>

using sensor_msgs::msg::CompressedImage;
using sensor_msgs::msg::Image;

namespace
{
constexpr std::array<uint8_t, 8> kPngSignature =
{0x89, 'P', 'N', 'G', '\r', '\n', 0x1A, '\n'};

// compressed_depth_image_transport::ConfigHeader 의 wire-format(2012년부터 불변)을
// 그대로 복제한 것. 그 패키지는 ament_cmake export를 제공하지 않아(런타임
// pluginlib 전용 설계) 헤더를 직접 include할 수 없으므로, 크기 계산에 필요한
// 만큼만 로컬로 정의해서 쓴다. data 레이아웃 = [format(4B)][depthParam[2](8B)] + PNG.
struct DepthConfigHeaderLayout
{
  int32_t format;
  float depth_param[2];
};
static_assert(
  sizeof(DepthConfigHeaderLayout) == 12,
  "compressed_depth_image_transport ConfigHeader 레이아웃 가정이 깨졌습니다");

uint32_t readBigEndianU32(const uint8_t * p)
{
  return (static_cast<uint32_t>(p[0]) << 24) |
         (static_cast<uint32_t>(p[1]) << 16) |
         (static_cast<uint32_t>(p[2]) << 8) |
         static_cast<uint32_t>(p[3]);
}
}  // namespace

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

    expected_h_ = this->declare_parameter<int>("expected_height", 720);
    expected_w_ = this->declare_parameter<int>("expected_width", 1280);

    // compressedDepth 구독/재발행 경로 on/off. 카메라 드라이버 쪽 compressedDepth
    // 퍼블리셔에 RELIABLE 구독자가 붙으면(이 노드 포함) 다른 스트림(RGB 등)에
    // 지연을 유발할 수 있어, 실제로 필요할 때만 켜서 쓸 수 있게 파라미터화한다.
    enable_compressed_depth_ = this->declare_parameter<bool>(
      "enable_compressed_depth", true);

    // RELIABLE QoS로 고정 (구독/발행 모두). 상대측(카메라 드라이버/구독자)의
    // QoS 도 RELIABLE 이어야 실제로 연결된다.
    auto qos = rclcpp::QoS(rclcpp::KeepLast(5)).reliable();

    // raw / compressedDepth 각각 별도 콜백 그룹 -> main()의 MultiThreadedExecutor
    // 에서 서로 다른 스레드로 처리된다.
    raw_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    // ---- raw 경로 ----
    raw_pub_ = this->create_publisher<Image>(output_topic_, qos);

    rclcpp::SubscriptionOptions raw_sub_opts;
    raw_sub_opts.callback_group = raw_cb_group_;
    raw_sub_ = this->create_subscription<Image>(
      input_topic_, qos,
      std::bind(&DepthResizeNode::rawImageCallback, this, std::placeholders::_1),
      raw_sub_opts);

    // ---- compressedDepth 경로 (디코드/인코드 없이 그대로 통과, 파라미터로 on/off) ----
    const std::string compressed_input_topic = input_topic_ + "/compressedDepth";
    const std::string compressed_output_topic = output_topic_ + "/compressedDepth";

    if (enable_compressed_depth_) {
      compressed_cb_group_ =
        this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

      compressed_pub_ = this->create_publisher<CompressedImage>(compressed_output_topic, qos);

      rclcpp::SubscriptionOptions compressed_sub_opts;
      compressed_sub_opts.callback_group = compressed_cb_group_;
      compressed_sub_ = this->create_subscription<CompressedImage>(
        compressed_input_topic, qos,
        std::bind(&DepthResizeNode::compressedDepthCallback, this, std::placeholders::_1),
        compressed_sub_opts);
    }

    RCLCPP_INFO(
      this->get_logger(),
      "depth_resize_node: [raw] %s -> %s, [compressedDepth] %s, expect %dx%d "
      "(해상도 불일치 시 drop)",
      input_topic_.c_str(), output_topic_.c_str(),
      enable_compressed_depth_
        ? (compressed_input_topic + " -> " + compressed_output_topic).c_str()
        : "disabled",
      expected_w_, expected_h_);
  }

private:
  void rawImageCallback(const Image::ConstSharedPtr & msg)
  {
    if (static_cast<int>(msg->height) != expected_h_ ||
      static_cast<int>(msg->width) != expected_w_)
    {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "[raw] Unexpected depth resolution %ux%u (expected %dx%d) - dropping frame",
        msg->width, msg->height, expected_w_, expected_h_);
      return;
    }

    raw_pub_->publish(*msg);
  }

  void compressedDepthCallback(const CompressedImage::ConstSharedPtr & msg)
  {
    uint32_t width = 0;
    uint32_t height = 0;
    if (!readPngDimensions(msg->data, width, height)) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "[compressedDepth] Failed to parse PNG header - dropping frame");
      return;
    }

    if (static_cast<int>(height) != expected_h_ || static_cast<int>(width) != expected_w_) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "[compressedDepth] Unexpected depth resolution %ux%u (expected %dx%d) - dropping frame",
        width, height, expected_w_, expected_h_);
      return;
    }

    // 압축을 풀지 않고 그대로 재발행 (byte-for-byte passthrough)
    compressed_pub_->publish(*msg);
  }

  // compressed_depth_image_transport 는 data 를 [ConfigHeader][PNG 파일 전체]
  // 형태로 채운다. PNG의 첫 번째 청크(IHDR, 항상 고정 오프셋)만 읽어서 압축을
  // 풀지 않고 width/height 를 얻는다.
  //   +0                : ConfigHeader (format + depthParam[2])
  //   +kHeaderOffset+0  ~ +8  (8B) : PNG 시그니처
  //   +kHeaderOffset+8  ~ +12 (4B) : IHDR 청크 길이
  //   +kHeaderOffset+12 ~ +16 (4B) : 청크 타입 "IHDR"
  //   +kHeaderOffset+16 ~ +20 (4B) : width  (big-endian)
  //   +kHeaderOffset+20 ~ +24 (4B) : height (big-endian)
  static bool readPngDimensions(
    const std::vector<uint8_t> & data, uint32_t & width, uint32_t & height)
  {
    constexpr size_t kHeaderOffset = sizeof(DepthConfigHeaderLayout);
    constexpr size_t kIhdrTypeOffset = kHeaderOffset + 12;
    constexpr size_t kWidthOffset = kHeaderOffset + 16;
    constexpr size_t kNeeded = kHeaderOffset + 24;

    if (data.size() < kNeeded) {
      return false;
    }
    if (!std::equal(kPngSignature.begin(), kPngSignature.end(), data.begin() + kHeaderOffset)) {
      return false;
    }
    if (std::memcmp(&data[kIhdrTypeOffset], "IHDR", 4) != 0) {
      return false;
    }

    width = readBigEndianU32(&data[kWidthOffset]);
    height = readBigEndianU32(&data[kWidthOffset + 4]);
    return true;
  }

  std::string input_topic_, output_topic_;
  int expected_h_{0}, expected_w_{0};
  bool enable_compressed_depth_{true};

  rclcpp::CallbackGroup::SharedPtr raw_cb_group_;
  rclcpp::CallbackGroup::SharedPtr compressed_cb_group_;

  rclcpp::Publisher<Image>::SharedPtr raw_pub_;
  rclcpp::Subscription<Image>::SharedPtr raw_sub_;

  rclcpp::Publisher<CompressedImage>::SharedPtr compressed_pub_;
  rclcpp::Subscription<CompressedImage>::SharedPtr compressed_sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DepthResizeNode>();

  // raw/compressedDepth 콜백이 서로 다른 콜백 그룹에 있으므로, 멀티스레드
  // executor로 돌려야 실제로 병렬 처리된다(SingleThreadedExecutor는 콜백
  // 그룹을 나눠도 여전히 순차 처리한다).
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
