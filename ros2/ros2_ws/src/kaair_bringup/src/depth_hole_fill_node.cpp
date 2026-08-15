// depth_hole_fill_node.cpp
//
// /hand/camera/depth/image_rect_raw 를 구독해, 노이즈 등으로 인해 값이
// 0(무효)으로 비어있는 픽셀을 주변 유효 픽셀들의 median 값으로 채운 뒤,
// 해상도를 그대로 유지한 채 재발행한다. 실제로 측정된 유효 depth 값은
// 건드리지 않고, 비어있는 홀(hole)만 메운다.
//
// 알고리즘 (프레임당, fill_passes 회 반복):
//   1. 입력 이미지를 그대로 출력 버퍼로 복사한다.
//   2. 입력에서 값이 0(무효)인 픽셀마다, (kernel_size x kernel_size) 윈도우
//      내에서 "입력" 이미지의 0이 아닌(유효) 값들을 모아 median을 계산한다.
//      (수정 중인 출력이 아니라 원본 입력만 참조하므로, 픽셀을 순회하는
//      순서에 결과가 좌우되지 않는다.)
//   3. 윈도우 내 유효 픽셀 수가 min_valid_neighbors 이상이면 그 median 값으로
//      채우고, 부족하면 이번 pass에서는 그대로 무효(0)로 남긴다.
//   4. 이전 pass의 출력을 다음 pass의 입력으로 사용해 반복하면, 한 번에
//      채우기 어려운 큰 홀도 바깥쪽부터 점진적으로 채워진다.
//
// 지원 인코딩: 16UC1(mm, RealSense 기본), mono16, 32FC1(m, 0 또는 NaN을 무효로 간주)
//
// image_transport::create_publisher 를 사용하므로, 시스템에 설치된 transport
// 플러그인에 따라 output_topic 하위에 raw 외 압축 토픽도 자동으로 함께 생성된다.

#include <algorithm>
#include <atomic>
#include <cmath>
#include <string>
#include <vector>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <opencv2/core.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>

using sensor_msgs::msg::Image;

namespace
{

template <typename T>
inline bool isInvalidDepth(T value);

template <>
inline bool isInvalidDepth<uint16_t>(uint16_t value)
{
  return value == 0;
}

template <>
inline bool isInvalidDepth<float>(float value)
{
  return value == 0.0f || std::isnan(value);
}

// 입력 이미지(input)를 참조해 0(무효) 픽셀을 median으로 채운 결과를
// output에 담는다. output이 input과 별도의 버퍼이므로 pass 내 순서 의존성이 없다.
template <typename T>
void fillHolesOnePass(const cv::Mat & input, cv::Mat & output, int kernel_size, int min_valid_neighbors)
{
  const int radius = kernel_size / 2;
  const int rows = input.rows;
  const int cols = input.cols;

  input.copyTo(output);

  std::vector<T> neighbors;
  neighbors.reserve(static_cast<size_t>(kernel_size) * static_cast<size_t>(kernel_size));

  for (int v = 0; v < rows; ++v) {
    const T * in_row = input.ptr<T>(v);
    T * out_row = output.ptr<T>(v);
    for (int u = 0; u < cols; ++u) {
      if (!isInvalidDepth<T>(in_row[u])) {
        continue;  // 이미 유효한 값은 건드리지 않는다
      }

      neighbors.clear();
      const int v0 = std::max(0, v - radius);
      const int v1 = std::min(rows - 1, v + radius);
      const int u0 = std::max(0, u - radius);
      const int u1 = std::min(cols - 1, u + radius);
      for (int vv = v0; vv <= v1; ++vv) {
        const T * win_row = input.ptr<T>(vv);
        for (int uu = u0; uu <= u1; ++uu) {
          const T val = win_row[uu];
          if (!isInvalidDepth<T>(val)) {
            neighbors.push_back(val);
          }
        }
      }

      if (static_cast<int>(neighbors.size()) >= min_valid_neighbors) {
        const size_t mid = neighbors.size() / 2;
        std::nth_element(neighbors.begin(), neighbors.begin() + mid, neighbors.end());
        out_row[u] = neighbors[mid];
      }
    }
  }
}

}  // namespace

class DepthHoleFillNode : public rclcpp::Node
{
public:
  DepthHoleFillNode()
  : Node("depth_hole_fill_node")
  {
    // ---- Parameters ----
    input_topic_ = this->declare_parameter<std::string>(
      "input_topic", "/hand/camera/depth/image_rect_raw");
    output_topic_ = this->declare_parameter<std::string>(
      "output_topic", "/hand/camera/depth/image_rect_filled");
    input_transport_ = this->declare_parameter<std::string>("input_transport", "raw");

    kernel_size_ = this->declare_parameter<int>("kernel_size", 5);
    if (kernel_size_ < 3) {
      kernel_size_ = 3;
    }
    if (kernel_size_ % 2 == 0) {
      ++kernel_size_;  // median 윈도우는 홀수 크기만 허용
    }

    min_valid_neighbors_ = this->declare_parameter<int>("min_valid_neighbors", 4);
    if (min_valid_neighbors_ < 1) {
      min_valid_neighbors_ = 1;
    }

    fill_passes_ = this->declare_parameter<int>("fill_passes", 2);
    if (fill_passes_ < 1) {
      fill_passes_ = 1;
    }

    // kernel_size / min_valid_neighbors / fill_passes 는 노드를 재시작하지
    // 않고도 `ros2 param set /depth_hole_filler <name> <value>` 로 실시간
    // 조정할 수 있도록 콜백을 등록한다 (topic/transport 파라미터는 구독/발행
    // 객체를 다시 만들어야 하므로 런타임 변경을 거부한다).
    param_callback_handle_ = this->add_on_set_parameters_callback(
      std::bind(&DepthHoleFillNode::onParamChange, this, std::placeholders::_1));

    auto qos = rclcpp::QoS(rclcpp::KeepLast(5)).reliable().get_rmw_qos_profile();

    pub_ = image_transport::create_publisher(this, output_topic_, qos);
    sub_ = image_transport::create_subscription(
      this, input_topic_,
      std::bind(&DepthHoleFillNode::imageCallback, this, std::placeholders::_1),
      input_transport_, qos);

    RCLCPP_INFO(
      this->get_logger(),
      "depth_hole_fill_node: %s (%s transport) -> %s, kernel=%dx%d "
      "min_valid_neighbors=%d fill_passes=%d",
      input_topic_.c_str(), input_transport_.c_str(), output_topic_.c_str(),
      kernel_size_.load(), kernel_size_.load(), min_valid_neighbors_.load(),
      fill_passes_.load());
  }

private:
  rcl_interfaces::msg::SetParametersResult onParamChange(
    const std::vector<rclcpp::Parameter> & params)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    for (const auto & p : params) {
      if (p.get_name() == "kernel_size") {
        int v = p.as_int();
        if (v < 3) {
          v = 3;
        }
        if (v % 2 == 0) {
          ++v;
        }
        kernel_size_.store(v);
      } else if (p.get_name() == "min_valid_neighbors") {
        int v = p.as_int();
        if (v < 1) {
          v = 1;
        }
        min_valid_neighbors_.store(v);
      } else if (p.get_name() == "fill_passes") {
        int v = p.as_int();
        if (v < 1) {
          v = 1;
        }
        fill_passes_.store(v);
      } else if (p.get_name() == "input_topic" || p.get_name() == "output_topic" ||
        p.get_name() == "input_transport")
      {
        result.successful = false;
        result.reason = "topic/transport 파라미터는 런타임에 변경할 수 없습니다 (노드 재시작 필요)";
      }
    }

    if (result.successful) {
      RCLCPP_INFO(
        this->get_logger(),
        "필터 파라미터 갱신: kernel=%dx%d min_valid_neighbors=%d fill_passes=%d",
        kernel_size_.load(), kernel_size_.load(), min_valid_neighbors_.load(),
        fill_passes_.load());
    }
    return result;
  }

  void imageCallback(const Image::ConstSharedPtr & msg)
  {
    cv_bridge::CvImageConstPtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvShare(msg, msg->encoding);
    } catch (const cv_bridge::Exception & e) {
      RCLCPP_ERROR_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "cv_bridge 변환 실패: %s", e.what());
      return;
    }

    // 프레임 하나를 처리하는 동안은 값이 바뀌어도 일관되도록 한 번만 읽어둔다.
    const int kernel_size = kernel_size_.load();
    const int min_valid_neighbors = min_valid_neighbors_.load();
    const int fill_passes = fill_passes_.load();

    cv::Mat filled;
    if (msg->encoding == sensor_msgs::image_encodings::TYPE_16UC1 ||
      msg->encoding == sensor_msgs::image_encodings::MONO16)
    {
      filled = runPasses<uint16_t>(cv_ptr->image, kernel_size, min_valid_neighbors, fill_passes);
    } else if (msg->encoding == sensor_msgs::image_encodings::TYPE_32FC1) {
      filled = runPasses<float>(cv_ptr->image, kernel_size, min_valid_neighbors, fill_passes);
    } else {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "지원하지 않는 depth 인코딩: %s (16UC1/mono16/32FC1만 지원) - "
        "필터링 없이 원본 그대로 재발행",
        msg->encoding.c_str());
      pub_.publish(msg);
      return;
    }

    cv_bridge::CvImage out;
    out.header = msg->header;
    out.encoding = msg->encoding;
    out.image = filled;
    pub_.publish(*out.toImageMsg());
  }

  template <typename T>
  cv::Mat runPasses(const cv::Mat & input, int kernel_size, int min_valid_neighbors, int fill_passes)
  {
    cv::Mat current = input;
    for (int i = 0; i < fill_passes; ++i) {
      cv::Mat next;
      fillHolesOnePass<T>(current, next, kernel_size, min_valid_neighbors);
      current = next;
    }
    return current;
  }

  std::string input_topic_, output_topic_, input_transport_;
  std::atomic<int> kernel_size_{5};
  std::atomic<int> min_valid_neighbors_{4};
  std::atomic<int> fill_passes_{2};

  image_transport::Publisher pub_;
  image_transport::Subscriber sub_;
  OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DepthHoleFillNode>());
  rclcpp::shutdown();
  return 0;
}
