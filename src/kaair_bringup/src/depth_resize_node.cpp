// depth_resize_node.cpp
//
// /femto/depth/image_raw 를 구독하여(입력 transport 는 파라미터로 설정, 기본
// compressedDepth) 수신된 이미지의 해상도가 정확히 720(h) x 1280(w) 인 경우에만
// depth 일관성 필터를 적용한 뒤 /femto/depth/aligned 로 재발행한다.
//
// depth_registration 이 켜진 카메라 드라이버가 이미 color 해상도(720x1280)에
// 맞춰 정렬된 depth 를 출력한다고 가정한다. 해상도가 기대값과 다르면 drop.
//
// Depth 일관성 필터 (포인트클라우드가 기울어지거나 "쉬프팅"되어 보이는
// 원인인 flying-pixel / 순간 depth 점프를 완화):
//   1) Spatial consistency:
//      각 유효 픽셀을 이웃(kernel) median 과 비교해
//      |z - median| > max(abs_thresh, rel_ratio * median) 이면
//      median 으로 교체(또는 invalidate). 실제 표면과 크게 어긋난
//      outlier depth만 바로잡고, 부드러운 표면은 유지한다.
//   2) Temporal EMA (선택):
//      직전 프레임과 블렌딩해 프레임 간 depth 흔들림을 줄인다.
//      갑자기 크게 점프한 픽셀은 이전 프레임 쪽 가중치를 높인다.
//
// 지원 인코딩: 16UC1/mono16(mm), 32FC1(m)
//
// image_transport::create_publisher 를 사용하므로 raw + compressedDepth 등이
// output_topic 하위에 자동으로 함께 advertise 된다.

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <opencv2/core.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>

using sensor_msgs::msg::Image;

namespace
{

constexpr float kInvalidDepth = std::numeric_limits<float>::quiet_NaN();

inline bool isInvalidFloat(float z)
{
  return !std::isfinite(z) || z <= 0.0f;
}

// 16UC1/mono16(mm) 또는 32FC1(m) → float meters (무효=NaN)
bool decodeToMeters(const cv::Mat & src, const std::string & encoding, cv::Mat & meters)
{
  if (encoding == sensor_msgs::image_encodings::TYPE_16UC1 ||
    encoding == sensor_msgs::image_encodings::MONO16)
  {
    meters.create(src.rows, src.cols, CV_32FC1);
    for (int v = 0; v < src.rows; ++v) {
      const uint16_t * in = src.ptr<uint16_t>(v);
      float * out = meters.ptr<float>(v);
      for (int u = 0; u < src.cols; ++u) {
        out[u] = (in[u] == 0) ? kInvalidDepth : static_cast<float>(in[u]) * 0.001f;
      }
    }
    return true;
  }

  if (encoding == sensor_msgs::image_encodings::TYPE_32FC1) {
    meters.create(src.rows, src.cols, CV_32FC1);
    for (int v = 0; v < src.rows; ++v) {
      const float * in = src.ptr<float>(v);
      float * out = meters.ptr<float>(v);
      for (int u = 0; u < src.cols; ++u) {
        const float z = in[u];
        out[u] = (!std::isfinite(z) || z <= 0.0f) ? kInvalidDepth : z;
      }
    }
    return true;
  }

  return false;
}

// float meters → 원본 인코딩 Mat
cv::Mat encodeFromMeters(const cv::Mat & meters, const std::string & encoding)
{
  if (encoding == sensor_msgs::image_encodings::TYPE_16UC1 ||
    encoding == sensor_msgs::image_encodings::MONO16)
  {
    cv::Mat out(meters.rows, meters.cols, CV_16UC1);
    for (int v = 0; v < meters.rows; ++v) {
      const float * in = meters.ptr<float>(v);
      uint16_t * o = out.ptr<uint16_t>(v);
      for (int u = 0; u < meters.cols; ++u) {
        if (isInvalidFloat(in[u])) {
          o[u] = 0;
        } else {
          const float mm = in[u] * 1000.0f;
          o[u] = static_cast<uint16_t>(std::min(std::max(mm + 0.5f, 0.0f), 65535.0f));
        }
      }
    }
    return out;
  }

  // 32FC1: 무효는 0.0으로 기록 (구독측 object_marker_server 등과 호환)
  cv::Mat out(meters.rows, meters.cols, CV_32FC1);
  for (int v = 0; v < meters.rows; ++v) {
    const float * in = meters.ptr<float>(v);
    float * o = out.ptr<float>(v);
    for (int u = 0; u < meters.cols; ++u) {
      o[u] = isInvalidFloat(in[u]) ? 0.0f : in[u];
    }
  }
  return out;
}

// 이웃 median 대비 차이가 큰 depth를 median으로 교체(또는 invalidate).
// 입력/출력은 float meters, 무효=NaN. 입력만 참조해 순서 의존성 없음.
void applySpatialConsistency(
  const cv::Mat & input,
  cv::Mat & output,
  int kernel_size,
  float abs_thresh_m,
  float rel_ratio,
  int min_valid_neighbors,
  bool invalidate_outliers)
{
  const int radius = kernel_size / 2;
  const int rows = input.rows;
  const int cols = input.cols;
  input.copyTo(output);

  std::vector<float> neighbors;
  neighbors.reserve(static_cast<size_t>(kernel_size) * static_cast<size_t>(kernel_size));

  for (int v = 0; v < rows; ++v) {
    const float * in_row = input.ptr<float>(v);
    float * out_row = output.ptr<float>(v);
    for (int u = 0; u < cols; ++u) {
      const float z = in_row[u];
      if (isInvalidFloat(z)) {
        continue;
      }

      neighbors.clear();
      const int v0 = std::max(0, v - radius);
      const int v1 = std::min(rows - 1, v + radius);
      const int u0 = std::max(0, u - radius);
      const int u1 = std::min(cols - 1, u + radius);
      for (int vv = v0; vv <= v1; ++vv) {
        const float * win = input.ptr<float>(vv);
        for (int uu = u0; uu <= u1; ++uu) {
          const float nz = win[uu];
          if (!isInvalidFloat(nz)) {
            neighbors.push_back(nz);
          }
        }
      }

      if (static_cast<int>(neighbors.size()) < min_valid_neighbors) {
        continue;
      }

      const size_t mid = neighbors.size() / 2;
      std::nth_element(neighbors.begin(), neighbors.begin() + mid, neighbors.end());
      const float med = neighbors[mid];
      const float thresh = std::max(abs_thresh_m, rel_ratio * med);
      if (std::fabs(z - med) > thresh) {
        out_row[u] = invalidate_outliers ? kInvalidDepth : med;
      }
    }
  }
}

// 직전 프레임과 EMA 블렌딩. 큰 점프는 이전 프레임 가중을 높여 "쉬프팅" 억제.
void applyTemporalEma(
  const cv::Mat & current,
  const cv::Mat & previous,
  cv::Mat & output,
  float alpha,
  float jump_thresh_m)
{
  output.create(current.rows, current.cols, CV_32FC1);
  const float keep = std::clamp(alpha, 0.0f, 0.95f);

  for (int v = 0; v < current.rows; ++v) {
    const float * cur = current.ptr<float>(v);
    const float * prev = previous.ptr<float>(v);
    float * out = output.ptr<float>(v);
    for (int u = 0; u < current.cols; ++u) {
      const float c = cur[u];
      const float p = prev[u];
      if (isInvalidFloat(c)) {
        out[u] = p;  // 현재 무효면 이전 값 유지(홀 깜빡임 완화)
        continue;
      }
      if (isInvalidFloat(p)) {
        out[u] = c;
        continue;
      }

      float a = keep;
      if (std::fabs(c - p) > jump_thresh_m) {
        // 급격한 점프: 이전 프레임을 더 신뢰 (쉬프팅/기울어짐 스파이크 억제)
        a = std::max(a, 0.7f);
      }
      out[u] = (1.0f - a) * c + a * p;
    }
  }
}

}  // namespace

class DepthResizeNode : public rclcpp::Node
{
public:
  DepthResizeNode()
  : Node("depth_resize_node")
  {
    input_topic_ = this->declare_parameter<std::string>(
      "input_topic", "/femto/depth/image_raw");
    output_topic_ = this->declare_parameter<std::string>(
      "output_topic", "/femto/depth/aligned");
    input_transport_ = this->declare_parameter<std::string>(
      "input_transport", "compressedDepth");

    expected_h_ = this->declare_parameter<int>("expected_height", 720);
    expected_w_ = this->declare_parameter<int>("expected_width", 1280);

    enable_filter_ = this->declare_parameter<bool>("enable_depth_filter", true);

    // Spatial consistency: 이웃 median과 이 이상 차이나면 보정
    consistency_kernel_ = this->declare_parameter<int>("consistency_kernel_size", 5);
    if (consistency_kernel_ < 3) {
      consistency_kernel_ = 3;
    }
    if (consistency_kernel_ % 2 == 0) {
      ++consistency_kernel_;
    }
    abs_thresh_m_ = static_cast<float>(
      this->declare_parameter<double>("max_depth_diff_m", 0.05));
    rel_ratio_ = static_cast<float>(
      this->declare_parameter<double>("max_depth_diff_ratio", 0.05));
    min_valid_neighbors_ = this->declare_parameter<int>("min_valid_neighbors", 5);
    invalidate_outliers_ = this->declare_parameter<bool>("invalidate_outliers", false);

    // Temporal EMA: 0이면 비활성. 0.2~0.4 권장
    temporal_alpha_ = static_cast<float>(
      this->declare_parameter<double>("temporal_alpha", 0.25));
    temporal_jump_m_ = static_cast<float>(
      this->declare_parameter<double>("temporal_jump_m", 0.08));

    auto qos = rclcpp::QoS(rclcpp::KeepLast(5)).reliable().get_rmw_qos_profile();
    pub_ = image_transport::create_publisher(this, output_topic_, qos);
    sub_ = image_transport::create_subscription(
      this, input_topic_,
      std::bind(&DepthResizeNode::imageCallback, this, std::placeholders::_1),
      input_transport_, qos);

    RCLCPP_INFO(
      this->get_logger(),
      "depth_resize_node: %s (%s) -> %s, expect %dx%d, filter=%s "
      "kernel=%d abs=%.3fm rel=%.2f temporal_alpha=%.2f",
      input_topic_.c_str(), input_transport_.c_str(), output_topic_.c_str(),
      expected_w_, expected_h_, enable_filter_ ? "on" : "off",
      consistency_kernel_, abs_thresh_m_, rel_ratio_, temporal_alpha_);
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

    if (!enable_filter_) {
      pub_.publish(msg);
      return;
    }

    cv_bridge::CvImageConstPtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvShare(msg, msg->encoding);
    } catch (const cv_bridge::Exception & e) {
      RCLCPP_ERROR_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "cv_bridge 변환 실패: %s", e.what());
      return;
    }

    cv::Mat meters;
    if (!decodeToMeters(cv_ptr->image, msg->encoding, meters)) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "지원하지 않는 depth 인코딩: %s - 필터 없이 원본 재발행",
        msg->encoding.c_str());
      pub_.publish(msg);
      return;
    }

    cv::Mat spatial;
    applySpatialConsistency(
      meters, spatial,
      consistency_kernel_, abs_thresh_m_, rel_ratio_,
      min_valid_neighbors_, invalidate_outliers_);

    cv::Mat filtered;
    if (temporal_alpha_ > 1e-6f &&
      !prev_meters_.empty() &&
      prev_meters_.size() == spatial.size())
    {
      applyTemporalEma(
        spatial, prev_meters_, filtered, temporal_alpha_, temporal_jump_m_);
    } else {
      filtered = spatial;
    }
    prev_meters_ = filtered.clone();

    cv_bridge::CvImage out;
    out.header = msg->header;
    out.encoding = msg->encoding;
    out.image = encodeFromMeters(filtered, msg->encoding);
    pub_.publish(*out.toImageMsg());
  }

  std::string input_topic_, output_topic_, input_transport_;
  int expected_h_{0}, expected_w_{0};

  bool enable_filter_{true};
  int consistency_kernel_{5};
  float abs_thresh_m_{0.05f};
  float rel_ratio_{0.05f};
  int min_valid_neighbors_{5};
  bool invalidate_outliers_{false};
  float temporal_alpha_{0.25f};
  float temporal_jump_m_{0.08f};

  cv::Mat prev_meters_;

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
