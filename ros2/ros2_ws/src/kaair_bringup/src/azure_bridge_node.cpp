// azure_bridge_node.cpp
//
// azure_kinect_container_humble 안에서 K4A wrapper(OrbbecSDK-K4A-Wrapper)로 Femto Bolt를
// 직접 여는 azure_kinect_ros_driver `node`는 body tracking(K4ABT)을 쓰기 위해 자기만의
// ROS_DOMAIN_ID(=25, "main domain")에 격리되어 떠 있다. 네이티브 orbbec_camera 드라이버
// (femto_bolt.launch.py, camera_name=femto)와 같은 물리 카메라를 동시에 열 수 없어서
// (USB 단독 클레임 경합), vision_runner.launch.py 쪽에서 is_azure 모드일 때는 femto_bolt
// 드라이버와 depth_resizer를 아예 끄고, 이 노드가 그 자리를 완전히 대체한다:
//
//   ROS_DOMAIN_ID=25(azure)         ROS_DOMAIN_ID=10(keti, 이 컨테이너의 기본 도메인)
//   ---------------------------     ---------------------------------------------
//   /rgb/camera_info            ->  /femto/color/camera_info
//   /rgb/image_raw              ->  /femto/color/image_raw
//   /depth_to_rgb/camera_info   ->  /femto/depth/camera_info
//   /depth_to_rgb/image_raw     ->  /femto/depth/aligned      (depth_resizer가 하던 일까지 포함)
//   /body_index_map/image_raw   ->  /body_index_map/image_raw (토픽명 동일)
//   /body_tracking_data         ->  /body_tracking_data       (토픽명 동일, marker별 frame_id 교정)
//
// 두 도메인은 rclcpp::Context를 각각 별도로 만들어서(InitOptions::set_domain_id) 완전히
// 분리한다. 즉 이 프로세스 하나가 25번 도메인 구독 + 10번 도메인 발행을 동시에 한다.
// 두 Context 모두 rclcpp::init() 를 거치지 않으므로, 전역 SIGINT 핸들러가 자동으로
// 설치되지 않는다 -- rclcpp::install_signal_handlers()를 직접 호출해야 Ctrl+C로
// 두 Context가 같이 내려간다(안 해주면 SIGINT를 그냥 무시하고 계속 돈다).
//
// 도메인 ID는 프로세스 시작 시(어떤 rclcpp::Context 도 만들기 전) 환경변수에서 읽는다:
//   AZURE_MAIN_DOMAIN_ID   -- azure_kinect_ros_driver node가 도는 도메인 (기본 25)
//   AZURE_TARGET_DOMAIN_ID -- 리네임된 토픽을 발행할 도메인 (기본: $ROS_DOMAIN_ID, 없으면 0)

#include <cstdlib>
#include <exception>
#include <memory>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

using sensor_msgs::msg::CameraInfo;
using sensor_msgs::msg::Image;
using visualization_msgs::msg::MarkerArray;

namespace
{

size_t domainIdFromEnv(const char * env_name, size_t default_value)
{
  const char * value = std::getenv(env_name);
  if (value == nullptr || std::string(value).empty()) {
    return default_value;
  }
  try {
    return static_cast<size_t>(std::stoul(value));
  } catch (const std::exception &) {
    return default_value;
  }
}

rclcpp::Context::SharedPtr makeDomainContext(int argc, char const * const * argv, size_t domain_id)
{
  auto context = std::make_shared<rclcpp::Context>();
  rclcpp::InitOptions init_options;
  init_options.set_domain_id(domain_id);
  context->init(argc, argv, init_options);
  return context;
}

}  // namespace

int main(int argc, char ** argv)
{
  // 두 Context 모두 rclcpp::init()를 거치지 않으므로 전역 SIGINT 핸들러가 자동으로
  // 설치되지 않는다. 여기서 명시적으로 한 번 설치해야 두 Context가 SIGINT에 같이 내려간다.
  rclcpp::install_signal_handlers();

  const size_t main_domain_id = domainIdFromEnv("AZURE_MAIN_DOMAIN_ID", 25);
  const size_t target_domain_id =
    domainIdFromEnv("AZURE_TARGET_DOMAIN_ID", domainIdFromEnv("ROS_DOMAIN_ID", 0));

  auto main_context = makeDomainContext(argc, argv, main_domain_id);
  auto target_context = makeDomainContext(argc, argv, target_domain_id);

  rclcpp::NodeOptions main_node_options;
  main_node_options.context(main_context);
  auto main_node = std::make_shared<rclcpp::Node>("azure_bridge_source", main_node_options);

  rclcpp::NodeOptions target_node_options;
  target_node_options.context(target_context);
  auto target_node = std::make_shared<rclcpp::Node>("azure_bridge_target", target_node_options);

  RCLCPP_INFO(main_node->get_logger(), "azure_bridge_node: 구독 ROS_DOMAIN_ID=%zu (AZURE_MAIN_DOMAIN_ID)", main_domain_id);
  RCLCPP_INFO(target_node->get_logger(), "azure_bridge_node: 발행 ROS_DOMAIN_ID=%zu (AZURE_TARGET_DOMAIN_ID)", target_domain_id);

  // target_node 쪽에 파라미터를 선언해서 토픽명/frame_id를 재빌드 없이 조정할 수 있게 한다.
  const std::string frame_id =
    target_node->declare_parameter<std::string>("frame_id", "femto_color_optical_frame");

  const std::string color_camera_info_in =
    target_node->declare_parameter<std::string>("color_camera_info_in", "/rgb/camera_info");
  const std::string color_image_in =
    target_node->declare_parameter<std::string>("color_image_in", "/rgb/image_raw");
  const std::string depth_camera_info_in =
    target_node->declare_parameter<std::string>("depth_camera_info_in", "/depth_to_rgb/camera_info");
  const std::string depth_image_in =
    target_node->declare_parameter<std::string>("depth_image_in", "/depth_to_rgb/image_raw");
  const std::string body_index_map_in =
    target_node->declare_parameter<std::string>("body_index_map_in", "/body_index_map/image_raw");
  const std::string body_tracking_data_in =
    target_node->declare_parameter<std::string>("body_tracking_data_in", "/body_tracking_data");

  const std::string color_camera_info_out =
    target_node->declare_parameter<std::string>("color_camera_info_out", "/femto/color/camera_info");
  const std::string color_image_out =
    target_node->declare_parameter<std::string>("color_image_out", "/femto/color/image_raw");
  const std::string depth_camera_info_out =
    target_node->declare_parameter<std::string>("depth_camera_info_out", "/femto/depth/camera_info");
  const std::string depth_image_out =
    target_node->declare_parameter<std::string>("depth_image_out", "/femto/depth/aligned");
  const std::string body_index_map_out =
    target_node->declare_parameter<std::string>("body_index_map_out", "/body_index_map/image_raw");
  const std::string body_tracking_data_out =
    target_node->declare_parameter<std::string>("body_tracking_data_out", "/body_tracking_data");

  const rclcpp::QoS qos(rclcpp::KeepLast(5));

  // ---- target 도메인 발행자 ----
  auto color_camera_info_pub = target_node->create_publisher<CameraInfo>(color_camera_info_out, qos);
  auto color_image_pub = target_node->create_publisher<Image>(color_image_out, qos);
  auto depth_camera_info_pub = target_node->create_publisher<CameraInfo>(depth_camera_info_out, qos);
  auto depth_image_pub = target_node->create_publisher<Image>(depth_image_out, qos);
  auto body_index_map_pub = target_node->create_publisher<Image>(body_index_map_out, qos);
  auto body_tracking_data_pub = target_node->create_publisher<MarkerArray>(body_tracking_data_out, qos);

  // ---- main 도메인 구독자: frame_id를 덮어써서 target 도메인으로 그대로 재발행 ----
  auto color_camera_info_sub = main_node->create_subscription<CameraInfo>(
    color_camera_info_in, qos,
    [color_camera_info_pub, frame_id](CameraInfo msg) {
      msg.header.frame_id = frame_id;
      color_camera_info_pub->publish(msg);
    });

  auto color_image_sub = main_node->create_subscription<Image>(
    color_image_in, qos,
    [color_image_pub, frame_id](Image msg) {
      msg.header.frame_id = frame_id;
      color_image_pub->publish(msg);
    });

  auto depth_camera_info_sub = main_node->create_subscription<CameraInfo>(
    depth_camera_info_in, qos,
    [depth_camera_info_pub, frame_id](CameraInfo msg) {
      msg.header.frame_id = frame_id;
      depth_camera_info_pub->publish(msg);
    });

  auto depth_image_sub = main_node->create_subscription<Image>(
    depth_image_in, qos,
    [depth_image_pub, frame_id](Image msg) {
      msg.header.frame_id = frame_id;
      depth_image_pub->publish(msg);
    });

  auto body_index_map_sub = main_node->create_subscription<Image>(
    body_index_map_in, qos,
    [body_index_map_pub, frame_id](Image msg) {
      msg.header.frame_id = frame_id;
      body_index_map_pub->publish(msg);
    });

  auto body_tracking_data_sub = main_node->create_subscription<MarkerArray>(
    body_tracking_data_in, qos,
    [body_tracking_data_pub, frame_id](MarkerArray msg) {
      for (auto & marker : msg.markers) {
        marker.header.frame_id = frame_id;
      }
      body_tracking_data_pub->publish(msg);
    });

  // 도메인이 다른 두 Context는 하나의 Executor로 같이 wait 할 수 없어서, 각자
  // 자기 Context에 바인딩된 Executor를 따로 돌려야 한다.
  rclcpp::ExecutorOptions main_executor_options;
  main_executor_options.context = main_context;
  rclcpp::executors::SingleThreadedExecutor main_executor(main_executor_options);
  main_executor.add_node(main_node);

  rclcpp::ExecutorOptions target_executor_options;
  target_executor_options.context = target_context;
  rclcpp::executors::SingleThreadedExecutor target_executor(target_executor_options);
  target_executor.add_node(target_node);

  std::thread main_spin_thread([&main_executor]() {main_executor.spin();});

  target_executor.spin();

  main_spin_thread.join();

  return 0;
}
