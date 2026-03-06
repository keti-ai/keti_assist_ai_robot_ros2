from dataclasses import dataclass
from rclpy.node import Node


@dataclass
class PipelineConfig:
    # topics
    input_topic: str
    output_topic: str

    # passthrough filter
    x_min: float
    x_max: float
    y_min: float
    y_max: float
    z_min: float
    z_max: float

    # voxel downsampling
    voxel_size: float

    # statistical outlier removal
    sor_nb_neighbors: int
    sor_std_ratio: float

    # RANSAC ground removal
    ransac_distance_threshold: float
    ransac_num_iterations: int

    # euclidean clustering
    cluster_tolerance: float
    cluster_min_size: int
    cluster_max_size: int


def load_config(node: Node) -> PipelineConfig:
    """ROS2 파라미터를 선언하고 PipelineConfig 로 반환"""

    node.declare_parameter('input_topic',  '/camera/depth/color/points')
    node.declare_parameter('output_topic', '/pointcloud/objects')

    node.declare_parameter('x_min', -3.0)
    node.declare_parameter('x_max',  3.0)
    node.declare_parameter('y_min', -3.0)
    node.declare_parameter('y_max',  3.0)
    node.declare_parameter('z_min',  0.05)
    node.declare_parameter('z_max',  2.5)

    node.declare_parameter('voxel_size', 0.05)

    node.declare_parameter('sor_nb_neighbors', 20)
    node.declare_parameter('sor_std_ratio', 2.0)

    node.declare_parameter('ransac_distance_threshold', 0.02)
    node.declare_parameter('ransac_num_iterations', 1000)

    node.declare_parameter('cluster_tolerance', 0.05)
    node.declare_parameter('cluster_min_size', 50)
    node.declare_parameter('cluster_max_size', 10000)

    return PipelineConfig(
        input_topic  = node.get_parameter('input_topic').value,
        output_topic = node.get_parameter('output_topic').value,

        x_min = node.get_parameter('x_min').value,
        x_max = node.get_parameter('x_max').value,
        y_min = node.get_parameter('y_min').value,
        y_max = node.get_parameter('y_max').value,
        z_min = node.get_parameter('z_min').value,
        z_max = node.get_parameter('z_max').value,

        voxel_size = node.get_parameter('voxel_size').value,

        sor_nb_neighbors = node.get_parameter('sor_nb_neighbors').value,
        sor_std_ratio    = node.get_parameter('sor_std_ratio').value,

        ransac_distance_threshold = node.get_parameter('ransac_distance_threshold').value,
        ransac_num_iterations     = node.get_parameter('ransac_num_iterations').value,

        cluster_tolerance = node.get_parameter('cluster_tolerance').value,
        cluster_min_size  = node.get_parameter('cluster_min_size').value,
        cluster_max_size  = node.get_parameter('cluster_max_size').value,
    )