#!/usr/bin/env python3

"""Publish the xArm end-effector pose and twist at the driver's full rate.

The xarm driver reports joint state but never the Cartesian velocity of
the tool: `RobotMsg` carries `pose` (position only) and the SDK's
`rt_tcp_spd` is a scalar magnitude that never reaches ROS at all. Data
collection and VLA inference both need the full 6-DOF twist, so this node
derives it from the joint state with the manipulator Jacobian.

Why the Jacobian rather than differentiating the reported pose:

  * `/xarm/joint_states` already carries measured joint velocities at
    ~100 Hz, so the twist is one linear transform away -- no differencing,
    no filter, and therefore no added lag.
  * `RobotMsg.pose` arrives at 5 Hz on the default `report_type: normal`,
    and its orientation is Euler angles. Differencing those introduces
    wrap-around and gimbal artifacts exactly where a demonstration is most
    interesting -- during fast reorientation.

Everything is reported at the TCP, not the flange. The teleop node drives
the arm with `vc_set_cartesian_velocity`, which acts at the TCP, so
observing anywhere else would mean the recorded action and the recorded
observation refer to different points on the robot.

The flange -> TCP offset is read out of the URDF rather than configured:
`link_eef -> tool_tcp_link` is a chain of fixed joints
(kaair_description/urdf/tool/tool.urdf.xacro), so the same description
that /tf, MoveIt and the servo `ee_frame_name` already agree on defines
the observation point too. A number typed into a config file would be a
fourth place that has to be kept in sync with the gripper.
"""

import rclpy
import PyKDL
from geometry_msgs.msg import PoseStamped, TwistStamped
from rcl_interfaces.srv import GetParameters
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from urdf_parser_py.urdf import URDF


def _kdl_tree_from_urdf(model):
    """Build a KDL tree from a parsed URDF model.

    kdl_parser_py is not packaged in this environment, so the conversion
    is done here. Only the joint types an xArm actually uses are handled;
    anything else becomes a fixed joint, which would show up immediately
    as a wrong DOF count rather than as silently wrong kinematics.
    """
    tree = PyKDL.Tree(model.get_root())

    def add_children(parent):
        for joint in model.joints:
            if joint.parent != parent:
                continue
            kdl_joint = _kdl_joint(joint)
            kdl_frame = _kdl_frame(joint.origin)
            # The xArm URDF carries no <inertial> on link_eef, and inertia
            # is irrelevant to FK and the Jacobian, so a default segment
            # inertia is fine here.
            segment = PyKDL.Segment(joint.child, kdl_joint, kdl_frame)
            tree.addSegment(segment, parent)
            add_children(joint.child)

    add_children(model.get_root())
    return tree


def _kdl_joint(joint):
    origin = _kdl_frame(joint.origin)
    if joint.type in ('revolute', 'continuous'):
        axis = PyKDL.Vector(*(joint.axis or [1.0, 0.0, 0.0]))
        return PyKDL.Joint(joint.name, origin.p, origin.M * axis,
                           PyKDL.Joint.RotAxis)
    if joint.type == 'prismatic':
        axis = PyKDL.Vector(*(joint.axis or [1.0, 0.0, 0.0]))
        return PyKDL.Joint(joint.name, origin.p, origin.M * axis,
                           PyKDL.Joint.TransAxis)
    return PyKDL.Joint(joint.name, PyKDL.Joint.Fixed)


def _kdl_frame(origin):
    if origin is None:
        return PyKDL.Frame()
    xyz = origin.xyz or [0.0, 0.0, 0.0]
    rpy = origin.rpy or [0.0, 0.0, 0.0]
    return PyKDL.Frame(PyKDL.Rotation.RPY(*rpy), PyKDL.Vector(*xyz))


class EefStatePublisher(Node):

    def __init__(self):
        super().__init__('eef_state_publisher')

        self._declare_parameters()
        self._load_parameters()

        self._build_chain()

        # Matches the driver's own joint_states publisher: this is a
        # fixed-rate stream where the freshest sample is what matters.
        sensor_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                                history=HistoryPolicy.KEEP_LAST, depth=10)

        self.pose_pub = self.create_publisher(
            PoseStamped, self.pose_topic, sensor_qos)
        self.twist_pub = self.create_publisher(
            TwistStamped, self.twist_topic, sensor_qos)

        self.create_subscription(
            JointState, self.joint_states_topic, self._on_joint_state, sensor_qos)

        self.get_logger().info(
            f'publishing TCP state from {self.joint_states_topic}: '
            f'{self.pose_topic} [{self.frame_id}], '
            f'{self.twist_topic} [{self.twist_frame_id}]')

    # ------------------------------------------------------------------
    # Parameters
    # ------------------------------------------------------------------

    def _declare_parameters(self):
        # The ufactory_driver stream, not the joint_state_merger's
        # /joint_states: the merger republishes at a fixed 50 Hz and
        # carries no velocities, so the twist would have to be
        # differenced back out of it.
        self.declare_parameter('joint_states_topic', '/xarm/joint_states')
        self.declare_parameter('pose_topic', '/observation/eef_pose')
        self.declare_parameter('twist_topic', '/observation/eef_twist')

        # arm_base is where robot.urdf.xacro attaches the xArm, with no
        # rotation, so it coincides with both link_base and the frame the
        # controller itself reports in. Naming it arm_base rather than
        # link_base keeps the output tied to this robot's URDF.
        self.declare_parameter('base_link', 'arm_base')
        # The arm chain ends at the flange; the tool is bolted on after it.
        self.declare_parameter('flange_link', 'link_eef')
        # Observation point. Same link the SRDF "arm" group uses as its
        # tip and the servo config as ee_frame_name, so the recorded
        # observation and the commanded motion pivot about one point.
        self.declare_parameter('tool_link', 'tool_tcp_link')
        self.declare_parameter('frame_id', 'arm_base')

        # Which axes the twist is written in. Either way it is the velocity
        # of the same point -- the TCP -- so this only re-expresses the two
        # vectors, it does not move the reference point.
        #
        #   'base' : base_link axes. What a Cartesian velocity command in
        #            the base frame is compared against, and what stays
        #            still while the tool rotates.
        #   'tool' : tool_link axes. "how fast am I moving along my own
        #            approach axis" -- the form a policy usually wants,
        #            because it is invariant to where the arm is pointing,
        #            which is why it is the default here.
        self.declare_parameter('twist_frame', 'tool')

    def _load_parameters(self):
        get = self.get_parameter
        self.joint_states_topic = get('joint_states_topic').value
        self.pose_topic = get('pose_topic').value
        self.twist_topic = get('twist_topic').value

        self.base_link = get('base_link').value
        self.flange_link = get('flange_link').value
        self.tool_link = get('tool_link').value
        self.frame_id = get('frame_id').value

        twist_frame = str(get('twist_frame').value).strip().lower()
        if twist_frame not in ('base', 'tool'):
            self.get_logger().warn(
                f"unknown twist_frame '{twist_frame}', assuming 'base'")
            twist_frame = 'base'
        self.twist_in_tool = twist_frame == 'tool'
        # The header has to name the frame the vectors are written in, or a
        # consumer transforming the twist would rotate it a second time.
        self.twist_frame_id = self.tool_link if self.twist_in_tool else self.frame_id

    # ------------------------------------------------------------------
    # Kinematics
    # ------------------------------------------------------------------

    def _build_chain(self):
        """Fetch the URDF from robot_state_publisher and build the chain.

        The description is read from the other node's parameter rather
        than from a file so this node always matches the model the rest
        of the stack is using.
        """
        urdf_xml = self._fetch_robot_description()
        model = URDF.from_xml_string(urdf_xml)
        tree = _kdl_tree_from_urdf(model)

        self.chain = tree.getChain(self.base_link, self.flange_link)
        self.dof = self.chain.getNrOfJoints()
        if self.dof == 0:
            raise RuntimeError(
                f'no movable joints between {self.base_link} and '
                f'{self.flange_link} -- check base_link/flange_link')

        # Joint order along the chain, used to reorder incoming
        # JointState by name: the driver's ordering is not guaranteed to
        # match the URDF's, and a silent mismatch produces a plausible
        # but wrong twist.
        self.joint_names = []
        for i in range(self.chain.getNrOfSegments()):
            joint = self.chain.getSegment(i).getJoint()
            if joint.getType() != PyKDL.Joint.Fixed:
                self.joint_names.append(joint.getName())

        self._tcp_offset = self._tool_offset_from_urdf(tree)

        self._fk = PyKDL.ChainFkSolverPos_recursive(self.chain)
        self._jac = PyKDL.ChainJntToJacSolver(self.chain)
        self._q = PyKDL.JntArray(self.dof)
        self._jacobian = PyKDL.Jacobian(self.dof)

        self.get_logger().info(
            f'chain {self.base_link} -> {self.flange_link}: '
            f'{self.dof} joints {self.joint_names}')

    def _tool_offset_from_urdf(self, tree):
        """Fixed flange -> tool transform, taken from the URDF itself.

        Refusing to run when a movable joint turns up in between is
        deliberate: this node would then be publishing the pose of a
        point whose position depends on a joint it never reads, and the
        error would look like a small constant bias rather than a bug.
        """
        chain = tree.getChain(self.flange_link, self.tool_link)
        if chain.getNrOfJoints() != 0:
            raise RuntimeError(
                f'{self.flange_link} -> {self.tool_link} has '
                f'{chain.getNrOfJoints()} movable joint(s); this node only '
                f'handles a fixed tool attachment')

        offset = PyKDL.Frame()
        solver = PyKDL.ChainFkSolverPos_recursive(chain)
        if solver.JntToCart(PyKDL.JntArray(0), offset) < 0:
            raise RuntimeError(
                f'FK failed for {self.flange_link} -> {self.tool_link}')

        self.get_logger().info(
            'tool offset {} -> {} from URDF: xyz={} m rpy={} rad'.format(
                self.flange_link, self.tool_link,
                [round(offset.p[i], 5) for i in range(3)],
                [round(v, 5) for v in offset.M.GetRPY()]))
        return offset

    def _fetch_robot_description(self):
        """Read robot_description from robot_state_publisher.

        Blocks until it is available: without the model there is nothing
        this node can compute, and coming up before the description is
        published is normal when everything launches together.
        """
        client = self.create_client(
            GetParameters, '/robot_state_publisher/get_parameters')
        while not client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn(
                'waiting for /robot_state_publisher to provide robot_description ...')

        request = GetParameters.Request()
        request.names = ['robot_description']
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        values = future.result().values
        if not values or not values[0].string_value:
            raise RuntimeError('robot_description is empty')
        return values[0].string_value

    # ------------------------------------------------------------------
    # Subscriptions
    # ------------------------------------------------------------------

    def _on_joint_state(self, msg):
        if not self._fill_joints(msg):
            return

        flange = PyKDL.Frame()
        if self._fk.JntToCart(self._q, flange) < 0:
            self.get_logger().warn('FK failed', throttle_duration_sec=5.0)
            return
        if self._jac.JntToJac(self._q, self._jacobian) < 0:
            self.get_logger().warn('Jacobian failed', throttle_duration_sec=5.0)
            return

        tcp = flange * self._tcp_offset
        twist = self._tcp_twist(msg, flange, self._tcp_offset)
        if twist is None:
            return

        self._publish(msg.header.stamp, tcp, twist)

    def _fill_joints(self, msg):
        """Copy positions into the KDL array in chain order.

        Returns False when the message does not carry every joint of the
        chain, which happens while a partial publisher is still coming up.
        """
        index = {name: i for i, name in enumerate(msg.name)}
        for i, name in enumerate(self.joint_names):
            j = index.get(name)
            if j is None or j >= len(msg.position):
                self.get_logger().warn(
                    f'joint {name} missing from {self.joint_states_topic}',
                    throttle_duration_sec=5.0)
                return False
            self._q[i] = msg.position[j]
        return True

    def _tcp_twist(self, msg, flange, offset):
        """Twist at the TCP, in the base frame.

        The Jacobian is taken at the flange, so the linear part has to be
        moved to the TCP: a rigid body rotating at omega about the flange
        carries the TCP along at an extra omega x r. Angular velocity is
        the same everywhere on a rigid body, so it transfers unchanged.
        """
        index = {name: i for i, name in enumerate(msg.name)}
        if len(msg.velocity) == 0:
            self.get_logger().warn(
                f'{self.joint_states_topic} carries no velocities -- '
                f'the driver reports them only on firmware >= 1.8.103',
                throttle_duration_sec=10.0)
            return None

        qdot = [0.0] * self.dof
        for i, name in enumerate(self.joint_names):
            j = index.get(name)
            if j is None or j >= len(msg.velocity):
                return None
            qdot[i] = msg.velocity[j]

        linear = PyKDL.Vector()
        angular = PyKDL.Vector()
        for row, target in ((0, linear), (3, angular)):
            for axis in range(3):
                total = 0.0
                for col in range(self.dof):
                    total += self._jacobian[row + axis, col] * qdot[col]
                target[axis] = total

        # r is the flange->TCP lever arm expressed in the base frame.
        lever = flange.M * offset.p
        linear = linear + angular * lever
        return linear, angular

    # ------------------------------------------------------------------
    # Publishing
    # ------------------------------------------------------------------

    def _publish(self, stamp, tcp, twist):
        linear, angular = twist

        if self.twist_in_tool:
            # Rotate into the tool's own axes. BOTH halves go together --
            # rotating only the linear part would leave the angular
            # velocity describing a different frame than the one the
            # header names. No translation is involved: the reference
            # point is already the TCP, which is the tool frame's origin.
            to_tool = tcp.M.Inverse()
            linear = to_tool * linear
            angular = to_tool * angular

        pose = PoseStamped()
        # Reuse the driver's stamp rather than "now": this state describes
        # the instant the arm was sampled, and every other recorded stream
        # is aligned against that.
        pose.header.stamp = stamp
        pose.header.frame_id = self.frame_id
        pose.pose.position.x = tcp.p[0]
        pose.pose.position.y = tcp.p[1]
        pose.pose.position.z = tcp.p[2]
        qx, qy, qz, qw = tcp.M.GetQuaternion()
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        self.pose_pub.publish(pose)

        msg = TwistStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = self.twist_frame_id
        msg.twist.linear.x = linear[0]
        msg.twist.linear.y = linear[1]
        msg.twist.linear.z = linear[2]
        msg.twist.angular.x = angular[0]
        msg.twist.angular.y = angular[1]
        msg.twist.angular.z = angular[2]
        self.twist_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = EefStatePublisher()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # ros2 launch tears the context down before this returns; that is
        # the normal shutdown path, not a failure.
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
