import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from armeo_msgs.msg import Trajectory, JointTrajectory, JointTarget  # Replace with your custom message if needed
from std_msgs.msg import Bool
from sensor_msgs.msg import JointState
import time

class MinimalPublisher(Node):

    def __init__(self):
        # Initialize the node with the name 'minimal_publisher'
        super().__init__('minimal_publisher')
        
        # Create a publisher that broadcasts String messages on the 'topic' topic
        # The queue size is set to 10
        self.publisher_ = self.create_publisher(Trajectory, '/trajectory', 10)
        self.pub_start = self.create_publisher(Bool, '/startstop', 10)

        # Subscribe to the joint states to get the current joint angles
        self.latest_joint_state = None
        self.sub_joint_states = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, qos_profile_sensor_data)
        
        # Create a timer that triggers the callback every 0.5 seconds
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def joint_state_callback(self, msg):
        self.latest_joint_state = msg

    def timer_callback(self):
        # Wait until we have received a joint state with at least 7 joints
        if self.latest_joint_state is None or len(self.latest_joint_state.position) < 7:
            self.get_logger().warn('Waiting for /joint_states...')
            return

        msg = Trajectory()
        msg.trajectories = [JointTrajectory(), JointTrajectory(), JointTrajectory(), JointTrajectory(), JointTrajectory(), JointTrajectory(), JointTrajectory()]


        ja = list(self.latest_joint_state.position[:7])
        
        j1_traj = JointTrajectory()
        j1_traj.targets = [JointTarget(), JointTarget()]
        j1_traj.targets[0].joint_angle = ja[0]
        j1_traj.targets[0].duration_sec = 2.0
        j1_traj.targets[1].joint_angle = 0.0
        j1_traj.targets[1].duration_sec = 2.0

        msg.trajectories[0] = j1_traj

        # Each Joint needs at least 1 target, you can set these to whatever the current joint angle is to make it not move, or some other static position.
        for i in range(1, 7):
            msg.trajectories[i].targets = [JointTarget()]
            msg.trajectories[i].targets[0].joint_angle = ja[i]
            msg.trajectories[i].targets[0].duration_sec = 1.0
        
        # Publish the message to the topic
        self.publisher_.publish(msg)

        time.sleep(0.5)

        start_msg = Bool()
        start_msg.data = True
        self.pub_start.publish(start_msg)

        while True:
            time.sleep(5.0)


def main(args=None):
    # Initialize the ROS 2 Python communications
    rclpy.init(args=args)

    # Create the publisher node
    minimal_publisher = MinimalPublisher()

    # Spin the node so its callback functions can be processed
    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly (optional, but good practice)
    minimal_publisher.destroy_node()
    
    # Shutdown the ROS 2 communications
    rclpy.shutdown()


if __name__ == '__main__':
    main()