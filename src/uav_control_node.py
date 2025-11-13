#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Bool

TARGET_ALTITUDE = 10.0 # meters

class UAVControl:
    def __init__(self):
        rospy.init_node('uav_control_node', anonymous=True)
        self.local_pos_sub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.pose_callback)
        self.cmd_vel_pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel_enu', Twist, queue_size=10)
        self.ready_pub = rospy.Publisher('/uav/status/ready_to_search', Bool, queue_size=1)
        
        self.current_altitude = 0.0
        self.is_armed = True
        self.is_taken_off = False
        self.rate = rospy.Rate(20)

    def pose_callback(self, msg):
        self.current_altitude = msg.pose.position.z

    def takeoff(self):
        rospy.loginfo("UAV: Starting Takeoff to 10m...")
        takeoff_vel = Twist()
        while self.current_altitude < TARGET_ALTITUDE - 0.5:
            takeoff_vel.linear.z = 1.0
            self.cmd_vel_pub.publish(takeoff_vel)
            self.rate.sleep()

        rospy.loginfo("UAV: Reached 10m altitude. Starting search mode.")
        self.is_taken_off = True
        takeoff_vel.linear.z = 0.0
        self.cmd_vel_pub.publish(takeoff_vel)
        self.ready_pub.publish(True)

    def run(self):
        if self.is_armed:
            self.takeoff()
        rospy.spin()

if __name__ == '__main__':
    try:
        controller = UAVControl()
        controller.run()
    except rospy.ROSInterruptException:
        pass