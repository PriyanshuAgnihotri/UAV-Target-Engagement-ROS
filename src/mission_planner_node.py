#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Point, Twist
from std_msgs.msg import Bool

KP_LATERAL = 0.5
KD_DESCENT = 0.8
STATE_SEARCH = 0
STATE_APPROACH = 1
STATE_HIT = 2

class MissionPlanner:
    def __init__(self):
        rospy.init_node('mission_planner_node', anonymous=True)
        self.bbox_sub = rospy.Subscriber('/cv/target_bbox_center', Point, self.bbox_callback)
        self.ready_sub = rospy.Subscriber('/uav/status/ready_to_search', Bool, self.ready_callback)
        self.cmd_vel_pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel_enu', Twist, queue_size=10)
        self.current_state = STATE_SEARCH
        self.target_center = Point()
        self.target_detected = False
        self.search_ready = False
        self.rate = rospy.Rate(20)

    def ready_callback(self, msg):
        self.search_ready = msg.data
        if self.search_ready:
            rospy.loginfo("PLANNER: Starting Search Pattern (Simulated).")

    def bbox_callback(self, msg):
        self.target_center = msg
        self.target_detected = True
        if self.current_state == STATE_SEARCH:
            self.current_state = STATE_APPROACH
            rospy.loginfo("PLANNER: Target lock acquired. Transitioning to APPROACH state.")

    def execute_search(self):
        rospy.loginfo_throttle(5, "PLANNER: Executing Search Pattern at 10m.")
        search_vel = Twist()
        search_vel.linear.x = 0.5
        self.cmd_vel_pub.publish(search_vel)

    def execute_approach(self):
        error_x = self.target_center.x
        error_y = self.target_center.y
        cmd_vel = Twist()
        cmd_vel.linear.x = KP_LATERAL * error_y
        cmd_vel.linear.y = KP_LATERAL * error_x
        cmd_vel.linear.z = -KD_DESCENT
        if abs(error_x) < 0.05 and abs(error_y) < 0.05:
            rospy.loginfo("PLANNER: Target centralized. Initiating HIT sequence.")
            self.current_state = STATE_HIT
        self.cmd_vel_pub.publish(cmd_vel)

    def execute_hit(self):
        rospy.loginfo("PLANNER: Executing final rapid descent (Hit).")
        hit_vel = Twist()
        hit_vel.linear.z = -5.0
        self.cmd_vel_pub.publish(hit_vel)
        rospy.signal_shutdown("Target Hit. Mission complete.")

    def run(self):
        while not rospy.is_shutdown():
            if not self.search_ready:
                self.rate.sleep()
                continue
            if self.current_state == STATE_SEARCH:
                self.execute_search()
            elif self.current_state == STATE_APPROACH:
                self.execute_approach()
            elif self.current_state == STATE_HIT:
                self.execute_hit()
            self.rate.sleep()

if __name__ == '__main__':
    try:
        planner = MissionPlanner()
        planner.run()
    except rospy.ROSInterruptException:
        pass