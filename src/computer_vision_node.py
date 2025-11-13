#!/usr/bin/env python
import rospy
import time
from geometry_msgs.msg import Point
from std_msgs.msg import Bool

class ComputerVision:
    def __init__(self):
        rospy.init_node('computer_vision_node', anonymous=True)
        self.bbox_pub = rospy.Publisher('/cv/target_bbox_center', Point, queue_size=10)
        self.status_sub = rospy.Subscriber('/uav/status/ready_to_search', Bool, self.status_callback)
        self.search_ready = False
        self.target_detected = False
        self.target_id = 1
        self.rate = rospy.Rate(1)

    def status_callback(self, msg):
        if msg.data:
            self.search_ready = True
            rospy.loginfo("CV: Received Ready Signal. Initiating visual search.")

    def run(self):
        while not self.search_ready:
            self.rate.sleep()

        rospy.loginfo("CV: Simulating 5 seconds of search...")
        time.sleep(5)
        self.target_detected = True
        rospy.loginfo("CV: Tank Detected! Starting tracking and publishing.")

        while not rospy.is_shutdown():
            if self.target_detected:
                simulated_center = Point()
                simulated_center.x = 0.1
                simulated_center.y = -0.05
                simulated_center.z = self.target_id
                self.bbox_pub.publish(simulated_center)
                rospy.loginfo(f"CV: Published Target ID {self.target_id} at ({simulated_center.x}, {simulated_center.y})")
            self.rate.sleep()

if __name__ == '__main__':
    try:
        cv_sim = ComputerVision()
        cv_sim.run()
    except rospy.ROSInterruptException:
        pass