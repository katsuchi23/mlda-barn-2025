#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist

class VelocityMixer:
    def __init__(self):
        self.kul_weight = rospy.get_param('~kul_weight', 0.3)
        self.rl_weight = rospy.get_param('~rl_weight', 0.7)
        self.debug = rospy.get_param('~debug', False)
        
        total_weight = self.kul_weight + self.rl_weight
        if total_weight != 1.0 and total_weight > 0:
            self.kul_weight /= total_weight
            self.rl_weight /= total_weight
        
        rospy.loginfo("Velocity Mixer initialized with KUL weight: {}, RL weight: {}".format(self.kul_weight, self.rl_weight))
        
        self.latest_kul_cmd = Twist()
        self.latest_rl_cmd = Twist()
        self.have_kul_cmd = False
        self.have_rl_cmd = False
        
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
        # Increase timer frequency to 50Hz for more responsive output
        self.timer = rospy.Timer(rospy.Duration(0.02), self.publish_combined_velocity)
        
        # Subscribe to input topics after publisher is set up
        rospy.Subscriber('/cmd_vel_kul', Twist, self.kul_callback)
        rospy.Subscriber('/cmd_vel_rl', Twist, self.rl_callback)
        
        rospy.loginfo("Velocity mixer node is ready")
    
    def kul_callback(self, msg):
        self.latest_kul_cmd = msg
        self.have_kul_cmd = True
        if self.debug:
            rospy.loginfo("Received KUL cmd: linear.x={}, angular.z={}".format(msg.linear.x, msg.angular.z))
        # Publish immediately when receiving new KUL commands for better responsiveness
        self.publish_combined_velocity(None)
    
    def rl_callback(self, msg):
        self.latest_rl_cmd = msg
        self.have_rl_cmd = True
        if self.debug:
            rospy.loginfo("Received RL cmd: linear.x={}, angular.z={}".format(msg.linear.x, msg.angular.z))
        # Publish immediately when receiving new RL commands for better responsiveness
        self.publish_combined_velocity(None)
    
    def publish_combined_velocity(self, event):
        if not (self.have_kul_cmd or self.have_rl_cmd):
            return
        
        combined_cmd = Twist()
        
        # If we have both commands, combine them according to weights
        if self.have_kul_cmd and self.have_rl_cmd:
            combined_cmd.linear.x = (self.kul_weight * self.latest_kul_cmd.linear.x + 
                                   self.rl_weight * self.latest_rl_cmd.linear.x)
            combined_cmd.angular.z = (self.kul_weight * self.latest_kul_cmd.angular.z + 
                                    self.rl_weight * self.latest_rl_cmd.angular.z)
        # If we only have KUL command
        elif self.have_kul_cmd:
            combined_cmd.linear.x = self.latest_kul_cmd.linear.x
            combined_cmd.angular.z = self.latest_kul_cmd.angular.z
        # If we only have RL command
        elif self.have_rl_cmd:
            combined_cmd.linear.x = self.latest_rl_cmd.linear.x
            combined_cmd.angular.z = self.latest_rl_cmd.angular.z
        
        self.cmd_vel_pub.publish(combined_cmd)
        if self.debug:
            rospy.loginfo("Publishing combined cmd: linear.x={}, angular.z={}".format(combined_cmd.linear.x, combined_cmd.angular.z))

if __name__ == '__main__':
    rospy.init_node('velocity_mixer')
    rospy.loginfo("Velocity Mixer is running")
    node = VelocityMixer()
    rospy.spin()
