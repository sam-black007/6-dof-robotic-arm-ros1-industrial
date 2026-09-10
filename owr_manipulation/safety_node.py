#!/usr/bin/env python3
# Safety node placeholder for joint limits and emergency stop
import rospy

def main():
    rospy.init_node('safety_node')
    rospy.loginfo('Safety node started - joint limits and emergency stop monitoring')
    rospy.spin()

if __name__ == '__main__':
    main()
