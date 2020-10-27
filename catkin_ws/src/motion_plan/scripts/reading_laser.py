#! /usr/bin/env python

import rospy

from sensor_msgs.msg import LaserScan


def clbk_laser(msg):
    
    for i in range(250):
     if msg.ranges[i] == 0.0:
      msgs=list(msg.ranges)
      msgs[i] = 10.0
      msg.ranges=tuple(msgs)
    # 260 / 8 = 32.5
    
    regions = [
        min(msg.ranges[0:32]),
        min(msg.ranges[33:65]),
        min(msg.ranges[66:98]),
        min(msg.ranges[99:131]),
        min(msg.ranges[132:164]),
        min(msg.ranges[165:197]),
        min(msg.ranges[198:230]),
        min(msg.ranges[231:259])
    ]
    rospy.loginfo(regions)

def main():
    rospy.init_node('reading_laser')

    sub = rospy.Subscriber('/scan', LaserScan, clbk_laser)

    rospy.spin()

if __name__ == '__main__':
    main()
