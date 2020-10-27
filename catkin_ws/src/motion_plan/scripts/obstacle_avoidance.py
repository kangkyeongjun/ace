#! /usr/bin/env python

import rospy

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

pub = None

def clbk_laser(msg):

    msgs = len(msg.ranges)
    
    if msgs == 250:
     for i in range(250):
      if msg.ranges[i] == 0.0:
       listt = list(msg.ranges)
       listt[i] = 10.0
       msg.ranges = tuple(listt)
       regions = {
        'right': min(min(msg.ranges[0:29]), 10),
        'fright': min(min(msg.ranges[30:59]), 10),
        'front': min(min(msg.ranges[60:99]), 10),
        'fleft': min(min(msg.ranges[100:129]), 10),
        'left': min(min(msg.ranges[130:159]), 10),
        'bleft': min(min(msg.ranges[160:189]), 10),
        'back': min(min(msg.ranges[190:219]), 10),
        'bright': min(min(msg.ranges[220:249]), 10)
    }
 
    # 260 / 8 = 32.5
    elif msgs == 260:
     for i in range(260):
      if msg.ranges[i] == 0.0:
       listt = list(msg.ranges)
       listt[i] = 10.0
       msg.ranges = tuple(listt)
       regions = {
        'right': min(min(msg.ranges[0:32]), 10),
        'fright': min(min(msg.ranges[33:65]), 10),
        'front': min(min(msg.ranges[66:98]), 10),
        'fleft': min(min(msg.ranges[99:131]), 10),
        'left': min(min(msg.ranges[132:164]), 10),
        'bleft': min(min(msg.ranges[165:197]), 10),
        'back': min(min(msg.ranges[198:230]), 10),
        'bright': min(min(msg.ranges[231:259]), 10)
    }


    take_action(regions)

def take_action(regions):
    msg = Twist()
    linear_x = 0
    angular_z = 0

    state_description = ''

    if regions['front'] > 0.15 and regions['left'] > 0.15 and regions['right'] > 0.15 and regions['back'] > 0.15: 
        if regions['fleft'] < 0.15 and regions['fright'] > 0.15 and regions['bleft'] > 0.15 and regions['bright'] > 0.15: #go right
	    linear_x = 0.5
	    angular_z = 1
	    state_description = 'case 5 - fleft'
        elif regions['fleft'] > 0.15 and regions['fright'] < 0.15 and regions['bleft'] > 0.15 and regions['bright'] > 0.15: #go left
	    linear_x = 0.5
	    angular_z = -1
	    state_description = 'case 6 - fright'
        elif regions['fleft'] > 0.15 and regions['fright'] > 0.15 and regions['bleft'] < 0.15 and regions['bright'] > 0.15: #go left
	    linear_x = 0.5
	    angular_z = -1
	    state_description = 'case 7 - bleft'
        elif regions['fleft'] > 0.15 and regions['fright'] > 0.15 and regions['bleft'] > 0.15 and regions['bright'] < 0.15: #go left
	    linear_x = 0.5
	    angular_z = -1
	    state_description = 'case 8 - bright'
	else:
	    state_description = 'case 21 - nothing' #go straight
            linear_x = 0.5
            angular_z = 0

    elif regions['front'] < 0.15: 
        if regions['fleft'] < 0.15 and regions['fright'] > 0.15: #turn right
	    linear_x = 0
	    angular_z = 1
	    state_description = 'case 1 - front and fleft'
        elif regions['fleft'] > 0.15 and regions['fright'] < 0.15: #turn left
	    linear_x = 0
	    angular_z = -1
	    state_description = 'case 2 - front and fright'
        elif regions['fleft'] < 0.15 and regions['fright'] < 0.15: #back
	    linear_x = -0.5
	    angular_z = 0
	    state_description = 'case 3 - front and fleft and fright'
        elif regions['fleft'] > 0.15 and regions['fright'] > 0.15: #go straight
	    linear_x = 0.5
	    angular_z = 0
	    state_description = 'case 4 - front'

    elif regions['right'] < 0.15: 
        if regions['bright'] < 0.15 and regions['fright'] > 0.15: #go left
	    linear_x = 0.5
	    angular_z = -1
	    state_description = 'case 9 - right and bright'
        elif regions['bright'] > 0.15 and regions['fright'] < 0.15: #turn left
	    linear_x = 0
	    angular_z = -1
	    state_description = 'case 10 - right and fright'
        elif regions['bright'] < 0.15 and regions['fright'] < 0.15: #turn left
	    linear_x = 0
	    angular_z = -1
	    state_description = 'case 11 - right and bright and fright'
        elif regions['bright'] > 0.15 and regions['fright'] > 0.15: #go straight
	    linear_x = 0.5
	    angular_z = 0
	    state_description = 'case 12 - right'

    elif regions['left'] < 0.15: 
        if regions['bleft'] < 0.15 and regions['fleft'] > 0.15: #go right
	    linear_x = 0.5
	    angular_z = 1
	    state_description = 'case 13 - left and bleft'
        elif regions['bleft'] > 0.15 and regions['fleft'] < 0.15: #turn right
	    linear_x = 0
	    angular_z = 1
	    state_description = 'case 14 - left and fleft'
        elif regions['bleft'] < 0.15 and regions['fleft'] < 0.15: #turn left
	    linear_x = 0
	    angular_z = 1
	    state_description = 'case 15 - left and bleft and fleft'
        elif regions['bleft'] > 0.15 and regions['fleft'] > 0.15: #go straight
	    linear_x = 0.5
	    angular_z = 0
	    state_description = 'case 16 - left'

    elif regions['back'] < 0.15: 
        if regions['bleft'] < 0.15 and regions['bright'] > 0.15: #go straight
	    linear_x = 0.5
	    angular_z = 0
	    state_description = 'case 17 - back and bleft'
        elif regions['bleft'] > 0.15 and regions['bright'] < 0.15: #go straight
	    linear_x = 0.5
	    angular_z = 0
	    state_description = 'case 18 - left and fleft'
        elif regions['bleft'] < 0.15 and regions['bright'] < 0.15: #go straight
	    linear_x = 0.5
	    angular_z = 0
	    state_description = 'case 19 - left and bleft and fleft'
        elif regions['bleft'] > 0.15 and regions['bright'] > 0.15: #go straight
	    linear_x = 0.5
	    angular_z = 0
	    state_description = 'case 20 - back'

    elif regions['front'] < 0.15 and regions['left'] < 0.15 and regions['right'] < 0.15 and regions['back'] < 0.15: 
        linear_x = 0
        angular_z = 0
        state_description = 'case 22 - stop'

    else:
        state_description = 'unknown case'
        rospy.loginfo(regions)

    rospy.loginfo(state_description)
    msg.linear.x = linear_x
    msg.angular.z = angular_z

    pub.publish(msg)

def main():
    global pub

    rospy.init_node('reading_laser')

    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

    sub = rospy.Subscriber('/scan', LaserScan, clbk_laser)

    rospy.spin()

if __name__ == '__main__':
    main()
