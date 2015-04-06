#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
import math

class Controller():
    def __init__(self):
        self.lastData = None
        rospy.Subscriber("joy", Joy, self._joyChanged)
        self.pubNav = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.pubSetBackLed = rospy.Publisher('set_back_led', Float32, queue_size=1)
        self.pubSetHeading = rospy.Publisher('set_heading', Float32, queue_size=1)
        self.offset = 0
        self.calibration = False
        # value = Float32()
        # value.data = 1.0
        # for i in range(0, 100):
            # self.pubSetBackLed.publish(value)

    def _joyChanged(self, data):
        self.lastData = data
        
    def run(self):
        while not rospy.is_shutdown():
            msg = Twist()
            value = Float32()
            if self.lastData != None:
                if self.lastData.buttons[0] == 1:
                    value.data = 1.0
                    self.pubSetBackLed.publish(value)
                    self.offset += self.lastData.axes[0] * math.pi / 50.0
                    msg.angular.x = self.offset
                    msg.linear.x = 0
                    self.calibration = True
                else:
                    if self.calibration:
                        value.data = self.offset
                        self.pubSetHeading.publish(value)
                        value.data = 0
                        self.pubSetBackLed.publish(value)
                        self.calibration = False
                        self.offset = 0
                    msg.linear.x = self.lastData.axes[3] * -255
                    msg.linear.y = self.lastData.axes[4] * 255
                
            self.pubNav.publish(msg)
            # self.pubSetHeading.publish(value)
            
            rospy.sleep(0.01)



if __name__ == '__main__':
    rospy.init_node('teleop', anonymous=True)
    controller = Controller()
    controller.run()
    # rospy.spin()
