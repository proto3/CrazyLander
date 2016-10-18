#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Joy

import logging
import time

from threading import Thread

import cflib
from cflib.crazyflie import Crazyflie

class MotorRampExample:
    """Example that connects to a Crazyflie and ramps the motors up/down and
    the disconnects"""

    def __init__(self, link_uri):
        """ Initialize and run the example with the specified link_uri """

        self._cf = Crazyflie()

        self._cf.connected.add_callback(self._connected)
        self._cf.disconnected.add_callback(self._disconnected)
        self._cf.connection_failed.add_callback(self._connection_failed)
        self._cf.connection_lost.add_callback(self._connection_lost)

        self._cf.open_link(link_uri)
        
        self._crazy_ready = False

        print('Connecting to %s' % link_uri)

    def _connected(self, link_uri):
        """ This callback is called form the Crazyflie API when a Crazyflie
        has been connected and the TOCs have been downloaded."""

        # Start a separate thread to do the motor test.
        # Do not hijack the calling thread!
        print "Connected"
        rospy.loginfo(rospy.get_caller_id() + "Connected to crazyflie")
        self._cf.commander.send_setpoint(0, 0, 0, 0)
        self._crazy_ready = True

    def _connection_failed(self, link_uri, msg):
        """Callback when connection initial connection fails (i.e no Crazyflie
        at the specified address)"""
        print('Connection to %s failed: %s' % (link_uri, msg))

    def _connection_lost(self, link_uri, msg):
        """Callback when disconnected after a connection has been made (i.e
        Crazyflie moves out of range)"""
        print('Connection to %s lost: %s' % (link_uri, msg))

    def _disconnected(self, link_uri):
        """Callback when the Crazyflie is disconnected (called in all cases)"""
        print('Disconnected from %s' % link_uri)

    def send_to_crazyflie(self, roll, pitch, yawrate, thrust):
        self._cf.commander.send_setpoint(roll, pitch, yawrate, thrust)
    #def _callback_joystick(self):
	#	print "uh" #data.axes        
	# rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.axes)

    #def _listener(self):
    #    print "listener"
    #    rospy.init_node('listener', anonymous=True)
    #    rospy.Subscriber("joy", Joy, self._callback_joystick)
    #    rospy.spin()
        
    '''
    def _ramp_motors(self):
        thrust_mult = 1
        thrust_step = 500
        thrust = 20000
        pitch = 0
        roll = 0
        yawrate = 0

        # Unlock startup thrust protection
        self._cf.commander.send_setpoint(0, 0, 0, 0)

        while thrust >= 20000:
            self._cf.commander.send_setpoint(roll, pitch, yawrate, thrust)
            time.sleep(0.1)
            if thrust >= 25000:
                thrust_mult = -1
            thrust += thrust_step * thrust_mult
        self._cf.commander.send_setpoint(0, 0, 0, 0)
        # Make sure that the last packet leaves before the link is closed
        # since the message queue is not flushed before closing
        time.sleep(0.1)
        self._cf.close_link()
    '''
def callback_joystick(data, crazy):
    #rospy.loginfo(rospy.get_caller_id() + "I heard ")
    global auto_thrust
    if data.axes[2] < 0:
        thrust = min(1,max(0, auto_thrust)) * 0xffff
    else:
        thrust = max(0, data.axes[1]) * 0xffff
    yaw = data.axes[0] * -100
    roll = data.axes[3] * -30
    pitch = data.axes[4] * 30
    rospy.loginfo(rospy.get_caller_id() + "send %s", str(auto_thrust))
    crazy.send_to_crazyflie(roll, pitch, yaw, thrust)
	# rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.axes)

def callback_joystick_autopilot(data, crazy):
    global auto_thrust
    auto_thrust = data.axes[1]

if __name__ == '__main__':
    global auto_thrust
    auto_thrust = 0
    # Initialize the low-level drivers (don't list the debug drivers)
    cflib.crtp.init_drivers(enable_debug_driver=False)
    # Scan for Crazyflies and use the first one found
    print('Scanning interfaces for Crazyflies...')
    available = cflib.crtp.scan_interfaces()
    print('Crazyflies found:')
    for i in available:
        print(i[0])

    if len(available) > 0:
        le = MotorRampExample(available[0][0])
        rospy.init_node('listener', anonymous=True)
        rospy.Subscriber("joy", Joy, callback_joystick, le)
        rospy.Subscriber("autopilot_joy", Joy, callback_joystick_autopilot, le)
        rospy.spin()
    else:
        print('No Crazyflies found, cannot run example')
