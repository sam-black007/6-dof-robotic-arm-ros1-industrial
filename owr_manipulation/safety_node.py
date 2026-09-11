#!/usr/bin/env python3
"""
safety_node - Joint-limit watch dog and emergency-stop repeater for the OWR arm.

Subscribes
  /joint_states            sensor_msgs/JointState    (50 Hz, arm controller)
  /emergency_stop          std_msgs/Bool             (hardware E-stop button)

Publishes
  /emergency_stop          std_msgs/Bool             (10 Hz repeater + limit breach)

Behaviour
  - Compares every joint position against the URDF hard limits below.
  - If any joint approaches its hard limit (within SAFETY_MARGIN radians),
    latches an emergency stop and re-broadcasts it.
  - If the hardware E-stop button publishes True, latches and re-broadcasts.
  - Keep-alive: publishes the current estop state at 10 Hz so monitoring
    tools always have fresh data.

Run
  rosrun owr_manipulation safety_node.py
"""

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool

# URDF hard limits in radians - owr_description/urdf/owr.urdf.xacro
JOINT_LIMITS = {
    'BJ':  (-2.0944, 2.0944),
    'SJ':  (-1.5708, 1.5708),
    'EJ':  (-3.9270, 1.0472),
    'W1J': (-1.5708, 1.5708),
    'W2J': (-1.0472, 2.6180),
    'W3J': (-3.1416, 3.1416),
}

SAFETY_MARGIN = 0.05       # radians; breach when within this of a hard limit
FINGER_LOWER, FINGER_UPPER = 0.0, 0.04


class SafetyNode(object):
    def __init__(self):
        self._estop = False
        self._estop_reason = None
        self._pub = rospy.Publisher('/emergency_stop', Bool, queue_size=10)
        self._last_states = rospy.Subscriber('/joint_states', JointState, self._on_joint_states)
        rospy.Subscriber('/emergency_stop', Bool, self._on_estop, callback_args=True)

    def _on_joint_states(self, msg):
        positions = dict(zip(msg.name, msg.position))
        for name, (lower, upper) in JOINT_LIMITS.items():
            if name not in positions:
                continue
            pos = positions[name]
            if pos < lower + SAFETY_MARGIN or pos > upper - SAFETY_MARGIN:
                self._trip('joint {0} at {1:.4f} rad, limit [{2:.4f}, {3:.4f}]'
                           .format(name, pos, lower, upper))
        finger = positions.get('finger_joint')
        if finger is not None:
            if finger < FINGER_LOWER or finger > FINGER_UPPER:
                self._trip('finger_joint at {0:.4f} m, limit [{1:.3f}, {2:.3f}]'
                           .format(finger, FINGER_LOWER, FINGER_UPPER))

    def _on_estop(self, msg, _):
        if msg.data and not self._estop:
            self._trip('hardware E-stop button')

    def _trip(self, reason):
        if not self._estop:
            rospy.logerr('SAFETY: emergency stop - {0}'.format(reason))
        self._estop = True
        self._estop_reason = reason

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self._estop:
                self._pub.publish(Bool(True))
            else:
                self._pub.publish(Bool(False))
            rate.sleep()


if __name__ == '__main__':
    rospy.init_node('safety_node')
    SafetyNode().run()