#!/usr/bin/env python
# Move the robot back during the pick process

from __future__ import print_function, division

import rospy

from geometry_msgs.msg import Twist
from actionlib_msgs.msg import GoalStatus
from assistance_msgs.msg import ExecutionEvent

from assistance_arbitrator.tracer import Tracer


# The actual class to command the base to move backwards

class MoveBackServer(object):
    """
    Wait until the wait is triggered and during that time, command the robot to
    move backwards
    """

    WAIT_ACTION_NAME = 'wait'
    RESET_DURATION = rospy.Duration(5.0)

    CMD_VEL_TOPIC = '/cmd_vel'
    CMD_VEL_MAGNITUDE = -0.6
    CMD_VEL_FREQUENCY = 10.0
    CMD_VEL_DURATION = rospy.Duration(0.5)

    def __init__(self):
        # Setup the publisher
        self._vel_pub = rospy.Publisher(MoveBackServer.CMD_VEL_TOPIC, Twist, queue_size=1)

        # State variables
        self._move_start_time = rospy.Time(0)

        # Subscriber
        self._trace_sub = rospy.Subscriber(Tracer.EXECUTION_TRACE_TOPIC, ExecutionEvent, self._on_trace)

    def _on_trace(self, msg):
        if not (msg.type == ExecutionEvent.TASK_STEP_EVENT
                and msg.name == MoveBackServer.WAIT_ACTION_NAME
                and msg.task_step_metadata.status == GoalStatus.ACTIVE):
            return

        if self._move_start_time > rospy.Time(0):
            return

        rospy.loginfo("Moving the robot back")
        self._move_start_time = rospy.Time.now()

    def spin(self):
        rate = rospy.Rate(MoveBackServer.CMD_VEL_FREQUENCY)
        while not rospy.is_shutdown():
            if rospy.Time.now() <= self._move_start_time + MoveBackServer.CMD_VEL_DURATION:
                msg = Twist()
                msg.linear.x = MoveBackServer.CMD_VEL_MAGNITUDE
                self._vel_pub.publish(msg)
            elif self._move_start_time > rospy.Time(0) \
                    and rospy.Time.now() >= self._move_start_time + MoveBackServer.RESET_DURATION:
                self._move_start_time = rospy.Time(0)

            rate.sleep()


# The main function
if __name__ == '__main__':
    rospy.init_node('move_back_server')
    server = MoveBackServer()
    server.spin()
