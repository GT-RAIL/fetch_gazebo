#!/usr/bin/env python
# Collide the robot during a move

from __future__ import print_function, division

import numpy as np

import rospy

from geometry_msgs.msg import Twist
from actionlib_msgs.msg import GoalStatus
from assistance_msgs.msg import ExecutionEvent

from assistance_arbitrator.tracer import Tracer


# The actual class to command the base to collide the base

class CollideRobotServer(object):
    """
    Move forward and collide the robot when departing
    """

    COLLISION_ACTION_NAME = 'setup'
    COLLISION_LINEAR_DURATION = rospy.Duration(13.0 + (2 * np.random.rand()))
    COLLISION_ANGULAR_DURATION = rospy.Duration(2.0 + (4 * np.random.rand()))

    CMD_VEL_TOPIC = '/cmd_vel'
    CMD_VEL_MAGNITUDE = (0.4 + (0.4 * np.random.rand()))
    CMD_VEL_FREQUENCY = 20.0

    def __init__(self):
        # Setup the publisher
        self._vel_pub = rospy.Publisher(CollideRobotServer.CMD_VEL_TOPIC, Twist, queue_size=1)

        # State variables
        self._move_start_time = rospy.Time(0)

        # Subscriber
        self._trace_sub = rospy.Subscriber(Tracer.EXECUTION_TRACE_TOPIC, ExecutionEvent, self._on_trace)

    def _on_trace(self, msg):
        if not (msg.type == ExecutionEvent.TASK_STEP_EVENT
                and msg.name == CollideRobotServer.COLLISION_ACTION_NAME
                and msg.task_step_metadata.status == GoalStatus.ACTIVE):
            return

        if self._move_start_time > rospy.Time(0):
            return

        rospy.loginfo("Colliding the robot")
        self._move_start_time = rospy.Time.now()

    def spin(self):
        rate = rospy.Rate(CollideRobotServer.CMD_VEL_FREQUENCY)
        while not rospy.is_shutdown():
            if rospy.Time.now() <= self._move_start_time + CollideRobotServer.COLLISION_ANGULAR_DURATION \
                    and self._move_start_time > rospy.Time(0):
                msg = Twist()
                msg.angular.z = CollideRobotServer.CMD_VEL_MAGNITUDE
                self._vel_pub.publish(msg)
            elif rospy.Time.now() <= (self._move_start_time
                                      + CollideRobotServer.COLLISION_ANGULAR_DURATION
                                      + CollideRobotServer.COLLISION_LINEAR_DURATION) \
                    and self._move_start_time > rospy.Time(0):
                msg = Twist()
                msg.linear.x = CollideRobotServer.CMD_VEL_MAGNITUDE
                self._vel_pub.publish(msg)
            # No else condition because the robot should be immovable after this

            rate.sleep()


# The main function
if __name__ == '__main__':
    rospy.init_node('collide_robot_server')
    server = CollideRobotServer()
    server.spin()
