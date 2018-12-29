#!/usr/bin/env python
# Look Away from the table after the robot looks forward in preparation to pick
# up the object

from __future__ import print_function, division

import rospy
import actionlib

from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
from actionlib_msgs.msg import GoalStatus
from assistance_msgs.msg import ExecutionEvent

from assistance_arbitrator.tracer import Tracer


# The actual class to command the head to look away

class LookAwayServer(object):
    """
    Wait until a wait is triggered and during that time, command the robot to
    look anywhere other than the table
    """

    WAIT_ACTION_NAME = 'wait'

    JOINT_TRAJECTORY_ACTION_SERVER = '/head_controller/follow_joint_trajectory'
    HEAD_JOINT_NAMES = ['head_pan_joint', 'head_tilt_joint']
    LOOK_AWAY_JOINT_POSITIONS = [-1.0, 0.7]
    LOOK_AWAY_DURATION = rospy.Duration(0.9)

    RESET_DURATION = rospy.Duration(5.0)

    def __init__(self):
        # Connect to the action server
        self._trajectory_client = actionlib.SimpleActionClient(
            LookAwayServer.JOINT_TRAJECTORY_ACTION_SERVER,
            FollowJointTrajectoryAction
        )

        # Create the goal to send
        self._look_away_goal = FollowJointTrajectoryGoal(
            goal_time_tolerance=LookAwayServer.LOOK_AWAY_DURATION
        )
        self._look_away_goal.trajectory.joint_names = LookAwayServer.HEAD_JOINT_NAMES
        self._look_away_goal.trajectory.points.append(JointTrajectoryPoint(
            positions=LookAwayServer.LOOK_AWAY_JOINT_POSITIONS,
            velocities=[0. for x in LookAwayServer.LOOK_AWAY_JOINT_POSITIONS],
            accelerations=[0. for x in LookAwayServer.LOOK_AWAY_JOINT_POSITIONS],
            effort=[0. for x in LookAwayServer.LOOK_AWAY_JOINT_POSITIONS],
            time_from_start=LookAwayServer.LOOK_AWAY_DURATION
        ))
        self._reset_timer = None

        # The subscriber
        self._trace_sub = rospy.Subscriber(Tracer.EXECUTION_TRACE_TOPIC, ExecutionEvent, self._on_trace)

    def _on_trace(self, msg):
        if not(msg.type == ExecutionEvent.TASK_STEP_EVENT
               and msg.name == LookAwayServer.WAIT_ACTION_NAME
               and msg.task_step_metadata.status == GoalStatus.ACTIVE):
            return

        if self._reset_timer is None:
            rospy.loginfo("Redirecting the gaze")
            self._look_away_goal.trajectory.header.seq += 1
            self._look_away_goal.trajectory.header.stamp = rospy.Time.now()
            self._trajectory_client.send_goal(self._look_away_goal)
            self._reset_timer = rospy.Timer(LookAwayServer.RESET_DURATION, self._reset, oneshot=True)

    def _reset(self, evt):
        self._reset_timer = None


# The main function

if __name__ == '__main__':
    rospy.init_node('look_away_server')
    server = LookAwayServer()
    rospy.spin()
