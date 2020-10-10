#!/usr/bin/env python
# coding: UTF-8

import rospy
from smach import StateMachine, Sequence, Concurrence, cb_interface, CBState
from smach_ros import SimpleActionState
from geometry_msgs.msg import Point, Quaternion
from vortex_msgs.msg import MoveGoal, MoveAction
from tf.transformations import quaternion_from_euler



def dp_move(x, y, z=-0.5, yaw_rad=0):
    goal = MoveGoal()

    goal.controller_name = 'DP'
    goal.target_pose.position = Point(x, y, z)
    goal.target_pose.orientation = Quaternion(*quaternion_from_euler(0, 0, yaw_rad))

    return SimpleActionState('move', MoveAction, goal=goal)


def los_move(x, y, z=-0.5):
    goal = MoveGoal()

    goal.controller_name = 'LOS'
    goal.target_pose.position = Point(x, y, z)

    return SimpleActionState('move', MoveAction, goal=goal)


def circle_move(target_point, direction):
    goal = MoveGoal()

    goal.controller_name = ''


def patrol_sequence(action_states):

    sm = Sequence(outcomes=['preempted', 'succeeded', 'aborted'], connector_outcome='succeeded')
    counter = 0

    with sm:

        for state in action_states:
            counter = counter + 1
            sm.add("State-%d" %counter, state)

    return sm


@cb_interface(
    outcomes=['alligned', 'wrong_direction'],
    input_keys=['alligned_pose'],
    output_keys=['alligned_pose']
)
def allignment_checker(userdata):
    # TODO
    pass


def allign_with_target(target):
    """
    Returns a state that alligns with the specified target. 

    target: string on the form 'GATE' or 'BOUY'

    The returned state is responsible for chaning the circeling direction when needed.

    """

    # TODO: get target position (x,y) from landmark server

    # TODO: create a move_goal
    move_goal = None

    allignment_attempt = Concurrence(
        outcomes=['succeeded', 'preempted', 'wrong_direction'],
        outcome_map={
            'succeeded': {'ALLIGNMENT_CHECKER': 'alligned'},
            'wrong_direction': {'ALLIGNMENT_CHECKER': 'wrong_direction'}
        },
        default_outcome=['preempted'],
        child_termination_cb=None   # TODO: should allways terminate
    )

    with allignment_attempt:

        Concurrence.add(
            'CIRCLE_GATE', 
            SimpleActionState('move', MoveAction, goal=move_goal) # TODO
        )
        Concurrence.add(
            'ALLIGNMENT_CHECKER', 
            CBState(allignment_checker)
        )
