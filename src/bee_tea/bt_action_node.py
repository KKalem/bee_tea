#! /usr/bin/env python
# -*- coding: utf-8 -*-
# vim:fenc=utf-8
#
# Author: Ozer Ozkahraman (ozkahramanozer@gmail.com)
# Date: 2018-06-14

from __future__ import print_function

import rospy
import actionlib

from bee_tea.msg import BTAction, BTFeedback, BTResult
from bee_tea.bt_states import SUCCESS, FAILURE, RUNNING

class BT_ActionNode:
    def __init__(self, name):
        rospy.init_node('bt_action__'+name)


        self._action_server = actionlib.SimpleActionServer(name, BTAction, self._execute_cb, False)
        self._action_server.register_preempt_callback(self._preempt_cb)
        self._action_server.start()
        self._name = name

        self._feedback = BTFeedback()

        self._current_goal = ""
        while True:
            while self._current_goal == "":
                pass

            P = start process(self.act, self._current_goal)

            while P is alive and self._current_goal is not "preempt":
                tree_feedback(RUNNING)

            if P is alive and goal is "preempt":
                kill P
                tree_feedback(FAILURE)

            get retrun P
            decide if S/F
            tree_feedback(decision)

            self._current_goal = ""




    def tree_feedback(self, s):
        """
        s = bt_states.X
        """
        self._feedback.bt_status = s
        self._action_server.publish_feedback(self._feedback)


    def act(self, goal):
        """
        return False if action failed, True otherwise
        """
        ###################################
        # actual action
        fail = False
        if goal == 'f':
            fail = True
        r = rospy.Rate(1)
        for i in range(6):
            # work
            print(i)
            if fail and i == 2:
                print('failed')
                return False
            r.sleep()
        print('SUCC')
        return True
        ###################################


    def _execute_cb(self, goal):
        rospy.loginfo(self._name+' received goal: '+goal.bt_action_goal)
        self._current_goal = goal.bt_action_goal


if __name__=='__main__':
    import sys

    args = sys.argv
    name = args[1]
    BT_ActionNode(name)
    rospy.spin()
