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
        self._result = BTResult()


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

    def _fail(self):
        # FAIL HERE
        self.tree_feedback(FAILURE)
        self._result.bt_status = FAILURE
        self._action_server.set_aborted(self._result)

    def _succeed(self):
        # SUCCESS !
        self.tree_feedback(SUCCESS)
        self._result.bt_status = SUCCESS
        self._action_server.set_succeeded(self._result)


    def _preempt_cb(self):
        # do stuff in case the action is cancelled by the tree
        # before it is done
        rospy.loginfo(self._name+' action received preemption!')
        return


    def _execute_cb(self, goal):
        rospy.loginfo(self._name+' received goal: '+goal.bt_action_goal)

        # let the tree know we are running
        self.tree_feedback(RUNNING)

        succeeded = self.act(goal.bt_action_goal)

        if succeeded:
            # work done successfully
            rospy.loginfo(self._name+' completed successfully.')
            self._succeed()
        else:
            rospy.loginfo(self._name+' failed to complete.')
            self._fail()




if __name__=='__main__':
    import sys

    args = sys.argv
    name = args[1]
    BT_ActionNode(name)
    rospy.spin()
