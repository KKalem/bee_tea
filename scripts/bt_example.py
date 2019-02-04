#! /usr/bin/env python
# -*- coding: utf-8 -*-
# vim:fenc=utf-8
#
# Author: Ozer Ozkahraman (ozkahramanozer@gmail.com)
# Date: 2018-09-24

from __future__ import print_function

import rospy
import actionlib

from bee_tea.msg import BTAction, BTGoal, BTFeedback
from bee_tea.bt_states import SUCCESS, FAILURE, RUNNING
from bee_tea.bt_pieces import ActionNodeLeaf, InstantLeaf, Seq, Fallback, Negate


class SomeThing:
    def __init__(self):
        """
        this is some robot state and methods that work on that state.
        just an example
        """

        # robot state
        self.a = 1
        self.b = 2
        self.c = 3

    # state-acting methods, these should not take any arguments
    # except self and they should return one of the bt_states
    def a_eq_b(self):
        if self.a == self.b:
            return SUCCESS
        return FAILURE

    def inc_a(self):
        self.a += 1
        return RUNNING

    def inc_b(self):
        self.b += 2
        return RUNNING

    def inc_c(self):
        self.c += 1
        return SUCCESS

    def c_eq_ten(self):
        if self.c == 10:
            return SUCCESS
        return FAILURE

    def a_larger_than_b(self):
        if self.a > self.b:
            return SUCCESS
        return FAILURE


if __name__ == '__main__':
    # initialize ros node as usual
    rospy.init_node('BT')
    rate = rospy.Rate(10)

    # instantiate the robot state
    s = SomeThing()
    # set some variables
    s.a = 0
    s.b = 9

    # create a tree
    fb0 = Fallback('root')

    fb1 = Fallback('fb1')
    # InstantLeaf-s can be used for conditions or actions that are always instantenous
    # they should never return 'running'
    fb1.add_child(InstantLeaf('a>b',s.a_larger_than_b))
    fb1.add_child(InstantLeaf('a+=1',s.inc_a))

    seq1 = Seq('seq1')
    seq1.add_child(Negate(InstantLeaf('c eq 10', s.c_eq_ten)))
    seq1.add_child(InstantLeaf('c+=1', s.inc_c))
    seq1.add_child(fb1)
    seq1.add_child(InstantLeaf('inc b', s.inc_b))

    fb0.add_child(seq1)
    # ActionNodeLeaf-s are ROS action clients. 
    # A ROS action server with the same name should be running before this node is ticked
    # otherwise this node will wait for the server to be ready.
    # this waiting is not a reliable behaviour
    anl = ActionNodeLeaf('test', goal='s')
    fb0.add_child(anl)

    # just a rename
    root = fb0

    # current return state of the entire tree
    r = FAILURE
    # less print-spam
    prev_s = ''

    print('Traversal:',root.traverse())

    # tick 100 times
    #  for i in range(10000):
    while not rospy.is_shutdown():
        # send a tick through the tree.
        # this is recursive
        r = root.tick()
        # s is a string that represents the tree, together with current states
        # and node names, indented.
        s = root.display(0)

        # less spam
        if s != prev_s:
            print(s)
        prev_s = s

        # a SUCCESS or FAILURE at the root node indicates 'done'
        if r == SUCCESS or r == FAILURE:
            print('We are done here')
            break

        # i am not as fast as the CPU, normally not needed
        rate.sleep()


