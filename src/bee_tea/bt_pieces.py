#! /usr/bin/env python
# -*- coding: utf-8 -*-
# vim:fenc=utf-8
#
# Author: Ozer Ozkahraman (ozkahramanozer@gmail.com)
# Date: 2018-06-14

from __future__ import print_function

import rospy
import actionlib

from bee_tea.msg import BTAction, BTGoal, BTFeedback
from bee_tea.bt_states import SUCCESS, FAILURE, RUNNING

import random

class AbstractLeafNode:
    def traverse(self, **kwargs):
        '''
        kwargs here just to absorb any keyword args passed
        '''

        r={'label':str(self._name),
           'id':self._unique_name,
           'status':str(self._status),
           'type':str(self._type),
           'children':[]}

        return r

    def traverse_graph(self, graph):
        """
        graph should be a networkx graph object
        returns the name of the graph node for this leaf so
        you can add an edge from the parent if need be
        """
        graph_id = '-'.join(self._unique_name)
        graph.add_node(graph_id)
        return self._unique_name

class AbstractBranchNode:
    def __init__(self):
        # will we show the children of this node?
        # they will still be processed
        self.expanded = True

    def add_child(self, child):
        self._children.append(child)
        child.parent = self

    def preempt(self):
        for child in self._children:
            child.preempt()

        self._status_string = str(self._status)+ ' -> None (Preempt)'
        self._status = None

    def traverse(self, recurse=True):
        '''
        if not recursive, the children are not traversed
        '''

        if self._name == 'root':
            self._unique_name = [0]

        c = []
        i = 0
        for child in self._children:
            child._unique_name = self._unique_name+[i]
            i += 1
            if recurse:
                c.append(child.traverse())
            else:
                c.append(child._unique_name)

        r={'label':str(self._name),
           'id':self._unique_name,
           'status':str(self._status),
           'type':str(self._type),
           'children':c}

        return r

    def traverse_graph(self, graph):
        """
        graph should be a networkx graph object
        returns the name of the graph node for this leaf so
        you can add an edge from the parent if need be

        adds edges to the children from this node
        """
        graph_id = '-'.join(self._unique_name)
        graph.add_node(graph_id)

        for child in self._children:
            child_name = child.traverse_graph(graph)
            graph.add_edge(self._unique_name, child_name)

        return self._unique_name


class ActionNodeLeaf(AbstractLeafNode):
    """
    A complete and long running action
    There needs to be an action server with the same name somewhere else, running.
    This leaf will block until the server is there.

    See bt_action_node.py for an example action server.
    That file can be used directly as it is.
    """

    def __init__(self, name, goal='no_goal', goal_fn=None):
        # this name should be the same as a running bt_action_node
        self._name = name
        self._type = 'ACT'
        self._unique_name = None
        self.parent = None

        # None means not checked
        # = {SUCCESS, FAILURE, RUNNING}
        self._status = None

        # the action this leaf connects to
        self._ac = actionlib.SimpleActionClient(self._name, BTAction)
        rospy.loginfo(self._name+' is waiting for its action node')
        self._ac.wait_for_server()
        rospy.loginfo(self._name+' connected to action node')

        # if this is a simple action, the goal can be defined from the get go
        # like "move left 10cm"
        # of course the action node needs to understand this
        # goal_fn should return a string representing the goal
        self._goal = BTGoal()
        if goal_fn is None:
            self._goal.bt_action_goal = goal
            self._goal_update_fn = None
        else:
            self._goal.bt_action_goal = goal_fn()
            self._goal_update_fn = goal_fn


        # a string for information purposes ONLY
        self._status_string = str(self._status)


    def tick(self):
        if self._status is None:
            # this is the first we are ticked
            # run the action
            # update the goal if need be
            if self._goal_update_fn is not None:
                self._goal.bt_action_goal = self._goal_update_fn()

            self._ac.send_goal(self._goal,
                               done_cb = None,
                               active_cb = None,
                               feedback_cb = self._feedback_cb)

            # we do not wait for the result
            # and give the control back to the rest of the tree
            # the feedback callback will update the status of this
            # leaf once it receives it
            # return RUNNING for now, just in case the feedback didnt
            # come back fast, we can afford to wait a single tick
            rospy.loginfo(self._name+' started')

            self._status_string = 'None -> RUNNING'
            self._status = RUNNING
            return RUNNING

        elif self._status == SUCCESS or self._status == FAILURE:
            # the action ran already and returned something
            # report this to the tree
            ret = self._status
            # reset the 'doneness' of the action so we can do it again
            self._status = None
            rospy.loginfo(self._name+' is done with:'+str(ret))

            self._status_string = str(ret)+' -> None'
            return ret

        elif self._status == RUNNING:
            # feedback is not a reliable channel for success/failure communication
            # check the status too if we are running
            result = self._ac.get_result()
            if result is not None:
                # feedback didnt happen
                # manual check found a resulting condition though
                # same as the above SUCCESS/FAIL checks
                # we let the tree know of the result immediately
                self._status = None
                ret = result.bt_status
                rospy.loginfo(self._name+' is done with:'+str(ret))

                self._status_string = 'RUNNING -> '+str(ret)+' -> None'
                return ret

            rospy.logdebug(self._name+' is running')

            self._status_string = 'RUNNING'
            return RUNNING

        else:
            # something has gone terribly wrong
            rospy.logerr(self._name+' has a bad status: '+str(self._status))

            self._status_string = 'BAD STATUS'
            return None

    def _feedback_cb(self, fb):
        rospy.loginfo(self._name+' received feedback:'+fb.bt_status)
        self._status = fb.bt_status

    def _active_cb(self, *args):
        print(self._name, 'still runs')

    def preempt(self):
        """
        tell the action to STOP
        but only if its running
        """
        if self._status == RUNNING:
            rospy.loginfo(self._name+' preempt requested')
            self._ac.cancel_all_goals()

            self._status_string = str(self._status)+ ' -> None (Preempt)'
            self._status = None

    def display(self, level):
        s = level*'   '+'A:['+self._name+']:'+str(self._status_string)+'\n'
        return s



class InstantLeaf(AbstractLeafNode):
    """
    This leaf instantly completes the job
    and therefore doesnt need an action server.

    Returns whatever the given function returns when ticked.

    Use for internal updates, flag setting, conditions etc.
    Everything in *args will be passed to this function.
    """

    def __init__(self, name, instant_act_function, *args, **kwargs):
        self._name = name
        self._type = 'ACT'
        self._unique_name = None
        self.parent = None

        self._instant_act_function = instant_act_function
        self._args = args
        self._kwargs = kwargs

        self._status = None
        self._status_string = 'None'

    def tick(self):
        """
        The instant act function should return either SUCCESS or
        FAILURE. It is assumed to be instant, therefore no RUNNING please
        """
        s = str(self._status)

        r = self._instant_act_function(*self._args, **self._kwargs)
        #  if not (r == SUCCESS or r == FAILURE):
            #  print("INSTANT ACTION RETURNED SOMETHING THAT IS NOT SUCCESS OR FAILURE")
        self._status = r

        self._status_string = s+' -> '+str(r)
        return r

    def preempt(self):
        """
        should not be needed, this is instant
        """
        self._status_string = str(self._status)+ ' -> None (Preempt)'
        self._status = None


    def display(self, level):
        s = level*'   '+'['+self._name+']:'+str(self._status)+'\n'
        return s



class Seq(AbstractBranchNode):
    """
    Ticks children in the given order as they return SUCCESS,
    returns the child's return if its not SUCCESS.
    """
    def __init__(self, name, children=None):
        AbstractBranchNode.__init__(self)
        if children is None:
            self._children = []
        else:
            self._children = children
        self._name = name
        self._unique_name = None
        self._type = 'SEQ'
        self.parent = None

        self._status = None
        self._status_string = 'None'


    def tick(self):
        for i in range(len(self._children)):
            child = self._children[i]
            r = child.tick()

            # if success, we will tick the next one
            # and not return or preempt the other children
            if r != SUCCESS:
                self._status_string = str(self._status)+' -> '+str(r)
                self._status = r
                # one child is running or failed
                # all next children must be 'unticked'
                # or status = None
                for j in range(i+1, len(self._children)):
                    self._children[j].preempt()

                return r

        # all children returned success, we return success
        self._status_string = str(self._status)+' -> SUCCESS'
        self._status = SUCCESS
        return SUCCESS

    def display(self, level):
        s = level*'   '+'SEQ:'+str(self._name)+':'+str(self._status)+'\n'
        if self.expanded:
            for child in self._children:
                s += str(child.display(level+1))
        return s



class Fallback(AbstractBranchNode):
    """
    Ticks children in the given order as they return FAILURE,
    returns the child's return if its not FAILURE
    """
    def __init__(self, name, children=None):
        AbstractBranchNode.__init__(self)
        if children is None:
            self._children = []
        else:
            self._children = children
        self._name = name
        self._type = 'FB'
        self._unique_name = None
        self.parent = None

        self._status = None
        self._status_string = 'None'

    def tick(self):
        for i in range(len(self._children)):
            child = self._children[i]
            r = child.tick()

            # if failure, we will tick the next one
            # and not return
            if r != FAILURE:
                self._status_string = str(self._status)+' -> '+ str(r)
                self._status = r
                # one child is running or succeeded
                # all next children must be 'unticked'
                # or status = None
                for j in range(i+1, len(self._children)):
                    self._children[j].preempt()
                return r

        # all children failed, we fail too
        self._status_string = str(self._status)+' -> FAILURE'
        self._status = FAILURE
        return FAILURE


    def display(self, level):
        s = level*'   '+'FB:'+str(self._name)+':'+str(self._status)+'\n'
        if self.expanded:
            for child in self._children:
                s += str(child.display(level+1))
        return s

class Negate(AbstractBranchNode):
    """
    Negates the return of the child when ticked.
    Does not do anything for "RUNNING"
    """
    def __init__(self, child):
        AbstractBranchNode.__init__(self)
        #  so we maintain interface with other 'has-a-child' nodes
        self._children = [child]
        self._name = '!('+child._name+')'
        self._type = 'NEG'
        self._unique_name = None
        self.parent = None

        self._status = None
        self._status_string = 'None'

    def tick(self):
        r = self._children[0].tick()
        if r == SUCCESS:
            self._status_string = str(self._status)+' -> '+FAILURE
            self._status = FAILURE
            return FAILURE
        if r == FAILURE:
            self._status_string = str(self._status)+' -> '+SUCCESS
            self._status = SUCCESS
            return SUCCESS

        self._status_string = str(self._status)+' -> '+str(r)
        self._status = r
        return r

    def display(self, level):
        s = level*'   '+'['+self._name+']:'+str(self._status)+'\n'
        return s

    def add_child(self, child):
        self._children[0] = child

    def preempt(self):
        """
        This should not be needed for this leaf ever
        This leaf is supposed to be instant!
        Mostly here for compatibility
        """
        self._children[0].preempt()
        self._status_string = str(self._status)+ ' -> None (Preempt)'
        self._status = None


