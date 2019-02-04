#! /usr/bin/env python
# -*- coding: utf-8 -*-
# vim:fenc=utf-8
#
# Author: Ozer Ozkahraman (ozkahramanozer@gmail.com)
# Date: 2018-10-02

from __future__ import print_function
import rospy
from std_msgs.msg import String
import json
import queue

import networkx as nx
import matplotlib.pyplot as plt
plt.ion()
import numpy as np
import pyqtgraph as pg
pg.setConfigOptions(antialias=False)
pg.setConfigOption('background', (240,240,240,255))

from pyqtgraph.Qt import QtCore, QtGui


colors = {'None':(150,150,150,255),
          'SUCCESS':(50,255,50,255),
          'FAILURE':(255,50,50,255),
          'RUNNING':(100,100,255,255)}

# made into tuples so that we can just + with the colors
widths = {'None':(1,),
          'SUCCESS':(2,),
          'FAILURE':(2,),
          'RUNNING':(3,)}


class VisualNode:
    def __init__(self, node, visual_tree, nx_graph):
        # we may want this at some point since change_label actually modifies the
        # label slightly
        self._original_label = node['label']

        self._is_minimized = False
        self._is_visible = True
        self.status = None

        self.children = []
        # this node is adjacent to its children
        self.adj = []
        # combine the lines of the children and the adj info into one
        # so that they are 1-to-1 always
        self.adjlines = []

        # this will be set by the master tree object, once all the nodes
        # are done creating themselves
        self.pos = None

        self._visual_tree = visual_tree
        self._nx_graph = nx_graph

        self.type = node['type']
        # set some stuff at the same time
        self._change_id(node['id'])
        self._change_label(node['label'])
        self._change_status(node['status'])

        nx_graph.add_node(self.str_id, label=self._original_label)


        # order of addition is the same as position in the list of nodes
        self.index = len(visual_tree._nodes)
        visual_tree.add_node(self)

        for child in node['children']:
            # recurse to create the tree
            visual_child = VisualNode(child, visual_tree, nx_graph)
            self.children.append(visual_child)

            # edges for both visual and analytical graphs
            nx_graph.add_edge(self.str_id, visual_child.str_id)
            self.adj.append((self.index, visual_child.index))
            self.adjlines.append((self.index, visual_child.index, visual_child.line))


        # change stuff now that we know the children
        self._change_status(node['status'])

    def re_index(self, visual_tree, nx_graph):
        """
        re-indexes the whole tree
        and adds each node to the tree node list
        """

        # re-set these, since some might not exist anymore
        self.adj = []
        self.adjlines = []

        # order of addition is the same as position in the list of nodes
        self.index = len(visual_tree._nodes)
        visual_tree.add_node(self)

        self._change_label(self._original_label)

        for child in self.children:
            if child._is_visible:
                child.re_index(visual_tree, nx_graph)
                # edges for both visual and analytical graphs
                nx_graph.add_edge(self.str_id, child.str_id)
                self.adj.append((self.index, child.index))
                self.adjlines.append((self.index, child.index, child.line))



    def _change_label(self, label):
        # we dont want very long labels.
        # first lets try splitting from spaces
        #  label = '\n'.join(label.split(' '))
        if self._is_minimized:
            self.label = label

        labels = {'SEQ':'-->',
                  'FB' :'?',
                  'ACT':label,
                  'CON':label,
                  'NEG':'!'}

        if label == 'root':
            self.label = label
        else:
            self.label = labels[self.type]


    def _change_id(self, new_id):
        self.id = new_id
        self.str_id = '-'.join(str(nid) for nid in new_id)


    def _change_status(self, new_status):
        global colors
        global widths

        changed = False
        if self.status != new_status:
            changed = True


        self.status = new_status
        self.color = colors[self.status]
        self.line = self.color+widths[self.status]
        self.brush = pg.mkBrush(self.color)
        self.pen = pg.mkPen(self.color)

        # for long texts, showing them vertically helps a little
        if len(self.children) == 0:# and len(self.label) > 10:
            angle = 0
            anchor = (0.5,0.5)
        else:
            angle = 0
            anchor = (0.5, 0.5)

        self.textbox = {'border':self.pen,
                        'fill':self.brush,
                        'angle':angle,
                        'anchor':anchor,
                        'tooltip':self._original_label,
                        'scale':0.5}

        return changed

    def dictify(self):
        """
        return a dictionary representation of the tree
        """

        r = {}
        r['id'] = self.id
        r['label'] = self._original_label
        r['type'] = self.type
        r['status'] = self.status

        r['children'] = []
        for child in self.children:
            r['children'].append(child.dictify())

        return r

    def set_visibilty(self, invisible):
        self._is_visible = not invisible
        # dont make children visible if we are supposed to be minimized
        if not self._is_minimized:
            for child in self.children:
                child.set_visibilty(invisible)

    def toggle_minimize(self):
        # dont minimize root
        if self._original_label == 'root':
            print("Can't minimize the root node!")
            return

        self._is_minimized = not self._is_minimized
        self._change_label(self._original_label)
        for child in self.children:
            child.set_visibilty(self._is_minimized)



    def refresh(self, node):
        """
        if all nodes are unchanged except their status's, the tree is kept as is
        and the status are modified.
        returns True if a complete re-make is required
        returns a second True if a visual update is needed.
        """
        self_changed = any([node['label'] != self._original_label,
                            node['id'] != self.id,
                            node['type'] != self.type])

        # no need to check children if self is changed completely, we need to
        # re-make the whole tree
        # since the parent will be using 'any' to asses if children are
        # changed, we need to return True to signal the change
        if self_changed:
            return self_changed, False

        children_changed = []
        visual_changed = False
        # number of children the same?
        if len(self.children) == len(node['children']):
            # are the children unchanged?
            # remake the adjlines so that if a child has changed status,
            # we will have the new color in us
            self.adjlines = []
            for dict_child, child in zip(node['children'], self.children):
                child_changed, child_visual_changed = child.refresh(dict_child)
                # collect the change flags
                children_changed.append(child_changed)
                visual_changed = child_visual_changed or visual_changed
                self.adjlines.append((self.index, child.index, child.line))

            # any one could be changed to trigger a re-build
            children_changed = any(children_changed)
        else:
            # not the same num of children!
            children_changed = True

        # if children changed, gotta signal that up
        if children_changed:
            return children_changed, False

        # nothing changed, good.
        # just update the status then
        status_chagned = self._change_status(node['status'])

        # no change!
        return False, status_chagned or visual_changed



class VisualTree:
    def __init__(self, bt_dict):
        """
        creates an object that mimics the given dictionary
        and adds a few extra fields and such

        bt_dict is a dictionary of dictionaries

        self._nodes and _nx_grap.nodes() have the same order by construction
        """

        self._create_new(bt_dict)


    def _create_new(self, bt_dict):
        # a list of VisualNode instances
        self._nodes = []
        # networkx graph, can do layouts and such
        self._nx_graph = nx.OrderedGraph()
        # construct the graph
        self.root_node = VisualNode(bt_dict, self, self._nx_graph)
        # create the id->position dictionary
        self.pos_dict = nx.drawing.nx_pydot.graphviz_layout(self._nx_graph,
                                                            prog='dot')

        # set the nodes positions
        for node in self._nodes:
            node.pos = self.pos_dict[node.str_id]

    def re_index(self):
        """
        re-create the indexes and such
        this takes into account the visibility of each node
        unlike the first creation
        """
        self._nodes = []
        self._nx_graph = nx.OrderedGraph()
        self.root_node.re_index(self, self._nx_graph)

    def add_node(self, visual_node):
        self._nodes.append(visual_node)

    def _get_from_nodes(self, field):
        for node in self._nodes:
            try:
                if node._is_visible:
                    yield node.__getattribute__(field)
                else:
                    pass
            except AttributeError:
                print('AttributeError:',field,'not found in node!')
                break


    def get_adj_lines(self, all_pos):
        all_adj = []
        all_lines = []
        for node in self._nodes:
            if node._is_visible:
                for i1,i2,line in node.adjlines:
                    all_adj.append((i1,i2))
                    all_lines.append(line)

        return all_adj, all_lines



    def refresh(self, bt_dict=None):
        bt_changed, visual_changed = self.root_node.refresh(bt_dict)
        # root node will have checked all the children if there is any
        # breaking changes in the structure of the tree
        if bt_changed:
            # if there is a structural change, re-make the whole thing
            self._create_new(bt_dict)
            return True, False

        return False, visual_changed


    def get_visuals(self):
        """
        return an array of pos, adj, lines and a list of text and textboxes
        """
        # one per node
        self.all_pos = np.array(list(self._get_from_nodes('pos')))
        self.all_text = np.array(list(self._get_from_nodes('label')))
        self.all_textbox = np.array(list(self._get_from_nodes('textbox')))

        # many per node
        edges, lines = self.get_adj_lines(self.all_pos)
        self.all_adj = np.array(edges)
        self.all_lines = np.array(lines,
                         dtype=[('red',np.ubyte),
                                ('green',np.ubyte),
                                ('blue',np.ubyte),
                                ('alpha',np.ubyte),
                                ('width',float)])

        return self.all_pos, self.all_text, self.all_textbox, self.all_adj, self.all_lines






class BTQT(pg.GraphItem):
    def __init__(self, bt_dict_topic):
        """
        A visualizer for behaviour trees that uses Qt for graphics.
        bt_dict is a dictionary of dictionaries that represent the tree.
        """


        self._initial_scale_done = False
        # initially assume scaling is 1
        self.xscale = 1
        self.yscale = 1

        # we need to create a window before we create a GraphItem
        self.window, self.view = self._create_window()
        # this needs to exist before the init to superclass
        self.textItems = []
        # initialize superclass so we get the default functions and such
        pg.GraphItem.__init__(self)
        # add outselves to the view
        self.view.addItem(self)

        # sub to the bt description topic
        self.bt_subs = rospy.Subscriber(bt_dict_topic, String, self.accept_json)

        self._initialized_tree = False
        self.visual_tree = None
        # needed for ros->qt problems
        self._visual_q = queue.Queue()
        # so that we can avoid needless graphics updates
        self._last_json = None

        # signal to the main run loop if we want to be done with life
        self._exit = False

    def accept_json(self, data):
        json_str = data.data
        if json_str != self._last_json:
            root = json.loads(json_str)
            self._visual_q.put(root)
            self._last_json = json_str

    def refresh_visual(self):
        # if the window is not visible, it was closed somehow
        # so we signal to close the whole thing
        self._exit = not self.window.isVisible()

        root = None
        try:
            while self._visual_q.qsize() > 2:
                root = self._visual_q.get_nowait()

            if root is not None:
                if self._initialized_tree:
                    overhaul_needed, visual_needed = self.visual_tree.refresh(root)
                    if visual_needed:
                        self._create_visual()
                else:
                    self.visual_tree = VisualTree(root)
                    self._create_visual()
                    self._initialized_tree = True

        except queue.Empty:
            return self._exit


        return self._exit

    def _update_scaling(self):
        """
        return true if scale has changed enough
        """

        xrng, yrng = self.view.viewRange()
        xmin, xmax = xrng
        ymin, ymax = yrng

        if not self._initial_scale_done:
            self.init_xscale = xmax-xmin
            self.init_yscale = ymax-ymin
            self._initial_scale_done = True

        oldxscale = self.xscale
        oldyscale = self.yscale

        self.xscale = self.init_xscale / (xmax-xmin)
        self.yscale = self.init_yscale / (ymax-ymin)

        if np.abs(self.xscale-oldxscale) > 0.01:
            return True

        return False

    def _create_visual(self):

        pos, text, textbox, adj, lines = self.visual_tree.get_visuals()

        # finally set the data, this probably triggers other stuff in superclass too
        self.setData(pos = pos,
                     size=50,
                     symbol='s',
                     adj = adj,
                     pen = lines,
                     text = text,
                     textbox = textbox,
                     symbolBrush = None,
                     symbolPen = None)


    def _create_window(self):
        # create a window and set some default things to look good
        window = pg.GraphicsWindow()
        window.setWindowTitle('Bee Tea')
        view = window.addViewBox()
        view.setAspectLocked()
        # connect to click listener
        window.scene().sigMouseClicked.connect(self._on_click)
        view.sigRangeChangedManually.connect(self._on_range_changed)
        return window, view

    def _on_range_changed(self, ev):
        scale_changed = self._update_scaling()
        if scale_changed:
            self.scaleTexts()


    def _on_click(self, ev):
        if ev.button() == QtCore.Qt.LeftButton:
            # this is aleft button click on a graph piece, minimize/maximize it
            pos = ev.pos()
            pts = self.scatter.pointsAt(pos)
            if len(pts) != 1:
                # ignore clicks that 'touches' too many things
                ev.ignore()
                return

            item_index = pts[0].data()[0]
            node = self.visual_tree._nodes[item_index]
            node.toggle_minimize()
            self.visual_tree.re_index()
            print('clicked vis node:', node.str_id, 'minimized:', node._is_minimized)
            self._last_clicked_node = node
            self._create_visual()

            ev.accept()
        else:
            ev.ignore()


    def setData(self, **kwds):
        # overwrite the superclass because of texts
        self.text = kwds.pop('text', [])
        self.textbox = kwds.pop('textbox', [])
        self.data = kwds
        if 'pos' in self.data:
            npts = self.data['pos'].shape[0]
            self.data['data'] = np.empty(npts, dtype=[('index', int)])
            self.data['data']['index'] = np.arange(npts)
        self.setTexts(self.text, self.textbox)
        self.updateGraph()

    def scaleTexts(self):
        self.setTexts(self.text, self.textbox)
        self.updateGraph()

    def setTexts(self, text, textbox):
        for i in self.textItems:
            i.scene().removeItem(i)
        self.textItems = []
        # to scale the textboxes
        scale = min(2.5, self.xscale)
        for t,tb in zip(text, textbox):
            if scale < 1:
                t = '.'
            item = pg.TextItem(t,
                               anchor=tb['anchor'],
                               color='k',
                               border=tb['border'],
                               fill=tb['fill'],
                               angle=tb['angle'])
            item.setScale(tb['scale'] * scale)
            item.setToolTip(tb['tooltip'])
            self.textItems.append(item)
            item.setParentItem(self)


    def updateGraph(self):
        pg.GraphItem.setData(self, **self.data)
        for i,item in enumerate(self.textItems):
            item.setPos(*self.data['pos'][i])



if __name__=='__main__':
    rospy.init_node('BT_visualizer')
    btqt = BTQT('/bt_response')
    rate = rospy.Rate(60)

    while not rospy.is_shutdown():
        exit = btqt.refresh_visual()
        pg.QtGui.QApplication.processEvents()
        if exit:
            pg.QtGui.QGuiApplication.quit()
            print('Done')
            break




