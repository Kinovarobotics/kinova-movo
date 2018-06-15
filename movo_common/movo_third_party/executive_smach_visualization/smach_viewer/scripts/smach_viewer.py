#!/usr/bin/env python

# Copyright (c) 2010, Willow Garage, Inc.
# All rights reserved.
# Copyright (c) 2013, Jonathan Bohren, The Johns Hopkins University
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# 
#   * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#   * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#   * Neither the name of the Willow Garage, Inc. nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Jonathan Bohren 

import rospy
import rospkg

from smach_msgs.msg import SmachContainerStatus,SmachContainerInitialStatusCmd,SmachContainerStructure

import sys
import os
import threading
import pickle
import pprint
import copy
import StringIO
import colorsys
import time

import wxversion
if wxversion.checkInstalled("2.8"):
    wxversion.select("2.8")
else:
    print("wxversion 2.8 is not installed, installed versions are {}".format(wxversion.getInstalled()))
import wx
import wx.richtext

import textwrap

## this import system (or ros-released) xdot
# import xdot
## need to import currnt package, but not to load this file
# http://stackoverflow.com/questions/6031584/importing-from-builtin-library-when-module-with-same-name-exists
def import_non_local(name, custom_name=None):
    import imp, sys

    custom_name = custom_name or name

    path = filter(lambda x: x != os.path.dirname(os.path.abspath(__file__)), sys.path)
    f, pathname, desc = imp.find_module(name, path)

    module = imp.load_module(custom_name, f, pathname, desc)
    if f:
        f.close()

    return module

smach_viewer = import_non_local('smach_viewer')
from smach_viewer import xdot
##
import smach
import smach_ros

### Helper Functions
def graph_attr_string(attrs):
    """Generate an xdot graph attribute string."""
    attrs_strs = ['"'+str(k)+'"="'+str(v)+'"' for k,v in attrs.iteritems()]
    return ';\n'.join(attrs_strs)+';\n'

def attr_string(attrs):
    """Generate an xdot node attribute string."""
    attrs_strs = ['"'+str(k)+'"="'+str(v)+'"' for k,v in attrs.iteritems()]
    return ' ['+(', '.join(attrs_strs))+']'

def get_parent_path(path):
    """Get the parent path of an xdot node."""
    path_tokens = path.split('/')
    if len(path_tokens) > 2:
        parent_path = '/'.join(path_tokens[0:-1])
    else:
        parent_path = '/'.join(path_tokens[0:1])
    return parent_path

def get_label(path):
    """Get the label of an xdot node."""
    path_tokens = path.split('/')
    return path_tokens[-1]

def hex2t(color_str):
    """Convert a hexadecimal color strng into a color tuple."""
    color_tuple = [int(color_str[i:i+2],16)/255.0    for i in range(1,len(color_str),2)]
    return color_tuple

class ContainerNode():
    """
    This class represents a given container in a running SMACH system. 

    Its primary use is to generate dotcode for a SMACH container. It has
    methods for responding to structure and status messages from a SMACH
    introspection server, as well as methods for updating the styles of a 
    graph once it's been drawn.
    """
    def __init__(self, server_name, msg):
        # Store path info
        self._server_name = server_name
        self._path = msg.path
        splitpath = msg.path.split('/')
        self._label = splitpath[-1]
        self._dir = '/'.join(splitpath[0:-1])

        self._children = msg.children
        self._internal_outcomes = msg.internal_outcomes
        self._outcomes_from = msg.outcomes_from
        self._outcomes_to = msg.outcomes_to

        self._container_outcomes = msg.container_outcomes

        # Status
        self._initial_states = []
        self._active_states = []
        self._last_active_states = []
        self._local_data = smach.UserData()
        self._info = ''

    def update_structure(self, msg):
        """Update the structure of this container from a given message. Return True if anything changes."""
        needs_update = False

        if self._children != msg.children\
                or self._internal_outcomes != msg.internal_outcomes\
                or self._outcomes_from != msg.outcomes_from\
                or self._outcomes_to != msg.outcomes_to\
                or self._container_outcomes != msg.container_outcomes:
            needs_update = True

        if needs_update:
            self._children = msg.children
            self._internal_outcomes = msg.internal_outcomes
            self._outcomes_from = msg.outcomes_from
            self._outcomes_to = msg.outcomes_to

            self._container_outcomes = msg.container_outcomes

        return needs_update

    def update_status(self, msg):
        """Update the known userdata and active state set and return True if the graph needs to be redrawn."""

        # Initialize the return value
        needs_update = False

        # Check if the initial states or active states have changed
        if set(msg.initial_states) != set(self._initial_states):
            self._structure_changed = True
            needs_update = True
        if set(msg.active_states) != set(self._active_states):
            needs_update = True

        # Store the initial and active states
        self._initial_states = msg.initial_states
        self._last_active_states = self._active_states
        self._active_states = msg.active_states

        # Unpack the user data
        while not rospy.is_shutdown():
            try:
                self._local_data._data = pickle.loads(msg.local_data)
                break
            except ImportError as ie:
                # This will only happen once for each package
                modulename = ie.args[0][16:]
                packagename = modulename[0:modulename.find('.')]
                roslib.load_manifest(packagename)
                self._local_data._data = pickle.loads(msg.local_data)

        # Store the info string
        self._info = msg.info

        return needs_update

    def get_dotcode(self, selected_paths, closed_paths, depth, max_depth, containers, show_all, label_wrapper, attrs={}):
        """Generate the dotcode representing this container.
        
        @param selected_paths: The paths to nodes that are selected
        @closed paths: The paths that shouldn't be expanded
        @param depth: The depth to start traversing the tree
        @param max_depth: The depth to which we should traverse the tree
        @param containers: A dict of containers keyed by their paths
        @param show_all: True if implicit transitions should be shown
        @param label_wrapper: A text wrapper for wrapping element names
        @param attrs: A dict of dotcode attributes for this cluster
        """

        dotstr = 'subgraph "cluster_%s" {\n' % (self._path)
        if depth == 0:
            #attrs['style'] = 'filled,rounded'
            attrs['color'] = '#00000000'
            attrs['fillcolor'] = '#0000000F'
        #attrs['rank'] = 'max'

        #,'succeeded','aborted','preempted'attrs['label'] = self._label
        dotstr += graph_attr_string(attrs)

        # Add start/terimate target
        proxy_attrs = {
                'URL':self._path,
                'shape':'plaintext',
                'color':'gray',
                'fontsize':'18',
                'fontweight':'18',
                'rank':'min',
                'height':'0.01'}
        proxy_attrs['label'] = '\\n'.join(label_wrapper.wrap(self._label))
        dotstr += '"%s" %s;\n' % (
                '/'.join([self._path,'__proxy__']),
                attr_string(proxy_attrs))

        # Check if we should expand this container
        if max_depth == -1 or depth <= max_depth:
            # Add container outcomes
            dotstr += 'subgraph "cluster_%s" {\n' % '/'.join([self._path,'__outcomes__'])
            outcomes_attrs = {
                    'style':'rounded,filled',
                    'rank':'sink',
                    'color':'#FFFFFFFF',#'#871C34',
                    'fillcolor':'#FFFFFF00'#'#FE464f3F'#'#DB889A'
                    }
            dotstr += graph_attr_string(outcomes_attrs)

            for outcome_label in self._container_outcomes:
                outcome_path = ':'.join([self._path,outcome_label])
                outcome_attrs = {
                        'shape':'box',
                        'height':'0.3',
                        'style':'filled,rounded',
                        'fontsize':'12',
                        'fillcolor':'#FE464f',#'#EDC2CC',
                        'color':'#780006',#'#EBAEBB',
                        'fontcolor':'#780006',#'#EBAEBB',
                        'label':'\\n'.join(label_wrapper.wrap(outcome_label)),
                        'URL':':'.join([self._path,outcome_label])
                        }
                dotstr += '"%s" %s;\n' % (outcome_path,attr_string(outcome_attrs))
            dotstr += "}\n"

            # Iterate over children
            for child_label in self._children:
                child_attrs = {
                        'style':'filled,setlinewidth(2)',
                        'color':'#000000FF',
                        'fillcolor':'#FFFFFF00'
                        }

                child_path = '/'.join([self._path,child_label])
                # Generate dotcode for children
                if child_path in containers:
                    child_attrs['style'] += ',rounded'

                    dotstr += containers[child_path].get_dotcode(
                            selected_paths,
                            closed_paths,
                            depth+1, max_depth,
                            containers,
                            show_all,
                            label_wrapper,
                            child_attrs)
                else:
                    child_attrs['label'] = '\\n'.join(label_wrapper.wrap(child_label))
                    child_attrs['URL'] = child_path
                    dotstr += '"%s" %s;\n' % (child_path, attr_string(child_attrs))

            # Iterate over edges
            internal_edges = zip(
                    self._internal_outcomes,
                    self._outcomes_from,
                    self._outcomes_to)

            # Add edge from container label to initial state
            internal_edges += [('','__proxy__',initial_child) for initial_child in self._initial_states]

            has_explicit_transitions = []
            for (outcome_label,from_label,to_label) in internal_edges:
                if to_label != 'None' or outcome_label == to_label:
                    has_explicit_transitions.append(from_label)

            # Draw internal edges
            for (outcome_label,from_label,to_label) in internal_edges:

                from_path = '/'.join([self._path, from_label])

                if show_all \
                        or to_label != 'None'\
                        or from_label not in has_explicit_transitions \
                        or (outcome_label == from_label) \
                        or from_path in containers:
                    # Set the implicit target of this outcome
                    if to_label == 'None':
                        to_label = outcome_label

                    to_path = '/'.join([self._path, to_label])

                    edge_attrs = {
                            'URL':':'.join([from_path,outcome_label,to_path]),
                            'fontsize':'12',
                            'label':'\\n'.join(label_wrapper.wrap(outcome_label))}
                    edge_attrs['style'] = 'setlinewidth(2)'

                    # Hide implicit
                    #if not show_all and to_label == outcome_label:
                    #    edge_attrs['style'] += ',invis'

                    from_key = '"%s"' % from_path
                    if from_path in containers:
                        if max_depth == -1 or depth+1 <= max_depth:
                            from_key = '"%s:%s"' % ( from_path, outcome_label)
                        else:
                            edge_attrs['ltail'] = 'cluster_'+from_path
                            from_path = '/'.join([from_path,'__proxy__'])
                            from_key = '"%s"' % ( from_path )

                    to_key = ''
                    if to_label in self._container_outcomes:
                        to_key = '"%s:%s"' % (self._path,to_label)
                        edge_attrs['color'] = '#00000055'# '#780006'
                    else:
                        if to_path in containers:
                            edge_attrs['lhead'] = 'cluster_'+to_path
                            to_path = '/'.join([to_path,'__proxy__'])
                        to_key = '"%s"' % to_path

                    dotstr += '%s -> %s %s;\n' % (
                            from_key, to_key, attr_string(edge_attrs))

        dotstr += '}\n'
        return dotstr

    def set_styles(self, selected_paths, depth, max_depth, items, subgraph_shapes, containers):
        """Update the styles for a list of containers without regenerating the dotcode.

        This function is called recursively to update an entire tree.
        
        @param selected_paths: A list of paths to nodes that are currently selected.
        @param depth: The depth to start traversing the tree
        @param max_depth: The depth to traverse into the tree
        @param items: A dict of all the graph items, keyed by url
        @param subgraph_shapes: A dictionary of shapes from the rendering engine
        @param containers: A dict of all the containers
        """

        # Color root container
        """
        if depth == 0:
            container_shapes = subgraph_shapes['cluster_'+self._path]
            container_color = (0,0,0,0)
            container_fillcolor = (0,0,0,0)

            for shape in container_shapes:
                shape.pen.color = container_color
                shape.pen.fillcolor = container_fillcolor
                """

        # Color shapes for outcomes

        # Color children
        if max_depth == -1 or depth <= max_depth:
            # Iterate over children
            for child_label in self._children:
                child_path = '/'.join([self._path,child_label])

                child_color = [0.5,0.5,0.5,1]
                child_fillcolor = [1,1,1,1]
                child_linewidth = 2

                active_color = hex2t('#5C7600FF')
                active_fillcolor = hex2t('#C0F700FF')

                initial_color = hex2t('#000000FF')
                initial_fillcolor = hex2t('#FFFFFFFF')

                if child_label in self._active_states:
                    # Check if the child is active
                    child_color = active_color
                    child_fillcolor = active_fillcolor
                    child_linewidth = 5
                elif child_label in self._initial_states:
                    # Initial style
                    #child_fillcolor = initial_fillcolor
                    child_color = initial_color
                    child_linewidth = 2

                # Check if the child is selected
                if child_path in selected_paths:
                    child_color = hex2t('#FB000DFF')

                # Generate dotcode for child containers 
                if child_path in containers:
                    subgraph_id = 'cluster_'+child_path
                    if subgraph_id in subgraph_shapes:
                        if child_label in self._active_states:
                            child_fillcolor[3] = 0.25
                        elif 0 and child_label in self._initial_states:
                            child_fillcolor[3] = 0.25
                        else:
                            if max_depth > 0:
                                v = 1.0-0.25*((depth+1)/float(max_depth))
                            else:
                                v = 0.85
                            child_fillcolor = [v,v,v,1.0]

                        
                        for shape in subgraph_shapes['cluster_'+child_path]:
                            pen = shape.pen
                            if len(pen.color) > 3:
                                pen_color_opacity = pen.color[3]
                                if pen_color_opacity < 0.01:
                                    pen_color_opacity = 0
                            else:
                                pen_color_opacity = 0.5
                            shape.pen.color = child_color[0:3]+[pen_color_opacity]
                            shape.pen.fillcolor = [child_fillcolor[i] for i in range(min(3,len(pen.fillcolor)))]
                            shape.pen.linewidth = child_linewidth

                        # Recurse on this child
                        containers[child_path].set_styles(
                                selected_paths,
                                depth+1, max_depth,
                                items,
                                subgraph_shapes,
                                containers)
                else:
                    if child_path in items:
                        for shape in items[child_path].shapes:
                            if not isinstance(shape,xdot.xdot.TextShape):
                                shape.pen.color = child_color
                                shape.pen.fillcolor = child_fillcolor
                                shape.pen.linewidth = child_linewidth
                    else:
                        #print child_path+" NOT IN "+str(items.keys())
                        pass

class SmachViewerFrame(wx.Frame):
    """
    This class provides a GUI application for viewing SMACH plans.
    """
    def __init__(self):
        wx.Frame.__init__(self, None, -1, "Smach Viewer", size=(720,480))

        # Create graph
        self._containers = {}
        self._top_containers = {}
        self._update_cond = threading.Condition()
        self._needs_refresh = True
        self.dotstr = ''

        vbox = wx.BoxSizer(wx.VERTICAL)


        # Create Splitter
        self.content_splitter = wx.SplitterWindow(self, -1,style = wx.SP_LIVE_UPDATE)
        self.content_splitter.SetMinimumPaneSize(24)
        self.content_splitter.SetSashGravity(0.85)


        # Create viewer pane
        viewer = wx.Panel(self.content_splitter,-1)

        # Create smach viewer 
        nb = wx.Notebook(viewer,-1,style=wx.NB_TOP | wx.WANTS_CHARS)
        viewer_box = wx.BoxSizer()
        viewer_box.Add(nb,1,wx.EXPAND | wx.ALL, 4)
        viewer.SetSizer(viewer_box)

        # Create graph view
        graph_view = wx.Panel(nb,-1)
        gv_vbox = wx.BoxSizer(wx.VERTICAL)
        graph_view.SetSizer(gv_vbox)

        # Construct toolbar
        toolbar = wx.ToolBar(graph_view, -1)

        toolbar.AddControl(wx.StaticText(toolbar,-1,"Path: "))

        # Path list
        self.path_combo = wx.ComboBox(toolbar, -1, style=wx.CB_DROPDOWN)
        self.path_combo .Bind(wx.EVT_COMBOBOX, self.set_path)
        self.path_combo.Append('/')
        self.path_combo.SetValue('/')
        toolbar.AddControl(self.path_combo)

        # Depth spinner
        self.depth_spinner = wx.SpinCtrl(toolbar, -1,
                size=wx.Size(50,-1),
                min=-1,
                max=1337,
                initial=-1)
        self.depth_spinner.Bind(wx.EVT_SPINCTRL,self.set_depth)
        self._max_depth = -1
        toolbar.AddControl(wx.StaticText(toolbar,-1,"    Depth: "))
        toolbar.AddControl(self.depth_spinner)

        # Label width spinner
        self.width_spinner = wx.SpinCtrl(toolbar, -1,
                size=wx.Size(50,-1),
                min=1,
                max=1337,
                initial=40)
        self.width_spinner.Bind(wx.EVT_SPINCTRL,self.set_label_width)
        self._label_wrapper = textwrap.TextWrapper(40,break_long_words=True)
        toolbar.AddControl(wx.StaticText(toolbar,-1,"    Label Width: "))
        toolbar.AddControl(self.width_spinner)

        # Implicit transition display
        toggle_all = wx.ToggleButton(toolbar,-1,'Show Implicit')
        toggle_all.Bind(wx.EVT_TOGGLEBUTTON, self.toggle_all_transitions)
        self._show_all_transitions = False

        toolbar.AddControl(wx.StaticText(toolbar,-1,"    "))
        toolbar.AddControl(toggle_all)

        toggle_auto_focus = wx.ToggleButton(toolbar, -1, 'Auto Focus')
        toggle_auto_focus.Bind(wx.EVT_TOGGLEBUTTON, self.toggle_auto_focus)
        self._auto_focus = False

        toolbar.AddControl(wx.StaticText(toolbar, -1, "    "))
        toolbar.AddControl(toggle_auto_focus)

        toolbar.AddControl(wx.StaticText(toolbar,-1,"    "))
        toolbar.AddLabelTool(wx.ID_HELP, 'Help',
                wx.ArtProvider.GetBitmap(wx.ART_HELP,wx.ART_OTHER,(16,16)) )
        toolbar.AddLabelTool(wx.ID_SAVE, 'Save',
                wx.ArtProvider.GetBitmap(wx.ART_FILE_SAVE,wx.ART_OTHER,(16,16)) )
        toolbar.Realize()

        self.Bind(wx.EVT_TOOL, self.ShowControlsDialog, id=wx.ID_HELP)
        self.Bind(wx.EVT_TOOL, self.SaveDotGraph, id=wx.ID_SAVE)

        # Create dot graph widget
        self.widget = xdot.wxxdot.WxDotWindow(graph_view, -1)

        gv_vbox.Add(toolbar, 0, wx.EXPAND)
        gv_vbox.Add(self.widget, 1, wx.EXPAND)

        # Create tree view widget
        self.tree = wx.TreeCtrl(nb,-1,style=wx.TR_HAS_BUTTONS)
        nb.AddPage(graph_view,"Graph View")
        nb.AddPage(self.tree,"Tree View")


        # Create userdata widget
        borders = wx.LEFT | wx.RIGHT | wx.TOP
        border = 4
        self.ud_win = wx.ScrolledWindow(self.content_splitter, -1)
        self.ud_gs = wx.BoxSizer(wx.VERTICAL)

        self.ud_gs.Add(wx.StaticText(self.ud_win,-1,"Path:"),0, borders, border)

        self.path_input = wx.ComboBox(self.ud_win,-1,style=wx.CB_DROPDOWN)
        self.path_input.Bind(wx.EVT_COMBOBOX,self.selection_changed)
        self.ud_gs.Add(self.path_input,0,wx.EXPAND | borders, border)


        self.ud_gs.Add(wx.StaticText(self.ud_win,-1,"Userdata:"),0, borders, border)

        self.ud_txt = wx.TextCtrl(self.ud_win,-1,style=wx.TE_MULTILINE | wx.TE_READONLY)
        self.ud_gs.Add(self.ud_txt,1,wx.EXPAND | borders, border)
        
        # Add initial state button
        self.is_button = wx.Button(self.ud_win,-1,"Set as Initial State")
        self.is_button.Bind(wx.EVT_BUTTON, self.on_set_initial_state)
        self.is_button.Disable()
        self.ud_gs.Add(self.is_button,0,wx.EXPAND | wx.BOTTOM | borders, border)

        self.ud_win.SetSizer(self.ud_gs)


        # Set content splitter
        self.content_splitter.SplitVertically(viewer, self.ud_win, 512)

        # Add statusbar
        self.statusbar = wx.StatusBar(self,-1)

        # Add elements to sizer
        vbox.Add(self.content_splitter, 1, wx.EXPAND | wx.ALL)
        vbox.Add(self.statusbar, 0, wx.EXPAND)

        self.SetSizer(vbox)
        self.Center()

        # smach introspection client
        self._client = smach_ros.IntrospectionClient()
        self._containers= {}
        self._selected_paths = []

        # Message subscribers
        self._structure_subs = {}
        self._status_subs = {}

        self.Bind(wx.EVT_IDLE,self.OnIdle)
        self.Bind(wx.EVT_CLOSE,self.OnQuit)

        # Register mouse event callback
        self.widget.register_select_callback(self.select_cb)
        self._path = '/'
        self._needs_zoom = True
        self._structure_changed = True

        # Start a thread in the background to update the server list
        self._keep_running = True
        self._server_list_thread = threading.Thread(target=self._update_server_list)
        self._server_list_thread.start()

        self._update_graph_thread = threading.Thread(target=self._update_graph)
        self._update_graph_thread.start()
        self._update_tree_thread = threading.Thread(target=self._update_tree)
        self._update_tree_thread.start()

    def OnQuit(self,event):
        """Quit Event: kill threads and wait for join."""
        with self._update_cond:
            self._keep_running = False
            self._update_cond.notify_all()

        self._server_list_thread.join()
        self._update_graph_thread.join()
        self._update_tree_thread.join()
        
        event.Skip()

    def update_graph(self):
        """Notify all that the graph needs to be updated."""
        with self._update_cond:
            self._update_cond.notify_all()

    def on_set_initial_state(self, event):
        """Event: Change the initial state of the server."""
        state_path = self._selected_paths[0]
        parent_path = get_parent_path(state_path)
        state = get_label(state_path)

        server_name = self._containers[parent_path]._server_name
        self._client.set_initial_state(server_name,parent_path,[state],timeout = rospy.Duration(60.0))

    def set_path(self, event):
        """Event: Change the viewable path and update the graph."""
        self._path = self.path_combo.GetValue()
        self._needs_zoom = True
        self.update_graph()

    def _set_path(self, path):
        self._path = path
        self._needs_zoom = True
        self.path_combo.SetValue(path)
        self.update_graph()

    def set_depth(self, event):
        """Event: Change the maximum depth and update the graph."""
        self._max_depth = self.depth_spinner.GetValue()
        self._needs_zoom = True
        self.update_graph()

    def _set_max_depth(self, max_depth):
        self._max_depth = max_depth
        self.depth_spinner.SetValue(max_depth)
        self._needs_zoom = True
        self.update_graph()

    def set_label_width(self, event):
        """Event: Change the label wrapper width and update the graph."""
        self._label_wrapper.width = self.width_spinner.GetValue()
        self._needs_zoom = True
        self.update_graph()

    def toggle_all_transitions(self, event):
        """Event: Change whether automatic transitions are hidden and update the graph."""
        self._show_all_transitions = not self._show_all_transitions
        self._structure_changed = True
        self.update_graph()

    def toggle_auto_focus(self, event):
        """Event: Enable/Disable automatically focusing"""
        self._auto_focus = not self._auto_focus
        self._needs_zoom = self._auto_focus
        self._structure_changed = True
        if not self._auto_focus:
            self._set_path('/')
            self._max_depth(-1)
        self.update_graph()

    def select_cb(self, item, event):
        """Event: Click to select a graph node to display user data and update the graph."""

        # Only set string status
        if not type(item.url) is str:
            return

        self.statusbar.SetStatusText(item.url)
        # Left button-up
        if event.ButtonUp(wx.MOUSE_BTN_LEFT):
            # Store this item's url as the selected path
            self._selected_paths = [item.url]
            # Update the selection dropdown
            self.path_input.SetValue(item.url)
            wx.PostEvent(
                    self.path_input.GetEventHandler(),
                    wx.CommandEvent(wx.wxEVT_COMMAND_COMBOBOX_SELECTED,self.path_input.GetId()))
            self.update_graph()

    def selection_changed(self, event):
        """Event: Selection dropdown changed."""
        path_input_str = self.path_input.GetValue()

        # Check the path is non-zero length
        if len(path_input_str) > 0:
            # Split the path (state:outcome), and get the state path
            path = path_input_str.split(':')[0]

            # Get the container corresponding to this path, since userdata is
            # stored in the containers
            if path not in self._containers:
                parent_path = get_parent_path(path)
            else:
                parent_path = path

            if parent_path in self._containers:
                # Enable the initial state button for the selection
                self.is_button.Enable()

                # Get the container
                container = self._containers[parent_path]

                # Store the scroll position and selection
                pos = self.ud_txt.HitTestPos(wx.Point(0,0))
                sel = self.ud_txt.GetSelection()

                # Generate the userdata string
                ud_str = ''
                for (k,v) in container._local_data._data.iteritems():
                    ud_str += str(k)+": "
                    vstr = str(v)
                    # Add a line break if this is a multiline value
                    if vstr.find('\n') != -1:
                        ud_str += '\n'
                    ud_str+=vstr+'\n\n'

                # Set the userdata string
                self.ud_txt.SetValue(ud_str)

                # Restore the scroll position and selection
                self.ud_txt.ShowPosition(pos[1])
                if sel != (0,0):
                    self.ud_txt.SetSelection(sel[0],sel[1])
            else:
                # Disable the initial state button for this selection
                self.is_button.Disable()

    def _structure_msg_update(self, msg, server_name):
        """Update the structure of the SMACH plan (re-generate the dotcode)."""

        # Just return if we're shutting down
        if not self._keep_running:
            return

        # Get the node path
        path = msg.path
        pathsplit = path.split('/')
        parent_path = '/'.join(pathsplit[0:-1])

        rospy.logdebug("RECEIVED: "+path)
        rospy.logdebug("CONTAINERS: "+str(self._containers.keys()))

        # Initialize redraw flag
        needs_redraw = False

        if path in self._containers:
            rospy.logdebug("UPDATING: "+path)

            # Update the structure of this known container
            needs_redraw = self._containers[path].update_structure(msg)
        else: 
            rospy.logdebug("CONSTRUCTING: "+path)

            # Create a new container
            container = ContainerNode(server_name, msg)
            self._containers[path] = container

            # Store this as a top container if it has no parent
            if parent_path == '':
                self._top_containers[path] = container

            # Append paths to selector
            self.path_combo.Append(path)
            self.path_input.Append(path)

            # We need to redraw thhe graph if this container's parent is already known
            if parent_path in self._containers:
                needs_redraw = True

        # Update the graph if necessary
        if needs_redraw:
            with self._update_cond:
                self._structure_changed = True
                self._needs_zoom = True # TODO: Make it so you can disable this
                self._update_cond.notify_all()

    def _status_msg_update(self, msg):
        """Process status messages."""

        # Check if we're in the process of shutting down
        if not self._keep_running:
            return

        if self._auto_focus and len(msg.info) > 0:
            self._set_path(msg.info)
            self._set_max_depth(msg.info.count('/')-1)

        # Get the path to the updating conainer
        path = msg.path
        rospy.logdebug("STATUS MSG: "+path)

        # Check if this is a known container
        if path in self._containers:
            # Get the container and check if the status update requires regeneration
            container = self._containers[path]
            if container.update_status(msg):
                with self._update_cond:
                    self._update_cond.notify_all()

            # TODO: Is this necessary?
            path_input_str = self.path_input.GetValue()
            if path_input_str == path or get_parent_path(path_input_str) == path:
                wx.PostEvent(
                        self.path_input.GetEventHandler(),
                        wx.CommandEvent(wx.wxEVT_COMMAND_COMBOBOX_SELECTED,self.path_input.GetId()))

    def _update_graph(self):
        """This thread continuously updates the graph when it changes.

        The graph gets updated in one of two ways:

          1: The structure of the SMACH plans has changed, or the display
          settings have been changed. In this case, the dotcode needs to be
          regenerated. 

          2: The status of the SMACH plans has changed. In this case, we only
          need to change the styles of the graph.
        """
        while self._keep_running and not rospy.is_shutdown():
            with self._update_cond:
                # Wait for the update condition to be triggered
                self._update_cond.wait()

                # Get the containers to update
                containers_to_update = {}
                if self._path in self._containers:
                    # Some non-root path
                    containers_to_update = {self._path:self._containers[self._path]}
                elif self._path == '/':
                    # Root path
                    containers_to_update = self._top_containers

                # Check if we need to re-generate the dotcode (if the structure changed)
                # TODO: needs_zoom is a misnomer
                if self._structure_changed or self._needs_zoom:
                    dotstr = "digraph {\n\t"
                    dotstr += ';'.join([
                        "compound=true",
                        "outputmode=nodesfirst",
                        "labeljust=l",
                        "nodesep=0.5",
                        "minlen=2",
                        "mclimit=5",
                        "clusterrank=local",
                        "ranksep=0.75",
                        # "remincross=true",
                        # "rank=sink",
                        "ordering=\"\"",
                        ])
                    dotstr += ";\n"

                    # Generate the rest of the graph
                    # TODO: Only re-generate dotcode for containers that have changed
                    for path,tc in containers_to_update.iteritems():
                        dotstr += tc.get_dotcode(
                                self._selected_paths,[],
                                0,self._max_depth,
                                self._containers,
                                self._show_all_transitions,
                                self._label_wrapper)
                    else:
                        dotstr += '"__empty__" [label="Path not available.", shape="plaintext"]'

                    dotstr += '\n}\n'
                    self.dotstr = dotstr
                    # Set the dotcode to the new dotcode, reset the flags
                    self.set_dotcode(dotstr,zoom=False)
                    self._structure_changed = False

                # Update the styles for the graph if there are any updates
                for path,tc in containers_to_update.iteritems():
                    tc.set_styles(
                            self._selected_paths,
                            0,self._max_depth,
                            self.widget.items_by_url,
                            self.widget.subgraph_shapes,
                            self._containers)

                # Redraw
                self.widget.Refresh()

    def set_dotcode(self, dotcode, zoom=True):
        """Set the xdot view's dotcode and refresh the display."""
        # Set the new dotcode
        if self.widget.set_dotcode(dotcode, None):
            self.SetTitle('Smach Viewer')
            # Re-zoom if necessary
            if zoom or self._needs_zoom:
                self.widget.zoom_to_fit()
                self._needs_zoom = False
            # Set the refresh flag
            self._needs_refresh = True
            wx.PostEvent(self.GetEventHandler(), wx.IdleEvent())

    def _update_tree(self):
        """Update the tree view."""
        while self._keep_running and not rospy.is_shutdown():
            with self._update_cond:
                self._update_cond.wait()
                self.tree.DeleteAllItems()
                self._tree_nodes = {}
                for path,tc in self._top_containers.iteritems():
                    self.add_to_tree(path, None)

    def add_to_tree(self, path, parent):
        """Add a path to the tree view."""
        if parent is None:
            container = self.tree.AddRoot(get_label(path))
        else:
            container = self.tree.AppendItem(parent,get_label(path))

        # Add children to tree
        for label in self._containers[path]._children:
            child_path = '/'.join([path,label])
            if child_path in self._containers.keys():
                self.add_to_tree(child_path, container)
            else:
                self.tree.AppendItem(container,label)

    def append_tree(self, container, parent = None):
        """Append an item to the tree view."""
        if not parent:
            node = self.tree.AddRoot(container._label)
            for child_label in container._children:
                self.tree.AppendItem(node,child_label)

    def OnIdle(self, event):
        """Event: On Idle, refresh the display if necessary, then un-set the flag."""
        if self._needs_refresh:
            self.Refresh()
            # Re-populate path combo
            self._needs_refresh = False

    def _update_server_list(self):
        """Update the list of known SMACH introspection servers."""
        while self._keep_running:
            # Update the server list
            server_names = self._client.get_servers()
            new_server_names = [sn for sn in server_names if sn not in self._status_subs]

            # Create subscribers for new servers
            for server_name in new_server_names:
                self._structure_subs[server_name] = rospy.Subscriber(
                        server_name+smach_ros.introspection.STRUCTURE_TOPIC,
                        SmachContainerStructure,
                        callback = self._structure_msg_update,
                        callback_args = server_name,
                        queue_size=50)

                self._status_subs[server_name] = rospy.Subscriber(
                        server_name+smach_ros.introspection.STATUS_TOPIC,
                        SmachContainerStatus,
                        callback = self._status_msg_update,
                        queue_size=50)

            # This doesn't need to happen very often
            rospy.sleep(1.0)
            
            
            #self.server_combo.AppendItems([s for s in self._servers if s not in current_servers])

            # Grab the first server
            #current_value = self.server_combo.GetValue()
            #if current_value == '' and len(self._servers) > 0:
            #    self.server_combo.SetStringSelection(self._servers[0])
            #    self.set_server(self._servers[0])

    def ShowControlsDialog(self,event):
        dial = wx.MessageDialog(None,
                "Pan: Arrow Keys\nZoom: PageUp / PageDown\nZoom To Fit: F\nRefresh: R",
                'Keyboard Controls', wx.OK)
        dial.ShowModal()

    def SaveDotGraph(self,event):
        timestr = time.strftime("%Y%m%d-%H%M%S")
        directory = rospkg.get_ros_home()+'/dotfiles/'
        if not os.path.exists(directory):
                os.makedirs(directory)
        filename = directory+timestr+'.dot'
        print('Writing to file: %s' % filename)
        with open(filename, 'w') as f:
            f.write(self.dotstr)

    def OnExit(self, event):
        pass

    def set_filter(self, filter):
        self.widget.set_filter(filter)

def main():
    from argparse import ArgumentParser
    p = ArgumentParser()
    p.add_argument('-f', '--auto-focus',
                 action='store_true',
                 help="Enable 'AutoFocus to subgraph' as default",
                 dest='enable_auto_focus')
    args = p.parse_args()
    app = wx.App()

    frame = SmachViewerFrame()
    frame.set_filter('dot')

    frame.Show()

    if args.enable_auto_focus:
        frame.toggle_auto_focus(None)

    app.MainLoop()

if __name__ == '__main__':
    rospy.init_node('smach_viewer',anonymous=False, disable_signals=True,log_level=rospy.INFO)
    sys.argv = rospy.myargv()
    main()
