import os
import rospy
import rospkg

import threading

from geometry_msgs.msg import Quaternion, Vector3
from std_msgs.msg import Empty, Bool

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget, QVBoxLayout, QLineEdit, QPushButton, QDoubleSpinBox, QToolButton, QLabel
from python_qt_binding.QtCore import QTimer, Slot

from .action_widget import ActionWidget
from .joystick_widget import PS3TeleopWidget
import riptide_controllers.msg

class ControllersWidget(QWidget):
    ui_file = os.path.join(rospkg.RosPack().get_path('riptide_rqt_plugins'), 'resource', 'ControllersPlugin.ui')
    namespace = ""

    LIGHT_STYLE_OFF = "QLabel{ color: rgb(186, 189, 182) }"
    LIGHT_STYLE_RED = "QLabel{ color: rgb(239, 41, 41) }"
    LIGHT_STYLE_GREEN = "QLabel{ color: rgb(78, 154, 6) }"

    stopping = False
    steady_light_data = False
    conflicting_publisher_light_data = False

    CONFLICT_EXPIRATION_TIME = rospy.Duration(30.0)
    CONFLICT_SEQUENCE_TIME = rospy.Duration(10.0)

    def __init__(self):
        super(ControllersWidget, self).__init__()
    
        # Load UI
        loadUi(self.ui_file, self)
        self.setObjectName('ControllersPluginUi')

        # Get all UI elements loaded into class and connected to callbacks
        
        # Configure all actions to be used
        self._actions_layout = self.findChild(QVBoxLayout, "actionsVerticalLayout")
        self._actions = []
        self.add_action("Calibrate Buoyancy", "calibrate_buoyancy", riptide_controllers.msg.CalibrateBuoyancyAction, has_results=True)
        self.add_action("Calibrate Drag", "calibrate_drag", riptide_controllers.msg.CalibrateDragAction, has_results=True)
        self.add_action("Thruster Test", "thruster_test", riptide_controllers.msg.ThrusterTestAction, has_results=False)

        self._teleop_widget = PS3TeleopWidget(self.namespace, self)
        self._teleop_widget_layout = self.findChild(QVBoxLayout, "teleopControlLayout")
        self._teleop_widget_layout.addWidget(self._teleop_widget)

        # Namespace config
        self._namespace_text = self.findChild(QLineEdit, "namespaceEdit")
        self._namespace_apply = self.findChild(QPushButton, "applyNamespaceButton")
        self._namespace_apply.clicked.connect(self._namespace_apply_callback)

        # Status Labels
        self._steady_light = self.findChild(QLabel, "steadyLabel")
        self._conflicting_publisher_light = self.findChild(QLabel, "conflictingPublisherLabel")

        # Target Values
        self._linear_target_title = self.findChild(QLabel, "linearTargetTitle")
        self._angular_target_title = self.findChild(QLabel, "angularTargetTitle")
        self._linear_target_label = self.findChild(QLabel, "linearTargetLabel")
        self._angular_target_label = self.findChild(QLabel, "angularTargetLabel")

        # Position Publishing
        self._position_x = self.findChild(QDoubleSpinBox, "positionXValue")
        self._position_y = self.findChild(QDoubleSpinBox, "positionYValue")
        self._position_z = self.findChild(QDoubleSpinBox, "positionZValue")
        self._orientation_x = self.findChild(QDoubleSpinBox, "orientationXValue")
        self._orientation_y = self.findChild(QDoubleSpinBox, "orientationYValue")
        self._orientation_z = self.findChild(QDoubleSpinBox, "orientationZValue")
        self._orientation_w = self.findChild(QDoubleSpinBox, "orientationWValue")

        self._publish_position = self.findChild(QPushButton, "publishPositionButton")
        self._stop_controller = self.findChild(QPushButton, "stopControllerButton")
        self._publish_position.clicked.connect(self._publish_position_callback)
        self._stop_controller.clicked.connect(self._stop_controller_callback)

        self._software_kill = self.findChild(QPushButton, "softwareKillButton")
        self._software_kill.setVisible(False)
        # TODO: Implement software kill button    

        # Add lock for competing publishers since multiple publishers share the code to check
        # if there's a competing publisher and race conditions occur without it
        self._competing_publishing_lock = threading.Lock()

        # Setup timer
        self._tick_timer = QTimer(self)
        self._tick_timer.timeout.connect(self.timer_tick)

        # Finally configure window to reset state
        self._reset_state()

    def _reset_state(self):
        # Resets the state of the ui to loading
        # This is the default state after any reload, such as loading or changing namespace

        self._publish_position.setEnabled(False)
        self._stop_controller.setEnabled(False)
        self._software_kill.setEnabled(False)
        self._software_kill.setChecked(False)

        self._linear_target_title.setText("Position:")
        self._linear_target_label.setText("Loading")
        self._linear_target_label.setToolTip("")
        self._angular_target_title.setText("Orientation:")
        self._angular_target_label.setText("Loading")
        self._angular_target_label.setToolTip("")

        self._linear_target_data = ["Position:", "No Data", "", None]
        self._angular_target_data = ["Orientation:", "No Data", "", None]

        self._steady_light.setStyleSheet(self.LIGHT_STYLE_OFF)
        self._conflicting_publisher_light.setStyleSheet(self.LIGHT_STYLE_OFF)
        self.steady_light_data = False
        self.conflicting_publisher_light_data = False
        with self._competing_publishing_lock:
            self._controller_publish_history = [[],[]]

    ########################################
    # Topic Management
    ########################################

    def _init_topics(self):
        self._position_pub = rospy.Publisher(self.namespace + "/position", Vector3, queue_size=1)
        self._orientation_pub = rospy.Publisher(self.namespace + "/orientation", Quaternion, queue_size=1)
        self._off_pub = rospy.Publisher(self.namespace + "/off", Empty, queue_size=1)

        self._position_sub = rospy.Subscriber(self.namespace + "/position", Vector3, self._position_callback, queue_size=1)
        self._orientation_sub = rospy.Subscriber(self.namespace + "/orientation", Quaternion, self._orientation_callback, queue_size=1)
        self._linear_velocity_sub = rospy.Subscriber(self.namespace + "/linear_velocity", Vector3, self._linear_velocity_callback, queue_size=1)
        self._angular_velocity_sub = rospy.Subscriber(self.namespace + "/angular_velocity", Vector3, self._linear_velocity_callback, queue_size=1)
        self._off_sub = rospy.Subscriber(self.namespace + "/off", Empty, self._off_callback, queue_size=1)
        self._steady_sub = rospy.Subscriber(self.namespace + "/steady", Bool, self._steady_callback, queue_size=1)

    def _cleanup_topics(self):
        self._position_pub.unregister()
        self._orientation_pub.unregister()
        self._off_pub.unregister()

        self._position_sub.unregister()
        self._orientation_sub.unregister()
        self._linear_velocity_sub.unregister()
        self._angular_velocity_sub.unregister()
        self._off_sub.unregister()
        self._steady_sub.unregister()

    def _track_publisher(self, node):
        with self._competing_publishing_lock:
            current_time = rospy.get_rostime()
            # Format for _controller_publish_history
            # First list is a list of previously published nodes
            # Second list is an ordered list of times, whose index correspond to the nodes in the first list

            # First cleanup any expired messages
            while len(self._controller_publish_history[0]) > 0 and (current_time - self._controller_publish_history[1][0]) > self.CONFLICT_EXPIRATION_TIME:
                tnode = self._controller_publish_history[0].pop(0)
                self._controller_publish_history[1].pop(0)

            # Then check for if the message has been published within expiration time
            if node in self._controller_publish_history[0]:
                if self._controller_publish_history[0][-1] != node and (current_time - self._controller_publish_history[1][-1]) <= self.CONFLICT_SEQUENCE_TIME:
                    self.conflicting_publisher_light_time = current_time
                    self.conflicting_nodes = [node, self._controller_publish_history[0][-1]]
                    self.conflicting_publisher_light_data = True
                prev_index = self._controller_publish_history[0].index(node)
                tnode = self._controller_publish_history[0].pop(prev_index)
                self._controller_publish_history[1].pop(prev_index)
            
            self._controller_publish_history[0].append(node)
            self._controller_publish_history[1].append(current_time)

    def _position_callback(self, msg):
        self._track_publisher(msg._connection_header['callerid'])
        self._linear_target_data[0] = "Position:"
        self._linear_target_data[1] = "({0:.2f}, {1:.2f}, {2:.2f})".format(msg.x, msg.y, msg.z)
        self._linear_target_data[2] = "Origin: " + msg._connection_header['callerid']
        self._linear_target_data[3] = rospy.get_rostime()

    def _orientation_callback(self, msg):
        self._track_publisher(msg._connection_header['callerid'])
        self._angular_target_data[0] = "Orientation:"
        self._angular_target_data[1] = "({0:.2f}, {1:.2f}, {2:.2f}. {3:.2f})".format(msg.x, msg.y, msg.z, msg.w)
        self._angular_target_data[2] = "Origin: " + msg._connection_header['callerid']
        self._angular_target_data[3] = rospy.get_rostime()

    def _linear_velocity_callback(self, msg):
        self._track_publisher(msg._connection_header['callerid'])
        self._linear_target_data[0] = "Lin Vel:"
        self._linear_target_data[1] = "({0:.2f}, {1:.2f}, {2:.2f})".format(msg.x, msg.y, msg.z)
        self._linear_target_data[2] = "Origin: " + msg._connection_header['callerid']
        self._linear_target_data[3] = rospy.get_rostime()

    def _angular_velocity_callback(self, msg):
        self._track_publisher(msg._connection_header['callerid'])
        self._angular_target_data[0] = "Ang Vel:"
        self._angular_target_data[1] = "({0:.2f}, {1:.2f}, {2:.2f})".format(msg.x, msg.y, msg.z)
        self._angular_target_data[2] = "Origin: " + msg._connection_header['callerid']
        self._angular_target_data[3] = rospy.get_rostime()

    def _off_callback(self, msg):
        self._linear_target_data = ["Position:", "No Data", "", None]
        self._angular_target_data = ["Orientation:", "No Data", "", None]

    def _steady_callback(self, msg):
        self.steady_light_data = msg.data

    ########################################
    # Button Callbacks
    ########################################
    
    @Slot()
    def _namespace_apply_callback(self):
        # Fix up entered namespace, and hide trailing slash for aesthetics
        namespace = self._namespace_text.text().rstrip("/")
        
        if len(namespace) != 0 and namespace[0] != "/":
            namespace = "/" + namespace
        
        if len(namespace) == 0:
            self._namespace_text.setText('/')    
        else:
            self._namespace_text.setText(namespace)

        if self.namespace != namespace:
            self.namespace = namespace
            self.update_namespace()

    @Slot()
    def _publish_position_callback(self):
        self._position_pub.publish(Vector3(x=self._position_x.value(), y=self._position_y.value(), z=self._position_z.value()))
        self._orientation_pub.publish(Quaternion(x=self._orientation_x.value(), y=self._orientation_y.value(), z=self._orientation_z.value(), w=self._orientation_w.value()))
    
    @Slot()
    def _stop_controller_callback(self):
        self._off_pub.publish(Empty())

    ########################################
    # Lifetime Management
    ########################################

    def start(self):
        """
        This method needs to be called to start updating topic pane.
        """
        self._init_topics()
        self._tick_timer.start(1000)

    def shutdown_plugin(self):
        self.stopping = True
        self._tick_timer.stop()
        self._cleanup_topics()
        self._teleop_widget.cleanup()

        for action in self._actions:
            action.cleanup_topics()

    @Slot()
    def timer_tick(self):
        if self.stopping:
            return

        # Tick child widgets
        self.tick_actions()
        self._teleop_widget.tick()

        # Update button status
        # We are subscribing to these topics as well, so there must be another subscriber for controllers to be present
        self._publish_position.setEnabled((self._position_pub.get_num_connections() > 1) and (self._orientation_pub.get_num_connections() > 1))
        self._stop_controller.setEnabled(self._off_pub.get_num_connections() > 1)

        # Update linear and angular targets for the controller
        current_time = rospy.get_rostime()
        linear_time_diff = ""
        if self._linear_target_data[3] is not None:
            linear_time_diff = " - {0} Secs. Ago".format((current_time - self._linear_target_data[3]).secs)
        self._linear_target_title.setText(self._linear_target_data[0])
        self._linear_target_label.setText(self._linear_target_data[1])
        self._linear_target_label.setToolTip(self._linear_target_data[2] + linear_time_diff)

        angular_time_diff = ""
        if self._angular_target_data[3] is not None:
            angular_time_diff = " - {0} Secs. Ago".format((current_time - self._angular_target_data[3]).secs)
        self._angular_target_title.setText(self._angular_target_data[0])
        self._angular_target_label.setText(self._angular_target_data[1])
        self._angular_target_label.setToolTip(self._angular_target_data[2] + angular_time_diff)

        # Update the status "lights"
        if self.steady_light_data:
            self._steady_light.setStyleSheet(self.LIGHT_STYLE_GREEN)
        else:
            self._steady_light.setStyleSheet(self.LIGHT_STYLE_OFF)

        if self.conflicting_publisher_light_data and (rospy.get_rostime() - self.conflicting_publisher_light_time) <= self.CONFLICT_EXPIRATION_TIME:
            self._conflicting_publisher_light.setStyleSheet(self.LIGHT_STYLE_RED)
            self._conflicting_publisher_light.setToolTip("Conflicts: {0} and {1}".format(self.conflicting_nodes[0], self.conflicting_nodes[1]))
        else:
            if self.conflicting_publisher_light_data:
                self.conflicting_publisher_light_data = False
                self.conflicting_publisher_light_time = None
            self._conflicting_publisher_light.setStyleSheet(self.LIGHT_STYLE_OFF)
            self._conflicting_publisher_light.setToolTip("")

    def save_settings(self, plugin_settings, instance_settings):
        # Save occurs before shutdown
        instance_settings.set_value('namespace', self.namespace)
        instance_settings.set_value('positionX', self._position_x.value())
        instance_settings.set_value('positionY', self._position_y.value())
        instance_settings.set_value('positionZ', self._position_z.value())
        instance_settings.set_value('orientationX', self._orientation_x.value())
        instance_settings.set_value('orientationY', self._orientation_y.value())
        instance_settings.set_value('orientationZ', self._orientation_z.value())
        instance_settings.set_value('orientationW', self._orientation_w.value())

    def restore_settings(self, plugin_settings, instance_settings):
        # Restore occurs after init
        if instance_settings.contains('namespace'):
            self.namespace = instance_settings.value('namespace')
            if len(self.namespace) == 0:
                self._namespace_text.setText('/')    
            else:
                self._namespace_text.setText(self.namespace)
            self.update_namespace()
        
        if instance_settings.contains('positionX'):
            self._position_x.setValue(float(instance_settings.value('positionX')))
        if instance_settings.contains('positionY'):
            self._position_y.setValue(float(instance_settings.value('positionY')))
        if instance_settings.contains('positionZ'):
            self._position_z.setValue(float(instance_settings.value('positionZ')))
        if instance_settings.contains('orientationX'):
            self._orientation_x.setValue(float(instance_settings.value('orientationX')))
        if instance_settings.contains('orientationY'):
            self._orientation_y.setValue(float(instance_settings.value('orientationY')))
        if instance_settings.contains('orientationZ'):
            self._orientation_z.setValue(float(instance_settings.value('orientationZ')))
        if instance_settings.contains('orientationW'):
            self._orientation_w.setValue(float(instance_settings.value('orientationW')))

    #def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure
        # This will enable a setting button (gear icon) in each dock widget title bar
        # Usually used to open a modal configuration dialog

    ########################################
    # Actions Management
    ########################################
    def add_action(self, name, topic, spec, has_results=False):
        action = ActionWidget(name, self.namespace, topic, spec, self._actions_layout, has_results)
        self._actions.append(action)

    def update_namespace(self):
        for action in self._actions:
            action.set_namespace(self.namespace)
        self._teleop_widget.set_namespace(self.namespace)
        self._cleanup_topics()
        self._reset_state()
        self._init_topics()


    def tick_actions(self):
        # Call periodic update on all actions
        # Typically called for timer callback for updating state without callbacks (like a disappearing topic)
        for action in self._actions:
            action.tick()

class ControllersPlugin(Plugin):

    def __init__(self, context):
        super(ControllersPlugin, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('ControllersPlugin')

        self._widget = ControllersWidget()
        self._widget.start()
        
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)
    
    def shutdown_plugin(self):
        self._widget.shutdown_plugin()

    def save_settings(self, plugin_settings, instance_settings):
        self._widget.save_settings(plugin_settings, instance_settings)

    def restore_settings(self, plugin_settings, instance_settings):
        self._widget.restore_settings(plugin_settings, instance_settings)