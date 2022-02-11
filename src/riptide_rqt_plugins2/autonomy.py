import os
import rospy
import rospkg

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget, QVBoxLayout, QLineEdit, QPushButton
from python_qt_binding.QtCore import QTimer, Slot

from .task_action_widget import TaskActionWidget

class AutonomyWidget(QWidget):
    ui_file = os.path.join(rospkg.RosPack().get_path('riptide_rqt_plugins2'), 'resource', 'AutonomyPlugin.ui')
    task_action_listener_topic = "task_action_listener"

    namespace = ""

    stopping = False

    def __init__(self):
        super(AutonomyWidget, self).__init__()
    
        # Load UI
        loadUi(self.ui_file, self)
        self.setObjectName('AutonomyPluginUi')
        
        # Configure all actions to be used
        self._actions_layout = self.findChild(QVBoxLayout, "actionsVerticalLayout")
        self._actions = []
        self.add_action("Flatten Test", "flatten_test.launch")
        self.add_action("Pitch Test", "pitch_test.launch")
        self.add_action("Roll Test", "roll_test.launch")

        # Namespace config
        self._namespace_text = self.findChild(QLineEdit, "namespaceEdit")
        self._namespace_apply = self.findChild(QPushButton, "applyNamespaceButton")
        self._namespace_apply.clicked.connect(self._namespace_apply_callback)

        # Setup timer
        self._tick_timer = QTimer(self)
        self._tick_timer.timeout.connect(self.timer_tick)

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

    ########################################
    # Lifetime Management
    ########################################

    def start(self):
        """
        This method needs to be called to start updating topic pane.
        """
        self._tick_timer.start(1000)

    def shutdown_plugin(self):
        self.stopping = True
        self._tick_timer.stop()

        for action in self._actions:
            action.cleanup_topics()

    @Slot()
    def timer_tick(self):
        if self.stopping:
            return

        # Tick child widgets
        self.tick_actions()

    def save_settings(self, plugin_settings, instance_settings):
        # Save occurs before shutdown
        instance_settings.set_value('namespace', self.namespace)

    def restore_settings(self, plugin_settings, instance_settings):
        # Restore occurs after init
        if instance_settings.contains('namespace'):
            self.namespace = instance_settings.value('namespace')
            if len(self.namespace) == 0:
                self._namespace_text.setText('/')    
            else:
                self._namespace_text.setText(self.namespace)
            self.update_namespace()

    #def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure
        # This will enable a setting button (gear icon) in each dock widget title bar
        # Usually used to open a modal configuration dialog

    ########################################
    # Actions Management
    ########################################
    def add_action(self, name, launch_file):
        action = TaskActionWidget(name, self.namespace, self.task_action_listener_topic, launch_file, self._actions_layout)
        self._actions.append(action)

    def update_namespace(self):
        for action in self._actions:
            action.set_namespace(self.namespace)

    def tick_actions(self):
        # Call periodic update on all actions
        # Typically called for timer callback for updating state without callbacks (like a disappearing topic)
        for action in self._actions:
            action.tick()


class AutonomyPlugin(Plugin):

    def __init__(self, context):
        super(AutonomyPlugin, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('AutonomyPlugin')

        self._widget = AutonomyWidget()
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