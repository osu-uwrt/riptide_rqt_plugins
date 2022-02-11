from typing import overload
import rospy
import rospkg
import actionlib
from actionlib_msgs.msg import GoalStatus

from python_qt_binding.QtWidgets import QWidget, QLabel, QPushButton, QTextEdit
from python_qt_binding.QtCore import Slot

import riptide_autonomy.msg

from .action_widget import ActionWidget

class TaskActionWidget(ActionWidget):
    _client_goal_handle = None

    def __init__(self, action_name, namespace, action_topic, launch_file, actions_layout):
        super(TaskActionWidget, self).__init__(action_name, namespace, action_topic, riptide_autonomy.msg.TaskActionListenerAction, actions_layout, False)

        self._action_goal = riptide_autonomy.msg.TaskActionListenerGoal()
        self._action_goal.autonomy_launchfile = launch_file


    def _check_id_matches(self, status_msg):
        if self._client_goal_handle is None:
            return False
        
        if not self._client_goal_handle.comm_state_machine:
            return False
        
        with self._client_goal_handle.comm_state_machine.mutex:
            return status_msg.goal_id.id == self._client_goal_handle.comm_state_machine.action_goal.goal_id.id

    ########################################
    # Function Overrides
    ########################################

    @Slot()
    def _start_callback(self):
        if self._action_client is not None:
            self._set_state(ActionWidget.STATE_LOADING)
            self._client_goal_handle = self._action_client.send_goal(self._action_goal)

    @Slot()
    def _cancel_callback(self):
        if self._client_goal_handle is not None:
            self._client_goal_handle.cancel()

    def cleanup_topics(self):
        self._client_goal_handle = None
        return super().cleanup_topics()

    def tick(self):
        if self._action_client is None:
            return

        server_available = self._action_server_available()

        # Get server connection status
        if server_available:
            if self._client_goal_handle is not None:
                action_state = self._client_goal_handle.get_goal_status()
            else:
                action_state = GoalStatus.PENDING

            # If there is a goal running, and it isn't the one expected, mark the server as busy
            status_list = self._action_client.last_status_msg.status_list
            if len(status_list) > 0:
                shared_action_state = self.STATE_BUSY
                for s in status_list:
                    if self._check_id_matches(s):
                        shared_action_state = action_state  # Action state is in ID server, it is valid
                action_state = shared_action_state

            self._set_state(action_state)
        else:
            self._set_state(GoalStatus.LOST)