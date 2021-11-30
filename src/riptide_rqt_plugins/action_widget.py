import os
import rospy
import rospkg
import actionlib
from actionlib_msgs.msg import GoalStatus

from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget, QLabel, QPushButton, QTextEdit
from python_qt_binding.QtCore import Slot

class ActionWidget(QWidget):
    # Constant Vars
    ui_file = os.path.join(rospkg.RosPack().get_path('riptide_rqt_plugins'), 'resource', 'ActionControl.ui')
    results_ui_file = os.path.join(rospkg.RosPack().get_path('riptide_rqt_plugins'), 'resource', 'TextWindow.ui')

    # Negative numbers for states, since actionserver uses non-negative numbers for status
    STATE_BUSY = -3
    STATE_UNINITIALIZED = -2
    STATE_LOADING = -1

    STYLE_NORMAL = "QLabel{ color: rgb(85, 87, 83) }"
    STYLE_RED = "QLabel{ color: rgb(239, 41, 41) }"
    STYLE_GREEN = "QLabel{ color: rgb(78, 154, 6) }"

    # Initialized instance variables
    state = STATE_UNINITIALIZED
    prev_num_goals = -1

    last_result = None
    results_window = None

    def __init__(self, action_name, namespace, action_topic, action_spec, actions_layout, has_results):
        super(ActionWidget, self).__init__()
        # Initialize instance variables
        self.name = action_name
        self.action_spec = action_spec
        self.namespace = namespace
        self.topic = action_topic
        self.has_results = has_results

        # Load UI into class
        loadUi(self.ui_file, self)

        self.setObjectName('ActionWidget-' + action_topic)
        
        self._action_name = self.findChild(QLabel, "actionTitle")
        self._action_name.setText(action_name)

        self._action_status = self.findChild(QLabel, "statusLabel")
        self._start_btn = self.findChild(QPushButton, "startActionButton")
        self._cancel_btn = self.findChild(QPushButton, "cancelActionButton")
        self._results_btn = self.findChild(QPushButton, "resultsButton")
        self._results_btn.setVisible(has_results)

        self._start_btn.clicked.connect(self._start_callback)
        self._cancel_btn.clicked.connect(self._cancel_callback)
        self._results_btn.clicked.connect(self._results_callback)

        # Configure UI
        self._set_state(ActionWidget.STATE_LOADING)
        self._init_topics()

        actions_layout.addWidget(self)

    ########################################
    # Private Functions
    ########################################

    def _init_topics(self):
        self._action_client = actionlib.ActionClient(self.namespace + "/" + self.topic, self.action_spec)
        self._set_state(ActionWidget.STATE_LOADING)

        self._results_btn.setEnabled(False)
        self.last_result = None

    def _action_server_available(self):
        try:
           return self._action_client.wait_for_server(rospy.Duration(-0.001))
        except TypeError:
            # If rqt starts <1ms after ros the first call might fail since the duration will be negative
            # But, duration of 0 means block until server is found, so a negative number is needed to never block
            return False

    ########################################
    # Callback Functions
    ########################################

    def _transition_callback(self, client_goal_handle):
        self.last_result = client_goal_handle.get_result()

    @Slot()
    def _start_callback(self):
        if self._action_client is not None:
            self._set_state(ActionWidget.STATE_LOADING)
            self.tracked_goal = self._action_client.send_goal(self.action_spec(), transition_cb=self._transition_callback)

    @Slot()
    def _cancel_callback(self):
        if self._action_client is not None:
            self._set_state(ActionWidget.STATE_LOADING)
            self._action_client.cancel_all_goals()

    @Slot()
    def _results_callback(self):
        if self.last_result is not None:
            if self.results_window is None:
                self.results_window = QWidget()
                loadUi(self.results_ui_file, self.results_window)

                self.results_window.setWindowTitle(self.name + " Results")
                title_label = self.results_window.findChild(QLabel, "titleLabel")
                title_label.setText(self.name + " Last Results:")

            text_area = self.results_window.findChild(QTextEdit, "textBox")
            text_area.setPlainText(str(self.last_result))
            
            self.results_window.show()

    ########################################
    # Public Functions
    ########################################

    def cleanup_topics(self):
        action_client = self._action_client
        self._action_client = None
        action_client.stop()
        self._set_state(ActionWidget.STATE_LOADING)

        if self.results_window is not None:
            self.results_window.destroy(destroyWindow = True)
            self.results_window = None

    def tick(self):
        if self._action_client is None:
            return
        
        self._results_btn.setEnabled(self.last_result is not None)

        server_available = self._action_server_available()

        # Get server connection status immediately
        if server_available:
            # last_status_msg is gaurenteed if wait_for_server returns true
            status_list = self._action_client.last_status_msg.status_list
            action_state = self._get_highest_priority_state(status_list)
            self._set_state(action_state, len(status_list))
        else:
            self._set_state(GoalStatus.LOST)

    def set_namespace(self, namespace):
        self.cleanup_topics()
        self.namespace = namespace
        self._init_topics()

    ########################################
    # State Decoding/Processing
    ########################################

    GOAL_PRIORITY_LIST = [
                            GoalStatus.ACTIVE,                              # Active highest priority, since it should be clear when running
                            GoalStatus.PREEMPTING, GoalStatus.RECALLING,    # Preempting/Recalling means that it could still be running
                            GoalStatus.REJECTED,                            # Rejected state means that there was an error processing goal
                            GoalStatus.PREEMPTED, GoalStatus.RECALLED,      # Preempted/Recalled means that it could be in an unstable state
                            GoalStatus.ABORTED,                             # Report failure of goal completion
                            GoalStatus.SUCCEEDED,                           # Finally report success of goal completion
                        ]

    def _get_highest_priority_state(self, statelist):
        action_status = GoalStatus.PENDING
        action_index = len(ActionWidget.GOAL_PRIORITY_LIST)
        for state in statelist:
            try:
                state_index = ActionWidget.GOAL_PRIORITY_LIST.index(state.status)
                if state_index < action_index:
                    action_index = state_index
                    action_status = state.status
            except ValueError:
                rospy.logwarn("Unexpected status from ActionServer: %d (%s)", state.status, str(state))
        
        return action_status

    def _set_state(self, state, num_goals=1):
        # Update the action state
        if self.state != state or self.prev_num_goals != num_goals:
            count_string = ""
            if num_goals > 1:
                count_string = " [{0}]".format(num_goals)
            
            self.prev_num_goals = num_goals
            self.state = state
            if state == ActionWidget.STATE_LOADING:
                self._action_status.setText("(Loading)" + count_string)
                self._action_status.setStyleSheet(ActionWidget.STYLE_NORMAL)
                self._action_status.setToolTip("State is loading. The status of action is unknown")

                self._start_btn.setEnabled(False)
                self._cancel_btn.setEnabled(False)
            elif state == ActionWidget.STATE_BUSY:
                self._action_status.setText("(Busy)" + count_string)
                self._action_status.setStyleSheet(ActionWidget.STYLE_NORMAL)
                self._action_status.setToolTip("The actionserver is busy with other requests at the time")

                self._start_btn.setEnabled(False)
                self._cancel_btn.setEnabled(False)
            elif state == GoalStatus.LOST:
                self._action_status.setText("(Not Found)" + count_string)
                self._action_status.setStyleSheet(ActionWidget.STYLE_RED)
                self._action_status.setToolTip("The action could not be found")

                self._start_btn.setEnabled(False)
                self._cancel_btn.setEnabled(False)
            elif state == GoalStatus.PENDING:
                self._action_status.setText("(Not Run)" + count_string)
                self._action_status.setStyleSheet(ActionWidget.STYLE_NORMAL)
                self._action_status.setToolTip("The goal has yet to be processed by the action server")

                self._start_btn.setEnabled(True)
                self._cancel_btn.setEnabled(False)
            elif state == GoalStatus.ACTIVE:
                self._action_status.setText("(Running)" + count_string)
                self._action_status.setStyleSheet(ActionWidget.STYLE_GREEN)
                self._action_status.setToolTip("The goal is currently being processed by the action server")

                self._start_btn.setEnabled(False)
                self._cancel_btn.setEnabled(True)
            elif state == GoalStatus.PREEMPTED:
                self._action_status.setText("(Cancelled)" + count_string)
                self._action_status.setStyleSheet(ActionWidget.STYLE_NORMAL)
                self._action_status.setToolTip("The goal received a cancel request after it started executing and has since completed its execution")

                self._start_btn.setEnabled(True)
                self._cancel_btn.setEnabled(False)
            elif state == GoalStatus.SUCCEEDED:
                self._action_status.setText("(Finished)" + count_string)
                self._action_status.setStyleSheet(ActionWidget.STYLE_NORMAL)
                self._action_status.setToolTip("The goal was achieved successfully by the action server")

                self._start_btn.setEnabled(True)
                self._cancel_btn.setEnabled(False)
            elif state == GoalStatus.ABORTED:
                self._action_status.setText("(Failure)" + count_string)
                self._action_status.setStyleSheet(ActionWidget.STYLE_RED)
                self._action_status.setToolTip("The goal was aborted during execution by the action server due to some failure")

                self._start_btn.setEnabled(True)
                self._cancel_btn.setEnabled(False)
            elif state == GoalStatus.REJECTED:
                self._action_status.setText("(Rejected)" + count_string)
                self._action_status.setStyleSheet(ActionWidget.STYLE_RED)
                self._action_status.setToolTip("The goal was rejected by the action server without being processed, because the goal was unattainable or invalid")

                self._start_btn.setEnabled(True)
                self._cancel_btn.setEnabled(False)
            elif state == GoalStatus.PREEMPTING:
                self._action_status.setText("(Cancelling)" + count_string)
                self._action_status.setStyleSheet(ActionWidget.STYLE_NORMAL)
                self._action_status.setToolTip("The goal received a cancel request after it started executing and has not yet completed execution")

                self._start_btn.setEnabled(False)
                self._cancel_btn.setEnabled(False)
            elif state == GoalStatus.RECALLING:
                self._action_status.setText("(Recalling)" + count_string)
                self._action_status.setStyleSheet(ActionWidget.STYLE_NORMAL)
                self._action_status.setToolTip("The goal received a cancel request before it started executing, but the action server has not yet confirmed that the goal is canceled")

                self._start_btn.setEnabled(False)
                self._cancel_btn.setEnabled(False)
            elif state == GoalStatus.RECALLING:
                self._action_status.setText("(Recalled)" + count_string)
                self._action_status.setStyleSheet(ActionWidget.STYLE_NORMAL)
                self._action_status.setToolTip("The goal received a cancel request before it started executing and was successfully cancelled")

                self._start_btn.setEnabled(True)
                self._cancel_btn.setEnabled(False)
            else:
                self._action_status.setText("(INVALID)" + count_string)
                self._action_status.setStyleSheet(ActionWidget.STYLE_RED)
                self._action_status.setToolTip("An invalid state was sent by the action server")

                self._start_btn.setEnabled(False)
                self._cancel_btn.setEnabled(False)