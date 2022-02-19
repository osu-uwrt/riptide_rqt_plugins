from riptide_msgs2.action import ExecuteTask

from .action_widget import ActionWidget

class TaskActionWidget(ActionWidget):

    def __init__(self, node, action_name, namespace, action_topic, behaviortree_file, actions_layout):
        super(TaskActionWidget, self).__init__(node, action_name, namespace, action_topic, ExecuteTask, actions_layout, True)

        self._behaviortree_file = behaviortree_file

        self._results_btn.setText("Last Console")

    ########################################
    # Function Overrides
    ########################################

    def _generate_goal(self):
        action_goal = ExecuteTask.Goal()
        action_goal.behaviortree_file = self._behaviortree_file
        return action_goal

    def _generate_results_text(self):
        return self.last_result.output