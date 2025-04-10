import time
import rclpy

import yasmin
from yasmin import State
from yasmin import Blackboard
from yasmin import StateMachine
from yasmin_ros import set_ros_loggers
from yasmin_viewer import YasminViewerPub

class Initialize(State):
    def __init__(self) -> None:
        super().__init__(outcomes=["1", "2"])

    def execute(self, blackboard: Blackboard) -> str:
        yasmin.YASMIN_LOG_INFO("Tool Change")

        if blackboard["current_tool"] == blackboard["vacuum_tool"]:
            time.sleep(3)  # TODO: tool changing
            blackboard["current_tool"] = blackboard["screwdriver_tool"]
            return "1"
        else:
            time.sleep(3)  # TODO: tool changing
            blackboard["current_tool"] = blackboard["vacuum_tool"]
            return "2"
        
        
class ToolChange(StateMachine):
    def __init__(self) -> None:
        super().__init__(outcomes=["screwdriver_tcp_attached", "vacuum_attached"])
        self.add_state(
            "ToolChangeInit",
            Initialize(),
            transitions={
                "1": "screwdriver_tcp_attached",
                "2": "vacuum_attached",
            },
        )
        self.set_start_state("ToolChangeInit")
    