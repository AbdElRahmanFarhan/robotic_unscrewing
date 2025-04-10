import time
import rclpy

import yasmin
from yasmin import State
from yasmin import Blackboard
from yasmin import StateMachine
from yasmin_ros import set_ros_loggers
from yasmin_viewer import YasminViewerPub

class RemoveScrews(StateMachine):
    def __init__(self) -> None:
        super().__init__(
            outcomes=[
                "remove_in_progress",
                "remove_successful",
                "remove_failed",
                "attach_vacuum",
            ]
        )
        self.add_state(
            "RemoveScrewsInit",
            Initialize(),
            transitions={
                "1": "remove_in_progress",
                "2": "remove_successful",
                "3": "remove_failed",
                "4": "attach_vacuum",
            },
        )
        self.set_start_state("RemoveScrewsInit")
        
class Initialize(State):

    def __init__(self) -> None:
        super().__init__(
            outcomes=[
                "1",
                "2",
                "3",
                "4",
            ]
        )
        self.n_remove_successful = 0

    def execute(self, blackboard: Blackboard) -> str:

        yasmin.YASMIN_LOG_INFO("Executing state RemoveScrews")
        
        if blackboard["current_tool"] != blackboard["screwdriver_tool"]:
            return "4"

        if self.n_remove_successful == blackboard["n_unscrew_successful"]:
            yasmin.YASMIN_LOG_INFO("All Screws Removed")
            return "2"
        else:
            yasmin.YASMIN_LOG_INFO(
                f"Attempting To Remove Screw {self.n_remove_successful+1}/{blackboard["n_unscrew_successful"]}"
            )
            time.sleep(3)
            remove_successful = True  # TODO: pick and place process
            if remove_successful:
                yasmin.YASMIN_LOG_INFO(
                    f"SUCCESS: Remove Screw {self.n_remove_successful+1}/{blackboard["n_unscrew_successful"]}"
                )
                self.n_remove_successful += 1
                blackboard["n_remove_successful"] = self.n_remove_successful
                return "1"
            else:
                yasmin.YASMIN_LOG_INFO(
                    f"FAILED: Remove Screw {self.n_remove_successful+1}/{blackboard["n_unscrew_successful"]}"
                )
                blackboard["n_remove_successful"] = self.n_remove_successful
                return "3"



