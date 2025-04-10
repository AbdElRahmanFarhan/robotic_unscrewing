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
        super().__init__(
            outcomes=[
                "1",
                "2",
                "3",
                "4",
            ]
        )
        self.n_unscrew_successful = 0

    def execute(self, blackboard: Blackboard) -> str:

        yasmin.YASMIN_LOG_INFO("Executing state Unscrew")
        if blackboard["current_tool"] != blackboard["screwdriver_tool"]:
            return "4"

        if self.n_unscrew_successful == blackboard["n_detected_screws"]:
            yasmin.YASMIN_LOG_INFO("All Screws Unscrewed")
            return "2"
        else:
            yasmin.YASMIN_LOG_INFO(
                f"Attempting To Unscrew Screw {self.n_unscrew_successful+1}/{blackboard["n_detected_screws"]}"
            )
            time.sleep(3)
            unscrew_successful = True  # TODO: unscrew process
            if unscrew_successful:
                yasmin.YASMIN_LOG_INFO(
                    f"SUCCESS: Unscrew Screw {self.n_unscrew_successful+1}/{blackboard["n_detected_screws"]}"
                )
                self.n_unscrew_successful += 1
                blackboard["n_unscrew_successful"] = self.n_unscrew_successful
                return "1"
            else:
                yasmin.YASMIN_LOG_INFO(
                    f"FAILED: Unscrew Screw {self.n_unscrew_successful+1}/{blackboard["n_detected_screws"]}"
                )
                blackboard["n_unscrew_successful"] = self.n_unscrew_successful
                return "3"
            

class Unscrew(StateMachine):
    def __init__(self) -> None:
        super().__init__(
            outcomes=[
                "unscrew_in_progress",
                "unscrew_successful",
                "unscrew_failed",
                "attach_screwdriver_bit",
            ]
        )
        self.n_unscrew_successful = 0
        self.add_state(
            "UnscrewInit",
            Initialize(),
            transitions={
                "1": "unscrew_in_progress",
                "2": "unscrew_successful",
                "3": "unscrew_failed",
                "4": "attach_screwdriver_bit",
            },
        )
        self.set_start_state("UnscrewInit")