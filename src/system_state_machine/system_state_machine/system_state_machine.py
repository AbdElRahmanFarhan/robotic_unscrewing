#!/usr/bin/env python3
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
        super().__init__(outcomes=["initialization_done"])

    def execute(self, blackboard: Blackboard) -> str:
        yasmin.YASMIN_LOG_INFO("Initialize")
        blackboard["current_tool"] = "screwdriver_bit"  # TODO:
        blackboard["n_detected_screws"] = 0
        blackboard["n_unscrew_successful"] = 0
        blackboard["n_remove_successful"] = 0
        return "initialization_done"


class DetectScrews(State):
    def __init__(self) -> None:
        super().__init__(outcomes=["screws_detected", "nothing_detected"])

    def execute(self, blackboard: Blackboard) -> str:

        yasmin.YASMIN_LOG_INFO("Executing state DetectScrews")
        time.sleep(3)
        n_detected_screws = 10  # TODO: get the detected screws
        if n_detected_screws < 0:
            return "nothing_detected"
        else:
            blackboard["n_detected_screws"] = n_detected_screws
            return "screws_detected"


class Unscrew(State):
    TOOL = "screwdriver_bit"

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

    def execute(self, blackboard: Blackboard) -> str:

        yasmin.YASMIN_LOG_INFO("Executing state Unscrew")
        if blackboard["current_tool"] != Unscrew.TOOL:
            return "attach_screwdriver_bit"

        if self.n_unscrew_successful == blackboard["n_detected_screws"]:
            yasmin.YASMIN_LOG_INFO("All Screws Unscrewed")
            return "unscrew_successful"
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
                return "unscrew_in_progress"
            else:
                yasmin.YASMIN_LOG_INFO(
                    f"FAILED: Unscrew Screw {self.n_unscrew_successful+1}/{blackboard["n_detected_screws"]}"
                )
                blackboard["n_unscrew_successful"] = self.n_unscrew_successful
                return "unscrew_failed"


class RemoveScrews(State):
    TOOL = "vacuum"

    def __init__(self) -> None:
        super().__init__(
            outcomes=[
                "remove_in_progress",
                "remove_successful",
                "remove_failed",
                "attach_vacuum",
            ]
        )
        self.n_remove_successful = 0

    def execute(self, blackboard: Blackboard) -> str:

        yasmin.YASMIN_LOG_INFO("Executing state RemoveScrews")

        if blackboard["current_tool"] != RemoveScrews.TOOL:
            return "attach_vacuum"

        if self.n_remove_successful == blackboard["n_unscrew_successful"]:
            yasmin.YASMIN_LOG_INFO("All Screws Removed")
            return "remove_successful"
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
                return "remove_in_progress"
            else:
                yasmin.YASMIN_LOG_INFO(
                    f"FAILED: Remove Screw {self.n_remove_successful+1}/{blackboard["n_unscrew_successful"]}"
                )
                blackboard["n_remove_successful"] = self.n_remove_successful
                return "remove_failed"


class SystemFailure(State):

    def __init__(self) -> None:
        super().__init__(outcomes=["failed"])

    def execute(self, blackboard: Blackboard) -> str:
        yasmin.YASMIN_LOG_INFO("System Failed")
        time.sleep(3)
        return "failed"


class ToolChange(State):
    def __init__(self) -> None:
        super().__init__(outcomes=["screwdriver_bit_attached", "vacuum_attached"])

    def execute(self, blackboard: Blackboard) -> str:
        yasmin.YASMIN_LOG_INFO("System Failed")

        if blackboard["current_tool"] == RemoveScrews.TOOL:
            time.sleep(3)  # TODO: tool changing
            blackboard["current_tool"] = Unscrew.TOOL
            return "screwdriver_bit_attached"
        else:
            time.sleep(3)  # TODO: tool changing
            blackboard["current_tool"] = RemoveScrews.TOOL
            return "vacuum_attached"


def main():

    yasmin.YASMIN_LOG_INFO("system state machine")
    rclpy.init()
    set_ros_loggers()

    sm = StateMachine(outcomes=["FAILED", "SUCCESSED"])

    sm.add_state(
        "Initialize",
        Initialize(),
        transitions={
            "initialization_done": "DetectScrews",
        },
    )

    sm.add_state(
        "DetectScrews",
        DetectScrews(),
        transitions={
            "screws_detected": "Unscrew",
            "nothing_detected": "SystemFailure",
        },
    )
    sm.add_state(
        "Unscrew",
        Unscrew(),
        transitions={
            "unscrew_in_progress": "Unscrew",
            "unscrew_successful": "RemoveScrews",
            "unscrew_failed": "SystemFailure",
            "attach_screwdriver_bit": "ToolChange",
        },
    )
    sm.add_state(
        "RemoveScrews",
        RemoveScrews(),
        transitions={
            "remove_in_progress": "RemoveScrews",
            "remove_successful": "SUCCESSED",
            "remove_failed": "SystemFailure",
            "attach_vacuum": "ToolChange",
        },
    )
    sm.add_state(
        "SystemFailure",
        SystemFailure(),
        transitions={
            "failed": "FAILED",
        },
    )
    sm.add_state(
        "ToolChange",
        ToolChange(),
        transitions={
            "screwdriver_bit_attached": "Unscrew",
            "vacuum_attached": "RemoveScrews",
        },
    )
    sm.set_start_state("Initialize")
    YasminViewerPub("system_state_machine", sm)

    try:
        outcome = sm()
        yasmin.YASMIN_LOG_INFO(outcome)
    except KeyboardInterrupt:
        if sm.is_running():
            sm.cancel_state()

    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
