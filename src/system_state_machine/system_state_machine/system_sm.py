#!/usr/bin/env python3
import time
import rclpy

import yasmin
from yasmin import State
from yasmin import Blackboard
from yasmin import StateMachine
from yasmin_ros import set_ros_loggers
from yasmin_viewer import YasminViewerPub
from system_state_machine.detect_screws_sm import DetectScrews
from system_state_machine.remove_screws_sm import RemoveScrews
from system_state_machine.tool_change_sm import ToolChange
from system_state_machine.unscrew_sm import Unscrew
from yasmin_ros import ServiceState

class Initialize(State):
    def __init__(self) -> None:
        super().__init__(outcomes=["initialization_done"])

    def execute(self, blackboard: Blackboard) -> str:
        yasmin.YASMIN_LOG_INFO("Initialize")
        blackboard["current_tool"] = "screwdriver_bit"  # TODO:
        blackboard["screwdriver_tool"] = "screwdriver_bit"  # TODO:
        blackboard["vacuum_tool"] = "screwdriver_vacuum"  # TODO:
        blackboard["n_detected_screws"] = 0
        blackboard["n_unscrew_successful"] = 0
        blackboard["n_remove_successful"] = 0
        return "initialization_done"
    
class SystemFailure(State):

    def __init__(self) -> None:
        super().__init__(outcomes=["failed"])

    def execute(self, blackboard: Blackboard) -> str:
        yasmin.YASMIN_LOG_INFO("System Failed")
        time.sleep(3)
        return "failed"

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
