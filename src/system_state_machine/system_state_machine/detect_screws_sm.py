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

        yasmin.YASMIN_LOG_INFO("Executing state DetectScrews")
        time.sleep(3)
        n_detected_screws = 10  # TODO: get the detected screws
        if n_detected_screws < 0:
            return "2"
        else:
            blackboard["n_detected_screws"] = n_detected_screws
            return "1" 

class DetectScrews(StateMachine):
    def __init__(self) -> None:
        super().__init__(outcomes=["screws_detected", "nothing_detected"])
        self.add_state(
        "DetectScrewsInit",
        Initialize(),
        transitions={
            "1": "screws_detected",
            "2": "nothing_detected",
        },
        )
        self.set_start_state("DetectScrewsInit")
    
        

