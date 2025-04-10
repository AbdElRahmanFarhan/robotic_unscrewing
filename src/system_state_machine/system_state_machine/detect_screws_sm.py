import time
import rclpy

import yasmin
from yasmin import State
from yasmin import Blackboard
from yasmin import StateMachine
from yasmin_ros import ActionState
from yasmin_ros import ServiceState
from yasmin_ros import set_ros_loggers
from lion_msgs.action import SetGoalRobot
from lion_msgs.msg import GoalType
from camera_msgs.srv import GetDetectedScrews
from yasmin_viewer import YasminViewerPub


class Initialize(State):
    def __init__(self) -> None:
        super().__init__(outcomes=["initialization_done"])

    def execute(self, blackboard: Blackboard) -> str:
        time.sleep(3)
        return "initialization_done"


class RobotAtCapturePose(ActionState):
    def __init__(self):
        super().__init__(
            SetGoalRobot,
            "/lion_control/go_to_goal",
            self.create_goal_handler,
            ["robot_at_capture_pose", "robot_failed"],
            self.response_handler,
            None,
        )

    def create_goal_handler(self, blackboard: Blackboard) -> SetGoalRobot.Goal:
        goal = SetGoalRobot.Goal()
        goal.end_effector = "screwdriver_bit"  # TODO: add camera to urdf and change the end effector here
        goal.goal_type.val = GoalType.NAMED
        goal.named_goal = "capture"  # according to srdf
        return goal

    def response_handler(
        self, blackboard: Blackboard, response: SetGoalRobot.Result
    ) -> str:
        if response.success:
            return "robot_at_capture_pose"
        else:
            return "robot_failed"


class CameraDetect(ServiceState):
    def __init__(self):
        super().__init__(
            GetDetectedScrews,
            "/camera/detect_screws",
            self.create_request_handler,
            ["detected", "detection_failed"],
            self.response_handler,
        )

    def create_request_handler(
        self, blackboard: Blackboard
    ) -> GetDetectedScrews.Request:
        req = GetDetectedScrews.Request()
        return req

    def response_handler(
        self, blackboard: Blackboard, response: GetDetectedScrews.Response
    ) -> str:
        if len(response.poses) > 0:
            blackboard["n_screws_detected"] = len(response.poses)
            blackboard["screws_poses"] = response.poses
            return "detected"
        else:
            return "detection_failed"


class DetectScrews(StateMachine):
    def __init__(self) -> None:
        super().__init__(outcomes=["screws_detected", "nothing_detected"])
        self.add_state(
            "DetectScrewsInit",
            Initialize(),
            transitions={
                "initialization_done": "RobotAtCapturePose",
            },
        )
        self.add_state(
            "RobotAtCapturePose",
            RobotAtCapturePose(),
            transitions={
                "robot_at_capture_pose": "CameraDetect",
                "robot_failed": "nothing_detected",
            },
        )
        self.add_state(
            "CameraDetect",
            CameraDetect(),
            transitions={
                "detected": "screws_detected",
                "detection_failed": "nothing_detected",
            },
        )
        self.set_start_state("DetectScrewsInit")


def main():

    yasmin.YASMIN_LOG_INFO("detect screws state machine")
    rclpy.init()
    set_ros_loggers()
    sm = DetectScrews()
    YasminViewerPub("detect screws state machine", sm)

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
