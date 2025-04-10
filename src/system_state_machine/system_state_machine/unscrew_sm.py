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
from weber_msgs.action import WeberUnscrew
from weber_msgs.msg import WeberUnscrewStatus
from copy import deepcopy

class Initialize(State):
    def __init__(self) -> None:
        super().__init__(
            outcomes=[
                "change_tool",
                "start",
            ]
        )

    def execute(self, blackboard: Blackboard) -> str:
        if blackboard["current_tool"] != blackboard["screwdriver_tool"]:
            return "change_tool"
        else:
            return "start"

# class Initialize(ServiceState):
#     def __init__(self):
#         super().__init__(
#             GetDetectedScrews,
#             "/camera/detect_screws",
#             self.create_request_handler,
#             ["change_tool", "start"],
#             self.response_handler,
#         )

#     def create_request_handler(
#         self, blackboard: Blackboard
#     ) -> GetDetectedScrews.Request:
#         req = GetDetectedScrews.Request()
#         return req

#     def response_handler(
#         self, blackboard: Blackboard, response: GetDetectedScrews.Response
#     ) -> str:
#         if len(response.poses) > 0:
#             blackboard["n_screws_detected"] = len(response.poses)
#             blackboard["n_unscrew_successful"] = 0
#             blackboard["screws"] = response
#             return "start"



class GoToScrew(ActionState):
    def __init__(self):
        super().__init__(
            SetGoalRobot,
            "/lion_control/go_to_goal",
            self.create_goal_handler,
            ["success", "failed"],
            self.response_handler,
            None,
        )

    def create_goal_handler(self, blackboard: Blackboard) -> SetGoalRobot.Goal:
        goal = SetGoalRobot.Goal()
        goal.end_effector = "screwdriver_tcp"
        goal.goal_type.val = GoalType.POSE
        screw_nr = blackboard["n_unscrew_successful"]
        screws = deepcopy(blackboard["screws"])
        goal.pose_goal.pose = screws.poses[screw_nr]
        z = goal.pose_goal.pose.position.z
        z += 0.2
        goal.pose_goal.pose.position.z = z
        goal.pose_goal.header.frame_id = screws.header.frame_id
        goal.pose_goal.pose.orientation.x = 0.
        goal.pose_goal.pose.orientation.y = 0.
        goal.pose_goal.pose.orientation.z = 0.
        goal.pose_goal.pose.orientation.w = 1.
        return goal

    def response_handler(
        self, blackboard: Blackboard, response: SetGoalRobot.Result
    ) -> str:
        if response.success:
            return "success"
        else:
            return "failed"

class ApproachScrew(ActionState):
    def __init__(self):
        super().__init__(
            SetGoalRobot,
            "/lion_control/go_to_goal",
            self.create_goal_handler,
            ["success", "failed"],
            self.response_handler,
            None,
        )

    def create_goal_handler(self, blackboard: Blackboard) -> SetGoalRobot.Goal:
        goal = SetGoalRobot.Goal()
        goal.end_effector = "screwdriver_tcp"
        goal.goal_type.val = GoalType.STRAIGHT
        screw_nr = blackboard["n_unscrew_successful"]
        screws = deepcopy(blackboard["screws"])
        goal.pose_goal.pose = screws.poses[screw_nr]
        z = goal.pose_goal.pose.position.z
        z += 0.05
        goal.pose_goal.pose.position.z = z
        goal.pose_goal.header.frame_id = screws.header.frame_id
        goal.pose_goal.pose.orientation.x = 0.
        goal.pose_goal.pose.orientation.y = 0.
        goal.pose_goal.pose.orientation.z = 0.
        goal.pose_goal.pose.orientation.w = 1.
        return goal

    def response_handler(
        self, blackboard: Blackboard, response: SetGoalRobot.Result
    ) -> str:
        if response.success:
            return "success"
        else:
            return "failed"

class StartWeberUnscrew(ActionState):
    def __init__(self):
        super().__init__(
            WeberUnscrew,
            "/weber_screwdriver/unscrew",
            self.create_goal_handler,
            ["success", "failed"],
            self.response_handler,
            None,
        )

    def create_goal_handler(self, blackboard: Blackboard) -> WeberUnscrew.Goal:
        goal = WeberUnscrew.Goal()
        goal.force = 0.1
        goal.torque = 0.2
        goal.stroke_dist = 0.05
        goal.retreat_dist = blackboard["screws"].length
        return goal

    def response_handler(
        self, blackboard: Blackboard, response: WeberUnscrew.Result
    ) -> str:
        if response.final_status.val == WeberUnscrewStatus.SUCCEEDED:
            return "success"
        else:
            return "failed"

class Retreat(ActionState):
    def __init__(self):
        super().__init__(
            SetGoalRobot,
            "/lion_control/go_to_goal",
            self.create_goal_handler,
            ["success", "failed"],
            self.response_handler,
            None,
        )

    def create_goal_handler(self, blackboard: Blackboard) -> SetGoalRobot.Goal:
        goal = SetGoalRobot.Goal()
        goal.end_effector = "screwdriver_tcp"
        goal.goal_type.val = GoalType.STRAIGHT
        screw_nr = blackboard["n_unscrew_successful"]
        screws = deepcopy(blackboard["screws"])
        goal.pose_goal.pose = screws.poses[screw_nr]
        z = goal.pose_goal.pose.position.z
        z += 0.2
        
        goal.pose_goal.pose.position.z = z
        goal.pose_goal.header.frame_id = screws.header.frame_id
        goal.pose_goal.pose.orientation.x = 0.
        goal.pose_goal.pose.orientation.y = 0.
        goal.pose_goal.pose.orientation.z = 0.
        goal.pose_goal.pose.orientation.w = 1.
        return goal

    def response_handler(
        self, blackboard: Blackboard, response: SetGoalRobot.Result
    ) -> str:
        if response.success:
            return "success"
        else:
            return "failed"

        

class Update(State):
    def __init__(self) -> None:
        super().__init__(
            outcomes=[
                "all",
                "still",
            ]
        )

    def execute(self, blackboard: Blackboard) -> str:
        n_unscrew_successful = blackboard["n_unscrew_successful"]
        n_unscrew_successful += 1
        if n_unscrew_successful == blackboard["n_screws_detected"]:
            yasmin.YASMIN_LOG_INFO("All Screws Unscrewed")
            return "all"
        else:
            blackboard["n_unscrew_successful"] = n_unscrew_successful
            return "still"

class Unscrew(StateMachine):
    def __init__(self) -> None:
        super().__init__(
            outcomes=[
                "unscrew_in_progress",
                "unscrew_successful",
                "unscrew_failed",
                "go_to_tool_changer",
            ]
        )
        self.add_state(
            "UnscrewInit",
            Initialize(),
            transitions={
                "change_tool": "go_to_tool_changer",
                "start": "GoToScrew",
            },
        )
        self.add_state(
            "GoToScrew",
            GoToScrew(),
            transitions={
                "success": "ApproachScrew",
                "failed": "unscrew_failed",
            },
        )
        self.add_state(
            "ApproachScrew",
            ApproachScrew(),
            transitions={
                "success": "StartWeberUnscrew",
                "failed": "unscrew_failed",
            },
        )
        self.add_state(
            "StartWeberUnscrew",
            StartWeberUnscrew(),
            transitions={
                "success": "Retreat",
                "failed": "unscrew_failed",
            },
        )
        self.add_state(
            "Retreat",
            Retreat(),
            transitions={
                "success": "Update",
                "failed": "unscrew_failed",
            },
        )
        self.add_state(
            "Update",
            Update(),
            transitions={
                "all": "unscrew_successful",
                "still": "unscrew_in_progress",
            },
        )
        self.set_start_state("UnscrewInit")

def main():

    yasmin.YASMIN_LOG_INFO("unscrew state machine")
    rclpy.init()
    set_ros_loggers()
    sm = Unscrew()
    YasminViewerPub("unscrewstate machine", sm)

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
