#!/usr/bin/env python3

import time
import rclpy
from rclpy.node import Node
from camera_msgs.srv import GetDetectedScrews
from weber_msgs.action import WeberUnscrew
from weber_msgs.msg import WeberUnscrewStatus
from weber_msgs.srv import AttachTool
from rclpy.action import ActionServer
from std_srvs.srv import Trigger, SetBool

class WeberScrewDriver(Node):
    UNSCREWING_PROCESS = [
        WeberUnscrewStatus.STARTED,
        WeberUnscrewStatus.ENGAGING,
        WeberUnscrewStatus.ENGAGED,
        WeberUnscrewStatus.UNFASTENING,
        WeberUnscrewStatus.UNFASTENED,
        WeberUnscrewStatus.RETREATING,
        WeberUnscrewStatus.RETREATED,
        WeberUnscrewStatus.SUCCEEDED,
    ]

    def __init__(self, node_name):
        super().__init__(node_name)
        self.attached_tool = "screwdriver"
        self.create_service(
            AttachTool, "/weber_screwdriver/attach_tool", self.attach_tool_cb
        )
        self.create_service(
            Trigger, "/weber_screwdriver/detach_tool", self.detach_tool_cb
        )
        self.create_service(
            Trigger, "/weber_screwdriver/get_attached_tool", self.get_attached_tool_cb
        )
        self.create_service(
            SetBool, "/weber_screwdriver/set_vacuum_on", self.vacuum_cb
        )

        ActionServer(
            self, WeberUnscrew, "/weber_screwdriver/unscrew", self.unscrew_cb
        )

    def attach_tool_cb(self, request, response):
        if self.attached_tool == "":
            self.attached_tool = request.tool
            response.success = True
        else:
            response.success = False
        return response
    
    def detach_tool_cb(self, request, response):
        if self.attached_tool != "":
            self.attached_tool = ""
            response.success = True
        else:
            response.success = False
        return response
    
    def get_attached_tool_cb(self, request, response):
        response.message = self.attached_tool
        return response
    
    def vacuum_cb(self, request, response):
        if request.data:
            self.get_logger().info("Setting vacuum on!")
        else:
            self.get_logger().info("Setting vacuum off!")
        return response
            
    def unscrew_cb(self, goal_handle):

        feedback_msg = WeberUnscrew.Feedback()

        for status in WeberScrewDriver.UNSCREWING_PROCESS:
            self.get_logger().info(f"Unscrweing in progress {status}/{len(WeberScrewDriver.UNSCREWING_PROCESS)}")
            if status == WeberUnscrewStatus.ABORTED:
                break
            else:
                feedback_msg.status.val = status
                goal_handle.publish_feedback(feedback_msg)
                time.sleep(1)
        
        goal_handle.succeed()
        result = WeberUnscrew.Result()
        result.final_status.val = WeberUnscrewStatus.SUCCEEDED
        return result


def main(args=None):
    rclpy.init(args=args)
    node = WeberScrewDriver("weber_screwdriver")
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
