import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty, Empty_Response, Empty_Request
import json

class Camera(Node):
    def __init__(self):
        super().__init__('camera')
        self.create_service(Empty, 'detect_screws_service', self.detect_screws)
        self.declare_parameter('screw_poses_file', '')
        self.screw_poses_file = self.get_parameter('screw_poses_file').value
        self.get_logger().info("Hello from the camera node!")

    def detect_screws(self, requst, response):
        screw_poses = self.read_screw_poses_from_json(self.screw_poses_file)
        response = Empty_Response()
        return response    
    
    def read_screw_poses_from_json(self, screw_poses_file):
        with open(screw_poses_file) as file:
            data = json.load(file)
        return data
    
def main(args=None):
    rclpy.init(args=args)
    camera = Camera()
    rclpy.spin(camera)
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()        