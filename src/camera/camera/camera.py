import rclpy
from rclpy.node import Node
from camera_msgs.srv import GetDetectedScrews
from camera.utils import read_screw_poses_from_json, json_data_to_ros, json_pose_to_pose_msg
    
class Camera(Node):
    def __init__(self):
        super().__init__('camera')
        self.create_service(GetDetectedScrews, '/camera/detect_screws', self.detect_screws)
        self.declare_parameter('screws_data_file', '')
        self.screws_data_file = self.get_parameter('screws_data_file').value
        self.get_logger().info("Camera node is running!")

    def detect_screws(self, requst, response):
        self.get_logger().info("Camera node received a request for detecting screws!")
        screws_data = read_screw_poses_from_json(self.screws_data_file)
        response = json_data_to_ros(screws_data)        
        return response
    
def main(args=None):
    rclpy.init(args=args)
    camera = Camera()
    rclpy.spin(camera)
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()        