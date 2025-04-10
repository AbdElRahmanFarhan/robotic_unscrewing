from geometry_msgs.msg import Pose
from camera_msgs.srv import DetectScrews
import json

def json_pose_to_pose_msg(json_pose):
    pose_msg = Pose()
    pose_msg.position.x = json_pose["position"][0]
    pose_msg.position.y = json_pose["position"][1]
    pose_msg.position.z = json_pose["position"][2]
    pose_msg.orientation.x = json_pose["orientation"][0]
    pose_msg.orientation.y = json_pose["orientation"][1]
    pose_msg.orientation.z = json_pose["orientation"][2]
    pose_msg.orientation.w = json_pose["orientation"][3]
    return pose_msg
    
    
def json_data_to_ros(screws_data):
    response = DetectScrews.Response()
    screws_poses = screws_data["screws"]["poses"]
    response.poses = [json_pose_to_pose_msg(screw_pose) for screw_pose in screws_poses]
    response.type = screws_data["screws"]["screw_type"]["drive"]["type"]
    response.size = screws_data["screws"]["screw_type"]["drive"]["size"]
    response.length = screws_data["screws"]["screw_type"]["length"]
    response.turns = screws_data["screws"]["screw_type"]["turns"]
    return response
    
    
def read_screw_poses_from_json(screws_data_file):
        with open(screws_data_file) as file:
            data = json.load(file)
        return data