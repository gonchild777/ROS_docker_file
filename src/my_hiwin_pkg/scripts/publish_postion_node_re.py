#!/usr/bin/env python
# For auto-publish position data as JSON to /target_points_json, simulate meta points
import rospy
import json
from std_msgs.msg import String

if __name__ == "__main__":
    rospy.init_node('my_pointarray_json_publisher')
    json_pub = rospy.Publisher('/target_points_json', String, queue_size=10)
    rospy.sleep(1.0)  # 等待 publisher 註冊完成

    # 範例：三個完整 pose（position + orientation）
    poses = [
        {
            "position": {"x": -0.4992596209049225, "y": 0.8769174218177795, "z": 0.7969591021537781},
            "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
        },
        {
            "position": {"x": -0.4992596209049225, "y": 0.8769174218177795, "z": 0.7969591021537781},
            "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
        },
        {
            "position": {"x": -0.07023268938064575, "y": 0.8089713454246521, "z": 0.9886242151260376},
            "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
        }
    ]

    # 組成 JSON 結構
    data = {"points": poses}

    # 序列化成字串並發布
    json_str = json.dumps(data)
    msg = String(data=json_str)
    json_pub.publish(msg)
    rospy.loginfo("已送出 %d 個 pose（含姿態，JSON）", len(poses))

    rospy.sleep(0.5)  # 稍等確保訊息送出
