#!/usr/bin/env python
import rospy
import json
from std_msgs.msg import String
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest

# 狀態機
STATE_IDLE = 0          # 尚未收到 target_points_json
STATE_GOT_JSON = 1      # 收到 target_points_json, 等待 plan
STATE_PLANNED = 2       # 已收到 plan, 等待 execute

state = STATE_IDLE
latest_joints_reply = None
reset_flag = False      # stop 訊號

def robot_signal_callback(msg: String):
    global state, reset_flag

    # 1. 尝试解析 JSON，获取 command 字段
    raw = msg.data.strip()
    try:
        payload = json.loads(raw)
        signal = str(payload.get('command', '')).strip().lower()
    except (json.JSONDecodeError, TypeError):
        # 如果不是合法 JSON，则当作简单字符串处理
        signal = raw.lower()

    rospy.loginfo("Received robot_signal: %s", signal)

    # 2. 处理 stop
    if signal == 'stop':
        reset_all()
        rospy.logwarn("流程收到 stop, 全部重置")
        return

    # 3. 根据当前状态，处理 plan/execute
    if state == STATE_IDLE:
        if signal in ('plan', 'execute'):
            rospy.logwarn("尚未收到 /target_points_json, 忽略指令 %s", signal)
        return

    if state == STATE_GOT_JSON:
        if signal == 'plan':
            publish_tmp_result()
            state = STATE_PLANNED
            rospy.loginfo("狀態變更: 已收到 plan, 已發佈 joints_tmp, 等待 execute")
        elif signal == 'execute':
            rospy.logwarn("還沒收到 plan 就收到 execute, 忽略")
        return

    if state == STATE_PLANNED:
        if signal == 'execute':
            publish_final_result()
            reset_all()
        elif signal == 'plan':
            rospy.logwarn("已經在等待 execute, 再收到 plan 忽略")


def compute_ik_and_get_joints(p, group='manipulator', frame_id='base_link', ik_link='tcp_link'):
    try:
        rospy.loginfo("Waiting for /compute_ik service…")
        rospy.wait_for_service('/compute_ik', timeout=5.0)
    except rospy.ROSException:
        rospy.logerr("/compute_ik service not available after timeout")
        return None

    ik_srv = rospy.ServiceProxy('/compute_ik', GetPositionIK)
    req = GetPositionIKRequest()
    req.ik_request.group_name = group
    # 這裡會使用傳入的 ik_link ('tcp_link')
    req.ik_request.ik_link_name = ik_link 
    req.ik_request.pose_stamped.header.frame_id = frame_id
    
    # Position
    req.ik_request.pose_stamped.pose.position.x = p['position']['x']
    req.ik_request.pose_stamped.pose.position.y = p['position']['y']
    req.ik_request.pose_stamped.pose.position.z = p['position']['z']
    
    # Orientation (Quaternion from Unity)
    req.ik_request.pose_stamped.pose.orientation.x = p['orientation']['x']
    req.ik_request.pose_stamped.pose.orientation.y = p['orientation']['y']
    req.ik_request.pose_stamped.pose.orientation.z = p['orientation']['z']
    req.ik_request.pose_stamped.pose.orientation.w = p['orientation']['w']
    
    req.ik_request.timeout = rospy.Duration(1.0)

    resp = ik_srv(req)
    if resp.error_code.val == 1:
        return resp.solution.joint_state
    else:
        return None

def json_callback(msg: String):
    global state, latest_joints_reply
    if state != STATE_IDLE:
        rospy.logwarn("流程尚未完成，忽略新收到的 /target_points_json")
        return

    try:
        data = json.loads(msg.data)
        pts = data.get('points', [])
    except json.JSONDecodeError as e:
        rospy.logerr("Cannot decode points JSON: %s", e)
        return

    rospy.loginfo("收到 %d 筆目標點（含姿態），開始 IK 計算…", len(pts))
    joints_list = []
    for i, p in enumerate(pts):
        # 驗證 pose 格式
        if 'position' not in p or 'orientation' not in p:
            rospy.logerr("Point %d 缺少 position 或 orientation: %s", i, p)
            json_pub_tmp.publish(json.dumps({'success': False, 'failed_index': i, 'error': 'missing_pose_data'}))
            reset_all()
            return
        
        js = compute_ik_and_get_joints(p)
        if js is None:
            rospy.logwarn("Point %d IK failed: %s", i, p)
            json_pub_tmp.publish(json.dumps({'success': False, 'failed_index': i, 'error': 'ik_failed'}))
            reset_all()
            return
        joints_list.append({'name': js.name, 'position': js.position})

    reply = {'success': True, 'joints': joints_list}
    latest_joints_reply = json.dumps(reply)
    rospy.loginfo("IK 已算完，等待 plan 指令 publish joints_tmp")
    state = STATE_GOT_JSON

def publish_tmp_result():
    global latest_joints_reply
    if latest_joints_reply:
        json_pub_tmp.publish(latest_joints_reply)
        rospy.loginfo("已發佈 /target_points_joints_json_tmp")
    else:
        rospy.logwarn("尚未有 joints result 可送出 (/target_points_joints_json_tmp)")

def publish_final_result():
    global latest_joints_reply
    if latest_joints_reply:
        json_pub.publish(latest_joints_reply)
        rospy.loginfo("已發佈 /target_points_joints_json")
    else:
        rospy.logwarn("尚未有 joints result 可送出 (/target_points_joints_json)")

def reset_all():
    global state, latest_joints_reply, reset_flag
    state = STATE_IDLE
    latest_joints_reply = None
    reset_flag = False

if __name__ == "__main__":
    rospy.init_node('ik_json_interface')

    # Publisher
    json_pub_tmp = rospy.Publisher('/target_points_joints_json_tmp', String, queue_size=1)
    json_pub = rospy.Publisher('/target_points_joints_json', String, queue_size=1)

    # Subscriber
    rospy.Subscriber('/target_points_json', String, json_callback)
    rospy.Subscriber('/robot_signal', String, robot_signal_callback)

    rospy.loginfo("IK JSON interface node started, waiting for /target_points_json …")
    rospy.spin()
