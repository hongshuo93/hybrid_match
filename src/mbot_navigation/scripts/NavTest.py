import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from visualization_msgs.msg import Marker, MarkerArray
from actionlib_msgs.msg import GoalStatus
import tf
import math

def get_robot_position(tf_listener):
    try:
        (trans, rot) = tf_listener.lookupTransform('map', 'base_link', rospy.Time(0))
        return trans  # 返回(x, y, z)
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        rospy.logwarn("无法获取机器人的当前位置")
        return None

def publish_all_markers(points):
    marker_array = MarkerArray()
    for idx, point in enumerate(points):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "points"
        marker.id = idx
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = point[0]
        marker.pose.position.y = point[1]
        marker.pose.position.z = 0
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker_array.markers.append(marker)

    # 发布MarkerArray
    marker_pub = rospy.Publisher('visualization_marker_array', MarkerArray, queue_size=10)
    # 等待发布者连接
    while marker_pub.get_num_connections() < 1:
        rospy.sleep(0.1)
    marker_pub.publish(marker_array)

def send_goals(points, distance_threshold):
    # 创建动作客户端
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()
    # 创建tf监听器
    listener = tf.TransformListener()

    for idx, point in enumerate(points):
        rospy.sleep(0.5)  # 等待tf缓冲

        # 获取机器人的当前位置
        robot_pos = get_robot_position(listener)
        if robot_pos is None:
            rospy.logwarn("无法获取机器人位置，跳过距离检查")
        else:
            # 计算距离
            dx = point[0] - robot_pos[0]
            dy = point[1] - robot_pos[1]
            distance = math.sqrt(dx*dx + dy*dy)
            rospy.loginfo("机器人到目标点的距离: {:.2f} 米".format(distance))
            if distance < distance_threshold:
                rospy.loginfo("距离小于阈值 {:.2f} 米，跳过该目标点".format(distance_threshold))
                continue  # 跳过当前点，继续下一个

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = point[0]
        goal.target_pose.pose.position.y = point[1]
        goal.target_pose.pose.orientation.w = 1.0

        rospy.loginfo("发送目标点: {}".format(point))
        client.send_goal(goal)
        wait = client.wait_for_result(rospy.Duration(30.0))
        if not wait:
            rospy.logerr("动作服务器不可用！")
            rospy.signal_shutdown("动作服务器不可用！")
        else:
            state = client.get_state()
            if state == GoalStatus.SUCCEEDED:
                rospy.loginfo("成功到达目标点 {}".format(point))
            else:
                rospy.logwarn("无法到达目标点 {}，跳过".format(point))
                continue  # 跳过当前点，继续下一个

if __name__ == '__main__':
    try:
        rospy.init_node('navigate_points')
        # 定义距离阈值，单位：米
        distance_threshold = 0.5  # 例如0.5米
        # 定义坐标点列表
        points = [
            [1.0, -1.0],
            [-3.0, -1.0],

            [1.0, 1.0],
            [-3.0, 1.0],
            # 在这里添加您的坐标点
        ]
        # 一次性发布所有Marker
        publish_all_markers(points)
        # 开始导航
        send_goals(points, distance_threshold)
    except rospy.ROSInterruptException:
        pass