#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

// Action类型别名
typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> TrajClient;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ur5e_move_client");
    ros::NodeHandle nh;

    // 连接eff_joint_traj_controller的action server
    TrajClient client("/eff_joint_traj_controller/follow_joint_trajectory", true);
    ROS_INFO("等待轨迹控制器Action服务器上线...");
    client.waitForServer();
    ROS_INFO("Action服务器已连接，可以发送轨迹！");

    // ===== 构造轨迹目标 =====
    control_msgs::FollowJointTrajectoryGoal goal;

    // UR5e关节名称，顺序不能错！
    goal.trajectory.joint_names.push_back("shoulder_pan_joint");
    goal.trajectory.joint_names.push_back("shoulder_lift_joint");
    goal.trajectory.joint_names.push_back("elbow_joint");
    goal.trajectory.joint_names.push_back("wrist_1_joint");
    goal.trajectory.joint_names.push_back("wrist_2_joint");
    goal.trajectory.joint_names.push_back("wrist_3_joint");

    // 轨迹点1：当前姿态保持一段时间（可选）
    trajectory_msgs::JointTrajectoryPoint p1;
    p1.positions = {0.0, -1.57, 0.0, -1.57, 0.0, 0.0};
    p1.time_from_start = ros::Duration(1.0);
    goal.trajectory.points.push_back(p1);

    // 轨迹点2：目标位置（弧度！）
    trajectory_msgs::JointTrajectoryPoint p2;
    p2.positions = {0.5, -1.0, 0.8, -1.2, 0.3, 0.0};
    p2.time_from_start = ros::Duration(4.0);  // 4秒内运动到该点
    goal.trajectory.points.push_back(p2);

    // 轨迹点3：回到另一个姿态
    trajectory_msgs::JointTrajectoryPoint p3;
    p3.positions = {-0.3, -1.2, 0.4, -1.0, 0.2, 0.5};
    p3.time_from_start = ros::Duration(7.0);
    goal.trajectory.points.push_back(p3);

    // 发送目标
    client.sendGoal(goal);
    ROS_INFO("轨迹指令已下发，等待机械臂执行完成");

    // 等待执行完成，最多等待10s
    bool finished_before_timeout = client.waitForResult(ros::Duration(10.0));
    if (finished_before_timeout)
    {
        actionlib::SimpleClientGoalState state = client.getState();
        ROS_INFO("动作执行结果: %s", state.toString().c_str());
    }
    else
    {
        ROS_ERROR("动作超时！");
    }

    ros::shutdown();
    return 0;
}
