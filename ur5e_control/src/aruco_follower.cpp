/**
 * @brief ArucoFollower 节点
 * @details 该节点用于实现ARUCO二维码跟随功能，将ARUCO二维码的中心位置作为目标位置，使用MoveIt!库实现机械臂的运动控制。
 * @author li.mao
 * @date 2026-10-18
*/




#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/Pose.h>
#include <visualization_msgs/Marker.h>
#include <memory>
#include <cmath>
#include <locale.h>   


//aruco视觉·二维码跟随
class ArucoFollower {
public:
    ArucoFollower()      // 构造函数，初始化tf监听器、move group接口，冒号后面是成员初始化列表
        : tf_listener_(tf_buffer_),
          move_group_("manipulator")
    {
        ros::NodeHandle pnh("~");  //实例化对象

        //调用pnh的param成员函数模板，读取参数


        /*
        *成员函数类模板
        template<typename T>
        T max(T a, T b) 
        {
            return a > b ? a : b;
        }
        max<int>(1, 2); 
        */


        pnh.param<double>("camera_distance", camera_distance_, 0.8); // 期望相机距离
        pnh.param<double>("loop_rate", loop_rate_hz_, 3.0); // 循环频率
        pnh.param<double>("planning_time", planning_time_, 3.0); // 单词规划最长时间


        //读取关键坐标系的名称，以便在tf变换中使用它们
        pnh.param<std::string>("base_frame",    base_frame_,    "base_link");  
        pnh.param<std::string>("marker_frame",  marker_frame_,  "aruco_marker_frame");
        pnh.param<std::string>("tool0_frame",   tool0_frame_,   "tool0");

        //配置move group接口
        move_group_.setEndEffectorLink(tool0_frame_);  //告诉move group接口，要控制的是tool0_frame_坐标系
        move_group_.setPlanningTime(planning_time_);  //给ompl规划器设置规划时间
        move_group_.setNumPlanningAttempts(5);        //设置规划尝试次数
        move_group_.setMaxVelocityScalingFactor(0.25);   //将关节最大速度限制为理论极限的25%,防止机械臂失控撞人
        move_group_.setMaxAccelerationScalingFactor(0.25);  //将关节最大加速度限制为理论极限的25%,防止机械臂失控撞人
        move_group_.setGoalPositionTolerance(0.04);        //允许终点位置有4厘米的误差，降低规划难度
        move_group_.setGoalOrientationTolerance(0.05);    //允许终点姿态有5度的误差，降低规划难度


        //调用pnh的advertise成员函数模板，创建发布目标位置标记的发布者
        marker_pub_ = pnh.advertise<visualization_msgs::Marker>("target_pose_marker", 10);

        ROS_INFO("========================================");
        ROS_INFO("ArucoFollower 节点已启动");
        ROS_INFO("  基座坐标系       = %s", base_frame_.c_str());
        ROS_INFO("  二维码坐标系     = %s", marker_frame_.c_str());
        ROS_INFO("  末端坐标系       = %s", tool0_frame_.c_str());
        ROS_INFO("  期望相机距离     = %.2f 米", camera_distance_);
        ROS_INFO("========================================");
    }


    /**
     * @details 以指定的频率，不断读取二维码位置，计算目标位置，发布目标位置，执行运动。
     */
    void run() {
        ros::Rate rate(loop_rate_hz_);

        while (ros::ok()) {
            try {
                /**
                 *T_base_marker: marker在机器人base_link坐标系下的变换矩阵
                 *T_cam_marker_current: marker相对于相机的位姿
                 *T_tool0_cam: 相机相对tool0的固定安装位姿
                 *T_base_tool0_target: 目标位置在机器人base坐标系下的变换矩阵
                 */
                tf2::Transform T_base_marker, T_cam_marker_current, T_tool0_cam;

                //调用lookupAllTransforms,查寻tf
                if (!lookupAllTransforms(T_base_marker, T_cam_marker_current, T_tool0_cam)) {
                    rate.sleep();
                    continue;
                }

                //调用computeTargetTool0
                tf2::Transform T_base_tool0_target =
                    computeTargetTool0(T_base_marker, T_cam_marker_current, T_tool0_cam);

                //定义一个ros的geometry_msgs::Pose消息对象
                geometry_msgs::Pose target_pose;

                //tf2::toMsg()是tf2库提供的转换函数，将tf2::Transform转换为ros的网络传输消息形式
                tf2::toMsg(T_base_tool0_target, target_pose);

                //发布红色箭头，在rviz中直观显示算出来的目标
                publishTargetMarker(target_pose);

                //将目标位姿交给moveit进行规划并执行
                planAndExecute(target_pose);



            } catch (const tf2::TransformException &ex) {
                ROS_WARN_THROTTLE(3.0, "TF 获取错误: %s", ex.what());
            } catch (const std::exception &e) {
                ROS_ERROR("发生异常: %s", e.what());
            }

            rate.sleep();
        }
    }

private:
    /**
     * @details 从tf_buffer_中查询所有必要的变换矩阵
     * @return true: 如果所有变换矩阵都成功获取
     * @return false: 如果任何变换矩阵获取失败
     */
    bool lookupAllTransforms(tf2::Transform &T_base_marker,
                             tf2::Transform &T_cam_marker,
                             tf2::Transform &T_tool0_cam) {
        //局部变量，用于临时接收tf查询结果的消息对象
        geometry_msgs::TransformStamped ts;

        ts = tf_buffer_.lookupTransform(base_frame_, marker_frame_,
                                        ros::Time(0), ros::Duration(1.0));
        tf2::fromMsg(ts.transform, T_base_marker);         ///将ros的网络传输消息形式转换为tf2::Transform类型

        ts = tf_buffer_.lookupTransform("camera_link", marker_frame_,
                                        ros::Time(0), ros::Duration(1.0));
        tf2::fromMsg(ts.transform, T_cam_marker);

        ts = tf_buffer_.lookupTransform(tool0_frame_, "camera_link",
                                        ros::Time(0), ros::Duration(1.0));
        tf2::fromMsg(ts.transform, T_tool0_cam);

        return true;
    }



    /**
     * @details  计算末端目标位姿
     * @param T_base_marker: marker在（0，0，0）的固定位置-二维码不会跑
     * @param T_cam_marker_current: 相机看到的二维码的姿势  -视觉实时给的
     * @param T_tool0_cam: 相机相对tool0的固定安装位姿 - 手眼标定的结果
     * @return T_base_tool0_target:  输出机械臂末端tool0的目标位姿，发给机械臂动起来
     */

    tf2::Transform computeTargetTool0(const tf2::Transform &T_base_marker,
                                       const tf2::Transform &T_cam_marker_current,
                                       const tf2::Transform &T_tool0_cam) {


        /**
        *1、机看到二维码的实际位姿是T_cam_marker_current
        *2、但是，我们希望相机正对着二维码，且距离是camera_distance_
        *3、所以，我们“虚拟“构造一个【期望的相机->二维码】位姿T_cam_marker_desired
        */
        tf2::Transform T_cam_marker_desired = T_cam_marker_current;
        T_cam_marker_desired.setOrigin(tf2::Vector3(0, 0, camera_distance_));

        tf2::Transform inv_T_cm  = T_cam_marker_desired.inverse();
        tf2::Transform inv_T_tc  = T_tool0_cam.inverse();

        tf2::Transform T_base_tool0_target = T_base_marker * inv_T_cm * inv_T_tc;

        return T_base_tool0_target;
    }

    void planAndExecute(const geometry_msgs::Pose &target_pose) {
        if (!std::isfinite(target_pose.position.x) ||
            !std::isfinite(target_pose.orientation.w)) {
            ROS_WARN_THROTTLE(2.0, "目标位姿包含无效数值 (NaN/Inf)，跳过本次规划");
            return;
        }

        double dist = std::sqrt(
            target_pose.position.x * target_pose.position.x +
            target_pose.position.y * target_pose.position.y +
            target_pose.position.z * target_pose.position.z);

        if (dist > 1.05) {
            ROS_WARN_THROTTLE(2.0, "目标距离 %.2f 米 > UR5e 极限臂展，跳过本次规划", dist);
            return;
        }

        move_group_.clearPoseTargets();
        move_group_.setPoseTarget(target_pose);

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        moveit::planning_interface::MoveItErrorCode result = move_group_.plan(plan);

        if (result == moveit::planning_interface::MoveItErrorCode::SUCCESS) {
            ROS_INFO("规划成功！目标位置: (%.3f, %.3f, %.3f) 米",
                     target_pose.position.x, target_pose.position.y, target_pose.position.z);
            move_group_.execute(plan);
        } else {
            ROS_WARN_THROTTLE(3.0, "规划失败 (错误码 %d)，目标位置: (%.3f, %.3f, %.3f)",
                              result.val,
                              target_pose.position.x,
                              target_pose.position.y,
                              target_pose.position.z);
        }
    }



    /**
     * @details 把传入的pose在rviz中用一个红色箭头画出来，方便可视化
     * @param pose: 目标位姿
     */
    void publishTargetMarker(const geometry_msgs::Pose &pose) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = base_frame_;
        marker.header.stamp = ros::Time::now();
        marker.ns = "target_pose";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::ARROW;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose = pose;
        marker.scale.x = 0.2;
        marker.scale.y = 0.03;
        marker.scale.z = 0.03;
        marker.color.r = 1.0f;
        marker.color.g = 0.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0f;
        marker_pub_.publish(marker);
    }

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    //成员变量声明 move_group_
    /**
    *moveit:顶层命名空间
    *planning_interface:moveit下的子命名空间，用于规划运动
    *MoveGroupInterface:moveit提供的接口类
    */
    moveit::planning_interface::MoveGroupInterface move_group_;
    ros::Publisher marker_pub_;

    double camera_distance_;
    double loop_rate_hz_;
    double planning_time_;
    std::string base_frame_;
    std::string marker_frame_;
    std::string tool0_frame_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "aruco_follower");
    setlocale(LC_ALL, "");  // 新增：允许显示中文
    ros::AsyncSpinner spinner(1);
    spinner.start();

    ArucoFollower follower;
    follower.run();
    return 0;
}