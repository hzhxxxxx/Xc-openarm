#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <example_interfaces/msg/string.hpp>
#include <openarmx_interfaces/msg/joint_command.hpp>
#include <openarmx_interfaces/msg/pose_command.hpp>

using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;
using String = example_interfaces::msg::String;
using JointCommand = openarmx_interfaces::msg::JointCommand;
using PoseCommand = openarmx_interfaces::msg::PoseCommand;
using namespace std::placeholders;

class BimanualCommander
{
public:
    BimanualCommander(std::shared_ptr<rclcpp::Node> node)
    {
        node_ = node;

        // 创建执行完成反馈发布者
        execution_done_pub_ = node_->create_publisher<String>("moveit_execution_done", 10);

        // 初始化四个 MoveGroupInterface
        left_arm_ = std::make_shared<MoveGroupInterface>(node_, "left_arm");
        left_arm_->setMaxVelocityScalingFactor(0.1);
        left_arm_->setMaxAccelerationScalingFactor(0.1);
        left_arm_->setEndEffectorLink("openarmx_left_hand_tcp");  // 设置末端执行器为夹爪TCP

        right_arm_ = std::make_shared<MoveGroupInterface>(node_, "right_arm");
        right_arm_->setMaxVelocityScalingFactor(0.1);
        right_arm_->setMaxAccelerationScalingFactor(0.1);
        right_arm_->setEndEffectorLink("openarmx_right_hand_tcp");  // 设置末端执行器为夹爪TCP

        left_gripper_ = std::make_shared<MoveGroupInterface>(node_, "left_gripper");
        right_gripper_ = std::make_shared<MoveGroupInterface>(node_, "right_gripper");

        // 创建订阅者 - 左臂
        left_arm_named_target_sub_ = node_->create_subscription<String>(
            "left_arm_named_target", 10, std::bind(&BimanualCommander::LeftArmNamedTargetCallback, this, _1));

        left_arm_joint_target_sub_ = node_->create_subscription<JointCommand>(
            "left_arm_joint_target", 10, std::bind(&BimanualCommander::LeftArmJointTargetCallback, this, _1));

        left_arm_pose_target_sub_ = node_->create_subscription<PoseCommand>(
            "left_arm_pose_target", 10, std::bind(&BimanualCommander::LeftArmPoseTargetCallback, this, _1));

        // 创建订阅者 - 右臂
        right_arm_named_target_sub_ = node_->create_subscription<String>(
            "right_arm_named_target", 10, std::bind(&BimanualCommander::RightArmNamedTargetCallback, this, _1));

        right_arm_joint_target_sub_ = node_->create_subscription<JointCommand>(
            "right_arm_joint_target", 10, std::bind(&BimanualCommander::RightArmJointTargetCallback, this, _1));

        right_arm_pose_target_sub_ = node_->create_subscription<PoseCommand>(
            "right_arm_pose_target", 10, std::bind(&BimanualCommander::RightArmPoseTargetCallback, this, _1));

        // 创建订阅者 - 夹爪
        left_gripper_named_target_sub_ = node_->create_subscription<String>(
            "left_gripper_named_target", 10, std::bind(&BimanualCommander::LeftGripperNamedTargetCallback, this, _1));

        right_gripper_named_target_sub_ = node_->create_subscription<String>(
            "right_gripper_named_target", 10, std::bind(&BimanualCommander::RightGripperNamedTargetCallback, this, _1));
    }

    // 用设定好的点位名字，作为目标：
    void goToNamedTarget(const std::shared_ptr<MoveGroupInterface> &arm, const std::string &name)
    {
        arm->setStartStateToCurrentState();
        arm->setNamedTarget(name);
        planAndExecute(arm);
    }

    // 用关节参数作为目标：
    void goToJointTarget(const std::shared_ptr<MoveGroupInterface> &arm, const std::vector<double> &joints)
    {
        arm->setStartStateToCurrentState();
        arm->setJointValueTarget(joints);
        planAndExecute(arm);
    }

    // 用末端位姿做目标
    void goToPoseTarget(const std::shared_ptr<MoveGroupInterface> &arm,
                        double x, double y, double z,
                        double roll, double pitch, double yaw,
                        const std::string &frame_id,
                        bool cartesian_path=false)
    {
        // 因为moveit里面用的不是rpx ，rpy这种欧拉角，而是四元数，所以需要一个人工转化的过程
        tf2::Quaternion q;
        q.setRPY(roll, pitch, yaw);
        q = q.normalize();

        geometry_msgs::msg::PoseStamped target_pose;
        target_pose.header.frame_id = frame_id;
        target_pose.pose.position.x = x;
        target_pose.pose.position.y = y;
        target_pose.pose.position.z = z;
        target_pose.pose.orientation.x = q.getX();
        target_pose.pose.orientation.y = q.getY();
        target_pose.pose.orientation.z = q.getZ();
        target_pose.pose.orientation.w = q.getW();

        arm->setStartStateToCurrentState();

        //如果只是普通的运动
        if (!cartesian_path){
            arm->setPoseTarget(target_pose);
            planAndExecute(arm);
        }
        //如果是笛卡尔直线运动
        else{
            std::vector<geometry_msgs::msg::Pose> waypoints;
            // 注意：不要添加当前位姿作为起点！MoveIt 会自动使用当前状态作为起点
            waypoints.push_back(target_pose.pose);  // 只添加目标点

            moveit_msgs::msg::RobotTrajectory trajectory;

            // MoveIt2 Humble 需要5个参数
            double fraction = arm->computeCartesianPath(waypoints, 0.01, 0.0, trajectory, true);

            if (fraction == 1){
                arm->execute(trajectory);
            }
        }
    }

    // 发布执行完成信号
    void publishDone(const std::string &action_name)
    {
        String msg;
        msg.data = action_name;
        execution_done_pub_->publish(msg);
    }

private:
    void planAndExecute(const std::shared_ptr<MoveGroupInterface> &interface)
    {
        MoveGroupInterface::Plan plan;
        bool success = (interface->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

        if (success) {
            interface->execute(plan);
        }
    }

    // 左臂回调函数
    void LeftArmNamedTargetCallback(const String &msg)
    {
        goToNamedTarget(left_arm_, msg.data);
        publishDone("left_arm_named");
    }

    void LeftArmJointTargetCallback(const JointCommand &msg)
    {
        std::vector<double> joints(msg.joint_positions.begin(), msg.joint_positions.end());
        goToJointTarget(left_arm_, joints);
        publishDone("left_arm_joint");
    }

    void LeftArmPoseTargetCallback(const PoseCommand &msg)
    {
        goToPoseTarget(left_arm_, msg.x, msg.y, msg.z,
                       msg.roll, msg.pitch, msg.yaw,
                       "openarmx_left_link0",
                       msg.cartesian_path);
        publishDone("left_arm_pose");
    }

    // 右臂回调函数
    void RightArmNamedTargetCallback(const String &msg)
    {
        goToNamedTarget(right_arm_, msg.data);
        publishDone("right_arm_named");
    }

    void RightArmJointTargetCallback(const JointCommand &msg)
    {
        std::vector<double> joints(msg.joint_positions.begin(), msg.joint_positions.end());
        goToJointTarget(right_arm_, joints);
        publishDone("right_arm_joint");
    }

    void RightArmPoseTargetCallback(const PoseCommand &msg)
    {
        goToPoseTarget(right_arm_, msg.x, msg.y, msg.z,
                       msg.roll, msg.pitch, msg.yaw,
                       "openarmx_right_link0",
                       msg.cartesian_path);
        publishDone("right_arm_pose");
    }

    // 夹爪回调函数
    void LeftGripperNamedTargetCallback(const String &msg)
    {
        goToNamedTarget(left_gripper_, msg.data);
        publishDone("left_gripper_named");
    }

    void RightGripperNamedTargetCallback(const String &msg)
    {
        goToNamedTarget(right_gripper_, msg.data);
        publishDone("right_gripper_named");
    }

    std::shared_ptr<rclcpp::Node> node_;

    // 执行完成反馈发布者
    rclcpp::Publisher<String>::SharedPtr execution_done_pub_;

    // 四个 MoveGroupInterface
    std::shared_ptr<MoveGroupInterface> left_arm_;
    std::shared_ptr<MoveGroupInterface> right_arm_;
    std::shared_ptr<MoveGroupInterface> left_gripper_;
    std::shared_ptr<MoveGroupInterface> right_gripper_;

    // 订阅者
    rclcpp::Subscription<String>::SharedPtr left_arm_named_target_sub_;
    rclcpp::Subscription<JointCommand>::SharedPtr left_arm_joint_target_sub_;
    rclcpp::Subscription<PoseCommand>::SharedPtr left_arm_pose_target_sub_;

    rclcpp::Subscription<String>::SharedPtr right_arm_named_target_sub_;
    rclcpp::Subscription<JointCommand>::SharedPtr right_arm_joint_target_sub_;
    rclcpp::Subscription<PoseCommand>::SharedPtr right_arm_pose_target_sub_;

    rclcpp::Subscription<String>::SharedPtr left_gripper_named_target_sub_;
    rclcpp::Subscription<String>::SharedPtr right_gripper_named_target_sub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("bimanual_commander");
    auto commander = BimanualCommander(node);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
