#include <ros/ros.h>

#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <eigen_conversions/eigen_msg.h>

#include <eigen_conversions/eigen_kdl.h>

#include <arf_moveit_wrapper/moveit_wrapper.h>
#include <gluebot_demo/ur_kin.h>




using MoveitPlan = moveit::planning_interface::MoveGroupInterface::Plan;

void printJointPose(std::vector<double>& q)
{
    ROS_INFO_STREAM("Joint Pose ======================");
    ROS_INFO_STREAM("( " << q[0] << ", "
    << q[1] << ", " << q[2] << ", "
    << q[3] << ", " << q[4] << ", "
    << q[5] << " )");
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_moveit_wrapper");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(2);
    spinner.start();

    namespace rvt = rviz_visual_tools;
    moveit_visual_tools::MoveItVisualToolsPtr visual_tools;
    visual_tools.reset(new moveit_visual_tools::MoveItVisualTools("world", "/moveit_visual_markers"));
    visual_tools->deleteAllMarkers();

    // std::vector<double> qtest = {0, 0, 0, 0, 0, 0};
    std::vector<double> qtest = { 0.549796, 4.0, 1.37888, 2.93743, 1.19235, 1.16189 };

    UR5 robot;
    //robot.printCurrentJointValues();
    auto base_link = robot.getLinkFixedRelativeTransform("base_link");
    auto tool0_to_tool_tip = robot.getLinkFixedRelativeTransform("glue_gun") * robot.getLinkFixedRelativeTransform("tool_tip");

    auto tool0 = robot.fk(qtest, "tool0");
    tool0 = base_link.inverse() * tool0;
    ROS_INFO_STREAM("==== test transform:");
    ROS_INFO_STREAM('\n' << tool0.translation().transpose());
    ROS_INFO_STREAM('\n' << tool0.rotation());

    auto ttest = robot.fk(qtest);
    //ttest = base_link.inverse() * ttest * tool0_to_tool_tip.inverse();
    ROS_INFO_STREAM("==== test transform:");
    ROS_INFO_STREAM('\n' << ttest.translation().transpose());
    ROS_INFO_STREAM('\n' << ttest.rotation());

    visual_tools->publishAxisLabeled(ttest, "ttest");
    visual_tools->trigger();

    // =========================================================================================
    ROS_INFO_STREAM("Calculating ik ur5");
    IKSolution joint_poses;

    // tranfrom pose to reference frame of 6dof robot
    auto sub_robot_pose = base_link.inverse() * ttest * tool0_to_tool_tip.inverse();

    ROS_INFO_STREAM("==== subrobot pose ====");
    ROS_INFO_STREAM('\n' << sub_robot_pose.translation().transpose());
    ROS_INFO_STREAM('\n' << sub_robot_pose.rotation());
    visual_tools->publishAxisLabeled(sub_robot_pose, "sub_robot_pose");
    visual_tools->trigger();

    double q_sols [8][6];
    Eigen::Affine3d pose_copy = sub_robot_pose;

    KDL::Frame frame;
    tf::transformEigenToKDL(sub_robot_pose, frame);

    double ik_pose[4][4];
    frame.Make4x4((double*) ik_pose);

    int num_sols = ur_kinematics::inverse((double*) ik_pose, (double*) q_sols);

    // transpose pose matrix, ur_kinematics uses row-major ordering
    // eigen uses column-major ordering as default.
    //pose_copy.matrix().transposeInPlace();
    
    //Eigen::Matrix4d M = sub_robot_pose.matrix();
    // double ik_pose[4][4] = {
    //     { M(0, 0), M(1, 0), M(2, 0), M(3, 0)},
    //     { M(0, 1), M(1, 1), M(2, 1), M(3, 1)},
    //     { M(0, 2), M(1, 2), M(2, 2), M(3, 2)},
    //     { M(0, 3), M(1, 3), M(2, 3), M(3, 3)}
    // };
    // int num_sols = ur_kinematics::inverse(pose_copy.data(), (double*) q_sols);

    ROS_INFO_STREAM("Found " << num_sols << " ik solutions.");

    std::vector<double> tmp(6);
    for (std::size_t i = 0; i < num_sols; ++i)
    {
        auto sol = q_sols[i];
        std::copy(sol, sol + 6, tmp.data());
        joint_poses.push_back(tmp);
        printJointPose(tmp);
    }

    
    
    while(ros::ok())
    {
        robot.plot(visual_tools, qtest);
        ros::Duration(2.0).sleep();

        for (auto q : joint_poses)
        {
            robot.plot(visual_tools, q);
            ros::Duration(0.5).sleep();
        }
    }

    // ros::waitForShutdown();
    ros::shutdown();
    return 0;
}
