#include <ros/ros.h>
#include <eigen3/Eigen/Geometry>

#include "ur_kinematics/ur_kin.h"

void printJointPose(double* q)
{
    ROS_INFO_STREAM("( " << q[0] << ", "
    << q[1] << ", " << q[2] << ", "
    << q[3] << ", " << q[4] << ", "
    << q[5] << " )");
}

int ik(Eigen::Affine3d& pose, double* q_sols)
{
    pose.matrix().transposeInPlace();
    return ur_kinematics::inverse(pose.data(), q_sols);
}

std::vector<Eigen::Affine3d> createGlueTask()
{
    std::vector<Eigen::Affine3d> waypoints;

    Eigen::Affine3d pose = Eigen::Affine3d::Identity() * Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitY());
    //pose.rotate(M_PI_2, Eigen::Axis)

    Eigen::Affine3d waypoint;

    waypoint.translation() << 0.7, -0.2, 0.0;
    waypoints.push_back(waypoint);

    for (int i = 0; i < 10; ++i)
    {
        waypoint.translation()[1] += 0.05;
        waypoints.push_back(waypoint);
    }
    return waypoints;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_moveit_wrapper");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    auto task = createGlueTask();

    // test ur kinematics
    double q_example[] = {0, 0, 0, 0, 0, 0};
    double T_example[4][4];

    Eigen::Affine3d T_eigen = Eigen::Affine3d::Identity();
    ur_kinematics::forward(q_example, T_eigen.data());
    T_eigen.matrix().transposeInPlace();
    ROS_INFO_STREAM("==== Ik result:");
    ROS_INFO_STREAM(T_eigen.translation());
    ROS_INFO_STREAM(T_eigen.rotation());

    ROS_INFO_STREAM(task[0].translation());
    ROS_INFO_STREAM(task[0].rotation());

    // IK calculations:
    // ================

    double q_sols[8][6];
    int num_sols = ik(T_eigen, q_sols[0]);

    ROS_INFO_STREAM("Number of ik solutions: " << num_sols);
    ROS_INFO_STREAM("Solutions:");

    for (int i = 0; i < num_sols; ++i)
    {
        printJointPose(q_sols[i]);
    }

    ros::shutdown();
    return 0;
}