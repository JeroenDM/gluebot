#ifndef _GLUEBOT_UTIL_H_
#define _GLUEBOT_UTIL_H_

#include <ros/ros.h>

#include <descartes_core/trajectory_pt.h>
#include <descartes_trajectory/axial_symmetric_pt.h>

#include <Eigen/Geometry>

void printJointPose(const std::vector<double>& q)
{
    ROS_INFO_STREAM("Joint Pose ======================");
    ROS_INFO_STREAM("( " << q[0] << ", "
    << q[1] << ", " << q[2] << ", "
    << q[3] << ", " << q[4] << ", "
    << q[5] << " )");
}

descartes_core::TrajectoryPtPtr makeTolerancedCartesianPoint(const Eigen::Affine3d& pose)
{
    using namespace descartes_core;
    using namespace descartes_trajectory;
    return TrajectoryPtPtr(new AxialSymmetricPt(pose, M_PI / 2.0, AxialSymmetricPt::Z_AXIS));
}

std::vector<descartes_core::TrajectoryPtPtr> makeDescartesTrajectory(EigenSTL::vector_Affine3d& path)
{
    std::vector<descartes_core::TrajectoryPtPtr> descartes_path;  // return value
    for (auto& point : path)
    {
        descartes_core::TrajectoryPtPtr pt = makeTolerancedCartesianPoint(point);
        descartes_path.push_back(pt);
    }
    return descartes_path;
}

EigenSTL::vector_Affine3d createLine(Eigen::Vector3d start, Eigen::Vector3d end)
{
    //std::vector<Eigen::Affine3d> waypoints;
    EigenSTL::vector_Affine3d waypoints;
    Eigen::Affine3d waypoint;
    int N = 5;
    Eigen::Vector3d step = (end - start) / N;

    for (std::size_t i = 0; i < N; ++i)
    {
        waypoint = Eigen::Translation3d(start + i * step);
        waypoint *= Eigen::AngleAxisd(-M_PI_2, Eigen::Vector3d::UnitZ());
        waypoint *= Eigen::AngleAxisd(M_PI + 0.5, Eigen::Vector3d::UnitY());
        waypoints.push_back(waypoint);
    }
    return waypoints;
}

std::vector<Eigen::Affine3d> createGlueTask(Eigen::Affine3d part_frame)
{
    std::vector<Eigen::Affine3d> waypoints;

    Eigen::Affine3d waypoint = Eigen::Affine3d::Identity() * Eigen::AngleAxisd(-M_PI_2, Eigen::Vector3d::UnitZ()) *
                               Eigen::AngleAxisd(M_PI + 0.3, Eigen::Vector3d::UnitY());

    // Eigen::Affine3d waypoint;
    // tf::poseEigenToMsg(pose, waypoint);

    waypoint.translation() << 0.0, 0.0, 0.0055;
    waypoints.push_back(waypoint);
    int N = 5; // should be +1 because of point above
    double step = 0.15 / ((double)N - 1);
    for (int i = 0; i < N; ++i)
    {
        waypoint.translation()[0] += step;
        waypoints.push_back(waypoint);
    }
    for (int i = 0; i < waypoints.size(); ++i)
    {
        waypoints[i] = part_frame * waypoints[i];
    }
    return waypoints;
}

std::vector<Eigen::Affine3d> createCircleTaskEigen(Eigen::Affine3d part_frame)
{
    //using namespace Eigen;
    std::vector<Eigen::Affine3d> waypoints;
    double radius = 0.02425;
    double y_rotation = 40.0 * M_PI / 180;
    struct Position
    {
        double x = 0.2 - 0.03485;
        double y = 0.05;
        double z = 0.0055;
    };
    Position position;
    int steps = 20;
    double step_size = 360.0 / (steps-1) * M_PI / 180.0;
    double z_rotation = 0.0;

    for (int i = 0; i < 5; ++i)
    {
        // create frame with correct rotation
        Eigen::Affine3d pose = Eigen::Affine3d::Identity() * Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitY());
        pose = pose * Eigen::AngleAxisd(z_rotation, Eigen::Vector3d::UnitZ());
        pose = pose * Eigen::AngleAxisd(y_rotation, Eigen::Vector3d::UnitY());

        pose.translation()[0] = position.x + radius * std::cos(-z_rotation);
        pose.translation()[1] = position.y + radius * std::sin(-z_rotation);
        pose.translation()[2] = position.z;

        waypoints.push_back(pose);

        z_rotation += step_size;
    }

    for (int i = 0; i < waypoints.size(); ++i)
    {
        waypoints[i] = part_frame * waypoints[i];
    }
    return waypoints;
}

std::vector<geometry_msgs::Pose> createCircleTask()
{
    std::vector<geometry_msgs::Pose> waypoints;
    double radius = 0.02425;
    double y_rotation = 40.0 * M_PI / 180;
    struct Position
    {
        double x = 0.2 - 0.03485;
        double y = 0.05;
        double z = 0.0055;
    };
    Position position;
    int steps = 20;
    double step_size = 360.0 / steps * M_PI / 180.0;
    double z_rotation = 0.0;

    for (int i = 0; i < steps; ++i)
    {
        // create frame with correct rotation
        Eigen::Affine3d pose = Eigen::Affine3d::Identity() * Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitY());
        pose = pose * Eigen::AngleAxisd(z_rotation, Eigen::Vector3d::UnitZ());
        pose = pose * Eigen::AngleAxisd(y_rotation, Eigen::Vector3d::UnitY());

        // go to message format
        geometry_msgs::Pose waypoint;
        tf::poseEigenToMsg(pose, waypoint);

        waypoint.position.x = position.x + radius * std::cos(-z_rotation);
        waypoint.position.y = position.y + radius * std::sin(-z_rotation);
        waypoint.position.z = position.z;

        waypoints.push_back(waypoint);

        z_rotation += step_size;
    }

    return waypoints;
}

#endif