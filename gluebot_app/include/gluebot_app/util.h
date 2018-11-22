#ifndef _GLUEBOT_UTIL_H_
#define _GLUEBOT_UTIL_H_

#include <ros/ros.h>

#include <moveit_visual_tools/moveit_visual_tools.h>
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

std::vector<Eigen::Affine3d> createLine(Eigen::Vector3d start, Eigen::Vector3d end)
{
    std::vector<Eigen::Affine3d> waypoints;
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

namespace rvt = rviz_visual_tools;

class VisualTools
{
  public:
    /**
     * \brief Constructor
     */
    VisualTools()
    {
        visual_tools_.reset(new moveit_visual_tools::MoveItVisualTools("world", "visual_tools"));
        visual_tools_->setPlanningSceneTopic("/move_group/monitored_planning_scene");
        visual_tools_->loadPlanningSceneMonitor();
        visual_tools_->loadMarkerPub(true);
        // visual_tools_->loadRobotStatePub("display_robot_state");
        visual_tools_->setManualSceneUpdating();

        // robot_state_ = visual_tools_->getSharedRobotState();
        // jmg_ = robot_state_->getJointModelGroup("manipulator");

        // Allow time to publish messages
        ros::spinOnce();
        ros::Duration(0.1).sleep();

        // Clear collision objects and markers
        visual_tools_->deleteAllMarkers();
        visual_tools_->removeAllCollisionObjects();
        visual_tools_->triggerPlanningSceneUpdate();
        ros::Duration(0.5).sleep();  // made this time bigger, now collision objects show up in planning scene

        // Show message
        Eigen::Affine3d text_pose = Eigen::Affine3d::Identity();
        text_pose.translation().z() = 2;
        visual_tools_->publishText(text_pose, "MoveIt! Visual Tools", rvt::WHITE, rvt::XLARGE, /*static_id*/ false);
    }

    void publishLabelHelper(const Eigen::Affine3d& pose, const std::string& label)
    {
        Eigen::Affine3d pose_copy = pose;
        pose_copy.translation().x() -= 0.2;
        visual_tools_->publishText(pose_copy, label, rvt::WHITE, rvt::LARGE, false);
    }

    void publishWorkobjectMesh(geometry_msgs::Pose pose, const std::string name = "part.stl")
    {
        ROS_INFO_STREAM_NAMED("visual_tools", "Publishing Collision Mesh");
        std::string file_path = "file://" + ros::package::getPath("gluebot_support");
        if (file_path == "file://")
            ROS_FATAL_STREAM_NAMED("visual_tools", "Unable to get "
                                                       << "gluebot_support"
                                                       << " package path ");
        file_path.append("/meshes/");
        file_path.append(name);
        visual_tools_->publishCollisionMesh(pose, "Part", file_path, rvt::CYAN);
        // Send ROS messages and give it some time to do it
        visual_tools_->triggerPlanningSceneUpdate();
    }

    void publishFrame(geometry_msgs::Pose frame)
    {
        visual_tools_->publishAxis(frame, rviz_visual_tools::MEDIUM);
        visual_tools_->trigger();
    }

    void publishFrame(Eigen::Affine3d frame)
    {
        visual_tools_->publishAxis(visual_tools_->convertPose(frame), rviz_visual_tools::MEDIUM);
        visual_tools_->trigger();
    }

    // For visualizing things in rviz
    moveit_visual_tools::MoveItVisualToolsPtr visual_tools_;

  private:
    // A shared node handle
    ros::NodeHandle nh_;

    

    // MoveIt Components
    planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;
    const moveit::core::JointModelGroup* jmg_;
    moveit::core::RobotStatePtr robot_state_;
};

#endif