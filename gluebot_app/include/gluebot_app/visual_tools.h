#ifndef _GLUEBOT_VISUAL_TOOLS_H_
#define _GLUEBOT_VISUAL_TOOLS_H_

#include <ros/ros.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

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

    void publishWorkobjectMesh(Eigen::Affine3d& pose, const std::string name = "part.stl")
    {
        ROS_INFO_STREAM_NAMED("visual_tools", "Publishing Collision Mesh");
        std::string file_path = "file://" + ros::package::getPath("gluebot_support");
        if (file_path == "file://")
            ROS_FATAL_STREAM_NAMED("visual_tools", "Unable to get "
                                                       << "gluebot_support"
                                                       << " package path ");
        file_path.append("/meshes/");
        file_path.append(name);
        visual_tools_->publishCollisionMesh(visual_tools_->convertPose(pose), "Part", file_path, rvt::CYAN);
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