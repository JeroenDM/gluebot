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
  ROS_INFO_STREAM("( " << q[0] << ", " << q[1] << ", " << q[2] << ", " << q[3] << ", " << q[4] << ", " << q[5] << " )");
}

int main(int argc, char** argv)
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
  robot.printCurrentJointValues();

  auto pose_tool0 = robot.fk(qtest, "tool_tip");
  visual_tools->publishAxisLabeled(pose_tool0, "fk_tool0");
  // robot.plot(visual_tools, qtest);
  visual_tools->trigger();

  IKSolution joint_poses;
  joint_poses = robot.ik(pose_tool0);

  int cnt = 3;
  while (cnt > 0)
  {
    robot.plot(visual_tools, qtest);
    ros::Duration(2.0).sleep();

    for (auto q : joint_poses)
    {
      if (robot.isInCollision(q))
        robot.plot(visual_tools, q, rviz_visual_tools::colors::RED);
      else
        robot.plot(visual_tools, q);
      ros::Duration(0.5).sleep();
    }
    cnt--;
  }

  // ros::waitForShutdown();
  ros::shutdown();
  return 0;
}
