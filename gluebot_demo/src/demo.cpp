#include <ros/ros.h>
#include <eigen3/Eigen/Geometry>
#include <string>
#include <vector>

#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <eigen_conversions/eigen_msg.h>

#include "arf_moveit_wrapper/moveit_wrapper.h"
#include "arf_graph/graph.h"
#include "arf_graph/util.h"
//#include "arf_planning/planner.h"
#include "arf_trajectory/trajectory.h"

#include "ur_kinematics/ur_kin.h"

using MoveitPlan = moveit::planning_interface::MoveGroupInterface::Plan;

std::vector<geometry_msgs::Pose> createGlueTask();

std::vector<geometry_msgs::Pose> createCircleTask();
std::vector<Eigen::Affine3d> createCircleTaskEigen();

MoveitPlan jointTrajectoryToMoveitPlan(std::vector<JointPose> joint_trajectory);

TrajectoryPoint poseToTrajectoryPoint(Eigen::Affine3d pose);

void printJointPose(JointPose& q)
{
    ROS_INFO_STREAM("Joint Pose ======================");
    ROS_INFO_STREAM("( " << q[0] << ", "
    << q[1] << ", " << q[2] << ", "
    << q[3] << ", " << q[4] << ", "
    << q[5] << " )");
}

class Planner
{
    std::vector<std::vector<std::vector<double>>> graph_data_;
    std::vector<std::vector<double>> shortest_path_;
public:
    bool createGraphData(RobotMoveitWrapper &robot, std::vector<TrajectoryPoint> &task);
    bool run(RobotMoveitWrapper &robot, std::vector<TrajectoryPoint> &task);
    std::vector<std::vector<double>> getShortestPath()
    {
        return shortest_path_;
    }
};

bool Planner::createGraphData(RobotMoveitWrapper &robot, std::vector<TrajectoryPoint> &task)
{
    ROS_INFO_STREAM("Creating graph data.");
    for (auto tp : task)
    {
        std::vector<std::vector<double>> new_data;
        for (auto pose : tp.getGridSamples())
        {
            for (auto q_sol : robot.ik(pose))
            {
                if (!robot.isInCollision(q_sol))
                {
                    //printJointPose(q_sol);
                    new_data.push_back(q_sol);
                }
            }
        }
        if (new_data.size() == 0)
        {
            ROS_ERROR("No collision free ik sol found for a tp");
            return false;
        }
        graph_data_.push_back(new_data);
    }
    return true;
}

bool Planner::run(RobotMoveitWrapper &robot, std::vector<TrajectoryPoint> &task)
{
    if (createGraphData(robot, task))
    {
        Graph g(graph_data_);
        g.runMultiSourceDijkstra();
        std::vector<Node*> sp = g.getShortestPath();
        for (auto node : sp)
        {
            //ROS_INFO_STREAM("=======Node " << (*node));
            shortest_path_.push_back(*(*node).jv);
        }
        return true;
    }
    return false;
}

void addPlans(MoveitPlan& plan_a, MoveitPlan& plan_b)
{
    //std::size_t Na = plan_a.trajectory_.joint_trajectory.points.size();
    ros::Duration last_time_a = plan_a.trajectory_.joint_trajectory.points.back().time_from_start;
    for (auto jtp : plan_b.trajectory_.joint_trajectory.points)
    {
        jtp.time_from_start += last_time_a;
        plan_a.trajectory_.joint_trajectory.points.push_back(jtp);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_moveit_wrapper");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    UR5 robot;
    robot.printCurrentJointValues();

    // test ur kinematics
    double q_example[] = {0, 0, 0, 0, 0, 0};
    double T_example[4][4];

    Eigen::Affine3d T_eigen = Eigen::Affine3d::Identity();
    ur_kinematics::forward(q_example, T_eigen.data());
    T_eigen.matrix().transposeInPlace();
    ROS_INFO_STREAM("==== Ik result:");
    ROS_INFO_STREAM(T_eigen.translation());
    ROS_INFO_STREAM(T_eigen.rotation());

    moveit::planning_interface::MoveGroupInterface move_group("manipulator");

    // std::string planner_id;
    // if (node_handle.getParam("/ompl_planner", planner_id))
    // {
    //     ROS_INFO_STREAM("OMPL planner read from config.yaml: " << planner_id);
    //     move_group.setPlannerId(planner_id);
    // }

    //moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    //std::string workcell = planning_scene_interface.getKnownObjectNames()[0];
    //ROS_INFO_STREAM("Planing scene: " << workcell);

    namespace rvt = rviz_visual_tools;
    moveit_visual_tools::MoveItVisualTools visual_tools("world", "/moveit_visual_markers");
    visual_tools.deleteAllMarkers();

    std::vector<geometry_msgs::Pose> task = createGlueTask();
    //std::vector<geometry_msgs::Pose> task = createCircleTask();
    //std::vector<TrajectoryPoint> taskEigen = createCircleTaskEigen();
    for (auto pose : task)
    {
        visual_tools.publishAxis(pose);
    }
    visual_tools.trigger();

    // =================================================================================================================
    // plan approach path
    moveit::planning_interface::MoveGroupInterface::Plan approach_plan;
    move_group.setPoseTarget(task[0]);
    //move_group.move();
    bool success = (move_group.plan(approach_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if (!success)
    {
        ROS_ERROR_STREAM("Failed to plan approach path");
        ros::shutdown();
        return 0;
    }

    std::vector<TrajectoryPoint> ee_trajectory;
    for (auto moveit_pose : task)
    {
        Eigen::Affine3d eigen_pose;
        tf::poseMsgToEigen(moveit_pose, eigen_pose);
        ee_trajectory.push_back(poseToTrajectoryPoint(eigen_pose));
    }

    // for (auto pose : task)
    // {
    //     Eigen::Affine3d pose_eigen;
    //     tf::poseMsgToEigen(pose, pose_eigen);
    //     robot.ik(pose_eigen);
    // }

    // std::vector<double> q_test = {0, 0, 0.1, 0.2, 0.3};
    // auto T_test = robot.fk(q_test);
    // ROS_INFO_STREAM("Test pose: " << T_test.translation());
    // ROS_INFO_STREAM("Test pose: " << T_test.rotation());

    // auto result = robot.ik(T_test);

    // =================================================================================================================
    //PLAN GLUE PATH

    Planner planner;
    if (!planner.run(robot, ee_trajectory))
    {
        ROS_ERROR_STREAM("Failed to find approach path.");
        move_group.execute(approach_plan);
        ros::Duration(2.0).sleep();
    }
    else
    {
        MoveitPlan glue_plan;
        glue_plan = jointTrajectoryToMoveitPlan(planner.getShortestPath());

        auto path = planner.getShortestPath();

        addPlans(approach_plan, glue_plan);
        move_group.execute(approach_plan);
        ros::Duration(2.0).sleep();
    }

    //move_group.execute(approach_plan);
    //ros::Duration(2.0).sleep();

    // moveit_msgs::RobotTrajectory trajectory;
    // const double jump_threshold = 0.0;
    // const double eef_step = 0.01;
    // double fraction = move_group.computeCartesianPath(task, eef_step, jump_threshold, trajectory);
    // ROS_INFO_NAMED("tutorial", "Visualizing plan 4 (Cartesian path) (%.2f%% acheived)", fraction * 100.0);

    // if (fraction == 1.0)
    // {
    //     moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    //     my_plan.trajectory_ = trajectory;
    //     move_group.execute(my_plan);
    // }
    // else
    // {
    //     ROS_ERROR_STREAM("Failed to plan glue path.");
    // }

    // =================================================================================================================
    // Move home again
    moveit::planning_interface::MoveGroupInterface::Plan home_plan;
    move_group.setNamedTarget("home");
    if (!(move_group.plan(home_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS))
    {
        ROS_ERROR_STREAM("Failed to plan to home position");
        ros::shutdown();
        return 0;
    }
    move_group.execute(home_plan);

    ROS_INFO("Wating 2 seconds before shutting down node.");
    ros::Duration(2.0).sleep();

    ros::shutdown();
    return 0;
}

std::vector<geometry_msgs::Pose> createGlueTask()
{
    std::vector<geometry_msgs::Pose> waypoints;

    Eigen::Affine3d pose = Eigen::Affine3d::Identity() * Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitY());
    //pose.rotate(M_PI_2, Eigen::Axis)

    geometry_msgs::Pose waypoint;
    tf::poseEigenToMsg(pose, waypoint);

    // waypoint.orientation.w = 1.0;
    waypoint.position.x = 0.7;
    waypoint.position.y = -0.2;
    waypoint.position.z = 0.0;
    waypoints.push_back(waypoint);

    for (int i = 0; i < 10; ++i)
    {
        waypoint.position.y += 0.05;
        waypoints.push_back(waypoint);
    }
    return waypoints;
}

std::vector<geometry_msgs::Pose> createCircleTask()
{
    std::vector<geometry_msgs::Pose> waypoints;
    double radius = 0.03485; //0.02425;
    double y_rotation = 45.0 * M_PI / 180;
    struct Position
    {
        double x = 0.55;
        double y = 0.03485;
        double z = -0.05;
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

std::vector<Eigen::Affine3d> createCircleTaskEigen()
{
    std::vector<Eigen::Affine3d> waypoints;
    double radius = 0.03485; //0.02425;
    double y_rotation = 45.0 * M_PI / 180;
    struct Position
    {
        double x = 0.55;
        double y = 0.03485;
        double z = -0.05;
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

        waypoints.push_back(pose);

        z_rotation += step_size;
    }

    return waypoints;
}

MoveitPlan jointTrajectoryToMoveitPlan(std::vector<JointPose> joint_trajectory)
{
    MoveitPlan plan;
    ros::Duration start_time(0.0);
    for (auto &q : joint_trajectory)
    {
        trajectory_msgs::JointTrajectoryPoint new_point;
        new_point.positions = q;
        new_point.time_from_start = start_time;
        plan.trajectory_.joint_trajectory.points.push_back(new_point);
        start_time += ros::Duration(0.2);
    }
    return plan;
}

TrajectoryPoint poseToTrajectoryPoint(Eigen::Affine3d pose)
{
    Eigen::Vector3d position = pose.translation();
    Eigen::Vector3d angles = pose.rotation().eulerAngles(0, 1, 2);

    Number x(position[0]), y(position[1]), z(position[2]);
    Number rx(angles[0]), ry(angles[1]), rz(angles[2]);

    TrajectoryPoint tp(x, y, z, rx, ry, rz);

    return tp;
}