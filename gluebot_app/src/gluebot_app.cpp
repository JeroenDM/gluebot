#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <eigen_conversions/eigen_msg.h>

#include <moveit/move_group_interface/move_group_interface.h>

#include "gluebot_app/util.h"

#include <ur5_demo_descartes/ur5_robot_model.h>
#include <descartes_planner/dense_planner.h>
#include <descartes_utilities/ros_conversions.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

using MoveitPlan = moveit::planning_interface::MoveGroupInterface::Plan;

class GluebotApp
{
    ros::NodeHandle nh_;
    ros::ServiceServer planServer_, moveHomeServer_, executeServer_;
    moveit::planning_interface::MoveGroupInterfacePtr move_group_;
    std::vector<MoveitPlan> plans_;  // {approach plan, glue plan, retract plan}
    bool has_plan_ = false;
    std::vector<geometry_msgs::Pose> task_;  // this should be in the service call in the future

    boost::shared_ptr<ur5_demo_descartes::UR5RobotModel> model_;
    descartes_planner::DensePlanner planner_;
    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ac_;

  public:
    GluebotApp(ros::NodeHandle& nh) : ac_("joint_trajectory_action", true)
    {
        if (!initDescartes())
            throw std::runtime_error("There was an issue initializing Descartes");

        move_group_.reset(new moveit::planning_interface::MoveGroupInterface("manipulator"));

        planServer_ = nh_.advertiseService("plan_path", &GluebotApp::plan, this);
        moveHomeServer_ = nh_.advertiseService("move_home", &GluebotApp::moveHome, this);
        executeServer_ = nh_.advertiseService("execute_path", &GluebotApp::execute, this);

        plans_.resize(3);
    }

    std::vector<std::string> getJointNames()
    {
        std::vector<std::string> names;
        nh_.getParam("controller_joint_names", names);
        return names;
    }

    bool initDescartes()
    {
        // Create a robot model
        model_ = boost::make_shared<ur5_demo_descartes::UR5RobotModel>();

        // Define the relevant "frames"
        const std::string robot_description = "robot_description";
        const std::string group_name = "manipulator";
        const std::string world_frame = "world";  // Frame in which tool poses are expressed
        const std::string tcp_frame = "tool_tip";

        // Using the desired frames, let's initialize Descartes
        if (!model_->initialize(robot_description, group_name, world_frame, tcp_frame))
        {
            ROS_WARN("Descartes RobotModel failed to initialize");
            return false;
        }

        if (!planner_.initialize(model_))
        {
            ROS_WARN("Descartes Planner failed to initialize");
            return false;
        }
        return true;
    }

    void setTask(std::vector<geometry_msgs::Pose>& task)
    {
        task_ = task;
    }

    void setPlannerStartState(std::vector<double>& joint_positions)
    {
        // terrible way to change start state of planner
        // moveit::core::RobotStatePtr current_state = move_group_->getCurrentState();
        // moveit::core::RobotState start_state = *current_state;
        // for (auto elem : move_group_->getNamedTargetValues("allZeros"))
        // {
        //     start_state.setJointPositions(elem.first, &elem.second);
        // }
        // move_group_->setStartState(start_state);

        const ros::V_string joint_names = move_group_->getJointNames();
        moveit::core::RobotStatePtr current_state = move_group_->getCurrentState();
        moveit::core::RobotState start_state = *current_state;

        for (std::size_t i = 0; i < joint_names.size(); ++i)
        {
            start_state.setJointPositions(joint_names[i], &joint_positions[i]);
        }
        move_group_->setStartState(start_state);
    }

    bool moveHome(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
    {
        ROS_INFO("Received service call to move to home.");
        // reset start state for planner
        move_group_->setStartState(*move_group_->getCurrentState());

        move_group_->setNamedTarget("home");
        MoveitPlan plan;
        if (move_group_->plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS)
        {
            ROS_INFO("Moving home.");
            move_group_->asyncExecute(plan);
            res.success = true;
            res.message = "Planned a successful path to home!";
            return true;
        }
        else
        {
            res.success = false;
            res.message = "Failed to plan home path.";
            return true;
        }
    }

    bool runDescartesPlanner(std::vector<double> start_joint_positions, std::vector<trajectory_msgs::JointTrajectoryPoint>& ros_trajectory)
    {
        ROS_INFO("Running descartes planner.");

        // convert to format needed by descartes util function
        EigenSTL::vector_Affine3d task_eigen;
        for (auto pose : task_)
        {
            Eigen::Affine3d tmp;
            tf::poseMsgToEigen(pose, tmp);
            task_eigen.push_back(tmp);
        }

        // create toleranced version of task, starting from the current joint state
        std::vector<descartes_core::TrajectoryPtPtr> path = makeDescartesTrajectory(task_eigen);
        descartes_core::TrajectoryPtPtr pt (new descartes_trajectory::JointTrajectoryPt(start_joint_positions));
        path.front() = pt;

        // set timing
        double step = 0.5;
        double t = 0.0;
        for (auto pt : path)
        {
            pt->setTiming( descartes_core::TimingConstraint(t) );
            t += step;
        }

        if (!planner_.planPath(path))
        {
            return false;
        }

        std::vector<descartes_core::TrajectoryPtPtr> result;
        if (!planner_.getPath(result))
        {
            return false;
        }

        // Convert the output trajectory into a ROS-formatted message
        descartes_utilities::toRosJointPoints(*model_, result, 1.0, ros_trajectory);
        return true;
    }

    bool plan(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
    {
        ROS_INFO("Received service call to plan a path.");
        // reset start state for planner
        move_group_->setStartState(*move_group_->getCurrentState());

        //-------------------------------------------------------------------------------------
        move_group_->setPoseTarget(task_[0]);
        MoveitPlan approach_plan;
        if (move_group_->plan(approach_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS)
        {
            ROS_INFO("Approach plan succesfully planned.");
            plans_[0] = approach_plan;
        }
        else
        {
            res.success = false;
            res.message = "Failed to plan approach path.";
            return true;
        }

        //-------------------------------------------------------------------------------------
        // set planner start at end of previous path
        // terrible way to change start state of planner
        std::vector<double> path_end = approach_plan.trajectory_.joint_trajectory.points.back().positions;
        printJointPose(path_end);
        
        // setPlannerStartState(path_end);
        // moveit_msgs::RobotTrajectory trajectory;
        // const double jump_threshold = 0.0;
        // const double eef_step = 0.01;
        // double fraction = move_group_->computeCartesianPath(task_, eef_step, jump_threshold, trajectory);
        // // ROS_INFO_NAMED("tutorial", "Visualizing plan 4 (Cartesian path) (%.2f%% acheived)", fraction * 100.0);

        // if (fraction == 1.0)
        // {
        //     moveit::planning_interface::MoveGroupInterface::Plan task_plan;
        //     task_plan.trajectory_ = trajectory;
        //     plans_[1] = task_plan;
        // }
        // else
        // {
        //     res.success = false;
        //     res.message = "Failed to plan glue path.";
        //     return true;
        // }
        // path_end = trajectory.joint_trajectory.points.back().positions;

        std::vector<trajectory_msgs::JointTrajectoryPoint> ros_trajectory;
        if (runDescartesPlanner(path_end, ros_trajectory))
        {
            //ros_trajectory.front().positions = path_end;
            ROS_INFO("Descartes planning successfull!");
            ROS_INFO_STREAM("Path length: " << ros_trajectory.size() );

            moveit::planning_interface::MoveGroupInterface::Plan task_plan;
            task_plan.trajectory_.joint_trajectory.points = ros_trajectory;
            task_plan.trajectory_.joint_trajectory.joint_names = getJointNames();
            task_plan.trajectory_.joint_trajectory.header.frame_id = "/world";
            plans_[1] = task_plan;
        }
        else {
            res.success = false;
            res.message = "Failed to plan glue path with descartes.";
            return true;
        }
        path_end = ros_trajectory.back().positions;

        //-------------------------------------------------------------------------------------
        
        

        setPlannerStartState(path_end);
        move_group_->setNamedTarget("home");
        MoveitPlan retract_plan;
        if (move_group_->plan(retract_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS)
        {
            ROS_INFO("Retract plan successfully planned.");
            plans_[2] = retract_plan;
        }
        else
        {
            res.success = false;
            res.message = "Failed to plan retract path.";
            return true;
        }

        res.success = true;
        res.message = "Planned a successful path!";
        has_plan_ = true;
        return true;
    }

    bool execute(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
    {
        ROS_INFO("Received service call to execute plan.");

        if (!has_plan_)
        {
            res.success = false;
            res.message = "There is no plan to execute,";
            return true;
        }

        //plans_[0].trajectory_.joint_trajectory.header.stamp = ros::Time::now();
        bool s0 = (move_group_->execute(plans_[0]) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

        plans_[1].trajectory_.joint_trajectory.header.stamp = ros::Time::now();
        bool s1 = (move_group_->execute(plans_[1]) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

        //plans_[2].trajectory_.joint_trajectory.header.stamp = ros::Time::now();
        bool s2 = (move_group_->execute(plans_[2]) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

        // alternative to execute the plans without moveit
        // control_msgs::FollowJointTrajectoryGoal goal;
        // goal.trajectory = plans_[1].trajectory_.joint_trajectory;
        // ac_.sendGoal(goal);
        // ac_.waitForResult();

        if (!s0 || !s1 || !s2)
        {
            res.success = false;
            res.message = "Executing failed.";
            return true;
        }

        res.success = true;
        res.message = "Plans done!";
        return true;
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "gluebot_app");
    ros::NodeHandle nh;

    //-------------------------------------------------------------------------------------
    // Here we will get the pose from halcon and convert it to een eigen pose
    Eigen::Affine3d part_frame =
        Eigen::Affine3d::Identity() * Eigen::AngleAxisd(M_PI_2 - 0.2, Eigen::Vector3d::UnitZ());
    part_frame.translation() << 0.7, -0.1, 0;
    geometry_msgs::Pose part_frame_msg;
    tf::poseEigenToMsg(part_frame, part_frame_msg);

    //-------------------------------------------------------------------------------------
    // fixed task, this can be read from file in future?
    // and then maybe transform after reading, to avoid reading again for each new part_frame
    // auto task = createGlueTask(part_frame);
    auto task = createCircleTaskEigen(part_frame);
    std::vector<geometry_msgs::Pose> task_msg;
    for (auto f : task)
    {
        geometry_msgs::Pose tmp;
        tf::poseEigenToMsg(f, tmp);
        task_msg.push_back(tmp);
    }

    // auto task_msg =  createCircleTask();
    // std::vector<Eigen::Affine3d> task;
    // for (auto pose : task_msg)
    // {
    //     Eigen::Affine3d tmp;
    //     tf::poseMsgToEigen(pose, tmp);
    //     tmp = part_frame * tmp;
    //     task.push_back(tmp);
    // }

    //-------------------------------------------------------------------------------------
    // setup service interface with gui
    GluebotApp app(nh);
    app.setTask(task_msg);

    // visualize stuff
    VisualTools vis;
    vis.visual_tools_->publishAxisLabeled(part_frame, "PART_FRAME", rvt::LARGE);
    vis.visual_tools_->trigger();

    ROS_INFO("Gluebot app node starting");
    ros::AsyncSpinner async_spinner(3);  // Nead more than one thread for difference service calls at once.
    async_spinner.start();

    for (auto f : task)
    {
        vis.publishFrame(f);
    }
    vis.publishWorkobjectMesh(part_frame_msg);

    ros::waitForShutdown();
}