#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <eigen_conversions/eigen_msg.h>

#include <moveit/move_group_interface/move_group_interface.h>

#include "gluebot_app/util.h"

using MoveitPlan = moveit::planning_interface::MoveGroupInterface::Plan;

class GluebotApp
{
    ros::NodeHandle nh_;
    ros::ServiceServer planServer_, moveHomeServer_, executeServer_;
    moveit::planning_interface::MoveGroupInterfacePtr move_group_;
    std::vector<MoveitPlan> plans_; // {approach plan, glue plan, retract plan}
    bool has_plan_ = false;
    std::vector<geometry_msgs::Pose> task_; // this should be in the service call in the future

  public:
    GluebotApp(ros::NodeHandle& nh)
    {
        if (!init())
            std::runtime_error("Could not initialize GluebotApp.");
        
        move_group_.reset(new moveit::planning_interface::MoveGroupInterface("manipulator"));
   
        planServer_ = nh_.advertiseService("plan_path", &GluebotApp::plan, this);
        moveHomeServer_ = nh_.advertiseService("move_home", &GluebotApp::moveHome, this);
        executeServer_ = nh_.advertiseService("execute_path", &GluebotApp::execute, this);

        plans_.resize(3);
    }

    bool init()
    {
        // Initialize planners
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
        std::vector<double> path_end =  approach_plan.trajectory_.joint_trajectory.points.back().positions;
        setPlannerStartState(path_end);

        moveit_msgs::RobotTrajectory trajectory;
        const double jump_threshold = 0.0;
        const double eef_step = 0.01;
        double fraction = move_group_->computeCartesianPath(task_, eef_step, jump_threshold, trajectory);
        // ROS_INFO_NAMED("tutorial", "Visualizing plan 4 (Cartesian path) (%.2f%% acheived)", fraction * 100.0);

        if (fraction == 1.0)
        {
            moveit::planning_interface::MoveGroupInterface::Plan task_plan;
            task_plan.trajectory_ = trajectory;
            plans_[1] = task_plan;
        }
        else
        {
            res.success = false;
            res.message = "Failed to plan glue path.";
            return true;
        }

        //-------------------------------------------------------------------------------------
        path_end = trajectory.joint_trajectory.points.back().positions;
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

        plans_[0].trajectory_.joint_trajectory.header.stamp = ros::Time::now();
        bool s0 = (move_group_->execute(plans_[0]) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

        plans_[1].trajectory_.joint_trajectory.header.stamp = ros::Time::now();
        bool s1 = (move_group_->execute(plans_[1]) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

        plans_[2].trajectory_.joint_trajectory.header.stamp = ros::Time::now();
        bool s2 = (move_group_->execute(plans_[2]) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

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
    Eigen::Affine3d part_frame = Eigen::Affine3d::Identity() * Eigen::AngleAxisd(M_PI_2 - 0.2, Eigen::Vector3d::UnitZ());
    part_frame.translation() << 0.7, -0.1, 0.0;
    geometry_msgs::Pose part_frame_msg;
    tf::poseEigenToMsg(part_frame, part_frame_msg);

    //-------------------------------------------------------------------------------------
    // fixed task, this can be read from file in future?
    // and then maybe transform after reading, to avoid reading again for each new part_frame
    auto task = createGlueTask(part_frame);
    std::vector<geometry_msgs::Pose> task_msg;
    for (auto f : task)
    {
      geometry_msgs::Pose tmp;
      tf::poseEigenToMsg(f, tmp);
      task_msg.push_back(tmp);
    }

    //-------------------------------------------------------------------------------------
    // setup service interface with gui
    GluebotApp app(nh);
    app.setTask(task_msg);

    // visualize stuff
    VisualTools vis;

    ROS_INFO("Gluebot app node starting");
    ros::AsyncSpinner async_spinner(3); // Nead more than one thread for difference service calls at once.
    async_spinner.start();

    for (auto f : task)
    {
        vis.publishFrame(f);
    }
    vis.publishWorkobjectMesh(part_frame_msg);


    ros::waitForShutdown();
}