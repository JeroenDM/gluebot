#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <std_srvs/SetBool.h>
#include <eigen_conversions/eigen_msg.h>

#include <moveit/move_group_interface/move_group_interface.h>

#include "gluebot_app/util.h"
#include "gluebot_app/visual_tools.h"

#include <ur5_demo_descartes/ur5_robot_model.h>
#include <descartes_planner/dense_planner.h>
#include <descartes_utilities/ros_conversions.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

#include <gluebot_app/GetPose2D.h>
#include <geometry_msgs/Pose2D.h>

using MoveitPlan = moveit::planning_interface::MoveGroupInterface::Plan;

const double TABLE_Z = -0.146;

class GluebotApp
{
    ros::NodeHandle nh_;
    ros::ServiceServer planServer_, moveHomeServer_, executeServer_;
    ros::ServiceClient glueGunClient_, halconClient_;
    moveit::planning_interface::MoveGroupInterfacePtr move_group_;

    boost::shared_ptr<ur5_demo_descartes::UR5RobotModel> model_;
    descartes_planner::DensePlanner planner_;

    // state variables related to planning
    std::vector<MoveitPlan> plans_;  // {approach plan, glue plan, retract plan}
    bool has_plan_ = false;
    EigenSTL::vector_Affine3d task_;  // this should be in the service call in the future?
    Eigen::Affine3d wobj_pose_;
    

    //actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ac_;
    //  : ac_("joint_trajectory_action", true)

    double glue_speed_ = 0.2;

    std::shared_ptr<VisualTools> vis_;

  public:
    GluebotApp(ros::NodeHandle& nh)
    {
        if (!initDescartes())
            throw std::runtime_error("There was an issue initializing Descartes");

        move_group_.reset(new moveit::planning_interface::MoveGroupInterface("manipulator"));

        planServer_ = nh_.advertiseService("plan_path", &GluebotApp::plan, this);
        moveHomeServer_ = nh_.advertiseService("move_home", &GluebotApp::moveHome, this);
        executeServer_ = nh_.advertiseService("execute_path", &GluebotApp::execute, this);

        glueGunClient_ = nh_.serviceClient<std_srvs::SetBool>("set_glue_gun");
        halconClient_ = nh_.serviceClient<gluebot_app::GetPose2D>("halcon");

        plans_.resize(3);

        updateGlueSpeed();
        // put dummy values in wobj_pose (useful for testing)
        wobj_pose_ = Eigen::Affine3d::Identity();
        wobj_pose_.translation() << 0.6, 0.1, TABLE_Z;

        vis_.reset(new VisualTools());
    }

    void updateGlueSpeed()
    {
        if (nh_.hasParam("glue_speed")) nh_.getParam("glue_speed", glue_speed_);
        else glue_speed_ = 0.2;
        ROS_INFO_STREAM("Glue speed set to: " << glue_speed_);
    }

   bool setWobjPoseFromHalcon()
    {
        Eigen::Affine3d pose = Eigen::Affine3d::Identity();
        gluebot_app::GetPose2D srv;
        if (halconClient_.call(srv))
        {
            ROS_INFO_STREAM("Halcon pose service call succesfull!");
            auto pose_2d = srv.response.pose;
            // rotate around z
            pose *= Eigen::AngleAxisd(pose_2d.theta * M_PI / 180.0, Eigen::Vector3d::UnitZ());
            // position in world frame
            pose.translation() << pose_2d.x, pose_2d.y, TABLE_Z;
            wobj_pose_ = pose;
            return true;
        }
        else
        {
            ROS_ERROR_STREAM("Failed to cal halcon service.");
            return false;
        }
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

    void setTask(EigenSTL::vector_Affine3d& task)
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

    void setGlueGun(bool val)
    {
        std_srvs::SetBool srv;
        srv.request.data = val;
        glueGunClient_.call(srv);
        if (srv.response.success)
        {
            ROS_INFO_STREAM("Glue gun service message: " << srv.response.message);
        }
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

    bool runDescartesPlanner(std::vector<double> start_joint_positions, EigenSTL::vector_Affine3d& task,
                             std::vector<trajectory_msgs::JointTrajectoryPoint>& ros_trajectory)
    {
        ROS_INFO("Running descartes planner.");

        // create toleranced version of task, starting from the current joint state
        std::vector<descartes_core::TrajectoryPtPtr> path = makeDescartesTrajectory(task);
        descartes_core::TrajectoryPtPtr pt(new descartes_trajectory::JointTrajectoryPt(start_joint_positions));
        path.front() = pt;

        // set timing
        double step = 0.2;
        double t = 0.0;
        for (auto pt : path)
        {
            pt->setTiming(descartes_core::TimingConstraint(t));
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
        //-------------------------------------------------------------------------------------
        // get wobj_pose from halcon and transform task
        if (setWobjPoseFromHalcon())
        {
            vis_->publishWorkobjectMesh(wobj_pose_, "part2.stl");
            vis_->visual_tools_->publishAxisLabeled(wobj_pose_, "PART_FRAME", rvt::MEDIUM);
            vis_->visual_tools_->trigger();
        }
        else
        {
            res.success = false;
            res.message = "Failed to get pose from halcon.";
            return true;
        }
        
        EigenSTL::vector_Affine3d transformed_task;
        for (auto pose : task_) transformed_task.push_back(wobj_pose_ * pose);
        for (auto f : transformed_task) vis_->publishFrame(f);

        //-------------------------------------------------------------------------------------
        // reset start state for planner
        move_group_->setStartState(*move_group_->getCurrentState());

        //-------------------------------------------------------------------------------------
        move_group_->setPoseTarget(transformed_task[0]);
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

        std::vector<trajectory_msgs::JointTrajectoryPoint> ros_trajectory;
        if (runDescartesPlanner(path_end, transformed_task, ros_trajectory))
        {
            // ros_trajectory.front().positions = path_end;
            ROS_INFO("Descartes planning successfull!");
            ROS_INFO_STREAM("Path length: " << ros_trajectory.size());

            moveit::planning_interface::MoveGroupInterface::Plan task_plan;
            task_plan.trajectory_.joint_trajectory.points = ros_trajectory;
            task_plan.trajectory_.joint_trajectory.joint_names = getJointNames();
            task_plan.trajectory_.joint_trajectory.header.frame_id = "/world";
            plans_[1] = task_plan;
        }
        else
        {
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
        updateGlueSpeed();

        if (!has_plan_)
        {
            res.success = false;
            res.message = "There is no plan to execute,";
            return true;
        }

        // plans_[0].trajectory_.joint_trajectory.header.stamp = ros::Time::now();
        bool s0 = (move_group_->execute(plans_[0]) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

        plans_[1].trajectory_.joint_trajectory.header.stamp = ros::Time::now();
        setGlueGun(true);
        bool s1 = (move_group_->execute(plans_[1]) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        setGlueGun(false);

        // plans_[2].trajectory_.joint_trajectory.header.stamp = ros::Time::now();
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

    // setup service interface with gui
    GluebotApp app(nh);

    Eigen::Vector3d start, end;
    start << 0.01, -0.03705, 0.015;
    end << 0.09, -0.03705, 0.015;
    auto task = createLine(start, end);

    app.setTask(task);


    ros::AsyncSpinner async_spinner(3);  // Nead more than one thread for difference service calls at once.
    async_spinner.start();


    ros::waitForShutdown();
}