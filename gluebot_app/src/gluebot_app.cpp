#include <ros/ros.h>

#include <std_srvs/Trigger.h>

class GluebotApp
{
    ros::NodeHandle nh_;
    ros::ServiceServer server_;
public:

GluebotApp(ros::NodeHandle& nh)
{
    if(!init())
        std::runtime_error("Could not initialize GluebotApp.");
    
    server_ = nh_.advertiseService("plan_path", &GluebotApp::plan, this);
}

bool init()
{
    // Initialize planners
    return true;
}

bool plan(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
{
    ROS_INFO("Planning a path service.");
    res.success = true;
    res.message = "Planned a successful path!";
    return true;
}

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "gluebot_app");

  ros::NodeHandle nh;
  GluebotApp planner (nh);

  ROS_INFO("Gluebot app node starting");
  ros::spin();
}