#include "faraday_cartesian_state/faraday_cartesian_state.h"

static std::vector<std::string> getDefaultFaradayJointNames()
{
    std::vector<std::string> names;
    names.reserve(5);
    names.push_back("joint_1");
    names.push_back("joint_2");
    names.push_back("joint_3");
    names.push_back("joint_4");
    names.push_back("joint_5");

    return names;
}

int main(int argc,char** argv)
{
    ros::init(argc,argv,"faraday_cartesian_state");
    ros::NodeHandle nh;

    std::vector<std::string> joint_names;
    joint_names.reserve(5);
    if(!nh.getParam("controller_joint_names",joint_names))
    {
        joint_names=getDefaultFaradayJointNames();
        ROS_INFO("Faraday Cartesian State: loading default joint names [joint_1 ... joint_5]");
    }
    else
    {
        ROS_INFO("Faraday Cartesian State: loaded joint names from param server");
    }
    
    faraday_cartesian_state::FaradayCartesianState pub("joint_states",joint_names);

    ros::spin();

    return 0;
}
