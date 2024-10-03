#include <ros/ros.h>

#include "dmce_msgs/RobotPosition.h"

class DockedPigeon
{
  private:
    ros::Subscriber sub;
    ros::Publisher pub;

  public:
    DockedPigeon(ros::NodeHandle *nh)
    {
      pub = nh->advertise<dmce_msgs::RobotPosition>("/robot0/RobotPosition", 10);
      sub = nh->subscribe("/robot1/RobotPosition", 10, &DockedPigeon::callback, this);
    }

    void callback(const dmce_msgs::RobotPosition& msg)
    {
      dmce_msgs::RobotPosition position;
      position.robotId = 0;
      position.x_position = msg.x_position;
      position.y_position = msg.y_position;
      pub.publish(position);
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "pigeon");
    ros::NodeHandle nodeHandle;
    DockedPigeon dp = DockedPigeon(&nodeHandle);
    ros::spin();

    return 0;
}
