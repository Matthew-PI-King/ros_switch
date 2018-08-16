#include<ros/ros.h>
#include <geometry_msgs/Twist.h>
class repeater
{
public:
  repeater(int n)
  :nh(), r(n)
  {
  myPub = nh.advertise <geometry_msgs::Twist> ("pioneer1/cmd_vel_ratio", 50, true);
  sub = nh.subscribe("slow_msgs", 5, &repeater::my_callback, this);
  std::cout<<"Finished Initialization!"<<std::endl;
  };

  void run()
  {
  while(ros::ok())
    {
      //std::cout<<"Looping..."<<std::endl;
      myPub.publish(latest_msg);
      ros::spinOnce();
      r.sleep();
    }
  };
	
private:

  void my_callback( const geometry_msgs::Twist &msg)
  {
  //std::cout<<"Recieved!"<<std::endl<<std::endl<<std::endl<<std::endl<<std::endl;
  latest_msg.linear.x=msg.linear.x;
  latest_msg.angular.z=msg.angular.z;
  }

  geometry_msgs::Twist latest_msg;
  ros::NodeHandle nh;
  ros::Rate r;
  ros::Publisher myPub;
  ros::Subscriber sub;
};


int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "repeater_node");

  repeater myrepeater(10); //10HZ
  myrepeater.run();

  return 0;
}
