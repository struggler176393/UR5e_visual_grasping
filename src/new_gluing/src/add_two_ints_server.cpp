#include "ros/ros.h"
#include "new_gluing/AddTwoInts.h"

// bool add(new_gluing::AddTwoInts::Request  &req,
//          new_gluing::AddTwoInts::Response &res)
// {
//   res.sum = req.a + req.b;
//   ROS_INFO("request: x=%ld, y=%ld", (long int)req.a, (long int)req.b);
//   ROS_INFO("sending back response: [%ld]", (long int)res.sum);
//   return true;
// }

// int main(int argc, char **argv)
// {
//   ros::init(argc, argv, "add_two_ints_server");
//   ros::NodeHandle n;

//   ros::ServiceServer service = n.advertiseService("add_two_ints", add);
//   ROS_INFO("Ready to add two ints.");
//   ros::spin();

//   return 0;
// }



// class作为回调函数
class AddTwo
{
public:
  bool add(new_gluing::AddTwoInts::Request& req,
           new_gluing::AddTwoInts::Response& res);
};


bool AddTwo::add(new_gluing::AddTwoInts::Request  &req,
         new_gluing::AddTwoInts::Response &res)
{
  res.sum = req.a + req.b;
  ROS_INFO("request: x=%ld, y=%ld", (long int)req.a, (long int)req.b);
  ROS_INFO("sending back response: [%ld]", (long int)res.sum);
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "add_two_ints_server");
  ros::NodeHandle n;
  AddTwo a;

  ros::ServiceServer service = n.advertiseService("add_two_ints", &AddTwo::add, &a);
  ROS_INFO("Ready to add two ints.");
  ros::spin();

  return 0;
}