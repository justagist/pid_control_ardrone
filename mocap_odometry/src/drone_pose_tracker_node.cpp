#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <string>
#include <sstream>

#include <cmath>

const std::string drone_frame_name("drone");


 void poseCallback(std_msgs::Float32 data){
   static tf::TransformBroadcaster br;
   tf::Transform transform;
   double theta = data.data;
   double x =  cos(theta);
   double y = sin(theta);
   transform.setOrigin( tf::Vector3(x, y, 0.0) );
   tf::Quaternion q;
   q.setRPY(0, theta, 0.0);
   transform.setRotation(q);
   br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", drone_frame_name));
 }

int main(int argc, char** argv){
  ros::init(argc, argv, "drone_pose_tracker_tf_broadcaster");

  ros::NodeHandle node;

  ros::Subscriber sub = node.subscribe(drone_frame_name+"/pose", 10, &poseCallback);

  ros::Publisher chatter_pub = node.advertise<std_msgs::String>("chatter", 1000);

  ros::Rate loop_rate(10);

  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
  int count = 0;
  double x = 0.0;
  while (ros::ok())
  {
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
    std_msgs::String msg;

    std::stringstream ss;
    ss << "hello world " << count;
    msg.data = ss.str();

    //ROS_INFO("%s", msg.data.c_str());

    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */
    chatter_pub.publish(msg);
    x = fmod(x+0.05, M_PI);
    std_msgs::Float32 xmsg;
    xmsg.data = x;
    poseCallback(xmsg);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }

  //ros::spin();
  return 0;
};
