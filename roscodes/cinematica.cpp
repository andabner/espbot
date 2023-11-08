#include <tf/tf.h>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <stdlib.h>

using namespace std;

double x, y, theta;

void odomCallback(const nav_msgs::Odometry &msg){

    x = msg.pose.pose.position.x;
    y = msg.pose.pose.position.y;
    
    tf::Quaternion q( msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w);
    tf::Matrix3x3 m(q);

    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    theta = yaw;

    //ROS_INFO("Pose [x, y, theta]: [%f %f %f]", msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.orientation.z);

}

int main(int argc, char **argv) {
   //Initializes ROS, and sets up a node
   ros::init(argc, argv, "cinematic_espbot");
   ros::NodeHandle nh;

   //Ceates the publisher, and tells it to publish
   //to the /cmd_vel topic, with a queue size of 100
   ros::Publisher pub=nh.advertise<geometry_msgs::Twist>("/car/cmd_vel", 100);
   //ros::Publisher pub=nh.advertise<geometry_msgs::Vector3>("/car/sensor", 100);

   // Creates the subscriber to subscribe the odometry
    ros::Subscriber odom_sub = nh.subscribe("/odom", 1000, odomCallback);

   //Sets the loop to publish at a rate of 10Hz
   ros::Rate rate(10);

   //Cinematic variables 
   x = 0.0;          // initial pose
   y = 0.0;
   theta = 0.0;
   
   double xf = 0.5;        // final pose
   double yf = 0.0;

   double v = 1.0;
   double w = 0.0;


   //Control variables
   double Dx, Dy;
   double rho, alpha, beta;

   // ganhos do controlador
   double k_r = 1.0, k_a = 10.0, k_b = -0.1;

   geometry_msgs::Twist msg;

   while(ros::ok()) {

      //Calls the callbacks functions
      ros::spinOnce();
   
      // Velocity control
      Dx = xf - x;
      Dy = yf - y;

      // calculo das variaveis polares
      rho = sqrt(pow(Dx,2.0) + pow(Dy,2.0));
      alpha = -theta + atan2(Dy,Dx);
      alpha = atan2(sin(alpha),cos(alpha));  // tratamento de alpha
      beta = -theta-alpha;
      beta = atan2(sin(beta),cos(beta));     // tratamento de beta

      if (rho > 0.1){
         // calculo das velocidades linear e angular
         v = k_r*rho;
         w = k_a*alpha + k_b*beta;
      }else {
         v = 0.0;
         w = 0.0;
      }
      
      cout << " ************************" << endl; 
      cout << "Dx = "  << Dx << "  Dy = " << Dy << endl;
      cout << "rho = " << rho << endl;
      cout << "alpha = " << alpha << endl;
      cout << "beta = " << beta << endl;
      cout << "v = " << v <<  "  w = " << w << endl; 
      cout << "x = " << x <<  "  y = " << y << "  th = " << theta << endl; 

      msg.linear.x = v;
      msg.angular.z = w;
      
      //Publish the message
      pub.publish(msg);
      //pub.publish(sensor);

   
      //Delays until it is time to send another message
      rate.sleep();
   }
}
