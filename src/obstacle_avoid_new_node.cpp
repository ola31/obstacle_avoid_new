#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "std_msgs/UInt16.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
#include "tf/tf.h"

#define MAX_RANGE 5
#define LEFT_ANG 90

#define RAD2DEG 57.2958
#define YAW_OFFSET 90

#define DIS_CONTINUE_THRESHOLD 0.5

int version = 2;

float Kp_lindar = 0.4;
float Kp_angular = 0.4;

float theta_y = 0.0;
float theta_p = 0.0;

float linear_x = 0.0;
float angular_z = 0.0;

int phase =0;
int count_1 = 0;

float yd_laserscan_arr[720]={0};
float laserscan_arr[180]={0};
float laserscan_arr_shift[179]={0};
float delta_arr[179]={0};
int discon_angle = 0;
float discon_distance = 100;

float pre_angular_z = 0.0;

void scan_Callback(const sensor_msgs::LaserScan::ConstPtr& msg){
/*
  int i = 0;
  for(i=0;i<180;i++){
    if(isinf(msg->ranges[i+LEFT_ANG])){
      laserscan_arr[i] = MAX_RANGE;
    }
    else laserscan_arr[i] = msg->ranges[i+LEFT_ANG];
  }


  */

if(version == 1){
int i=0;

  yd_laserscan_arr[0]=msg->ranges[1];
    for(i=1;i<719;i++){
      if(msg->ranges[i]==0){
          yd_laserscan_arr[i]=yd_laserscan_arr[i-1];
      }
      else{
        yd_laserscan_arr[i]=msg->ranges[i];
      }
    }


    for(i=0;i<90;i++){
      if(isinf(msg->ranges[i*2])){
        laserscan_arr[89-i] = MAX_RANGE;
      }
      else laserscan_arr[89-i]=yd_laserscan_arr[i*2];
    }
    for(i=269;i<359;i++){
      if(isinf(msg->ranges[i*2])){
        laserscan_arr[89+(359-i)] = MAX_RANGE;
      }
      laserscan_arr[89+(359-i)]=yd_laserscan_arr[i*2];
    }



  for(i=0;i<179;i++){
    laserscan_arr_shift[i] = laserscan_arr[i+1];
    delta_arr[i] = abs(laserscan_arr_shift[i]-laserscan_arr[i]);
  }
  float dist=0.0;
  for(i=0;i<179;i++){
    if(delta_arr[i]>DIS_CONTINUE_THRESHOLD){
      dist = (laserscan_arr_shift[i]>laserscan_arr[i]) ? laserscan_arr[i]:laserscan_arr_shift[i];
      if(dist < discon_distance){
        discon_distance = dist;
        discon_angle = i;
      }
    }
  }
  if(discon_angle<90 && discon_distance!=100){
    if(laserscan_arr[discon_angle-5]>laserscan_arr[discon_angle+5]){
      angular_z= 0.02;
      linear_x = 0.1;
      ROS_INFO("1");
    }
    else{
      angular_z = -1*Kp_angular*(laserscan_arr[180-discon_angle]-discon_distance);
      linear_x = Kp_lindar*0.5*(discon_distance +laserscan_arr[90+(90-discon_angle)]);
      ROS_INFO("2");
    }
  }
  else if(discon_angle>90&&discon_distance!=100){
    if(laserscan_arr[discon_angle-5]<laserscan_arr[discon_angle+5]){
      angular_z= -0.02;
      linear_x = 0.1;
      ROS_INFO("3");
    }
    else{
      angular_z = Kp_angular*(laserscan_arr[180-discon_angle]-discon_distance);
      linear_x = Kp_lindar*0.5*(discon_distance +laserscan_arr[90+(90-discon_angle)]);
      ROS_INFO("4");
    }
  }
  else if(discon_angle == 90 && discon_distance!=100){
    if(laserscan_arr[discon_angle-5]>laserscan_arr[discon_angle+5]){
      angular_z= 0.05;
      linear_x = 0.1;
      ROS_INFO("5");
    }
    else{
      angular_z= -0.05;
      linear_x = 0.1;
      ROS_INFO("6");
    }
  }
  else if(discon_distance==100){
    ROS_WARN("dd..");
  }
  ROS_INFO("discontinue dist %f, angle : %d",discon_distance,discon_angle);
  discon_distance = 100;

  float angular_z_new = angular_z*0.6 + pre_angular_z*0.4;
  pre_angular_z = angular_z_new;


}
  if(version ==2){

    int i=0;
    float laser_min = 10;

      yd_laserscan_arr[0]=msg->ranges[1];
        for(i=1;i<719;i++){
          if(msg->ranges[i]==0){
              yd_laserscan_arr[i]=yd_laserscan_arr[i-1];
          }
          else{
            yd_laserscan_arr[i]=msg->ranges[i];
          }
        }


        for(i=0;i<90;i++){
          if(isinf(yd_laserscan_arr[i*2])){
            laserscan_arr[89-i] = MAX_RANGE;
          }
          else laserscan_arr[89-i]=yd_laserscan_arr[i*2];
        }
        for(i=269;i<359;i++){
          if(isinf(yd_laserscan_arr[i*2])){
            laserscan_arr[89+(359-i)] = MAX_RANGE;
          }
          laserscan_arr[89+(359-i)]=yd_laserscan_arr[i*2];
        }




        for(i=0;i<80;i++){

          if(i<30){
            laserscan_arr[i]*=1.6;
          }
          if(i>60){
            laserscan_arr[i]*=0.5;
          }

          if(laserscan_arr[i]<laser_min){
            laser_min = laserscan_arr[i];
          }
        }
        linear_x = 0.2;
        angular_z = -2*(0.6-laser_min);
       /* if(laserscan_arr[30]>2){
          angular_z = 1*0.5*(laserscan_arr[5]-laserscan_arr[175]);
          ROS_INFO("testing1");

        }*/
        if(laserscan_arr[5]>2 || laserscan_arr[175]>2){
          angular_z = 0.0;
          linear_x = 0.0;
          ROS_INFO("testing2");

        }
        ROS_INFO("laser_min : %f",laser_min);

        angular_z = 0.3*angular_z + 0.7*pre_angular_z;
        pre_angular_z = angular_z;

  }
  if(version ==3){

    int i=0;
    float left_area=0;
    float right_area=0;

      yd_laserscan_arr[0]=msg->ranges[1];
        for(i=1;i<719;i++){
          if(msg->ranges[i]==0){
              yd_laserscan_arr[i]=yd_laserscan_arr[i-1];
          }
          else{
            yd_laserscan_arr[i]=msg->ranges[i];
          }
        }


        for(i=0;i<90;i++){
          if(isinf(yd_laserscan_arr[i*2])){
            laserscan_arr[89-i] = MAX_RANGE;
          }
          else laserscan_arr[89-i]=yd_laserscan_arr[i*2];
        }
        for(i=269;i<359;i++){
          if(isinf(yd_laserscan_arr[i*2])){
            laserscan_arr[89+(359-i)] = MAX_RANGE;
          }
          else laserscan_arr[89+(359-i)]=yd_laserscan_arr[i*2];
        }



        for(i=0;i<180;i++){

          if(i<30){
            laserscan_arr[i]*=0.5;
          }
          if(i>=30 && i<60){
            laserscan_arr[i]*=1;
          }
          if(i>=60 && i<90){
            laserscan_arr[i]*=1.5;
          }
          if(i>=90 && i<120){
            laserscan_arr[i]*=1.5;
          }
          if(i>=120 && i<150){
            laserscan_arr[i]*=1;
          }
          if(i>=150 && i<180){
            laserscan_arr[i]*=0.5;
          }

          for(i=0;i<90;i++){
            left_area+=laserscan_arr[i];
          }
          for(i=90;i<179;i++){
            right_area+=laserscan_arr[i];
          }

        }
        linear_x = 0.3;
        angular_z =-0.07*(right_area - left_area);
       /* if(laserscan_arr[30]>2){
          angular_z = 1*0.5*(laserscan_arr[5]-laserscan_arr[175]);
          ROS_INFO("testing1");

        }*/
        if(laserscan_arr[5]>2 || laserscan_arr[175]>2){
          angular_z = 0.0;
          linear_x = 0.0;
          ROS_INFO("testing2");

        }
        ROS_INFO("angularz = %f",angular_z);
        ROS_INFO("right area = %f",right_area);
        ROS_INFO("error = %f",right_area-left_area);
        angular_z = 0.3*angular_z + 0.7*pre_angular_z;
        pre_angular_z = angular_z;

  }
}

void imuCallback(const sensor_msgs::Imu::ConstPtr msg){
/*
  tf::Quaternion q(
    msg->orientation.x,
    msg->orientation.y,
    msg->orientation.z,
    msg->orientation.w
    );
    tf::Matrix3x3 m(q);
    double roll,pitch,yaw;
    m.getRPY(roll,pitch,yaw);
    theta_y = yaw*RAD2DEG+YAW_OFFSET;
    theta_p = -1*pitch*RAD2DEG;

    if(theta_y < 0){
      theta_y +=360;
    }
   // ROS_INFO("yaw : %f",theta_y);
   // ROS_INFO("pitch : %f",theta_p);

    if(phase == 0){
      ROS_INFO("phase0");
      linear_x = 0.5;
      angular_z = 0.0;

      int count = 0;
      for(int j = 80; j <100; j++){
       if(laserscan_arr[j] <0.5){
         count += 1;
        }
      }
      if (count > 15)
      {
        linear_x = 0.0;
        angular_z = 0.0;
        phase ++;
      }
    }

    if(phase == 1){
      ROS_INFO("phase1");
      linear_x = 0.7;
      angular_z = 0.05*(90-theta_y);

      if(theta_p>15){
        phase++;
      }
    }

    if(phase == 2){
      ROS_INFO("phase2");

      linear_x = 0.7;
      angular_z = 0.05*(90-theta_y);

      if(theta_p < 15){
        count_1++;
        if(count_1 > 10){
          ROS_INFO("counting");
          linear_x = 0.0;
          angular_z = 0.0;
          phase++;
        }
      }
    }

    if(phase == 3){
      ROS_INFO("phase3");
    }
*/

}
int main(int argc, char **argv)
{
  ros::init(argc, argv, "obstacle_avoid_new_node");
  ros::NodeHandle nh;

  //ros::Publisher yaw_angle_pub = nh.advertise<std_msgs::Float32>("yaw_angle", 1000);
  //ros::Subscriber imu_sub = nh.subscribe("imu",1,imuCallback);
  ros::Subscriber scan_sub = nh.subscribe("/scan", 1000, scan_Callback);
  ros::Publisher cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
  ros::Publisher status_pub = nh.advertise<std_msgs::UInt16>("/obstacle_state", 1000);


  ros::Rate loop_rate(60);
  while (ros::ok())
  {
    std_msgs::UInt16 status_msg;
    status_msg.data = 1;
    status_pub.publish(status_msg);

    std_msgs::Float32 msg;
    geometry_msgs::Twist msg1;

    msg.data = theta_y;

    msg1.linear.x = linear_x;
    msg1.angular.z = angular_z;

    //yaw_angle_pub.publish(msg);
    cmd_vel_pub.publish(msg1);

    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}
