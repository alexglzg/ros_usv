#include <iostream>
#include <math.h>
#include <eigen3/Eigen/Dense>

#include "ros/ros.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Vector3.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float32.h"

#include "std_msgs/UInt8.h"
#include <ros/console.h>

#include "nav_msgs/Odometry.h"
#include <tf/tf.h>

using namespace Eigen;

class AdaptiveSlidingModeControl
{
public:
  //Thruster outputs
  float starboard_t;
  float port_t;

  //Sensor feedback
  float x=0;
  float y=0;
  float psi=0;
  float u=0;
  float v=0;
  float r=0;

  float integral_step;

  int testing;
  //int arduino;

  //Tracking variables
  float u_d=0;
  float psi_d=0;
  float udot_d=0;
  float r_d=0;
  float rdot_d=0;
  //Condition to restarting initial conditions
  float starting=0;

  //Auxiliry variables
  float e_u0=0;
  float e_psi0=0;
  float edot_psi0=0;
  float e_u=0;
  float e_psi=0;
  float edot_psi=0;
  float s_u=0;
  float s_psi=0;

  float ei_u=0;
  float eidot_u=0;
  float eidot_u_last=0;
  float ei_psi=0;
  float eidot_psi=0;
  float eidot_psi_last=0;

  int sign_u=0;
  int sign_psi=0;
  int sign_u_sm=0;
  int sign_psi_sm=0;
  int sign_su=0;
  int sign_spsi=0;

  //Model pysical parameters
  float Xu;
  float Yv;
  float Nr;
  float X_u_dot;
  float Y_v_dot;
  float N_r_dot;
  float Xuu;
  float Yvv;
  float Nrr;
  float m; //mass
  float Iz; //moment of inertia
  float B; //centerline-to-centerline separation
  float c; //thruster correction factor

  float f_u;
  float g_u;
  float f_psi;
  float g_psi;
  
  float Tx=0;
  float Tz=0;
  float Ka_u=0;
  float Ka_psi=0;
  float Ka_dot_u=0;
  float Ka_dot_psi=0;
  float Ka_dot_last_u=0;
  float Ka_dot_last_psi=0;
  float ua_u=0;
  float ua_psi=0;

  //Controller gains
  float k_u;
  float k_psi;
  float kmin_u;
  float kmin_psi;
  float k2_u;
  float k2_psi;
  float mu_u;
  float mu_psi;
  float alpha_u;
  float alpha_psi;
  float beta_psi;
  float tc_u;
  float tc_psi;
  float q_u;
  float q_psi;
  float p_u;
  float p_psi;

  Matrix2f Ja;
  Vector2f Ju;
  Vector2f vd;
  Vector2f alpha_eidot_xi;
  Vector2f ua_xi;
  Vector2f xi_dot;

  std_msgs::Float32 right_thruster;
  std_msgs::Float32 left_thruster;
  
  std_msgs::Float64 u_gain;
  std_msgs::Float64 psi_gain;

  std_msgs::Float64 u_error;
  std_msgs::Float64 psi_error;

  std_msgs::Float64 u_sigma;
  std_msgs::Float64 psi_sigma;

  geometry_msgs::Pose2D ctrl_input;

  AdaptiveSlidingModeControl()
  {
    //ROS Publishers for each required sensor data
    right_thruster_pub = n.advertise<std_msgs::Float32>("/motors/right_thrust", 10);
    left_thruster_pub = n.advertise<std_msgs::Float32>("/motors/left_thrust", 10);

    surge_gain_pub = n.advertise<std_msgs::Float64>("/usv_control/aitsmc/speed_gain", 10);
    surge_error_pub = n.advertise<std_msgs::Float64>("/usv_control/controller/speed_error", 10);
    surge_sigma_pub = n.advertise<std_msgs::Float64>("/usv_control/aitsmc/speed_sigma", 10);
    yaw_sigma_pub = n.advertise<std_msgs::Float64>("/usv_control/aitsmc/heading_sigma", 10);
    yaw_gain_pub = n.advertise<std_msgs::Float64>("/usv_control/aitsmc/heading_gain", 10);
    yaw_error_pub = n.advertise<std_msgs::Float64>("/usv_control/controller/heading_error", 10);
    control_input_pub = n.advertise<geometry_msgs::Pose2D>("/usv_control/controller/control_input", 10);
    edot_psi_pub = n.advertise<std_msgs::Float64>("/usv_control/aitsmc/debug/edot_psi", 10);

    //ROS Subscribers
    desired_speeds_sub = n.subscribe("/desired_values", 10, &AdaptiveSlidingModeControl::desiredTrajCallback, this);
    desired_speedsdot_sub = n.subscribe("/desired_values_derivative", 10, &AdaptiveSlidingModeControl::desiredTrajDotCallback, this);
    ins_pose_sub = n.subscribe("/vectornav/ins_2d/NED_pose", 10, &AdaptiveSlidingModeControl::insCallback, this);
    local_vel_sub = n.subscribe("/vectornav/ins_2d/local_vel", 10, &AdaptiveSlidingModeControl::velocityCallback, this);
    odom_sub = n.subscribe("/imu/converted_odometry", 10, &AdaptiveSlidingModeControl::odomCallback, this);
    //flag_sub = n.subscribe("/arduino_br/ardumotors/flag", 10, &AdaptiveSlidingModeControl::flagCallback, this);
    //ardu_sub = n.subscribe("arduino", 10, &AdaptiveSlidingModeControl::arduinoCallback, this);


    static const float dk_u = 1.0;
    static const float dk_psi = 1.0;
    static const float dkmin_u = 0.01;
    static const float dkmin_psi = 0.01;
    static const float dk2_u = 0.01;
    static const float dk2_psi = 0.01;
    static const float dmu_u = 0.001;
    static const float dmu_psi = 0.001;
    static const float dalpha_u = 1.0;
    static const float dalpha_psi = 1.0;
    static const float dbeta_psi = 1.0;
    static const float dq_u = 3;
    static const float dq_psi = 3;
    static const float dp_u = 5;
    static const float dp_psi = 5;

    n.param("/aitsmc_heading/k_u", k_u, dk_u);
    n.param("/aitsmc_heading/k_psi", k_psi, dk_psi);
    n.param("/aitsmc_heading/kmin_u", kmin_u, dkmin_u);
    n.param("/aitsmc_heading/kmin_psi", kmin_psi, dkmin_psi);
    n.param("/aitsmc_heading/k2_u", k2_u, dk2_u);
    n.param("/aitsmc_heading/k2_psi", k2_psi, dk2_psi);
    n.param("/aitsmc_heading/mu_u", mu_u, dmu_u);
    n.param("/aitsmc_heading/mu_psi", mu_psi, dmu_psi);
    n.param("/aitsmc_heading/alpha_u", alpha_u, dalpha_u);
    n.param("/aitsmc_heading/alpha_psi", alpha_psi, dalpha_psi);
    n.param("/aitsmc_heading/beta_psi", beta_psi, dbeta_psi);
    n.param("/aitsmc_heading/q_u", q_u, dq_u);
    n.param("/aitsmc_heading/q_psi", q_psi, dq_psi);
    n.param("/aitsmc_heading/p_u", p_u, dp_u);
    n.param("/aitsmc_heading/p_psi", p_psi, dp_psi);

    X_u_dot = -2.25;
    Y_v_dot = -23.13;
    N_r_dot = -2.79;
    Yvv = -99.99;
    Nrr = -3.49;
    m = 30; //mass
    Iz = 4.1; //moment of inertia
    B = 0.41; //centerline-to-centerline separation
    c = 0.78; //thruster correction factor

    g_u = (1 / (m - X_u_dot));
    g_psi = (1 / (Iz - N_r_dot));
    testing = 1;
    //alpha_u = 0.8;
    //alpha_psi = 0.8;
    //beta_psi = 1.0;
  }

  void desiredTrajCallback(const geometry_msgs::Pose2D::ConstPtr& _pd)
  {
    u_d = _pd -> x;
    starting = _pd -> y;
    psi_d = _pd -> theta;
    //ROS_FATAL_STREAM("start = " << starting);
  }

  void desiredTrajDotCallback(const geometry_msgs::Pose2D::ConstPtr& _pdotd)
  {
    udot_d = _pdotd -> x;
    r_d = _pdotd -> theta;
  }

  void insCallback(const geometry_msgs::Pose2D::ConstPtr& _ins)
  {
    x = _ins -> x;
    y = _ins -> y;
    psi = _ins -> theta;
  }

  void velocityCallback(const geometry_msgs::Vector3::ConstPtr& _vel)
  {
    u = _vel -> x;
    v = _vel -> y;
    r = _vel -> z;
  }

  void odomCallback(const nav_msgs::Odometry::ConstPtr& o)
  {
        tf::Quaternion q(
        o->pose.pose.orientation.x,
        o->pose.pose.orientation.y,
        o->pose.pose.orientation.z,
        o->pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    psi = yaw;

    x = o->pose.pose.position.x;
    y = o->pose.pose.position.y;
    
    u = o->twist.twist.linear.x;
    v = o->twist.twist.linear.y;
    r = o->twist.twist.angular.z;
    ROS_INFO("x: %f, y: %f, u: %f v: %f, r: %f psi: %f", x, y, u, v, r, psi);
  }

  /*void flagCallback(const std_msgs::UInt8::ConstPtr& _flag)
  {
    testing = _flag -> data;
  }

  void arduinoCallback(const std_msgs::UInt8::ConstPtr& _ardu)
  {
    arduino = _ardu -> data;
  }*/

  void control()
  {
    if (testing == 1){
    //if (starting > 0){}
      Xu = -25;
      Xuu = 0;
      if (std::abs(u) > 1.2){
        Xu = 64.55;
        Xuu = -70.92;
      }

      Nr = (-0.52)*pow(pow(u,2) + pow(v,2),0.5);

      f_u = (((m - Y_v_dot)*v*r + (Xuu*std::abs(u)*u + Xu*u)) / (m - X_u_dot));
      f_psi = (((-X_u_dot + Y_v_dot)*u*v + (Nrr*std::abs(r)*r + Nr*r)) / (Iz - N_r_dot));

      e_u = u_d - u;
      e_psi = psi_d - psi;
      if (std::abs(e_psi) > 3.141592){
          e_psi = (e_psi/std::abs(e_psi))*(std::abs(e_psi) - 2*3.141592);
      }

      edot_psi = r_d-r;
      std_msgs::Float64 edot_psi_msg;
      edot_psi_msg.data = edot_psi;
      edot_psi_pub.publish(edot_psi_msg);

      if (e_u == 0){
        sign_u = 0;
      }
      else {
        sign_u = copysign(1,e_u);
      }
      if (e_psi == 0){
        sign_psi = 0;
      }
      else {
        sign_psi = copysign(1,e_psi);
      }

      eidot_u = sign_u*pow(std::abs(e_u),q_u/p_u);
      eidot_psi = sign_psi*pow(std::abs(e_psi),q_psi/p_psi);
      ei_u = (integral_step)*(eidot_u + eidot_u_last)/2 + ei_u; //u integral error
      eidot_u_last = eidot_u;
      ei_psi = (integral_step)*(eidot_psi + eidot_psi_last)/2 + ei_psi; //r integral error
      eidot_psi_last = eidot_psi;

      if (starting == 1.0){
          e_u0 = e_u;
          e_psi0 = e_psi;
          edot_psi0 = edot_psi;
          //alpha_u = (pow(std::abs(e_u0),1-q_u/p_u))/(tc_u*(1-q_u/p_u));
          //alpha_psi = (pow(std::abs(e_psi0),1-q_psi/p_psi))/(tc_psi*(1-q_psi/p_psi));
          ei_u = -e_u0/alpha_u;
          ei_psi = -(edot_psi0 + beta_psi*e_psi0)/alpha_psi;
          ROS_FATAL_STREAM("alpha_u = " << alpha_u);
          ROS_FATAL_STREAM("alpha_psi = " << alpha_psi);
          ROS_FATAL_STREAM("e_u0 = " << e_u0);
          ROS_FATAL_STREAM("e_psi0 = " << e_psi0);
      }
      
      s_u = e_u + alpha_u*ei_u;
      s_psi = edot_psi + beta_psi*e_psi + alpha_psi*ei_psi;

      if (Ka_u > kmin_u){
          float signvar = std::abs(s_u) - mu_u;
          if (signvar == 0){
            sign_u_sm = 0;
          }
          else {
            sign_u_sm = copysign(1,signvar);
          }
          Ka_dot_u = k_u * sign_u_sm;
      }
      else{
        Ka_dot_u = kmin_u;
      } 

      Ka_u = (integral_step)*(Ka_dot_u + Ka_dot_last_u)/2 + Ka_u; //integral to get the u adaptative gain
      Ka_dot_last_u = Ka_dot_u;

      if (Ka_psi > kmin_psi){
          float signvar = std::abs(s_psi) - mu_psi;
          if (signvar == 0){
            sign_psi_sm = 0;
          }
          else {
            sign_psi_sm = copysign(1,signvar);
          }
          Ka_dot_psi = k_psi * sign_psi_sm;
      }
      else{
        Ka_dot_psi = kmin_psi;
      }

      Ka_psi = (integral_step)*(Ka_dot_psi + Ka_dot_last_psi)/2 + Ka_psi; //integral to get the r adaptative gain
      Ka_dot_last_psi = Ka_dot_psi;

      if (s_u == 0){
        sign_su = 0;
      }
      else {
        sign_su = copysign(1,s_u);
      }
      ua_u = ((-Ka_u) * pow(std::abs(s_u),0.5) * sign_su) - (k2_u*s_u);

      if (s_psi == 0){
        sign_spsi = 0;
      }
      else {
        sign_spsi = copysign(1,s_psi);
      }
      ua_psi = ((-Ka_psi) * pow(std::abs(s_psi),0.5) * sign_spsi) - (k2_psi*s_psi);

      Tx = (udot_d + (alpha_u * eidot_u) - f_u - ua_u) / g_u; //surge force
      Tz = (rdot_d + (alpha_psi * eidot_psi) + (beta_psi * edot_psi) - f_psi - ua_psi) / g_psi; //yaw rate moment
      
      if (Tx > 73){
        Tx = 73;
      }
      else if (Tx < -60){
        Tx = -60;
      }
      if (Tz > 14){
        Tz = 14;
      }
      else if (Tz < -14){
        Tz = -14;
      }
      
      if (starting == 0){
        Tx = 0;
        Tz = 0;
        u_d = 0;
        r_d = 0;
        Ka_u = kmin_u;
        Ka_dot_last_u = 0;
        Ka_psi = kmin_psi;
        Ka_dot_last_psi = 0;
        ei_u = -e_u/alpha_u;
        eidot_u_last = 0;
        ei_psi = -e_psi/alpha_psi;
        eidot_psi_last = 0;
      }

      port_t = (Tx / 2) + (Tz / B);
      starboard_t = (Tx / (2*c)) - (Tz / (B*c));

      if (starboard_t > 36.5){
        starboard_t = 36.5;
      }
      else if (starboard_t < -30){
        starboard_t = -30;
      }
      if (port_t > 36.5){
        port_t = 36.5;
      }
      else if (port_t < -30){
        port_t = -30;
      }

      //Data publishing
      right_thruster.data = starboard_t;
      left_thruster.data = port_t;
      //ROS_INFO("Right Thruster: %f Left Thruster: %f", starboard_t, port_t);

      u_gain.data = Ka_u;
      psi_gain.data = Ka_psi;

      u_error.data = e_u;
      psi_error.data = e_psi;

      u_sigma.data = s_u;
      psi_sigma.data = s_psi;
    
      ctrl_input.x = Tx;
      ctrl_input.theta = Tz;

      right_thruster_pub.publish(right_thruster);
      left_thruster_pub.publish(left_thruster);

      surge_gain_pub.publish(u_gain);
      surge_error_pub.publish(u_error);
      surge_sigma_pub.publish(u_sigma);
      yaw_gain_pub.publish(psi_gain);
      yaw_error_pub.publish(psi_error);
      yaw_sigma_pub.publish(psi_sigma);
      
      control_input_pub.publish(ctrl_input);
    }
  }

private:
  ros::NodeHandle n;

  ros::Publisher right_thruster_pub;
  ros::Publisher left_thruster_pub;
  ros::Publisher surge_gain_pub;
  ros::Publisher surge_error_pub;
  ros::Publisher surge_sigma_pub;
  ros::Publisher yaw_sigma_pub;
  ros::Publisher yaw_gain_pub;
  ros::Publisher yaw_error_pub;
  ros::Publisher control_input_pub;
  ros::Publisher edot_psi_pub;

  ros::Subscriber desired_speeds_sub;
  ros::Subscriber desired_speedsdot_sub;
  ros::Subscriber ins_pose_sub;
  ros::Subscriber local_vel_sub;
  ros::Subscriber odom_sub;
  //ros::Subscriber flag_sub;
  //ros::Subscriber ardu_sub;
};

// Main
int main(int argc, char *argv[])
{
  ros::init(argc, argv, "aitsmc_heading");
  AdaptiveSlidingModeControl adaptiveSlidingModeControl;
  adaptiveSlidingModeControl.integral_step = 0.01;
  int rate = 100;
  ros::Rate loop_rate(rate);

  while (ros::ok())
  {
    adaptiveSlidingModeControl.control();
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}