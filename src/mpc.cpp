#include <iostream>
#include <math.h>
#include <eigen3/Eigen/Dense>

#include "ros/ros.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Vector3.h"
#include "nav_msgs/Odometry.h"
#include <tf/tf.h>
#include "std_msgs/Float64.h"
#include "std_msgs/UInt8.h"
#include <ros/console.h>
#include <ocp/StageOCPApplication.hpp>

using namespace fatrop;
class MPC
{
    public:

        //Sensor feedback
        float x = 0.0;
        float y = 0.0;
        float psi = 0.0;
        float u = 0.0;
        float v = 0.0;
        float r = 0.0;

        float u_d;
        float r_d;
        float u_d_dot;
        float r_d_dot;
        float starting_flag=0.0;
        float counter_start = 0.0;

        float s = 0.0;

        const double x0[7] = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0};

        geometry_msgs::Pose2D desired_speeds;
        geometry_msgs::Pose2D desired_accelerations;

        std::shared_ptr<StageOCPApplication> app;

        MPC()
        {

            desired_speeds_pub = n.advertise<geometry_msgs::Pose2D>("/desired_speeds", 1);
            desired_accelerations_pub = n.advertise<geometry_msgs::Pose2D>("/desired_speeds_derivative", 1);

            odom_sub = n.subscribe("/imu/odometry", 10, &MPC::odomCallback, this);
            ins_pose_sub = n.subscribe("/vectornav/ins_2d/NED_pose", 1000, &MPC::insCallback, this);
            local_vel_sub = n.subscribe("/vectornav/ins_2d/local_vel", 1000, &MPC::velocityCallback, this);

            app = StageOCPApplicationBuilder::FromRockitInterface("/ws/foobar/casadi_codegen.so",
            "/ws/foobar/casadi_codegen.json");
            
            ///  no dynamic memory allocation
            // app->Optimize();
            // ///  retrieve solution
            // app->LastStageOCPSolution().Eval(eval_expression, u0_result);
            app->SetOption("tol", 1e-3);
            app->SetOption("mu_init", 1e-3);
            app->SetOption("bound_push", 1e-7); // all *_bound_push variables
            app->SetOption("warm_start_mult_bound_push", 1e-7); 
            app->SetOption("accept_every_trial_step", false);
            app->SetOption("iterative_refinement", false); // fast_step_computation
            app->SetOption("warm_start_init_point", true);

        }

        void odomCallback(const nav_msgs::Odometry::ConstPtr& o)
        {
            x = o->pose.pose.position.x;
            y = o->pose.pose.position.y;

            tf::Quaternion q(
                o->pose.pose.orientation.x,
                o->pose.pose.orientation.y,
                o->pose.pose.orientation.z,
                o->pose.pose.orientation.w);
            tf::Matrix3x3 m(q);
            double roll, pitch, yaw;
            m.getRPY(roll, pitch, yaw);
            psi = yaw;

            u = o->twist.twist.linear.x;
            v = o->twist.twist.linear.y;
            r = o->twist.twist.angular.z;
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

        void mpc()
        {   
            // TODO: figure out why an extra iteration
            
            auto param = app->GetParameterSetter("X_0");
            const double x0[7] = {x, y, psi, u, v, r, s};
            param->SetValue(x0);

            app->Optimize();

            auto eval_expression = app->GetExpression("Urdot")->at_t0();
            std::vector<double> u0_result(eval_expression -> Size());

            auto eval_r = app->GetExpression("r")->at_tk(1);
            std::vector<double> r_result(eval_r -> Size());

            auto eval_s = app->GetExpression("s_min")->at_t0();
            std::vector<double> s_result(eval_s -> Size());

            app->LastStageOCPSolution().Eval(eval_expression, u0_result);
            app->LastStageOCPSolution().Eval(eval_r, r_result);
            app->LastStageOCPSolution().Eval(eval_s, s_result);
            app->SetInitial(app->LastStageOCPSolution());

            starting_flag = 1.0;
            counter_start += 1.0;
            if (counter_start >= 50.0){
                starting_flag = 2.0;
            }

            desired_speeds.x = 0.5;
            desired_speeds.y = starting_flag;
            desired_speeds.theta = r_result[0];
            s = s_result[0];

            desired_speeds_pub.publish(desired_speeds);
        }

    private:
        ros::NodeHandle n;

        ros::Publisher desired_speeds_pub;
        ros::Publisher desired_accelerations_pub;
        
        ros::Subscriber ins_pose_sub;
        ros::Subscriber local_vel_sub;
        ros::Subscriber odom_sub;

};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mpc");
    MPC mpc;

    ros::Rate loop_rate(100);

    while (ros::ok())
    {
        mpc.mpc();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
