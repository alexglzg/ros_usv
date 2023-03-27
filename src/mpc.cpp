#include <iostream>
#include <math.h>
#include <eigen3/Eigen/Dense>

#include "ros/ros.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Vector3.h"
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

        float x_amplitude = 3.0;
        float x_start = 1.0;
        float x_freq = 3*3.141592/40;
        float y_multiplier = -1.0;
        float y_start = 1.0;
        float x_d;
        float y_d;

        const double x0[7] = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0};

        geometry_msgs::Pose2D desired_speeds;
        geometry_msgs::Pose2D desired_accelerations;
        geometry_msgs::Pose2D desired_position;

        std::shared_ptr<StageOCPApplication> app;

        MPC()
        {

            desired_speeds_pub = n.advertise<geometry_msgs::Pose2D>("/desired_speeds", 10);
            desired_accelerations_pub = n.advertise<geometry_msgs::Pose2D>("/desired_speeds_derivative", 10);
            desired_position_pub = n.advertise<geometry_msgs::Pose2D>("/desired_position", 10);

            ins_pose_sub = n.subscribe("/vectornav/ins_2d/NED_pose", 10, &MPC::insCallback, this);
            local_vel_sub = n.subscribe("/vectornav/ins_2d/local_vel", 10, &MPC::velocityCallback, this);

            app = StageOCPApplicationBuilder::FromRockitInterface("/home/alex/Documents/rockit/examples/ASV_examples/foobar/casadi_codegen.so",
            "/home/alex/Documents/rockit/examples/ASV_examples/foobar/casadi_codegen.json");
             
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

        float desired_x(float s_var)
        {
            return x_amplitude*std::sin(s_var*x_freq) + x_start;
        }

        float desired_y(float s_var)
        {
            return y_multiplier*s_var +  y_start;
        }

        void mpc()
        {   
            // TODO: figure out why an extra iteration
            
            auto param = app->GetParameterSetter("X_0");
            const double x0[7] = {x, y, psi, u, v, r, s};
            param->SetValue(x0);

            app->Optimize();

            auto eval_expression = app->GetExprEvaluator("Urdot")->at_t0();
            std::vector<double> u0_result(eval_expression -> Size());

            auto eval_r = app->GetExprEvaluator("r")->at_tk(1);
            std::vector<double> r_result(eval_r -> Size());

            auto eval_s = app->GetExprEvaluator("s_min")->at_t0();
            std::vector<double> s_result(eval_s -> Size());

            app->LastStageOCPSolution().Eval(eval_expression, u0_result);
            app->LastStageOCPSolution().Eval(eval_r, r_result);
            app->LastStageOCPSolution().Eval(eval_s, s_result);
            app->SetInitial(app->LastStageOCPSolution());

            starting_flag = 1.0;
            counter_start += 1.0;
            if (counter_start >= 5.0){
                starting_flag = 2.0;
            }

            desired_speeds.x = 0.5;
            desired_speeds.y = starting_flag;
            desired_speeds.theta = r_result[0];
            s = s_result[0];

            x_d = desired_x(s);
            y_d = desired_y(s);
            desired_position.x = x_d;
            desired_position.y = y_d;

            desired_speeds_pub.publish(desired_speeds);
            desired_position_pub.publish(desired_position);
        }

    private:
        ros::NodeHandle n;

        ros::Publisher desired_speeds_pub;
        ros::Publisher desired_accelerations_pub;
        ros::Publisher desired_position_pub;
        
        ros::Subscriber ins_pose_sub;
        ros::Subscriber local_vel_sub;
          

};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mpc");
    MPC mpc;

    ros::Rate loop_rate(100);
    ros::Rate start_delay(0.2);
    start_delay.sleep(); //Five second delay to start

    while (ros::ok())
    {
        mpc.mpc();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
