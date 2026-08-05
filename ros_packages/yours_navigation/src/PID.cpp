//
// Created by liuyoupei on 18-7-23.
//

#include "../include/yours_navigation/PID.h"
#include "ros/ros.h"
//#include <follow_path/pid.h>
#include "../include/yours_navigation/macro.h"
namespace PID
{

        PID::PID(ros::NodeHandle &n):P(0),I(0),D_fdbk(0),err_uper_limit(20),err_lower_limit(-20),output_uper_limit(0.5),output_lower_limit(-0.5),
                                     p_uper_limit(0.5),p_lower_limit(-0.5),I_uper_limit(0.5),I_lower_limit(-0.5),D_uper_limit(0.5),D_lower_limit(-0.5)
        {
            n_ = n;
            n_.param<std::float_t >("kp_fdbk", kp_fdbk, 0.025);
            n_.param<std::float_t>("ki_fdbk", ki_fdbk, 0.001);
            n_.param<std::float_t>("err_uper_limit", err_uper_limit, 0.5);
            n_.param<std::float_t >("err_lower_limit", err_lower_limit, -0.5);
            n_.param<std::float_t>("output_uper_limit", output_uper_limit, 0.5);
            n_.param<std::float_t>("output_lower_limit", output_lower_limit, -0.5);
            n_.param<std::float_t>("p_uper_limit", p_uper_limit, 0.5);
            n_.param<std::float_t>("p_lower_limit", p_lower_limit, -0.5);
            n_.param<std::float_t>("I_uper_limit", I_uper_limit, 0.5);
            n_.param<std::float_t>("I_lower_limit", I_lower_limit, -0.5);
            n_.param<std::float_t>("D_uper_limit", D_uper_limit, 0.5);
            n_.param<std::float_t>("D_lower_limit", D_lower_limit, -0.5);

            ros::param::get("~kp_fdbk", kp_fdbk);
            ros::param::get("~ki_fdbk", ki_fdbk);
            ros::param::get("~kd_fdbk", kd_fdbk);
            ros::param::get("~err_uper_limit", err_uper_limit);
            ros::param::get("~err_lower_limit", err_lower_limit);
            ros::param::get("~output_uper_limit", output_uper_limit);
            ros::param::get("~output_lower_limit", output_lower_limit);
            ros::param::get("~p_uper_limit", p_uper_limit);
            ros::param::get("~p_lower_limit", p_lower_limit);
            ros::param::get("~I_uper_limit", I_uper_limit);
            ros::param::get("~I_lower_limit", I_lower_limit);
            ros::param::get("~D_uper_limit", D_uper_limit);
            ros::param::get("~D_lower_limit", D_lower_limit);
            ROS_INFO("kp_fdbk = %lf", kp_fdbk);
            ROS_INFO("ki_fdbk = %lf", ki_fdbk);
            ROS_INFO("kd_fdbk = %lf", kd_fdbk);
            ROS_INFO("err_uper_limit = %lf", err_uper_limit);
            ROS_INFO("err_lower_limit = %lf", err_lower_limit);
            ROS_INFO("output_uper_limit = %lf", output_uper_limit);
            ROS_INFO("output_lower_limit = %lf", output_lower_limit);
            ROS_INFO("p_uper_limit = %lf", p_uper_limit);
            ROS_INFO("p_lower_limit = %lf", p_lower_limit);
            ROS_INFO("I_uper_limit = %lf", I_uper_limit);
            ROS_INFO("I_lower_limit = %lf", I_lower_limit);
            ROS_INFO("D_uper_limit = %lf", D_uper_limit);
            ROS_INFO("D_lower_limit = %lf", D_lower_limit);

            pid_debug = n_.advertise<yours_navigation::pid>("pid_debug_",10);

        }

        float PID::constrain(float val, float min, float max)
        {
            if (val < min)
                return min;
            else if (val > max)
                return max;
            else
                return val;
        }

        void PID::pid_reset()
        {
            D_fdbk             = 0;
            I                  = 0;
            P                  = 0;
            err                = 0;
            err_old            = 0;
            PID_out            = 0;
        }

        void PID::pid_set_param_pid(float kp,float ki,float kd)
        {
            kp_fdbk = kp;
            ki_fdbk = ki;
            kd_fdbk = kd;
        }

        void PID::pid_set_param_limit(float errlow,    float errupper,
                                      float p_low  ,    float p_upper,
                                      float ilow  ,    float iupper,
                                      float d_lower,   float d_upper,
                                      float outlow,    float outupper
                                     )
        {
            err_uper_limit    = errupper;
            err_lower_limit   = errlow;
            output_uper_limit = outupper;
            output_lower_limit= outlow;
            p_uper_limit      = p_upper;
            p_lower_limit     = p_low;
            I_uper_limit      = iupper;
            I_lower_limit     = ilow;
            D_uper_limit      = d_upper;
            D_lower_limit      = d_lower;
        }

        void PID::pid_set_param_limit_err(float errlow,float errupper)
        {
            err_uper_limit    = errupper;
            err_lower_limit   = errlow;
        }
        void PID::pid_set_param_limit_out(float outlow,float outupper)
        {
            output_uper_limit = outupper;
            output_lower_limit= outlow;
        }

        void PID::pid_set_param_limit_D(float outlow,float outupper)
        {
            D_uper_limit=outupper;
            D_lower_limit=  outlow;
        }
        void PID::pid_set_param_limit_I(float outlow,float outupper)
        {
            I_uper_limit= outupper;
            I_lower_limit= outlow;
        }
        float pid_p =  0.03f;
        float pid_i =  0.002;
        float pid_d =  0.004;
        float PID::anti_saturation(float currentError, float perOut, float delt_timer)
        {
            if(perOut >= output_uper_limit)
            {
                if(currentError <= 0)
                {
                    I+= (ki_fdbk * (err *  delt_timer));
                }
            }
            else if(perOut < output_lower_limit)
            {
                if(currentError >= 0)
                {
                    I+= (ki_fdbk * (err *  delt_timer));
                }
            }else{
                I+= (ki_fdbk * (err *  delt_timer));
            }
            return I;
        }

        float PID::pid_core_exe(float cmd ,float fdbk ,float dltT)
        {
            err 		= fdbk - cmd;
            err		= constrain(err,err_lower_limit,err_uper_limit);
            diff_fdbk = (((err - err_old)) / dltT);
            P		    = (kp_fdbk * err);
            P           = constrain(P, p_lower_limit, p_uper_limit);
            I		   = anti_saturation(err, PID_out, dltT);
            I         = constrain(I, I_lower_limit, I_uper_limit);
            D_fdbk    = (kd_fdbk * diff_fdbk);
            D_fdbk    = constrain(D_fdbk,D_lower_limit,D_uper_limit);
            PID_out   = P + I + D_fdbk;
            PID_out   = constrain(PID_out,output_lower_limit,output_uper_limit);
            pid_msg_.pid_out = PID_out;
            pid_msg_.p_out = P;
            pid_msg_.i_out = I;
            pid_msg_.d_out = D_fdbk;
            pid_msg_.currentAngle = cmd;
            pid_msg_.goalAngle = fdbk;
            pid_debug.publish(pid_msg_);
/*            ROS_INFO("err = %f, P = %f, I = %f, D = %f, PID_out = %f",
                    err, P, I, D_fdbk, PID_out);*/
            err_old	= err;

            return PID_out;
        }
        float PID::pid_inc_core_exe(float cmd ,float fdbk ,float dltT)
        {
            float cur_diff;
            err 		= fdbk;
            err		= constrain(err,err_lower_limit,err_uper_limit);
            cur_diff            = ((err  - err_old) / dltT);

            P		    = (kp_fdbk * cur_diff);
            P           = constrain(P, p_lower_limit, p_uper_limit);
            I		    = (ki_fdbk * err);
            I         = constrain(I, I_lower_limit, I_uper_limit);
            D_fdbk    = (kd_fdbk * (cur_diff / diff_fdbk));
            D_fdbk    = constrain(D_fdbk, D_lower_limit, D_uper_limit);
            PID_out  += (P + I + D_fdbk);
            PID_out   = constrain(PID_out,output_lower_limit,output_uper_limit);

            ROS_INFO("err = %f, P = %f, I = %f, D = %f, PID_out = %f",
                    err, P, I, D_fdbk, PID_out);
            err_old	= err;
            cur_diff!=0 ? diff_fdbk = cur_diff: diff_fdbk = diff_fdbk;
            return PID_out;
        }

        float PID::get_pid_turn_angle_speed(float delt_error)
        {
            float tmp = pid_core_exe(0,delt_error,PID_TIME_INTERVAL);
            return tmp;
        }
}
