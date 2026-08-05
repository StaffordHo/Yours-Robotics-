//
// Created by liuyoupei on 18-7-23.
//

#ifndef GMAPPING_TEST_PID_H
#define GMAPPING_TEST_PID_H

#include <iostream>
#include <ros/ros.h>
#include <yours_navigation/pid.h>
//#define PID_TIME_INTERVAL 0.05
#define PID_TIME_INTERVAL 0.1

/*#define MAX_POSITIVE  100

#define MAX_NEGATIVE  (-100)

#define PID_PARAM_TURN_P            0.001f

#define PID_PARAM_TURN_I            0.0f

#define PID_PARAM_TURN_D            0.0f

#define ERROR_LOW                  (-20)
#define ERROR_UPPER                (20)
#define P_OUTPUT_LOW               (-0.2)
#define P_OUTPUT_UPPER              (0.2)
#define I_OUTPUT_LOW               (-0.1)
#define I_OUTPUT_UPPER             (0.1)
#define D_OUTPUT_LOW               (-0.1)
#define D_OUTPUT_UPPER              (0.1)*/
/*
#define OUTPUT_LOW                (-0.3)
#define OUTPUT_UPPER              (0.3)
#define SLOW_OUTPUT_LOW                (-0.15)
#define SLOW_OUTPUT_UPPER              (0.15)
*/
#define OUTPUT_LOW                (-0.3)
#define OUTPUT_UPPER              (0.3)
#define SLOW_OUTPUT_LOW                (-0.65)
#define SLOW_OUTPUT_UPPER              (0.65)

namespace PID
{
    class PID {
        private:
            ros::Publisher pid_debug;
            yours_navigation::pid pid_msg_;
            ros::NodeHandle n_;
            float kp_fdbk;
            float ki_fdbk;
            float kd_fdbk;

            float err;
            float err_old;
            float diff_fdbk;
            float P;
            float I;
            float D_fdbk;
            float PID_out;


            float p_lower_limit;
            float p_uper_limit;
            float I_lower_limit;
            float I_uper_limit;
            float err_lower_limit;
            float err_uper_limit;
            float output_lower_limit;
            float output_uper_limit;
            float D_uper_limit;
            float D_lower_limit;
        public:
            PID(ros::NodeHandle &n);

            float anti_saturation(float currentError, float perOut, float delt_timer);

            void pid_reset();

            float pid_core_exe(float expectVal ,float curVal,float dltTime);

            float pid_inc_core_exe(float cmd ,float fdbk ,float dltT);

            void pid_set_param_pid(float p,float i,float d);

            void pid_set_param_limit(float errlow,    float errupper,
                                      float p_low  ,    float p_upper,
                                      float ilow  ,    float iupper,
                                      float d_lower,   float d_upper,
                                      float outlow,    float outupper
             );
            void pid_set_param_limit_err(float errlow,float errupper);

            void pid_set_param_limit_out(float outlow,float outupper);

            void pid_set_param_limit_D(float outlow,float outupper);

            void pid_set_param_limit_I(float outlow,float outupper);

            float constrain(float val, float min, float max);

            float get_pid_turn_angle_speed(float delt_error);
        };
}


#endif //GMAPPING_TEST_PID_H
