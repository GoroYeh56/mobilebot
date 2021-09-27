/*******************************************************************************
* mb_odometry.c
*
* TODO: Implement these functions to add odometry and dead rekoning 
*
*******************************************************************************/

#include "../mobilebot/mobilebot.h"
#include "mb_defs.h"
#include <math.h>

#define PI 3.14159265358979323846

/*******************************************************************************
* mb_initialize_odometry() 
*
* TODO: initialize odometry
* NOTE: you should initialize from Optitrack data if available
*
*******************************************************************************/
void mb_initialize_odometry(mb_odometry_t* mb_odometry, float x, float y, float theta){
    mb_odometry->x = x;
    mb_odometry->y = y;
    mb_odometry->theta = theta;
}


/*******************************************************************************
* mb_update_odometry() 
*
* TODO: calculate odometry from internal variables
*       publish new odometry to lcm ODOMETRY_CHANNEL
*
*******************************************************************************/
void mb_update_odometry(mb_odometry_t* mb_odometry, mb_state_t* mb_state){
    float left_encoder_delta_theta = 2 * PI * mb_state->left_encoder_delta / ENCODER_RES / GEAR_RATIO;
    float right_encoder_delta_theta = 2 * PI * mb_state->right_encoder_delta / ENCODER_RES / GEAR_RATIO;
    float left_delta_s = (WHEEL_DIAMETER/2) * left_encoder_delta_theta;
    float right_delta_s = (WHEEL_DIAMETER/2) * right_encoder_delta_theta;

    float delta_theta = (right_delta_s - left_delta_s) / WHEEL_BASE;
    // delta_theta = mb_clamp_radians(delta_theta);
    float delta_d = (right_delta_s + left_delta_s) / 2;
    float delta_x = delta_d * cos(mb_odometry->theta + delta_theta / 2);
    float delta_y = delta_d * sin(mb_odometry->theta + delta_theta / 2);
    float delta_gyro = mb_state->tb_angles[2] - mb_state->last_yaw;
    // printf("\rleft_dtheta %.4f, right_dtheta %.4f, dth %.4f, d_d %.4f, dx %.4f",left_encoder_delta_theta, right_encoder_delta_theta,delta_theta, delta_d,  delta_x);
    mb_odometry->x = mb_odometry->x + delta_x;
    mb_odometry->y = mb_odometry->y + delta_y;

    // printf("\rodometry.x,y,theta: %.4f, %.4f, %.4f", mb_odometry->x, mb_odometry->y, mb_odometry->theta);

    //update gyro
    float angle_threshold = (0.01 / 180) * PI;
    float angle_diff = mb_angle_diff_radians(delta_theta, delta_gyro);
    // printf("%f\n", fabs(angle_diff));
    // if(fabs(angle_diff) > angle_threshold){
        // mb_odometry->theta = mb_clamp_radians(mb_odometry->theta + delta_gyro);
    // }
    // else{
        mb_odometry->theta = mb_clamp_radians(mb_odometry->theta + delta_theta);
    // }

    // printf("x: %f, y: %f, theta: %f\n", mb_odometry->x, mb_odometry->y, mb_odometry->theta);
}



/*******************************************************************************
* mb_clamp_radians() 
* clamp an angle from -PI to PI
*******************************************************************************/
float mb_clamp_radians(float angle){

    if(angle < -PI)
    {
        for(; angle < -PI; angle += 2.0*PI);
    }
    else if(angle > PI)
    {
        for(; angle > PI; angle -= 2.0*PI);
    }

    return angle;
}


/*******************************************************************************
* mb_angle_diff_radians() 
* computes difference between 2 angles and wraps from -PI to PI
*******************************************************************************/
float mb_angle_diff_radians(float angle1, float angle2){
    float diff = angle2 - angle1;
    while(diff < -PI) diff+=2.0*PI;
    while(diff > PI) diff-=2.0*PI;
    return diff;
}