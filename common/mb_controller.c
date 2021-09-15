#include "../mobilebot/mobilebot.h"

/*******************************************************************************
* int mb_initialize()
*
* this initializes all the PID controllers from the configuration file
* you can use this as is or modify it if you want a different format
*
* return 0 on success
*
*******************************************************************************/

int mb_initialize_controller(){
    // mb_load_controller_config();
    left_wheel_velocity_pid = rc_filter_empty();
    right_wheel_velocity_pid = rc_filter_empty();
    rc_filter_pid(&left_wheel_velocity_pid, 1.0 ,0.0, 0.0, 0.1, DT);
    rc_filter_pid(&right_wheel_velocity_pid, 1.0 ,0.0, 0.0, 0.1, DT);
    rc_filter_enable_saturation(&left_wheel_velocity_pid, -1.0, 1.0);
    rc_filter_enable_saturation(&right_wheel_velocity_pid, -1.0, 1.0);
    return 0;
}

/*******************************************************************************
* int mb_load_controller_config()
*
* this provides a basic configuration load routine
* you can use this as is or modify it if you want a different format
*
* return 0 on success
*
*******************************************************************************/


int mb_load_controller_config(){
    FILE* file = fopen(CFG_PATH, "r");
    if (file == NULL){
        printf("Error opening pid.cfg\n");
    }

/******
*
*   Example of loading a line from .cfg file:
*
*    fscanf(file, "%f %f %f %f", 
*        &pid_params.kp,
*        &pid_params.ki,
*        &pid_params.kd,
*        &pid_params.dFilterHz
*        );
*
******/

    fclose(file);
    return 0;
}

/*******************************************************************************
* int mb_controller_update()
* 
* TODO: Write your PID controller here
* take inputs from the global mb_state
* write outputs to the global mb_state
*
* return 0 on success
*
*******************************************************************************/

int mb_controller_update(mb_state_t* mb_state, mb_setpoints_t* mb_setpoints){  
    float left_error = mb_setpoints->fwd_velocity - mb_state->left_velocity ;
    float right_error = mb_setpoints->fwd_velocity - mb_state->right_velocity;
    mb_state->left_cmd = rc_filter_march(&left_wheel_velocity_pid, left_error);
    mb_state->right_cmd = rc_filter_march(&right_wheel_velocity_pid, right_error);
    return 0;
}


/*******************************************************************************
* int mb_destroy_controller()
* 
* TODO: Free all resources associated with your controller
*
* return 0 on success
*
*******************************************************************************/

int mb_destroy_controller(){
    rc_filter_free(&left_wheel_velocity_pid);
    rc_filter_free(&right_wheel_velocity_pid);
    return 0;
}
