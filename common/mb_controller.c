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



/*
    Within mb_controller.c, create two controllers, 
    1. control each wheel velocity using your open loop calibration
    2. control each wheel’s speed using PID controllers.  

    The controllers should keep the wheels moving the desired speed 
    so the robot moves in as straight a line as possible without 
    heading correction but likely this won’t be perfect.  

    These controllers will be a starting point for a more sophisticated controller.

    You can modify the python scripts 'drive_test.py' and 'drive_square.py' 
    to generate velocity commands to test your controller.

*/


int mb_initialize_controller(){
    mb_load_controller_config();
    left_vel_pid_filter = rc_filter_empty();
    right_vel_pid_filter = rc_filter_empty();
    rc_filter_pid(&left_vel_pid_filter, pid_params.lkp, pid_params.lki, pid_params.lkd, pid_params.dFilterHz,DT );
    rc_filter_pid(&right_vel_pid_filter,  pid_params.rkp, pid_params.rki, pid_params.rkd, pid_params.dFilterHz,DT );
    rc_filter_enable_saturation(&left_vel_pid_filter, -1.0, 1.0);
    rc_filter_enable_saturation(&right_vel_pid_filter, -1.0, 1.0);
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

    /*
        *****
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
        *****
    */

    // pid_params = malloc(sizeof(pid_parameters));
    fscanf(file, "lkp: %f\nlki: %f\nlkd: %f\nrkp: %f\nrki: %f\nrkd: %f\ndFilterHz: %f", 
        &pid_params.lkp,
        &pid_params.lki,
        &pid_params.lkd,
        &pid_params.rkp,
        &pid_params.rki,
        &pid_params.rkd,        
        &pid_params.dFilterHz
        );

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


// float Kp, Ki, Kd;
// float Kp = pid_params.kp;
// float Kd = pid_params.kd;
// float Ki = pid_params.ki;

   
/* PID algorithm:
    // x : left / right velocity
    // xdot: left/right acc
    // float Kp, Kd, Ki;
    // float left_up, left_ui, left_ud, right_up, right_ui, right_ud;
    // float left_speed, right_speed;
    // float left_acc=0, right_acc=0, left_acc_last=0, right_acc_last=0;
    // float left_ui_last, right_ui_last;
    // float left_speed_last, right_speed_last;
    // float left_cmd, right_cmd;

    // float left_err, left_err_d, left_err_i;
    // float right_err, right_err_d, right_err_i;
    // float left_last_err, right_last_err;
*/

void mb_controller_openloop(mb_state_t* mb_state, mb_setpoints_t* mb_setpoints){

  /*
    Left motor :1.09*x + -0.0399
    Right motor:1.08*x + -0.0171
  */ 
  float v = mb_setpoints->fwd_velocity;
  float w = mb_setpoints->turn_velocity;
  float left_vel_cmd = v - w * (WHEEL_BASE/2);
  float right_vel_cmd = v + w * (WHEEL_BASE/2);
  mb_state->left_cmd  =  (left_vel_cmd+0.0399)/1.09;
  mb_state->right_cmd  = (right_vel_cmd+0.0171)/1.08;

}


int mb_controller_update(mb_state_t* mb_state, mb_setpoints_t* mb_setpoints){  

  // simply march it!
  // TODO : 1.5 Covert fwd, turn to left/right wheel velocity.
  float v = mb_setpoints->fwd_velocity;
  float w = mb_setpoints->turn_velocity;
  // float left_vel_cmd = v;
  // float right_vel_cmd = v;
  float left_vel_cmd = v - w * (WHEEL_BASE/2);
  float right_vel_cmd = v + w * (WHEEL_BASE/2);
  float left_error = left_vel_cmd - mb_state->left_velocity;
  float right_error = right_vel_cmd - mb_state->right_velocity;
  
  //Convert the 'speed' command to corresponding 'pwm' signals
  /*
    Left motor :1.09*x + -0.0399
    Right motor:1.08*x + -0.0171
  */
 
  // Feed Forward
  mb_state->left_cmd  =  (left_vel_cmd+0.0399)/1.09;
  mb_state->right_cmd  = (right_vel_cmd+0.0171)/1.08;

  mb_state->left_cmd  += rc_filter_march( &left_vel_pid_filter, left_error); //+0.0399)/1.09;
  mb_state->right_cmd  += rc_filter_march( &right_vel_pid_filter, right_error); //+0.0171)/1.08;

/*
    Kp = pid_params.kp;
    Kd = pid_params.kd;
    Ki = pid_params.ki;
    // int dt = time_offset;
    int dt = 1;
    // 1. Convert setpoints linear.x & angular.z to left/right speed.
    float x = mb_setpoints->fwd_velocity;  // linear x
    float z = mb_setpoints->turn_velocity; // angular z

    // Compute Setpoints
    float left_setpoint = (x- z*WHEEL_BASE/2) / (WHEEL_DIAMETER/2);
    float right_setpoint = (x+ z*WHEEL_BASE/2) / (WHEEL_DIAMETER/2);

    // 2. Get current speed (state)
    left_speed = mb_state->left_velocity;
    right_speed = mb_state -> right_velocity;

    // Compute 'speed' error.
    left_err =  left_setpoint - left_speed;
    right_err = right_setpoint - right_speed;

    // 3. Compute x_dot (acceleration)
    // x_dot = (x - x_k-1 ) / dt
    left_acc = (left_speed - left_speed_last)/dt;
    right_acc = (right_speed - right_speed_last)/dt;

    // 4. Compute up, ui and ud
    // up = Kp * err
    left_up  = Kp * left_err;
    right_up = Kp * right_err;

    // ui = ui_k-1 + Ki*(err)
    left_ui = left_ui_last + Ki * left_err;
    right_ui = right_ui_last + Ki * right_err;

    // ud = Kd* ( x_dot - x_dotk-1)
    left_ud = Kd * (left_acc - left_acc_last);
    right_ud = Kd * (right_acc - right_acc_last);
   
    // U = up + ui + ud
    left_cmd = left_up + left_ui + left_ud;
    right_cmd = right_up + right_ui + right_ud;

    // 5. Send commad ! (Write state.)

    // Print command out first
    // printf("left_cmd right_cmd: (%.3f, %.3f)\r", left_cmd, right_cmd);
    mb_state->right_cmd = right_cmd;
    mb_state->left_cmd = left_cmd;


    ///////////// 6. UPDATE /////////////
    // update 'last' speed for next-iter left/right acc
    left_speed_last = left_speed;
    right_speed_last = right_speed;

    // update 'last' ui for next-iter ui
    left_ui_last = left_ui;
    right_ui_last = right_ui;

    // update 'last' acc for next-iter ud
    left_acc_last = left_acc;
    right_acc_last = right_acc;

    // // update last_err
    // left_last_err = left_err;
    // right_last_err = right_err;
*/  
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

    // Free filters
    rc_filter_free(&left_vel_pid_filter);
    rc_filter_free(&right_vel_pid_filter);

    return 0;
}
