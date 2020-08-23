/*******************************************************************************
* mb_motors.c
*
* Control external Brushed DC Motor Drivers
*
*******************************************************************************/
#include <stdio.h>
#include <rc/motor.h>
#include <rc/model.h>
#include <rc/gpio.h>
#include <rc/pwm.h>
#include <rc/adc.h>
#include "mb_motor.h"
#include "mb_defs.h"

// preposessor macros
#define unlikely(x) __builtin_expect (!!(x), 0)

// global initialized flag
static int init_flag = 0;

/*******************************************************************************
* int mb_motor_init()
* 
* initialize mb_motor with default frequency
*******************************************************************************/
int mb_motor_init(){   
    return mb_motor_init_freq(DEFAULT_PWM_FREQ);
}

/*******************************************************************************
* int mb_motor_init_freq()
* TODO
* set up pwm channels, gpio assignments and make sure motors are left off.
*******************************************************************************/
int mb_motor_init_freq(int pwm_freq_hz){
    init_flag = 1;
    return 0;
}

/*******************************************************************************
* mb_motor_cleanup()
* TODO
* clean up and release memory after using the motors
*******************************************************************************/
int mb_motor_cleanup(){
    if(!init_flag) return 0;
    return 0;
}

#ifdef MRC_VERSION_2v1
/*******************************************************************************
* mb_motor_brake()
* TODO
* allows setting the brake function on the motor drivers
* returns 0 on success, -1 on failure
*******************************************************************************/
int mb_motor_brake(int brake_en){
    
    if(unlikely(!init_flag)){
        fprintf(stderr,"ERROR: trying to enable brake before motors have been initialized\n");
        return -1;
    }
   return 0
}
#endif

/*******************************************************************************
* int mb_disable_motors()
* TODO
* disables PWM output signals
* returns 0 on success
*******************************************************************************/
int mb_motor_disable(){
    
    if(unlikely(!init_flag)){
        fprintf(stderr,"ERROR: trying to disable motors before motors have been initialized\n");
        return -1;
    }

    return 0;
}


/*******************************************************************************
* int mb_motor_set(int motor, double duty)
* TODO
* set a motor direction and power
* motor is from 1 to N, duty is from -1.0 to +1.0
* use defines in mb_defs.h
* returns 0 on success
*******************************************************************************/
int mb_motor_set(int motor, double duty){
    
    if(unlikely(!init_flag)){
        fprintf(stderr,"ERROR: trying to rc_set_motor_all before they have been initialized\n");
        return -1;
    }
    return 0;
}

/*******************************************************************************
* int mb_motor_set_all(double duty)
* TODO
* applies the same duty cycle argument to both motors
*******************************************************************************/
int mb_motor_set_all(double duty){

    if(unlikely(!init_flag)){
        printf("ERROR: trying to rc_set_motor_all before they have been initialized\n");
        return -1;
    }

    return 0;
}


#ifdef MRC_VERSION_2v1
/*******************************************************************************
* int mb_motor_read_current(int motor)
* TODO
* returns the measured current in A
*******************************************************************************/
double mb_motor_read_current(int motor){
    return 0.0;
}
#endif