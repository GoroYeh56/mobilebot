/*******************************************************************************
* drive_simple.c
*
* 
*******************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/types.h>

#include <lcm/lcm.h>
#include "../lcmtypes/mbot_encoder_t.h"
#include "../lcmtypes/mbot_motor_pwm_t.h"

#include <rc/start_stop.h>
#include <rc/encoder_eqep.h>
#include <rc/time.h>
#include <rc/motor.h>

//LCM
lcm_t * lcm;
#define MBOT_ENCODER_CHANNEL        "MBOT_ENCODERS"
#define MBOT_MOTOR_PWM_CHANNEL  "MBOT_MOTOR_PWM"
//global watchdog_timer to cut off motors if no lcm messages recieved
float watchdog_timer;
//functions
void motor_pwm_handler(const lcm_recv_buf_t *rbuf, const char *channel,
                          const mbot_motor_pwm_t *msg, void *user);
void publish_encoder_msg();

/*******************************************************************************
* int main() 
*
*******************************************************************************/
int main(){
	// make sure another instance isn't running
    if(rc_kill_existing_process(2.0)<-2) return -1;

	// start signal handler so we can exit cleanly
    if(rc_enable_signal_handler()==-1){
                fprintf(stderr,"ERROR: failed to start signal handler\n");
                return -1;
    }

	if(rc_motor_init()<0){
        fprintf(stderr,"ERROR: failed to initialze motors\n");
        return -1;
    }

    lcm = lcm_create("udpm://239.255.76.67:7667?ttl=1");

    // make PID file to indicate your project is running
	// due to the check made on the call to rc_kill_existing_process() above
	// we can be fairly confident there is no PID file already and we can
	// make our own safely.
	rc_make_pid_file();

    //subscribe to pwm commands
    mbot_motor_pwm_t_subscribe(lcm, 
    							   MBOT_MOTOR_PWM_CHANNEL, 
    							   motor_pwm_handler, 
    							   NULL);

	// done initializing so set state to RUNNING

    rc_encoder_eqep_init();

	rc_set_state(RUNNING);
    
    watchdog_timer = 0.0;
    printf("Running...\n");
	while(rc_get_state()==RUNNING){
        watchdog_timer += 0.01;
        if(watchdog_timer >= 0.25)
        {
            rc_motor_set(1,0.0);
            rc_motor_set(2,0.0);
            printf("timeout...\n");
        }
		// define a timeout (for erroring out) and the delay time
        lcm_handle_timeout(lcm, 1);
        rc_nanosleep(1E9 / 100); //handle at 10Hz
	}
    rc_motor_cleanup();
    rc_encoder_eqep_cleanup();
    lcm_destroy(lcm);
	rc_remove_pid_file();   // remove pid file LAST
	return 0;
}

/*******************************************************************************
*  motor_pwm_handler()
*
*  sets motor PWMS from incoming lcm message
*
*******************************************************************************/
void motor_pwm_handler(const lcm_recv_buf_t *rbuf, const char *channel,
                          const mbot_motor_pwm_t *msg, void *user){
    printf("CMD: %f | %f \r",msg->left_motor_pwm,msg->right_motor_pwm);
    rc_motor_set(1,msg->left_motor_pwm);
    rc_motor_set(2,msg->right_motor_pwm);
    publish_encoder_msg();
    watchdog_timer = 0.0;
}

/*******************************************************************************
* void publish_encoder_msg()
*
* publishes LCM message of encoder reading
* 
*******************************************************************************/
void publish_encoder_msg(){
    mbot_encoder_t encoder_msg;

    encoder_msg.utime = rc_nanos_since_epoch();
    encoder_msg.left_delta = 0;
    encoder_msg.right_delta = 0;
    encoder_msg.leftticks = rc_encoder_eqep_read(1);
    encoder_msg.rightticks = rc_encoder_eqep_read(2);
    
    mbot_encoder_t_publish(lcm, MBOT_ENCODER_CHANNEL, &encoder_msg);
}