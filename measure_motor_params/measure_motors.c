/*******************************************************************************
* measure_motor_params.c
*   Template code 
*   Complete this code to automatically measure motor parameters
*   or print out data to be namalyzed in numpy
* 
*******************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <rc/start_stop.h>
#include <rc/encoder_eqep.h>
#include <rc/encoder.h>
#include <rc/adc.h>
#include <rc/time.h>
#include <rc/motor.h>
#include "../common/mb_motor.h"
#include "../common/mb_defs.h"


float enc2meters = (WHEEL_DIAMETER * M_PI) / (GEAR_RATIO * ENCODER_RES);

void test_speed(char filename1[], char filename2[], int direction);

/*******************************************************************************
* int main() 
*
*******************************************************************************/
int main(){

	// make sure another instance isn't running
    // if return value is -3 then a background process is running with
    // higher privaledges and we couldn't kill it, in which case we should
    // not continue or there may be hardware conflicts. If it returned -4
    // then there was an invalid argument that needs to be fixed.
    if(rc_kill_existing_process(2.0)<-2) return -1;

	// start signal handler so we can exit cleanly
    if(rc_enable_signal_handler()==-1){
                fprintf(stderr,"ERROR: failed to start signal handler\n");
                return -1;
    }

#if defined(MRC_VERSION_1v3) || defined(MRC_VERSION_2v1)
    if(mb_motor_init()<0){
        fprintf(stderr,"ERROR: failed to initialze mb_motors\n");
        return -1;
    }
#endif

#if defined(BEAGLEBONE_BLUE)
    if(rc_motor_init()<0){
        fprintf(stderr,"ERROR: failed to initialze motors\n");
        return -1;
    }
#endif

    if(rc_encoder_eqep_init()<0){
        fprintf(stderr,"ERROR: failed to initialze encoders\n");
        return -1;
    }
    
    // make PID file to indicate your project is running
	// due to the check made on the call to rc_kill_existing_process() above
	// we can be fairly confident there is no PID file already and we can
	// make our own safely.
	rc_make_pid_file();

	// done initializing so set state to RUNNING
	rc_set_state(RUNNING);

	if(rc_get_state()==RUNNING){
		rc_nanosleep(1E9); //sleep for 1s
        //TODO: write routine here
	}
	
	// TODO: Plase exit routine here
    test_speed("leftBackward.txt", "rightBackward.txt", -1);

    // remove pid file LAST
	rc_remove_pid_file();   
	return 0;
}

void test_speed(char filename1[], char filename2[], int direction){
    rc_encoder_eqep_init(); // initialize subsystems
    rc_motor_init();
    FILE *f1 = fopen(filename1, "w");
    FILE *f2 = fopen(filename2, "w");
    if (f1 == NULL || f2 == NULL)
    {
        printf("Error opening file!\n");
        exit(1);
    }

    for(float pwm = 0.0; pwm <= 1.0 ;pwm += 0.05)
    {
        rc_motor_set(LEFT_MOTOR, LEFT_MOTOR_POLAR * pwm * direction); // set motor command
        rc_motor_set(RIGHT_MOTOR, RIGHT_MOTOR_POLAR * pwm * direction); // set motor command
        rc_nanosleep(2E8); // give time to reach steady state
        rc_encoder_eqep_write(LEFT_MOTOR,0); // reset encoder
        rc_encoder_eqep_write(RIGHT_MOTOR,0); // reset encoder
        rc_nanosleep(1E9); // measure for 1s
        int LeftTicks = rc_encoder_eqep_read(LEFT_MOTOR); // read encoder
        float LeftSpeed = enc2meters * LeftTicks * LEFT_ENC_POLAR; // convert to speed in rpm or rad/sec
        int RightTicks = rc_encoder_eqep_read(RIGHT_MOTOR); // read encoder
        float RightSpeed = enc2meters * RightTicks * RIGHT_ENC_POLAR; // convert to speed in rpm or rad/sec
        fprintf(f1, "%f,%f\n", pwm, LeftSpeed); // print to terminal
        fprintf(f2, "%f,%f\n", pwm, RightSpeed); // print to terminal
        printf("%f,%f\n", pwm, LeftSpeed);
        printf("%f,%f\n", pwm, RightSpeed);
    }
    fclose(f1);
    fclose(f2);
    rc_motor_set(LEFT_MOTOR,0.0); // cleanup
    rc_motor_set(RIGHT_MOTOR,0.0); // cleanup
    rc_encoder_eqep_cleanup();
    rc_motor_cleanup();
    return;
}