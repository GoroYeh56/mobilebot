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

void test_speed(int motor_ch, float du, float dtime_s);

/*******************************************************************************
* int main() 
*
*******************************************************************************/





// #define GEAR_RATIO 78
#define ENC_CNTS 20
#define PI 3.14159265358979323846
// #define WHEEL_DIAMETER 0.083 // 8.33 cm

// Global speed numpy arrays
float speed_left[21];
float speed_right[21];

void Write_Output_File(float* speed_left, float* speed_right, char* filename);

int main(int argc, char** argv){

    if(argc <2){
        printf("Wrong input format.\n");
        printf("Type: sudo ./measure_motors <output_filename.txt>\n");
        return 0;
    }
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
    printf("Ready to Calibrate...\n");
    printf("Calibrating Left Motor...\n");
    test_speed(1, 0.5, 0.5);
    printf("Calibrating Right Motor...\n");
    test_speed(2, 0.5, 0.5);
    printf("Done Calibration.\n");

    // char filename[30] = "1.1.txt";

    // char filename[30] = argv[1]; 
    // char* 
    Write_Output_File(speed_left, speed_right, argv[1]);


    printf("\n ==== Profiling Results: =======\n");
    for(int i=0; i<=20; i++){
        printf("PWM: %f, Left: %f, Right: %f\n", i*0.05, speed_left[i] ,speed_right[i]);
    }
    // remove pid file LAST
	rc_remove_pid_file();   
	return 0;
}


void Write_Output_File(float* speed_left, float* speed_right, char* filename){
    FILE *f = fopen(filename, "w");
    if (f == NULL)
    {
        printf("Error opening file!\n");
        exit(1);
    }

    /* print some text */
    // const char *text = "Duty Cycles v.s. Motor Speed (m/s)";
    // fprintf(f, "Some text: %s\n", text);

    // /* print integers and floats */
    // int i = 1;
    // float pi= 3.1415927;
    // fprintf(f, "Integer: %d, float: %f\n", i, pi);

    for(int i=0; i<21; i++){
        fprintf(f,"%f,%f\n",speed_left[i], speed_right[i]);
        // fprintf(f, "Duty cycle: %f, Left: %f, Right %f\n", i*0.05, speed_left[i], speed_right[i]);
    }


    /* printing single chatacters */
    // char c = 'A';
    // fprintf(f, "A character: %c\n", c);

    fclose(f);    
}

// Call every second
float ticks_to_speed(float number_of_ticks){
    float speed = 0;
    float wheel_rps = 0;
    float mps = 0;
    speed = abs(number_of_ticks);                 // counts per second
    wheel_rps = speed / (GEAR_RATIO*ENC_CNTS);    // wheel revs per second
    mps = wheel_rps * WHEEL_DIAMETER * PI;        // (m/s) or the robot (circumference = 2*PI*r)
    return mps;
    // return speed;
}

void test_speed(int motor_ch, float duty, float dtime_s){
    // run duty cycle 'duty' with time dtime_s

    int index = 0;
    rc_encoder_eqep_init();
    rc_motor_init();
    for (float pwm=0.0; pwm<= 1.1; pwm += 0.05){
        rc_motor_set(motor_ch, pwm);
        rc_nanosleep(2E8);
        rc_encoder_eqep_write(motor_ch, 0); // reset encoder counts
        rc_nanosleep(1E9); // sleep 1 second.
        int ticks = rc_encoder_eqep_read(motor_ch); // read channel one
        float speed = ticks_to_speed(ticks);
        if(motor_ch == 1){
            // left 
            speed_left[index++] = speed;
        }
        else{
            speed_right[index++] = speed;
        }
        printf("Duty Cycle: %f, Speed: %f (m/s)\n", pwm, speed);
        // printf("Duty Cycle: %f, Speed: %f (counts/second)\n", pwm, speed);
    }
    rc_motor_set(motor_ch, 0.0);
    rc_encoder_eqep_cleanup();
    rc_motor_cleanup();
    // record these data into a numpy array (a one-d float array)
}

/*
    TODO:
    1. Check gear ratio: test encoder program : 1635/78~20.96 =>
        - One revolution for encoder: 20 counts
        - 20 * 78 = 1560 counts per wheel revolution!
    2. Output .txt file and use python or excel to plot curve

*/


/*
measure the 'loaded steady state wheel speed' vs. 'input PWM'.  
You will likely want to have the program output data for analysis with 'numpy'.  

You will need to f'ill in parameters of the robot in the file common/mb_defs.h '
which includes compiler #defines for parameters like 
(1) wheel diameter
(2) gear ratio
(3) encoder resolution 
then use these in your code.  

You can find these values by consulting the MBot-Mini documentation or 
by measuring parameters in the lab.


- Report - :
1. motor speed vs. pwm

Q:
How much variation is there in the calibration functions?
What do you think is the source of variation?


*/
