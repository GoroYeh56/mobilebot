/*******************************************************************************
*                 Mobilebot Template Code for MBot/MBot-Mini
*                           pgaskell@umich.edu
*       
*    This code is a template for completing a fully featured
*    mobile robot controller
*   
*    Functions that need completing are marked with "TODO:"
*
*******************************************************************************/

bin/			      : Binaries folder
mobilebot/mobilebot.c/.h      : Main setup and threads
test_motors/test_motors.c/.h  : Program to test motor implementation
meas..params/meas..params.c/.h: Program to measure motor parameters
common/mb_controller.c/.h     : Contoller for manual and autonomous nav
common/mb_defs.h              : Define hardware config
common/mb_odometry.c/.h	      : Odometry and dead reckoning 
lcmtypes/                     : lcmtypes for Mobilebot
java/                         : java build folder for lcmtypes for lcm-spy
setenv.sh                     : sets up java PATH variables for lcm-spy (run with: source setenv.sh)
