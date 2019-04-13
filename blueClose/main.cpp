#include "robot-config.h"
using namespace vex::this_thread;



/*---------------------------------------------------------------------------*/
/*                                                                           */
/*        Description: Competition template for VCS VEX V5                   */
/*                                                                           */
/*---------------------------------------------------------------------------*/

//Creates a competition object that allows access to Competition methods.
vex::competition    Competition;

/*user control globals*/
//orientation variables
bool g_reverse_orientation = false;
bool g_release_orient = true;
bool g_release_intake = true;
bool g_released_wheelLockButton = true;
bool g_halfspeed = false;
int g_deadband = 20;


//atn variables
double g_distance = 4;
double g_degrees  = 90;

//atn constants
double g_wheel_radius = 2.04173228;
double g_robot_radius = 6.78125;
double g_ticks_per_rev = 900;

//user control flags
bool wheelLock = false;
bool shootHold = false;
//function definitions

//FuNcTiOn DeClArAtIoNs
void atn_KOBE();

//Setter Functions
void set_left_drive_speed(int speed) {
    if (speed >=0){
        drive_LFT.spin(vex::directionType::fwd, speed, vex::velocityUnits::pct);
        drive_LFTBCK.spin(vex::directionType::fwd, speed, vex::velocityUnits::pct);

    }
    else{
        speed *= -1;
        drive_LFT.spin(vex::directionType::rev, speed, vex::velocityUnits::pct);
        drive_LFTBCK.spin(vex::directionType::rev, speed, vex::velocityUnits::pct);
    }
}
void set_right_drive_speed(int speed) {
    if(speed >=0){
        drive_RT.spin(vex::directionType::fwd, speed, vex::velocityUnits::pct);
        drive_RTBCK.spin(vex::directionType::fwd, speed, vex::velocityUnits::pct);

    }
    else{
        speed *= -1;
        drive_RT.spin(vex::directionType::rev, speed, vex::velocityUnits::pct);
        drive_RTBCK.spin(vex::directionType::rev, speed, vex::velocityUnits::pct);
    }
}

void set_motor_intake(int speed) {
    if (speed >= 0){
	    intake_motor.spin(vex::directionType::fwd, speed, vex::velocityUnits::pct);
    }
    else{
        speed = abs(speed);
        intake_motor.spin(vex::directionType::rev, speed, vex::velocityUnits::pct);
    }
}
void set_PID_Vel_accum(int dist_target, float* kP, float* kI, float* kD, float* vel_accum){//sets PID and lagging ticks velocity accumulator for controlling drive and lagging ticks respectively
	//checks passed args to make sure they aren't pointing to NULL

	if (dist_target <= 12){//CHANGE THESE, THESE VALUES ARE A PLACEHOLDER UNTIL TESTING IS DONE
		*kP = .035;
		*kI = 0.0001;
		*kD = 0;
		*vel_accum = 1.2;

	}
	else if(dist_target <= 24){
		*kP = .035;
		*kI = 0.0001;
		*kD = 0;
		*vel_accum = 1.2;

	}
	else{
		*kP = .035;
		*kI = 0.000055;
		*kD = 0;
		*vel_accum = 1.2;
	}
}

/*-----------------------------
-----------------------------
-----------------------------
USER FUNCTIONS
-----------------------------
-----------------------------
*/
void usr_drive()
{
    if (g_halfspeed == true){
        
        if (g_reverse_orientation == false){
            if(abs(vexRT.Axis3.value()) > g_deadband)
                set_left_drive_speed(vexRT.Axis3.value()*0.35);
            else
                set_left_drive_speed(0);
          if(abs(vexRT.Axis2.value()) > g_deadband)
                set_right_drive_speed(vexRT.Axis2.value()*0.35);
            else
                set_right_drive_speed(0);
            return;
        }
        else{
            if(abs(vexRT.Axis2.value()) > g_deadband)
                set_left_drive_speed(-0.35*(vexRT.Axis2.value()));
            else
                set_left_drive_speed(0);
          if(abs(vexRT.Axis3.value()) > g_deadband)
                set_right_drive_speed(-0.35*(vexRT.Axis3.value()));
            else
                set_right_drive_speed(0);
            return;

        }
        
    }
    else if(g_halfspeed == false){
        
        if (g_reverse_orientation == false){
            if(abs(vexRT.Axis3.value()) > g_deadband)
                set_left_drive_speed(vexRT.Axis3.value()*0.5);
            else
                set_left_drive_speed(0);
          if(abs(vexRT.Axis2.value()) > g_deadband)
                set_right_drive_speed(vexRT.Axis2.value()*0.5);
            else
                set_right_drive_speed(0);
            return;
        }
        else{
            if(abs(vexRT.Axis2.value()) > g_deadband)
                set_left_drive_speed(-0.5*(vexRT.Axis2.value()));
            else
                set_left_drive_speed(0);
          if(abs(vexRT.Axis3.value()) > g_deadband)
                set_right_drive_speed(-0.5*(vexRT.Axis3.value()));
            else
                set_right_drive_speed(0);
            return;

        }
	//more motors attached later
    }
}


void usr_halfspeed(){
   	if (vexRT.ButtonB.pressing()){//checks if button has been g_released prior to changing
		if (g_halfspeed == true){
            g_halfspeed = false;
        }
        else{
            g_halfspeed =true;
        }
	}
    while(vexRT.ButtonB.pressing()){
        
    }
}

void usr_change_orientation()
{//change back to front, right front wheel always right stick
	if (!(vexRT.ButtonY.pressing())){//checks if button has been g_released prior to changing
		g_release_orient = true;
	}
	if ((vexRT.ButtonY.pressing()) && (g_release_orient == true) ){//front wheels become back
		if (g_reverse_orientation == false){
			g_reverse_orientation = true;
		}
		else{
			g_reverse_orientation = false;
		}
		g_release_orient = false;

	}

	return;



}

void usr_activate_intake()
{
	if (!(vexRT.ButtonR1.pressing()) && !(vexRT.ButtonR2.pressing())){//checks if buttons have been released prior to changing
		g_release_intake = true;
	}
	//change intake to on, reverse, or off
	if ((vexRT.ButtonR1.pressing()) && (g_release_intake == true) ){//either turn motor forward or turn off
      if(intake_motor.velocity(vex::velocityUnits::pct) >= 50 || intake_motor.velocity(vex::velocityUnits::pct) <= -50){
        set_motor_intake(0);
      }
       else{
         set_motor_intake(60);

       }
       g_release_intake = false;
     }
    else if ((vexRT.ButtonR2.pressing()) && (g_release_intake == true) ){//either turn motor backward or turn off
      if(intake_motor.velocity(vex::velocityUnits::pct) >= 50 || intake_motor.velocity(vex::velocityUnits::pct) <= -50){ //running at 120/127 in either direction
        set_motor_intake(0);
      }
       else{
         set_motor_intake(-60);
       }
    g_release_intake = false;
	}

	return;
}

void usr_shoot(){//DRAW BACK AT FULL SPEED
    
    if(vexRT.ButtonL2.pressing()){
        drive_RT.stop(vex::brakeType::hold);
	    drive_RTBCK.stop(vex::brakeType::hold);
	    drive_LFT.stop(vex::brakeType::hold);
	    drive_LFTBCK.stop(vex::brakeType::hold);
		atn_KOBE();
	} else {
        shooter_motor.spin(vex::directionType::fwd, 0, vex::velocityUnits::pct);
        shooter_motor2.spin(vex::directionType::rev, 0, vex::velocityUnits::pct);
    }
}


void usr_lockwheels(){//DRAW BACK AT FULL SPEED
    if (vexRT.ButtonA.pressing() && !wheelLock)
    {
        wheelLock = true;
        while(vexRT.ButtonA.pressing());
    }
    if (vexRT.ButtonA.pressing() && wheelLock)
    {
        wheelLock = false;
        while(vexRT.ButtonA.pressing());
    }
    if (wheelLock)
    {
        drive_RT.stop(vex::brakeType::hold);
		drive_RTBCK.stop(vex::brakeType::hold);
		drive_LFT.stop(vex::brakeType::hold);
		drive_LFTBCK.stop(vex::brakeType::hold);
    }
}

void usr_flipper(){ 
    if(vexRT.ButtonX.pressing() && flipper_motor.rotation(vex::rotationUnits::deg) < 80)
    {
        flipper_motor.spin(vex::directionType::fwd, 20, vex::velocityUnits::pct);
        flipper_motor2.spin(vex::directionType::fwd, 20, vex::velocityUnits::pct);
    }
    else
    {
        if(flipper_motor.rotation(vex::rotationUnits::deg) > 40)
        {
            flipper_motor.spin(vex::directionType::rev, 8, vex::velocityUnits::pct);
            flipper_motor2.spin(vex::directionType::rev, 8, vex::velocityUnits::pct);
        }
        else
        {
            int flipper_motor_vel = flipper_motor.rotation(vex::rotationUnits::deg) * 0.2;
            flipper_motor.spin(vex::directionType::rev, flipper_motor_vel, vex::velocityUnits::pct);
            flipper_motor2.spin(vex::directionType::rev, flipper_motor_vel, vex::velocityUnits::pct);
        }
    }
}


/*-----------------------------
-----------------------------
-----------------------------
AUTON FUNCTIONS
-----------------------------
-----------------------------
*/


void atn_drive(double distance, int speed = 49){
    //use g_distance
    int destination_ticks = int((distance *360)/(2*M_PI*2.04173228)); //convert target distance to ticks
    
    if (speed >= 50){//if a user sets the speed above 50, the auton max spec, allow that speed;otherwise, set based on distance
 
    }
    else if(distance <= 4){
        speed = 10;
    }
    else if(distance <= 8){
        speed = 25;
    }
    else{
        speed = 30;
    }

    
    drive_LFTBCK.startRotateFor(destination_ticks,vex::rotationUnits::deg,speed,vex::velocityUnits::pct);
    drive_RTBCK.startRotateFor(destination_ticks,vex::rotationUnits::deg,speed,vex::velocityUnits::pct);
    drive_LFT.startRotateFor(destination_ticks,vex::rotationUnits::deg,speed,vex::velocityUnits::pct);
    drive_RT.startRotateFor(destination_ticks,vex::rotationUnits::deg,speed,vex::velocityUnits::pct);
    while(drive_RT.isSpinning()  || drive_RTBCK.isSpinning() || 
          drive_LFT.isSpinning() || drive_LFTBCK.isSpinning()) {
        
    }
}

void atn_turn(double degrees){
    double arc_length = degrees / 360.0 * 2 * M_PI * g_robot_radius;
    int dest_ticks = int((arc_length *360)/(2*M_PI*2.04173228));
    
    drive_LFT.startRotateFor(-dest_ticks,vex::rotationUnits::deg,20,vex::velocityUnits::pct);
    drive_RT.startRotateFor(dest_ticks,vex::rotationUnits::deg,20,vex::velocityUnits::pct);
    drive_LFTBCK.startRotateFor(-dest_ticks,vex::rotationUnits::deg,20,vex::velocityUnits::pct);
    drive_RTBCK.startRotateFor(dest_ticks,vex::rotationUnits::deg,20,vex::velocityUnits::pct);
    
    while(drive_RT.isSpinning()  || drive_RTBCK.isSpinning() || 
          drive_LFT.isSpinning() || drive_LFTBCK.isSpinning()) {
        //Wait for end of move
    }
    
    drive_LFTBCK.stop(vex::brakeType::hold);
    drive_LFT.stop(vex::brakeType::hold);
    drive_RTBCK.stop(vex::brakeType::hold);
    drive_RT.stop(vex::brakeType::hold);
}

void atn_KOBE(){
    
    shooter_motor.startRotateFor(vex::directionType::rev, 360, vex::rotationUnits::deg, 100, vex::velocityUnits::pct);
    shooter_motor2.startRotateFor(vex::directionType::fwd, 360, vex::rotationUnits::deg, 100, vex::velocityUnits::pct);
     while(shooter_motor.isSpinning()  || shooter_motor2.isSpinning() ) {
        
    }
}

void atn_activate_intake(){
    set_motor_intake(-60);
}
void atn_deactivate_intake(){
    set_motor_intake(0);
}

//RETURNS - int dist_traveled: the distance in inches that the robot moved to complete the action
double atn_intake(){
    //Record Starting Encoder Tick
    drive_RT.resetRotation();
    
    set_motor_intake(-60);
    set_right_drive_speed(20);
    set_left_drive_speed(20);
    
    while(intake_empty.value()){
        //Wait for ball to enter robot
    }
    
    int deg_traveled = abs(drive_RT.rotation(vex::rotationUnits::deg));
    double dist_traveled = deg_traveled * 2 * M_PI * g_wheel_radius / 360;
    
    set_right_drive_speed(0);
    set_left_drive_speed(0);
    
    while(shooter_empty.value()) {
        //Wait for ball to be loaded into shooter
    }
    
    set_motor_intake(0);
    
    return dist_traveled;
}

void atn_mount() {
    atn_drive(-25, 127);
}

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the cortex has been powered on and    */ 
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

void pre_auton( void ) {
	/*while (gyro1.isCalibrating()){//wait for calibration to finish use startCalibration() to restart calibration if necessary
	vex::this_thread::sleep_for(100);
    }*/
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void autonomous( void ) {

  atn_drive(5.3); //to hit invisible wall
  vex::this_thread::sleep_for(100);
  atn_KOBE();
  atn_drive(-12);//bck up  to center align
  atn_turn(90);
  atn_activate_intake();
  atn_drive(45, 50);
  vex::this_thread::sleep_for(1000);
  atn_deactivate_intake();
  atn_turn(-87);
 
  atn_drive(20);
  atn_KOBE();

    atn_drive(35);

}

/*---------------------------------------------------------------------------*/
/*                                                                            */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void usercontrol( void ) {
  // User control code here, inside the loop

  while (1) {
    if(!wheelLock)
    {
        usr_drive();
    }
    usr_halfspeed();
	usr_change_orientation();
	usr_activate_intake();
    usr_shoot();
    usr_flipper();
   
    vex::this_thread::sleep_for(20);

    
    //Sleep the task for a short amount of time to prevent wasted resources. 
  }
}

//
// Main will set up the competition functions and callbacks.
//

int main() {
    
    //Run the pre-autonomous function. 
    pre_auton();
    
    //Set up callbacks for autonomous and driver control periods.
    Competition.autonomous( autonomous );
    Competition.drivercontrol( usercontrol );

    //Prevent main from exiting with an infinite loop.                        
    while(1) {
      vex::task::sleep(100);//Sleep the task for a short amount of time to prevent wasted resources.
    }    
       
}




        
                            
     
                





