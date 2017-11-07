#pragma config(Sensor, in1,    mobile,         sensorPotentiometer)
#pragma config(Sensor, dgtl3,  claw,           sensorDigitalOut)
#pragma config(Motor,  port1,           conveyor,      tmotorVex393_HBridge, openLoop, scaled)
#pragma config(Motor,  port2,           rf,            tmotorVex393_MC29, openLoop, scaled)
#pragma config(Motor,  port3,           rm,            tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port4,           rb,            tmotorVex393_MC29, openLoop, scaled)
#pragma config(Motor,  port5,           lf,            tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port6,           lm,            tmotorVex393_MC29, openLoop, scaled)
#pragma config(Motor,  port7,           lb,            tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port8,           goal,          tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port9,           goal2,         tmotorVex393_MC29, openLoop, scaled)
#pragma config(Motor,  port10,          conveyor2,     tmotorVex393_HBridge, openLoop)
/*---------------------------------------------------------------------------*/
/*                                                                           */
/*        Description: Competition template for VEX EDR                      */
/*                                                                           */
/*---------------------------------------------------------------------------*/

// This code is for the VEX cortex platform
#pragma platform(VEX2)

// Select Download method as "competition"
#pragma competitionControl(Competition)

//Main competition background code...do not modify!
#include "Vex_Competition_Includes.c"

int scale = 1;
int half = 1;
int rl=0;
int hl=0;
int sl=0;

task lower(){
	while(sensorValue(mobile) > 5000){
		motor[goal] = -127;
		motor[goal2] = 127;
	}
	while(sensorValue(mobile) < 3585 ){
		motor[goal] = -127;
		motor[goal2] = -127;
	}
		motor[goal] = 5;
		motor[goal2] = 5;
	wait1Msec(250);
		motor[goal] = 0;
		motor[goal2] = 0;
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

void pre_auton()
{
  // Set bStopTasksBetweenModes to false if you want to keep user created tasks
  // running between Autonomous and Driver controlled modes. You will need to
  // manage all user created tasks if set to false.
  bStopTasksBetweenModes = true;

	// Set bDisplayCompetitionStatusOnLcd to false if you don't want the LCD
	// used by the competition include file, for example, you might want
	// to display your team name on the LCD in this function.
	// bDisplayCompetitionStatusOnLcd = false;

  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
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

void drivespeed(int x) {
	motor[lm] = x;
	motor[rm] = x;
	motor[lb] = x;
	motor[lf] = x;
	motor[rb] = x;
	motor[rf] = x;
}

task autonomous()
{

		motor[goal] = 127;
	motor[goal2] = 127;
	while (sensorValue(mobile)<2100 || sensorValue(mobile)>2300) {
		wait1Msec(1);
	}
	motor[goal] = 0;
	motor[goal2] = 0;
	/*
	drivespeed(-127);
	delay(800);
	drivespeed(127);
	delay(800);
	drivespeed(-127);
	delay(800);
	drivespeed(127);
	delay(800);
	drivespeed(0);

  /*for (int x =0; x < 6; x++) {
		drivespeed(127*((x%2)*2-1));
		delay(500);
	}
	int liftTime = 250;
	int driveTime = 1000;
	motor[conveyor] = 127; //check if this is towards the robot or away from it
	motor[conveyor2] = 127;

	delay(liftTime);

	drivespeed(127);

	delay(driveTime);

	drivespeed(0);

	motor[conveyor] = 127; //check if this is towards the robot or away from it
	motor[conveyor2] = 127;

	delay(liftTime);

	drivespeed(-127);

	delay(driveTime);

	drivespeed(0);*/

}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

task usercontrol()
{
  // User control code here, inside the loop
while(true){
		motor[lf] = scale * (vexRT[Ch3] + scale*(vexRt[Ch1]+ vexRT[Ch4]))/half;
		motor[lm] = scale * (vexRT[Ch3] + scale*(vexRt[Ch1]+ vexRT[Ch4]))/half;
		motor[lb] = scale * (vexRT[Ch3] + scale*(vexRt[Ch1]+ vexRT[Ch4]))/half;
		motor[rf] = scale * (vexRT[Ch3] - scale*(vexRt[Ch1]+ vexRT[Ch4]))/half;
		motor[rm] = scale * (vexRT[Ch3] - scale*(vexRt[Ch1]+ vexRT[Ch4]))/half;
		motor[rb] = scale * (vexRT[Ch3] - scale*(vexRt[Ch1]+ vexRT[Ch4]))/half;

		displayLCDNumber(0,0,SensorValue(mobile));
		if(vexRT[Btn6U]){
			motor[goal] = 127;
			motor[goal2] = 127;
		}
		else if (vexRt[Btn6D]/*&&(SensorValue(mobile) < 3600 || sensorValue(mobile) > 5000)*/){
			motor[goal] = -127;
			motor[goal2] = -127;
	}
		else{
			motor[goal] = 0;
			motor[goal2] = 0;
	}
	if(vexRT[Btn5U]){
			motor[conveyor] = 127;
			motor[conveyor2] = 127;
		}
		else if (vexRt[Btn5D]){
			motor[conveyor] = -127;
		  motor[conveyor2] = -127;
	}
		else{
			motor[conveyor] = 0;
			motor[conveyor2] = 0;
	}

		if(vexRT[Btn8D]!=rl && rl){
			scale *= -1;
		}
		rl = vexRT[Btn8D];
		if(vexRT[Btn7D]!=hl && hl){
			half == 1 ? half = 2 : half = 1;
		}
		hl = vexRT[Btn7D];
		if (vexRT[Btn8R]!=sl && sl) {
				SensorValue[dgtl3] = !SensorValue[dgtl3];
		}
		sl = vexRT[Btn8R];
	}

}
