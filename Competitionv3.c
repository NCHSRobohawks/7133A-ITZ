#pragma config(Sensor, in1,    dr4bL,          sensorPotentiometer)
#pragma config(Sensor, in2,    dr4bR,          sensorPotentiometer)
#pragma config(Sensor, in3,    mogo,           sensorPotentiometer)
#pragma config(Sensor, in4,    manipulator,    sensorPotentiometer)
#pragma config(Sensor, dgtl1,  L,              sensorQuadEncoder)
#pragma config(Sensor, dgtl3,  R,              sensorQuadEncoder)
#pragma config(Sensor, dgtl5,  ultrasonic,     sensorSONAR_inch)
#pragma config(Motor,  port1,           mogoL,         tmotorVex393_HBridge, openLoop)
#pragma config(Motor,  port2,           left,          tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port3,           liftL1,        tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port4,           liftL2,        tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port5,           manipulatorL,  tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port6,           manipulatorR,  tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port7,           liftR1,        tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port8,           liftR2,        tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port9,           right,         tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port10,          mogoR,         tmotorVex393_HBridge, openLoop)
#pragma platform(VEX2)
#pragma competitionControl(Competition)

#include "Vex_Competition_Includes.c"

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the cortex has been powered on and    */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/
float ticksPerFoot = 344;
float error[5] =  {0,0,0,0,0}
float pError[5] =  {0,0,0,0,0};
float kP[4] = {0.12,0.48,0.3,0.25};
float kI[4] = {0.1,0.06,0.012,0};
float kD[4] = {0.2,1.1,1.5 ,0.3};
float I[5] = {0,0,0,0,0};

void straighten(int target){
	error[2] = target - SensorValue(in5);
	I[2] += error[2];
	if(error[2] > 50 || abs(error[2]) > 10){
		I[2]  = 0;
	}
	motor[left] += -(error[2] * kP[1] + I[2] * kI[1] + (error[2] - pError[2]) * kD[1]);
	motor[right] += error[2] * kP[1] * kP[1] + I[2] * kI[1] + + (error[2] - pError[2]) * kD[1];
	pError[2] = error[2];
}

void pd_base(int target, int degrees10){
	error[0] = target - SensorValue(L);
	error[1] = target - SensorValue(R);
	I[0] += error[0];
	if(abs(error[0]) > 50 || abs(error[0]) < 5){
		I[0] = 0;
	}
	I[1] += error[0];
	if(abs(error[1]) > 50 || abs(error[1]) < 5){
		I[1] = 0;
	}
	motor[left] = error[0] * kP[0] + (error[0] - pError[0]) * kD[0];
	motor[right] = error[1]* kP[0] + (error[1] - pError[1]) * kD[0];
	pError[0] = error[0];
	pError[1] = error[1];
	straighten(degrees10);
}

void pid_turn(int target){
	error[2] = target - SensorValue(in5);
	I[2] += error[2];
	if(error[2] > 35 || abs(error[2]) == 0){
		I[2]  = 0;
	}
	motor[left] = -(error[2] * kP[1] + I[2] * kI[1] + (error[2] - pError[2]) * kD[1]);
	motor[right] = error[2] * kP[1] * kP[1] + I[2] * kI[1] + + (error[2] - pError[2]) * kD[1];
	pError[2] = error[2];
}



void pid_lift(int target){
	error[3] = SensorValue[dr4bR]- target;
	I[3] += error[3];
	writeDebugStreamLine("Error: %d", SensorValue(dr4bR));

	if(abs(error[3]) > 200 || abs(error[3]) < 10){
		I[3]  = 0;
}
	motor[liftL1] = error[3] * kP[2] + I[3] * kI[2] + (error[3] - pError[3])*kD[2];
	motor[liftR1] = error[3] * kP[2] + I[3] * kI[2] + (error[3] - pError[3])*kD[2];
	motor[liftR2] = error[3] * kP[2] + I[3] * kI[2] + (error[3] - pError[3])*kD[2];
	motor[liftR2] = error[3] * kP[2] + I[3] * kI[2] + (error[3] - pError[3])*kD[2];
	pError[3] = error[3];
}

void pid_manipulator(int target){
	error[4] = target - SensorValue(manipulator);
	I[4] += error[4]
	if(abs(error[4]) < 20){
		I[4]  = 0;
}
	motor[manipulatorL] = error[4] *kP[3] + I[4] * kI[3] + (error[4] - pError[4]) * kD[3];
	motor[manipulatorR] = error[4] *kP[3] + I[4] * kI[3] + (error[4] - pError[4]) * kD[3];
	pError[4] = error[4];
}

void move(int target){
	SensorValue[L] = 0;
	SensorValue[R] = 0;
	pError[0] = 0;
	pError[1] = 0;
	int degrees = SensorValue[in5];
	while(abs(SensorValue(L) - target) > 30 && abs(SensorValue(R) - target) > 30){
		pd_base(target, degrees);
	}
	motor[left] = 0;
	motor[right] = 0;
}

void turn(int degrees10){
	if(SensorType[in5] != sensorGyro){
		SensorType[in5] = sensorNone;
		wait1Msec(1000);
		SensorType[in5] = sensorGyro;
		wait1Msec(2000);
		}
	while(abs(SensorValue(in5)) < degrees10){

		pid_turn(degrees10);
	}
}

void lift_set(int target){
	I[4] = 0;
	while((Sensorvalue[dr4bR] - target) > 15){
		pid_lift(target)
		wait1Msec(25);
	}
}

void manipulator_set(int target){
	I[2] = 0;
	while((Sensorvalue[manipulator] - target) > 15){
		pid_manipulator(target)
		wait1Msec(25);
	}
}
int target_lift;
int target_manipulator = 2950;
task control(){
	while(true){
	pid_manipulator(target_manipulator);
	wait1Msec(25);
}
}

int pot_target, sonar;
void autostack(){
	motor[mogoL] = -50;
	motor[mogoR] = -50;
	startTask(control);
	int begin;
	begin = SensorValue(dr4bR);
	while(SensorValue(ultrasonic) < 12){
		pid_lift(1250);

		wait1Msec(25);
	}
	pot_target = SensorValue(dr4bR);
	while(abs(SensorValue(dr4bR) - (pot_target-100)) > 50 && !vexRT[Btn8L]){
		pid_lift(pot_target-100);
		wait1Msec(25);
		writeDebugStreamLine("Stuck: %d", 1);
	}
	target_manipulator =2050
	while(abs(SensorValue(manipulator) - target_manipulator) > 20 && !vexRT[Btn8L]){

		pid_lift(pot_target - 100);
		wait1Msec(25);
		writeDebugStreamLine("Stuck: %d", 2);

	}
	wait1Msec(500);
	while(abs(SensorValue(dr4bR) - (pot_target + 300)) > 50 && !vexRT[Btn8L]) {
		pid_lift(pot_target+300);
		wait1Msec(25);
		writeDebugStreamLine("Stuck: %d", 3);
	}
	while(abs(SensorValue(manipulator) - 2950) > 90 && !vexRT[Btn8L]){
		target_manipulator = 2950;
		pid_lift(pot_target+300);
		wait1Msec(25);
		writeDebugStreamLine("Stuck: %d", 4);
	}
	while(abs(SensorValue(dr4bR) - begin) > 100 && !vexRT[Btn8L]) {
		pid_lift(begin);
		wait1Msec(25);
		writeDebugStreamLine("Stuck: %d", 5);
	}
	target_manipulator=2950;
	stopTask(control);


}
bool near_zone, left_side, enabled;
void pre_auton()
{
  bStopTasksBetweenModes = true;
  /*
  displayLCDCenteredString(0, "Auton?");
  displayLCDCenteredString(1, "Yes                   No");
  while((nLCDButtons == 0 || nLCDButtons == 2) && enabled == true );
  if(nLCDButtons == 1){
  	enabled = true;
  }
  else if (nLCDButtons == 4){
  	enabled = false;
  }
  clearLCDLine(0);
 	clearLCDLine(1);
 	if(enabled){
 	displayLCDCenteredString(0, "Auton Enabled");
}
	else{
		displayLCDCenteredString(0, "Auton Disabled");
		return;
	}
 	wait1Msec(1000);
 	clearLCDLine(0);
  displayLCDCenteredString(0, "Side:");
  displayLCDCenteredString(1, "Left               Right");
  while((nLCDButtons == 0 || nLCDButtons == 2) && enabled == true);
  if(nLCDButtons == 1){
  	left_side = true;
  }
  else if (nLCDButtons == 4){
  	left_side = false;
  }
 	clearLCDLine(0);
 	clearLCDLine(1);
 	if(left){
 		displayLCDCenteredString(0, "Left Side Selected");
 	}
 	else{
 		displayLCDCenteredString(0, "Right Side Selected");
 	}
 	wait1Msec(1000);
 	clearLCDLine(0);
 	displayLCDCenteredString(0, "Zone:");
  displayLCDCenteredString(1, "Near               White");
 	while((nLCDButtons == 0 || nLCDButtons == 2) && enabled == true);
  if(nLCDButtons == 1){
  	near_zone = true;
  }
  else if (nLCDButtons == 4){
  	near_zone = false;
  }
  clearLCDLine(0);
 	clearLCDLine(1);
 	if(near_zone){
 		displayLCDCenteredString(0, "Near Zone Selected");
 	}
 	else{
 		displayLCDCenteredString(0, "5 pt. Zone Selected");
 	}
 	wait1Msec(1000);
 	clearLCDLine(0);
 	displayLCDCenteredString(0, "Calibrating Gyro");
 	SensorType[in5] = sensorNone;
	wait1Msec(1000);
	SensorType[in5] = sensorGyro;
	wait1Msec(2000);
	displayLCDCenteredString(0, "Gyro Calibrated");
	*/
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

task autonomous()
{
	motor[left] = 127;
	motor[right] = 127;
	wait1Msec(800);
	motor[left] = 0;
	motor[right] = 0;

	/*
	SensorType[in5] = SensorNone;
	wait1Msec(1000);
	SensorType[in5] = SensorGyro;
	SensorValue[in5] = 0;
	manipulator_set(2000);
	lift_set(2600);

	motor[liftL1] = 15;
	motor[liftR1] = 15;
	motor[liftL2] = 15;
	motor[liftR2] = 15;
	while(SensorValue(mogo) > 1950){
		motor[mogoL] = 127;
		motor[mogoR] = 127;
	}
	while(abs(SensorValue[L] - 3.5*ticksPerFoot)){
		pd_base(3.5 * ticksPerFoot, 0);
	}
	while(SensorValue(mogo) < 2800){
		motor[mogoL] = 127;
		motor[mogoR] = 127;
	}
	lift_set(2750);
	manipulator_set(2950);

	*/
  AutonomousCodePlaceholderForTesting();

}



task usercontrol()
{
	int prev_joystick[4] = {0,0,0,0};
	int mogo_power = 0;
  target_manipulator = SensorValue(manipulator);
  target_lift = SensorValue(dr4bR);
  //startTask(control);
  while(true){
		motor[left] = vexRT[Ch3] + vexRT[Ch4];
		motor[right] = vexRT[Ch3] - vexRT[Ch4];
		/*
		if(abs(vexRT[Ch2]) > 15){
			motor[mogoL] += vexRT[Ch2];
			motor[mogoR] += vexRT[Ch2];
		}
		*/
		if(vexRT[Btn6U]){
			motor[mogoL] = 127;
			motor[mogoR] = 127;
		}
		else if(vexRT[Btn6D]){
			motor[mogoL] = -127;
			motor[mogoR] = -127;
		}
		else{
			motor[mogoL] = -15;
			motor[mogoR] = -15;
		}

		if(vexRT[Btn8D] && vexRT[Btn8D] != prev_joystick[0]){
			if(mogo_power ==  0){
				mogo_power = -30;
			}
			else{
				mogo_power = 0;
			}
		}


		if(abs(vexRT[Ch3Xmtr2]) > 15){
			motor[liftL1] = vexRT[Ch3Xmtr2];
			motor[liftL2] = vexRT[Ch3Xmtr2];
			motor[liftR1] = vexRT[Ch3Xmtr2];
			motor[liftR2] = vexRT[Ch3Xmtr2];
		}
		else if(vexRT[Btn5U]){
			motor[liftL1] = 127;
			motor[liftL2] = 127;
			motor[liftR1] = 127;
			motor[liftR2] = 127;
		}
		else if(vexRT[Btn5D]){
			motor[liftL1] = -127;
			motor[liftL2] = -127;
			motor[liftR1] = -127;
			motor[liftR2] = -127;
		}

		else{
		 	motor[liftL1] = 15;
			motor[liftL2] = 15;
			motor[liftR1] = 15;
			motor[liftR2] = 15;
		}

		/*
		if(vexRT[Btn5U] && vexRT[Btn5U] != prev_joystick[1]){
			if(target_manipulator == 2950){
				target_manipulator = 2500;
			}
			else{
				target_manipulator = 2950;
			}
		}
		*/

		if(vexRT[Btn8R]){
			pid_manipulator(2100);
			wait1Msec(25);
		}
		else if(vexRT[Btn8U]){
			pid_manipulator(2200);
			wait1Msec(25);
		}
		else if(vexRT[Btn8D]){
			pid_manipulator(2000);
			wait1Msec(25);
		}
		else if(vexRT[Btn8L]){
			pid_manipulator(1950);
			wait1Msec(25);
		}
		else if (abs(vexRT[Ch2]) > 15){
			motor[manipulatorL] = vexRT[Ch2];
			motor[manipulatorR] = vexRT[Ch2];
		}
		else{
			pid_manipulator(2950);
			wait1Msec(25);
		}
		if(target_manipulator > 2900){
			target_manipulator = 2900;
		}
		if(target_manipulator < 2000){
			target_manipulator = 2000;
		}
		if(vexRT[Btn8D]){
			stopTask(control);
			enabled = true;
		}
		if (VexRT[Btn7D]){
			startTask(control);
			enabled = false;
		}
		if(vexRT[Btn5D] && vexRT[Btn5d] != prev_joystick[2]){
			//autostack();
		}
		prev_joystick[0] = vexRT[Btn8D];
		prev_joystick[1] = vexRT[Btn5U];
		prev_joystick[2] = vexRT[Btn5D];
}
}
