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
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

task main()
{

while(true){

	motor[left] = vexRT[Ch3] + vexRT[Ch4];
	motor[right] = vexRT[Ch3] - vexRT[Ch4];

	if(vexRT[Btn6U]){
		motor[mogoL] = 127;
		motor[mogoR] = 127;
	}
	else if(vexRT[Btn6D]){
		motor[mogoL] = -127;
		motor[mogoR] = -127;
	}
	else{
		motor[mogoL] = -20;
		motor[mogoR] = -20;
	}

	if(vexRT[Btn5U]){
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
		motor[liftL1] = 0;
		motor[liftL2] = 0;
		motor[liftR1] = 0;
		motor[liftR2] = 0;
	}

	if(vexRT[Btn8U]){
		motor[manipulatorL] = 127;
		motor[manipulatorR] = 127;
	}
	else if(vexRT[Btn8R]){
		motor[manipulatorL] = -127;
		motor[manipulatorR] = -127;
	}
	else{
		motor[manipulatorL] = 0;
		motor[manipulatorR] = 0;
	}
}

}
