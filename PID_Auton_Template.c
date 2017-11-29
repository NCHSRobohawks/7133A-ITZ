
#pragma config(Sensor, in1,    mobile1,        sensorPotentiometer)
#pragma config(Sensor, in2,    mobile2,        sensorPotentiometer)
#pragma config(Sensor, dgtl1,  left,           sensorQuadEncoder)
#pragma config(Sensor, dgtl3,  right,          sensorQuadEncoder)
#pragma config(Sensor, dgtl6,  claw,           sensorDigitalOut)
#pragma config(Sensor, dgtl7,  elevator,       sensorQuadEncoder)
#pragma config(Sensor, dgtl11, bar,            sensorQuadEncoder)
#pragma config(Motor,  port1,           bar1,          tmotorVex393_HBridge, openLoop)
#pragma config(Motor,  port2,           rf,            tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port3,           rb,            tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port4,           lf,            tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port5,           lb,            tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port6,           goal2,         tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port7,           goal1,         tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port8,           lift1,         tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port9,           lift2,         tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port10,          bar2,          tmotorVex393_HBridge, openLoop)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

float kP[4], kI[4], kD[4], kL[4];
long tolerance[4];
long ticksPerRotation, ticksPerFoot, waitBetweenPID;
bool turning = false;
long degrees = 0;
/*
Constants
* * *
0 - wheel base
1 - Mobile goal intake
2 - Elevator
3 - Chain bar
* * *
*/

void init() {
    //Wheel base constants
    kP[0] = -0.08;
    kI[0] = -0.3;
    kD[0] = 1.1;
    kL[0] = 0.0;
    tolerance[0] = 10;
    nMotorEncoder[port2] = 0;
    nMotorEncoder[port3] = 0;

    //Mobile goal lift constants
    kP[1] = 0.3;
    kI[1] = 0.04;
    kD[1] = -0.00;
    kL[1] = 20;
    tolerance[1] = 5;

    //Elevator constants
    kP[2] = 0.0;
    kI[2] = 0.0;
    kD[2] = 0.0;
    kL[2] = 0.0;
    tolerance[2] = 0;

    //Chain bar constants
    kP[3] = 10;
    kI[3] = 4;
    kD[3] = 2;
    kL[3] = 0.0;
    tolerance[3] = 0;

    ticksPerRotation = 0; // Number of ticks per full rotation
    ticksPerFoot = 298.1; // Number of ticks per foot traveled


    waitBetweenPID = 1000; // Number of milliseconds to wait after each PID move

    SensorType[in3] = sensorNone;
    wait1Msec(1000);
    SensorType[in3] = sensorGyro;
    wait1Msec(2000);
    //SensorScale[in3] = 260;
    SensorFullCount[in3] = 3600;


}

long target[6] = {0, 0, sensorValue(mobile1), sensorValue(mobile2), 0, 0};

/*
Targets
* * *
0 - left wheels
1 - right wheels
2 - left mobile goal
3 - right mobile goal
4 - elevator
5 - chain bar
* * *
*/
task pid() {

    long error[6] = {0, 0, 0, 0, 0, 0};
    long pError[6] = {0, 0, 0, 0, 0, 0};
    long p[6] = {0, 0, 0, 0, 0, 0};
    long i[6] = {0, 0, 0, 0, 0, 0};
    long d[6] = {0, 0, 0, 0, 0, 0};

    while(true) {
				if(turning){
	        error[0] = degrees - SensorValue(in3);
	        p[0] = error[0];
	        i[0] = abs(i[0] + error[0]) < kL[0] ? i[0] + error[0] : sgn(i[0] + error[0])*kL[0];
	        d[0] = error[1] - pError[1];
	        motor[port4] = -(p[0]*-0.095 + i[0]*-0.73 + d[0]*0);
	        motor[port5] = -(p[0]*-0.095 + i[0]*-0.73 + d[0]*0);
	        motor[port2] = p[0]*-0.095 + i[0]*-0.73 + d[0]*0;
	        motor[port3] = p[0]*-0.095 + i[0]*-0.73 + d[0]*0;
      }
      	else{
      		error[0] = target[0] - SensorValue(left);
	        error[1] = target[1] - SensorValue(right);
	        p[0] = error[0];
	        p[1] = error[1];
	        i[0] = abs(i[0] + error[0]) < kL[0] ? i[0] + error[0] : sgn(i[0] + error[0])*kL[0];
	        i[1] = abs(i[1] + error[1]) < kL[0] ? i[1] + error[1] : sgn(i[1] + error[1])*kL[0];
	        d[1] = error[1] - pError[1];
	        d[2] = error[2] - pError[2];
	        motor[port4] = p[0]*kP[0] + i[0]*kI[0] + d[0]*kD[0];
	        motor[port5] = p[0]*kP[0] + i[0]*kI[0] + d[0]*kD[0];
	        motor[port2] = p[1]*kP[0] + i[1]*kI[0] + d[1]*kD[0];
	        motor[port3] = p[1]*kP[0] + i[1]*kI[0] + d[1]*kD[0];
	      }

        error[2] = target[2] - SensorValue(mobile1);
        error[3] = target[3] - SensorValue(mobile2);
        error[4] = target[4] - SensorValue(elevator);
        error[5] = target[5] - SensorValue(bar);

        p[2] = error[2];
        p[3] = error[3];
        p[4] = error[4];
        p[5] = error[5];

        i[2] = abs(i[2] + error[2]) < kL[0] ? i[2] + error[2] : sgn(i[2] + error[2])*kL[0];
        i[3] = abs(i[3] + error[3]) < kL[0] ? i[3] + error[3] : sgn(i[3] + error[3])*kL[0];
        i[4] = abs(i[4] + error[4]) < kL[0] ? i[4] + error[4] : sgn(i[4] + error[4])*kL[0];
        i[5] = abs(i[5] + error[5]) < kL[0] ? i[5] + error[5] : sgn(i[5] + error[5])*kL[0];
        d[0] = error[0] - pError[0];
        //displayLCDNumber(0,1, d[0]);
				d[2] = error[3] - pError[3];
        d[3] = error[3] - pError[3];
        d[4] = error[4] - pError[4];
        d[5] = error[5] - pError[5];
        motor[port7] = p[2]*kP[1] + i[2]*kI[1] + d[2]*kD[1];
        motor[port6] = p[3]*kP[1] + i[3]*kI[1] + d[3]*kD[1];
        motor[port8] = p[4]*kP[2] + i[4]*kI[2] + d[4]*kD[2];
        motor[port9] = p[4]*kP[2] + i[4]*kI[2] + d[4]*kD[2];
        motor[port1] = p[5]*kP[3] + i[5]*kI[3] + d[5]*kD[3];
        motor[port10] = p[5]*kP[3] + i[5]*kI[3] + d[5]*kD[3];

        pError[0] = error[0];
        pError[1] = error[1];
        pError[2] = error[2];
        pError[3] = error[3];
        pError[4] = error[4];
        pError[5] = error[5];
        wait1Msec(25);

    }

}

int reverse = 1;
int half = 1;
int rl=0;
int hl=0;
int sl=0;
int ll=0
bool lower = False;
long error[2] = {0, 0};
long i[2] = {0,0};

void mogo_forward(int inches){
	SensorValue[right] = 0;
	SensorValue[left] = 0;
	target[0] = target[1] = inches*ticksPerFoot/12;
	while(abs(SensorValue(left) - target[0]) > tolerance[0] && abs(SensorValue(right) - target[1]) > tolerance[0]);

}

void forward(int inches){
	SensorValue[right] = 0;
	SensorValue[left] = 0;
	target[0] = target[1] = -inches*ticksPerFoot/12;
	while(abs(SensorValue(left) - target[0]) > tolerance[0] && abs(SensorValue(right) - target[1]) > tolerance[0]);
}

void turn(int degrees10){
	turning = True;
	degrees += degrees10;
	while(True);
	//while(abs(SensorValue(in3) - degrees) > 10);
	//turning = False;
}
void arm_raise(){
	target[5] = 60;
}
void mogo_lower() {
	target[2] = 730;
	target[3] = 345;
}

task red_left_far(){
  SensorValue[claw] = 1;
  //startTask(deploy);
  wait1Msec(500);
  startTask(pid);
  mogo_lower();
  while(abs(SensorValue(mobile1) - target[2]) > tolerance[3]);
  wait1Msec(waitBetweenPID);
	mogo_forward(47);
  while(abs(nMotorEncoder[port4] - target[0]) > tolerance[0] && abs(nMotorEncoder[port2] - target[1]) > tolerance[0]);
}


task main() {
	sensorValue[bar] = 0;
	init();
	startTask(pid);
	SensorValue[claw] = 1;
	arm_raise();
	wait1Msec(waitBetweenPID);
	mogo_lower();
	mogo_forward(50);
	while(True);
}
