#pragma config(Motor,  port1,           conveyor,      tmotorVex393_HBridge, openLoop, reversed)
#pragma config(Motor,  port2,           rf,            tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port3,           rm,            tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port4,           rb,            tmotorServoContinuousRotation, openLoop, reversed)
#pragma config(Motor,  port5,           lf,            tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port6,           lm,            tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port7,           lb,            tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port8,           goal,          tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port9,           goal2,         tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port10,          conveyor2,     tmotorVex393_HBridge, openLoop)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

task main()
{

while(true){
	motor[lf] = vexRT[Ch3] - vexRt[Ch4];
	motor[lm] = vexRT[Ch3] - vexRt[Ch4];
	motor[lb] = vexRT[Ch3] - vexRt[Ch4];
	motor[rf] = vexRt[Ch3] + vexRT[Ch4];
	motor[rm] = vexRt[Ch3] + vexRT[Ch4];
	motor[rb] = vexRt[Ch3] + vexRT[Ch4];

	if(vexRT[Btn6U]){
		motor[goal] = 127;
		motor[goal2] = 127;
	}
	else if (vexRt[Btn6D]){
		motor[goal] = -127;
		motor[goal2] = -127;
}
	else{
		motor[goal] = 0;
		motor[goal2] = 0;
}
if(vexRT[Btn7U]){
		motor[conveyor] = 127;
		motor[conveyor2] = 127;
	}
	else if (vexRt[Btn7D]){
		motor[conveyor] = -127;
	  motor[conveyor2] = -127;
}
	else{
		motor[conveyor] = 0;
		motor[conveyor2] = 0;
}
}
}
