#include "observer.h"
#include <fstream>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <iomanip>
#include <string>
#include <math.h>

//lcm
static lcmt_encoder_subscription_t * lcm_sub_hcp_y;
static lcmt_encoder_subscription_t * lcm_sub_hcp_theta;
lcmt_hcp_state state;
static lcm_t * lcm;
//end lcm

static const double PI = 3.14159265358979;

static double meas[2] = {0.0,0.0};

static double speedVkhz = .3;
static double dt = .001/speedVkhz;

static void hcp_dynamics(const double* q, double* q_dot, const double* u);

static inline double mod_by_2PI(double th) { return fmod(fmod(th,2*PI)+3*PI,2*PI)-PI;; }

int main(int argc, char** argv)
{
	operatingMode = 1; //default to recentering operation
	if(argc > 1){
		sscanf(argv[1],"%d",&operatingMode);
	}

	switch(operatingMode)
	{
	case 1: //normal operation
		printf("Normal operation...\n");
		break;

	default:
		printf("Command not understood. Turning off.\n\n");
		return 0;
		break;
	}


	lcm = lcm_create ("udpm://");
	if (!lcm)
		return 1;

	lcm_sub_hcp_y = lcmt_encoder_subscribe (lcm, "hcp_y", &my_handler, NULL);
	lcm_sub_hcp_theta = lcmt_encoder_subscribe (lcm, "hcp_theta", &my_handler, NULL);

	HANDLE lcm_watcher_thread;
	unsigned lcm_watcher_thread_ID;
	static HANDLE mutex = CreateMutex(NULL, false, "hcp_enc_receive_mutex");
	ReleaseMutex(mutex);
	//lcm_watcher_thread =(HANDLE)_beginthreadex( NULL , 0,&lcm_watcher, lcm, 0, &lcm_watcher_thread_ID);

	unsigned int iter=0;

	state.y=0.0; state.theta=0.0; state.ydot=0.0; state.thetadot=0.0;

	double x0[] = {0.0, 0.0, 0.0, 0.0};
	FGSE observer(hcp_dynamics, 4, dt, x0);
	const double* state_est;

	while(keepRunning)//begin control loop
	{
		iter++;

		if(WaitForSingleObject(mutex, 10000)==WAIT_TIMEOUT)
		{
			printf("MUTEX TIMEOUT ERROR 2\n");
		}
		else
		{
			meas[0] = y_ticks;
			meas[1] = theta_ticks;
		}
		ReleaseMutex(mutex);


		meas[0] = y_ticks;
		meas[1] = theta_ticks;

		//observer
		observer.update(meas, &lastu);
		state_est = observer.getEstimate();
		state.y=state_est[0]; state.theta=state_est[1]; state.ydot=state_est[2]; state.thetadot=state_est[3];

		switch(operatingMode)
		{

		case 1: //normal operation

			break;

		default:
			keepRunning=false; //should never reach here
		}

		send_message (lcm, &myComm);
	}


	return 0;
}
////////////////////////////
//Helper functions
////////////////////////////
static void hcp_dynamics(const double* q, double* q_dot, const double* u){
 	q_dot[0]=q[2];//xd
	q_dot[1]=q[3];//td
	q_dot[2]=*u;//xdd

	double a=17.4;
	double b=9.69;
	double c=4.61;
	double d=0.900;

	//tdd at upright    theta     yd       thetad     u
	q_dot[3]=            d*q[1] +b*q[2]   -c*q[3]  +a*(*u);
}

//LCM Code
unsigned __stdcall lcm_watcher(void *param)
{
	sub = lcmt_servotubeState_subscribe (lcm, "wt_state", &my_handler, NULL);
	while(1)
	{
		lcm_handle((lcm_t*) param);
	}

	_endthreadex(0);
	return 0;
}

static void my_handler (const lcm_recv_buf_t *rbuf, const char * channel, 
						const lcmt_servotubeState * msg, void* user)
{
	static HANDLE mutex = CreateMutex(NULL, false, "wt_state_mutex");

	if(WaitForSingleObject(mutex, 10000)==WAIT_TIMEOUT)
	{
		printf("MUTEX TIMEOUT ERROR 2\n");
	}
	else
	{
		position= msg->position;
		secondaryPosition= msg->positionSecondary;
		msgNum++;
	}
	ReleaseMutex(mutex);
}

static void send_message (lcm_t * lcm, lcmt_servotubeCommand* my_data)
{	
	//printf("Sent: %f\n",my_data->commandValue);
	lcmt_servotubeCommand_publish (lcm, "wt_command", my_data);
}