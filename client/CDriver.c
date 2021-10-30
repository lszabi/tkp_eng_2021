#include "CDriver.h"
#include <math.h>

#ifdef VS_DEBUG
	#include <WinSock.h>
	#include <stdio.h>
	SOCKET socketDescriptor;
	struct sockaddr_in serverAddress;
#endif

const int REVERSE = 1;

const float PI = 3.14159265f;

/* Gear Changing Constants*/
const int gearUp[6] = {6500,7000,7000,7500,7500,0};
const int gearDown[6] = {0,2500,2500,3000,3000,3500};

/* Stuck constants*/
const int stuckTime = 25;
const float stuckAngle = 3.1415f / 3.0f;

/* Accel and Brake Constants*/
const float maxSpeedDist = 70.0f;
const float maxSpeed = 200.0f;
const float sin5 = 0.08716f;
const float cos5 = 0.99619f;

/* Steering constants*/
const float steerLock = 0.785398f;
const float steerSensitivityOffset = 90.0f;
const float wheelSensitivityCoeff = 1.0f;

/* ABS Filter Constants */
const float wheelRadius[4] = { 0.3179f, 0.3179f, 0.3276f, 0.3276f };
const float absSlip = 2.0f;
const float absRange = 3.0f;
const float absMinSpeed = 3.0f;

/* Clutch constants */
const float clutchMax = 0.5f;
const float clutchDelta = 0.05f;
const float clutchRange = 0.82f;
const float clutchDeltaTime = 0.02f;
const float clutchDeltaRaced = 10.0f;
const float clutchDec = 0.01f;
const float clutchMaxModifier = 1.3f;
const float clutchMaxTime = 1.5f;

int stuck;
float clutch;

/* Filtering */
#define ACCEL_FILT_N 50
float accel_filt[ACCEL_FILT_N] = {0, };
int accel_filt_i = 0;

float accel_int = 0;
float steer_int = 0;

float sensor_prev[19] = { 0, };
float sensor_prevprev[19] = { 0, };

/* Logging */
int tick = 0;

struct maplog {
	float distance;
	float steer;
	float speed;
	float accel;
	float track_pos;
};

struct maplog map[256000];
int map_n = 0, map_i = 0;

int firstlap = 1;

void filter(float *dest, float* source, int n, int order) {
	for (int i = 0; i < n; i++)
	{
		float sum = 0;
		for (int k = 0; k < order; k++)
		{
			int c = i + k - order / 2;
			if (c > 0 && c < n) {
				sum += source[c];
			}
		}
		dest[i] = sum / order;
	}
}

float track_pos[256000], track_pos_new[256000];

void process_map() {
	for (int i = 0; i < map_n; i++)
	{
		if (map[i].steer > 0.05)
		{
			track_pos[i] = 0.8;
		}
		else if (map[i].steer < -0.05)
		{
			track_pos[i] = -0.8;
		}
		else
		{
			track_pos[i] = 0;
		}
	}
	filter(track_pos_new, track_pos, map_n, 50);
	for (int i = 0; i < map_n; i++)
	{
		map[i].track_pos = track_pos_new[i];
	}
}

int getGear(structCarState* cs)
{

	int gear = cs->gear;
	int rpm = cs->rpm;

	// if gear is 0 (N) or -1 (R) just return 1 
	if (gear < 1)
	{
		return 1;
	}
	// check if the RPM value of car is greater than the one suggested 
	// to shift up the gear from the current one     
	if (gear < 6 && rpm >= gearUp[gear - 1])
	{
		return gear + 1;
	}
	// check if the RPM value of car is lower than the one suggested 
	// to shift down the gear from the current one
	else if (gear > 1 && rpm <= gearDown[gear - 1])
	{
		return gear - 1;
	}
	else // otherwhise keep current gear
	{
		return gear;
	}
}

float filterABS(structCarState* cs, float brake)
{
	// convert speed to m/s
	float speed = cs->speedX / 3.6f;
	// when speed lower than min speed for abs do nothing
	if (speed < absMinSpeed)
		return brake;

	// compute the speed of wheels in m/s
	float slip = 0.0f;
	for (int i = 0; i < 4; i++)
	{
		slip += cs->wheelSpinVel[i] * wheelRadius[i];
	}
	// slip is the difference between actual speed of car and average speed of wheels
	slip = speed - slip / 4.0f;
	// when slip too high apply ABS
	if (slip > absSlip)
	{
		brake = brake - (slip - absSlip) / absRange;
	}

	// check brake is not negative, otherwise set it to zero
	if (brake < 0)
		return 0;
	else
		return brake;
}

void clutching(structCarState* cs, float* clutch)
{
	float maxClutch = clutchMax;

	// Check if the current situation is the race start
	if (cs->curLapTime < clutchDeltaTime && cs->stage == RACE && cs->distRaced < clutchDeltaRaced)
		*clutch = maxClutch;

	// Adjust the current value of the clutch
	if (clutch > 0)
	{
		float delta = clutchDelta;
		if (cs->gear < 2)
		{
			// Apply a stronger clutch output when the gear is one and the race is just started
			delta /= 2;
			maxClutch *= clutchMaxModifier;
			if (cs->curLapTime < clutchMaxTime)
				*clutch = maxClutch;
		}

		// check clutch is not bigger than maximum values
		*clutch = fmin(maxClutch, *clutch);

		// if clutch is not at max value decrease it quite quickly
		if (*clutch != maxClutch)
		{
			*clutch -= delta;
			*clutch = fmax(0.0, *clutch);
		}
		// if clutch is at max value decrease it very slowly
		else
		{
			*clutch -= clutchDec;
		}
	}
}

float getSteer(structCarState* cs)
{
	// steering angle is compute by correcting the actual car angle w.r.t. to track 
	// axis [cs->angle] and to adjust car position w.r.t to middle of track [cs->trackPos*0.5]
	float targetAngle = (cs->angle - cs->trackPos * 0.5f);
	// at high speed reduce the steering command to avoid loosing the control
	if (cs->speedX > steerSensitivityOffset)
	{
		return targetAngle / (steerLock * (cs->speedX - steerSensitivityOffset) * wheelSensitivityCoeff);
	}
	else
	{
		return (targetAngle) / steerLock;
	}
}

float getAccel(structCarState* cs)
{
	// checks if car is out of track
	if (cs->trackPos < 1 && cs->trackPos > -1)
	{
		// reading of sensor at +5 degree w.r.t. car axis
		float rxSensor = cs->track[10];
		// reading of sensor parallel to car axis
		float cSensor = cs->track[9];
		// reading of sensor at -5 degree w.r.t. car axis
		float sxSensor = cs->track[8];

		float targetSpeed;

		// track is straight and enough far from a turn so goes to max speed
		if (cSensor > maxSpeedDist || (cSensor >= rxSensor && cSensor >= sxSensor))
		{
			targetSpeed = maxSpeed;
		}
		else
		{
			// approaching a turn on right
			if (rxSensor > sxSensor)
			{
				// computing approximately the "angle" of turn
				float h = cSensor * sin5;
				float b = rxSensor - cSensor * cos5;
				float sinAngle = b * b / (h * h + b * b);
				// estimate the target speed depending on turn and on how close it is
				targetSpeed = maxSpeed * (cSensor * sinAngle / maxSpeedDist);
			}
			// approaching a turn on left
			else
			{
				// computing approximately the "angle" of turn
				float h = cSensor * sin5;
				float b = sxSensor - cSensor * cos5;
				float sinAngle = b * b / (h * h + b * b);
				// estimate the target speed depending on turn and on how close it is
				targetSpeed = maxSpeed * (cSensor * sinAngle / maxSpeedDist);
			}

		}

		// accel/brake command is expontially scaled w.r.t. the difference between target speed and current one
		return 2 / (1 + exp(cs->speedX - targetSpeed)) - 1.0f;
	}
	else
	{
		return 0.3f; // when out of track returns a moderate acceleration command
	}
}

void filterSensors(structCarState* cs) {
	for (int i = 0; i < 19; i++)
	{
		if (cs->curLapTime < 0.5f)
		{
			sensor_prev[i] = cs->track[i];
			sensor_prevprev[i] = cs->track[i];
		}
		else
		{
			//printf("%.04f\t%.04f\n", fabs(cs->track[i] - sensor_prev[i]), fabs(sensor_prev[i] - sensor_prevprev[i]));
			if (cs->track[i] > 200 || cs->track[i] < 0)
			{
				cs->track[i] = sensor_prev[i];
			}
			else
			{
				sensor_prevprev[i] = sensor_prev[i];
				sensor_prev[i] = cs->track[i];
			}
		}
	}
}

void customDrive(structCarState* cs, float* accel, float* steer) {
	float speed = cs->speedX;

	//filterSensors(cs);

	// angle: negative to left, positive to right
	// trackpos: positive to left, negative to right
	float angle_err = 0;
	float target_pos = 0;
	float pos_err = -cs->trackPos;
	int steer_brake = 0;

	float rSensor = cs->track[12]; // +5 deg to car axis, but clockwise
	float cSensor = cs->track[9]; // parallel to car axis
	float lSensor = cs->track[6]; // -5 deg to car axis, but clockwise

	float target_speed = 280;
	float sensor_max = fmax(cSensor, fmax(lSensor, rSensor));

	if (firstlap)
	{
		if (sensor_max < 190) {
			if (cSensor < 30) {
				target_speed = cSensor + 30;
			}
			else if (sensor_max < 75)
			{
				target_speed = sensor_max * 1.1f + 30;
			}
			else
			{
				target_speed = sensor_max * 1.3f;
			}
		}
	}
	else
	{
		while (map[map_i].distance < cs->distFromStart && map_i < map_n - 1) {
			map_i++;
		}
		target_speed = map[map_i].speed;
		target_pos = map[map_i].track_pos;
		if (map_i >= map_n - 1 || map[map_i].distance > cs->distFromStart + 1000)
		{
			map_i = 0;
		}
	}

	if (speed < target_speed) {
		*accel = 1;
	}
	else if (speed > (target_speed + 10)) {
		*accel = -1;
	}

	float corner_dir = 1;
	/*
	if (rSensor > lSensor) {
		corner_dir = -1;
	}
	*/

	//target_pos = (200 - sensor_max) * 0.004f + cs->angle; // feed back angle
	pos_err = corner_dir * target_pos - cs->trackPos;
	angle_err = -cs->angle - 0.5 * pos_err;

	*steer = -PI * angle_err / steerLock;
	
	if (firstlap)
	{
		if (firstlap == 1 && cs->distFromStart < 500) {
			firstlap = 2;
		}
		if (cs->distFromStart + 1000 < cs->distRaced) {
			// second lap
			firstlap = 0;
			process_map();
		#ifdef VS_DEBUG
			FILE* f = fopen("debug.txt", "w");
			for (int i = 0; i < map_n; i++)
			{
				fprintf(f, "%.04f,%.04f,%.04f,%.04f,%.04f\n", map[i].distance, map[i].speed, map[i].steer, map[i].accel, map[i].track_pos);
			}
			fclose(f);
		#endif // VS_DEBUG

		}
		if (firstlap == 2 && (map_n == 0 || map[map_n - 1].distance + 1 < cs->distFromStart)) {
			map[map_n].distance = cs->distFromStart;
			map[map_n].speed = speed;
			map[map_n].steer = *steer;
			map[map_n].accel = *accel;
			map[map_n].track_pos = 0;
			map_n++;
		}
	}

	/*
	// at high speed reduce the steering command to avoid loosing the control
	if (speed > steerSensitivityOffset)
	{
		*steer =  -PI * angle_err / (steerLock * (cs->speedX - steerSensitivityOffset));
	}
	else
	{
		*steer = -PI * angle_err / steerLock;
	}
	*/

#ifdef VS_DEBUG
	char str[512] = "";
	sprintf(str,	"Angle: %10.2f     Track pos: %10.2f\n"
					"L-Sensor: %10.2f     C-Sensor: %10.2f     R-Sensor: %10.2f\n"
					"target_speed: %.02f\n"
					"target_pos: %10.2f\n"
					"sensor_max: %.02f\n"
					"map_i: %d\n"
					"dist: %.02f\n"
					"distfromstart: %.02f\n"
					"firstlap: %d"
					//"|s -6;0.5;%f|s -4;0.5;%f|s -2;0.5;%f|s 0;0.5;%f|s +2;0.5;%f|s +4;0.5;%f|s +6;0.5;%f",
					"|steer;100;%f|speed;0.5;%f",
		cs->angle, cs->trackPos, lSensor, cSensor, rSensor, target_speed, target_pos, sensor_max, map_i, cs->distRaced, cs->distFromStart, firstlap,
		//cs->track[6], cs->track[7], cs->track[8], cs->track[9], cs->track[10], cs->track[11], cs->track[12]
		*steer, speed
	);
	sendto(socketDescriptor, str, strlen(str), 0, (struct sockaddr*) & serverAddress, sizeof(serverAddress));
#endif // VS_DEBUG
}

structCarControl CDrive(structCarState cs)
{
	if (cs.stage != cs.prevStage)
	{
		cs.prevStage = cs.stage;
	}

	float gear_m = 1;
	if (REVERSE)
	{
		gear_m = -1;
		if (cs.angle > PI)
		{
			cs.angle -= PI;
		}
		else {
			cs.angle += PI;
		}

		if (fabs(cs.angle) > PI / 2) {
			float accel = 0.3;
			float steer = 1;
			float gear = -1;

			// Calculate clutching
			clutching(&cs, &clutch);

			// build a CarControl variable and return it
			structCarControl cc = { accel, 0.0f, gear, steer, clutch };
			return cc;
		}
	}


	// check if car is currently stuck
	if (fabs(cs.angle) > stuckAngle)
	{
		// update stuck counter
		stuck++;
	}
	else
	{
		// if not stuck reset stuck counter
		stuck = 0;
	}

	// after car is stuck for a while apply recovering policy
	if (stuck > stuckTime)
	{
		/* set gear and steering command assuming car is
		 * pointing in a direction out of track */

		 // to bring car parallel to track axis
		float steer = -cs.angle / steerLock;
		int gear = -1 * gear_m; // gear R

		// if car is pointing in the correct direction revert gear and steer  
		if (cs.angle * cs.trackPos > 0)
		{
			gear = 1 * gear_m;
			steer = -steer;
		}

		// Calculate clutching
		clutching(&cs, &clutch);

		// build a CarControl variable and return it
		structCarControl cc = { 0.3f, 0.0f, gear, steer, clutch };
		return cc;
	}

	else // car is not stuck
	{
		// compute gear 
		int gear = getGear(&cs);

		if (REVERSE)
		{
			gear = -1;
		}

		float accel_and_brake = 0, steer = 0;
		//customDrive(&cs, &accel_and_brake, &steer);
		
		// compute accel/brake command
		//accel_and_brake = getAccel(&cs);
		// compute steering
		//steer = getSteer(&cs);

		// normalize steering
		if (steer < -1)
			steer = -1;
		if (steer > 1)
			steer = 1;

		// set accel and brake from the joint accel/brake command 
		float accel, brake;
		if (accel_and_brake >= 0)
		{
			accel = accel_and_brake;
			brake = 0;
		}
		else
		{
			accel = 0;
			// apply ABS to brake
			brake = filterABS(&cs, -accel_and_brake);
		}

		// Calculate clutching
		clutching(&cs, &clutch);

		// build a CarControl variable and return it
		structCarControl cc = { accel,brake,gear,steer,clutch };
		return cc;
	}
}

//gives 19 angles for the distance sensors
void Cinit(float* angles)
{

#ifdef VS_DEBUG
	socketDescriptor = socket(AF_INET, SOCK_DGRAM, 0);
	struct hostent* hostInfo = gethostbyname("localhost");
	serverAddress.sin_family = hostInfo->h_addrtype;
	memcpy((char*)&serverAddress.sin_addr.s_addr, hostInfo->h_addr_list[0], hostInfo->h_length);
	serverAddress.sin_port = htons(9000);
	/*
	if (firstlap) {
		firstlap = 0;
		FILE* f = fopen("debug_in.txt", "r");
		int r = 5;
		while (r == 5)
		{
			float a=0, b=0, c=0, d=0, e=0;
			r = fscanf(f, "%f%f%f%f%f", &a, &b, &c, &d, &e);
			map[map_n].distance = a;
			map[map_n].speed = b;
			map[map_n].steer = c;
			map[map_n].accel = d;
			map[map_n].track_pos = e;
			//printf("%d: %.02f,%.02f,%.02f,%.02f,%.02f\n", map_n, map[map_n].distance, map[map_n].speed, map[map_n].steer, map[map_n].accel, map[map_n].track_pos);
			map_n++;
		}
		fclose(f);
	}*/
#endif // VS_DEBUG

	/*
	// set angles as {-90,-75,-60,-45,-30,-20,-15,-10,-5,0,5,10,15,20,30,45,60,75,90}
	for (int i = 0; i < 5; i++)
	{
		angles[i] = -90 + i * 15;
		angles[18 - i] = 90 - i * 15;
	}

	for (int i = 5; i < 9; i++)
	{
		angles[i] = -20 + (i - 5) * 5;
		angles[18 - i] = 20 - (i - 5) * 5;
	}
	*/

	// set angles as -18..+18 (2 deg)
	for (int i = 0; i < 9; i++)
	{
		angles[i] = -18 + 2 * i;
		angles[18 - i] = 18 - 2 * i;
	}
	angles[9] = 0;
}

void ConShutdown()
{
	printf("Bye bye!");
}

void ConRestart()
{
	printf("Restarting the race!");
}