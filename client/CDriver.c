#include "CDriver.h"
#include <math.h>

/* Gear Changing Constants*/
const int gearUp[6] = {7500,8000,8000,8500,8500,0};
const int gearDown[6] = {0,3000,4000,4000,4000,4500};

/* Stuck constants*/
const int stuckTime = 25;
const float stuckAngle = .523598775; //PI/6

/* Accel and Brake Constants*/
const float maxSpeedDist = 70;
const float maxSpeed = 200;
const float sin5 = 0.08716;
const float cos5 = 0.99619;

/* Steering constants*/
const float steerLock = 0.785398;
const float steerSensitivityOffset = 80.0;
const float wheelSensitivityCoeff = 1;

/* ABS Filter Constants */
const float wheelRadius[4] = { 0.3179,0.3179,0.3276,0.3276 };
const float absSlip = 2.0;
const float absRange = 3.0;
const float absMinSpeed = 3.0;

/* Clutch constants */
const float clutchMax = 0.5;
const float clutchDelta = 0.05;
const float clutchRange = 0.82;
const float clutchDeltaTime = 0.02;
const float clutchDeltaRaced = 10;
const float clutchDec = 0.01;
const float clutchMaxModifier = 1.3;
const float clutchMaxTime = 1.5;

int stuck;
float clutch;

/* Filtering */
#define ACCEL_FILT_N 50
float accel_filt[ACCEL_FILT_N] = {0, };
int accel_filt_i = 0;

float accel_int = 0;
float steer_int = 0;

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
	float speed = cs->speedX / 3.6;
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
	float targetAngle = (cs->angle - cs->trackPos * 0.5);
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
		return 2 / (1 + exp(cs->speedX - targetSpeed)) - 1;
	}
	else
	{
		return 0.3; // when out of track returns a moderate acceleration command
	}
}

int msg = 1;

void customDrive(structCarState* cs, float* accel, float* steer) {
	/*
	if (cs->distFromStart < 150 || cs->distFromStart > 2000) {
		*accel = 1;
	}
	else
	{
		if (cs->speedX < 3)
		{
			if (msg == 2)
			{
				printf("Distance: %.02f\n", cs->distFromStart);
				msg = 3;
			}
			*accel = 0;
		}
		else
		{
			if (msg == 1)
			{
				printf("Braking now, speed: %.02f\n", cs->speedX);
				msg = 2;
			}
			*accel = -1;
		}
	}
	*/
	float speed = cs->speedX;

	// angle: negative to left, positive to right
	// trackpos: positive to left, negative to right
	float angle_err = 0;
	float target_pos = 0;
	float pos_err = cs->trackPos;
	int steer_brake = 0;

	// checks if car is out of track
	if (abs(cs->trackPos) < 1)
	{
		float lSensor = cs->track[10]; // +5 deg to car axis
		float cSensor = cs->track[9]; // parallel to car axis
		float rSensor = cs->track[8]; // -5 deg to car axis

		float brake_distance = speed * speed * speed / 20000; // magic formula

		if (cSensor > brake_distance || lSensor > brake_distance || rSensor > brake_distance)
		{
			// pedal to the metal
			*accel = 1;
			/*
			if (speed > 50)
			{
				*accel = 0;
			}
			*/
		}
		else if (cSensor > brake_distance)
		{
			// coast
			*accel = 0;
		}
		else
		{
			// brake
			*accel = -1;
		}

		if (cSensor < lSensor || cSensor < rSensor)
		{
			// corner ahead
			if (lSensor > rSensor)
			{
				// turn left
				target_pos = -0.85;
			}
			else
			{
				// turn right
				target_pos = 0.85;
			}

			pos_err = cs->trackPos - target_pos;
			if (pos_err > 0.3)
			{
				steer_brake = 1;
			}
			if (pos_err < -0.3)
			{
				steer_brake = 1;
			}

			if (steer_brake)
			{
				if (*accel > -0.1)
				{
					*accel -= 0.5;
				}
			}

			angle_err -= 0.5 * pos_err;
		}
		else
		{
			if (angle_err < 0 && pos_err > 0)
			{
				angle_err -= 0.3 * pos_err;
			}
			if (angle_err > 0 && pos_err < 0)
			{
				angle_err -= 0.3 * pos_err;
			}
			if (pos_err > 0.3 || pos_err < -0.3)
			{
				angle_err -= 0.3 * pos_err;
			}
		}
		
		//printf("angle: %3.02f, trackpos: %3.02f\r", angle_err, );

		*steer = angle_err * 1.1;

		/*

		// track is straight and enough far from a turn so goes to max speed
		if (cSensor > brake_distance || (cSensor >= lSensor && cSensor >= rSensor))
		{
			// pedal to the metal
			*accel = 1;

			// steering angle is computed by correcting the actual car angle w.r.t. to track 
			// axis [cs->angle] and to adjust car position w.r.t to middle of track [cs->trackPos*0.5]
			targetAngle = cs->angle - cs->trackPos * 0.5;
		}
		else
		{
			float targetSpeed = 0;
			// approaching a turn on right
			if (lSensor > rSensor)
			{
				// computing approximately the "angle" of turn
				float h = cSensor * sin5;
				float b = rSensor - cSensor * cos5;
				float sinAngle = b * b / (h * h + b * b);
				// estimate the target speed depending on turn and on how close it is
				targetSpeed = maxSpeed * (cSensor * sinAngle / maxSpeedDist);
			}
			// approaching a turn on left
			else
			{
				// computing approximately the "angle" of turn
				float h = cSensor * sin5;
				float b = lSensor - cSensor * cos5;
				float sinAngle = b * b / (h * h + b * b);
				// estimate the target speed depending on turn and on how close it is
				targetSpeed = maxSpeed * (cSensor * sinAngle / maxSpeedDist);
			}
			// accel/brake command is expontially scaled w.r.t. the difference between target speed and current one
			*accel = 2 / (1 + exp(speed - targetSpeed)) - 1;
		}
		*/
	}
	else
	{
		*accel = 0.3; // when out of track returns a moderate acceleration command
	}

	/*
	// at high speed reduce the steering command to avoid loosing the control
	if (speed > steerSensitivityOffset)
	{
		*steer = targetAngle / (steerLock * (cs->speedX - steerSensitivityOffset) * wheelSensitivityCoeff);
	}
	else
	{
		*steer = (targetAngle) / steerLock;
	}
	*/

	/*
	// Filter acceleration
	accel_filt[accel_filt_i] = *accel;
	accel_filt_i = (accel_filt_i + 1) % ACCEL_FILT_N;
	float avg = 0;
	for (int i = 0; i < ACCEL_FILT_N; i++)
	{
		avg += accel_filt[i];
	}
	*accel = avg / ACCEL_FILT_N;
	*/
	//*accel = *accel / 2 + accel_int / 4;
	//accel_int = accel_int / 2 + *accel;
}

structCarControl CDrive(structCarState cs)
{
	if (cs.stage != cs.prevStage)
	{
		cs.prevStage = cs.stage;
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
		int gear = -1; // gear R

		// if car is pointing in the correct direction revert gear and steer  
		if (cs.angle * cs.trackPos > 0)
		{
			gear = 1;
			steer = -steer;
		}

		// Calculate clutching
		clutching(&cs, &clutch);

		// build a CarControl variable and return it
		structCarControl cc = { 1.0f,0.0f,gear,steer,clutch };
		return cc;
	}

	else // car is not stuck
	{
		// compute gear 
		int gear = getGear(&cs);

		float accel_and_brake = 0, steer = 0;
		customDrive(&cs, &accel_and_brake, &steer);
		
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

	// set angles as {-90,-75,-60,-45,-30,20,15,10,5,0,5,10,15,20,30,45,60,75,90}

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