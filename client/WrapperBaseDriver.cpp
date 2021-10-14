/***************************************************************************
 
    file                 : WrapperBaseDriver.cpp
    copyright            : (C) 2007 Daniele Loiacono
 
 ***************************************************************************/

/***************************************************************************
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 ***************************************************************************/
#include "WrapperBaseDriver.h"

string
WrapperBaseDriver::drive(string sensors)
{
	structCarState cs;
	cs.stage = this->stage;
	SimpleParser::parse(sensors, "angle", cs.angle);
	SimpleParser::parse(sensors, "curLapTime", cs.curLapTime);
	SimpleParser::parse(sensors, "damage", cs.damage);
	SimpleParser::parse(sensors, "distFromStart", cs.distFromStart);
	SimpleParser::parse(sensors, "distRaced", cs.distRaced);
	SimpleParser::parse(sensors, "focus", cs.focus, FOCUS_SENSORS_NUM);
	SimpleParser::parse(sensors, "fuel", cs.fuel);
	SimpleParser::parse(sensors, "gear", cs.gear);
	SimpleParser::parse(sensors, "lastLapTime", cs.lastLapTime);
	SimpleParser::parse(sensors, "opponents", cs.opponents, OPPONENTS_SENSORS_NUM);
	SimpleParser::parse(sensors, "racePos", cs.racePos);
	SimpleParser::parse(sensors, "rpm", cs.rpm);
	SimpleParser::parse(sensors, "speedX", cs.speedX);
	SimpleParser::parse(sensors, "speedY", cs.speedY);
	SimpleParser::parse(sensors, "speedZ", cs.speedZ);
	SimpleParser::parse(sensors, "track", cs.track, TRACK_SENSORS_NUM);
	SimpleParser::parse(sensors, "trackPos", cs.trackPos);
	SimpleParser::parse(sensors, "wheelSpinVel", cs.wheelSpinVel, 4);
	SimpleParser::parse(sensors, "z", cs.z);

	structCarControl cc = CDrive(cs);

	string str;
	str  = SimpleParser::stringify("accel", cc.accel);
	str += SimpleParser::stringify("brake", cc.brake);
	str += SimpleParser::stringify("gear",  cc.gear);
	str += SimpleParser::stringify("steer", cc.steer);
	str += SimpleParser::stringify("clutch", cc.clutch);
	str += SimpleParser::stringify("focus",  cc.focus);
	str += SimpleParser::stringify("meta", cc.meta);
	
	return str;	
}

void
WrapperBaseDriver::init(float *angles)
{
	Cinit(angles);
}

void
WrapperBaseDriver::onRestart()
{
	ConRestart();
}

void
WrapperBaseDriver::onShutdown()
{
	ConShutdown();
}