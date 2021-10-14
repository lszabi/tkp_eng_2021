/***************************************************************************
 
    file                 : WrapperBaseDriver.h
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
#ifndef WRAPPERBASEDRIVER_H_
#define WRAPPERBASEDRIVER_H_

#include "SimpleParser.h"
#include "CDriver.h"

#include <cmath>
#include <cstdlib>

class WrapperBaseDriver
{
public:
	
	// the drive function with string input and output
	string drive(string sensors);

	// Print a shutdown message 
	void onShutdown();
	
	// Print a restart message 
	void onRestart();

	// Initialization of the desired angles for the rangefinders
	void init(float *angles);

	tstage stage;

	char trackName[100];

};



#endif /*WRAPPERBASEDRIVER_H_*/
