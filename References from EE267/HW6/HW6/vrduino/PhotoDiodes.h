#ifndef _PhotoDiodes_h_
#define _PhotoDiodes_h_

#include "LighthouseSensor.h"

/*
	Photodiode Ordering when looking at the front side of the VRduino
	with the text right side up.

	[0]-----------------[1]
	 |					 |
	 |		Vrduino		 |
	 |		Front		 |
	 |		Face		 |
	[3]-----------------[2]
*/

class PhotoDiodes
{
private:
	// Debug variable to turn on print statements for lighthouse settings and photodiode readings
	bool debug = false; 

	// Input pin to select the lighthouse to listen to
	int lightHouseSelectPin_ = 11;

	// Photodiode corresponding to the lighthouse select pin
	// (either 0 for Lighthouse A/B or 1 for Lighthouse C)
	int lighthouseToWatch_;

	// Int to hold which lighthouse the values were recorded from. 0 for B, 1 for C
	int lighthousePolled_;

	// Output pin to enable or disable the TPS6366
	int standbyPin_ = 12;

	// Array of 4 photodiodes
	LighthouseSensor sensors_[4];

	// Flags indicated x and y tick values for photodiodes have been updated
	// Formatted : [x_0 y_0 x_1 y_1 x_2 y_2 x_3 y_3]
	int updatedDiodes[8] = {0, 0, 0, 0, 0, 0, 0, 0};

public:
	// Array of raw clock ticks from each individual photodiode
	// Formatted : [x_0 y_0 x_1 y_1 x_2 y_2 x_3 y_3]
	uint32_t clockTicks_[8];

	// Array of 2D measurements on plane at unit distance
	// from each individual photodiode
	// Formatted : [x_0 y_0 x_1 y_1 x_2 y_2 x_3 y_3]
	float projection2D_[8];

	// Coordinates of the photodiodes on the VRduino.
	// Formatted : [x_0 y_0 x_1 y_1 x_2 y_2 x_3 y_3]
	float locations_[8] =	{-42.0, 25.0, 42.0, 25.0, 42.0, -25.0, -42.0, -25.0};

	PhotoDiodes() {};
	~PhotoDiodes() {};

	// Initializes interrupts for each of the photodiodes, as well as certain
	// pins for light house selection.
	void initTracking();

	// Prints out the base station the photodiodes are currently searching for.
	void printBaseStation();

	// Updates the clockTicks_ member with the the most up-to-date ticks from
	// the 4 photodiodes. Because the Teensy runs at 48Mhz, which is much
	// faster than the operating rates of the lighthouse sweep, it will not
	// always be able to get new values. In each call it will update as many
	// of the clockTicks_ values (x and y for each photodiode) as it can.
	// Only after all 8 values have been updated will the function return true,
	// indicating that the values are up to date and ready to be used for
	// computation.
	bool updateClockTicks();

	// Updates the 2D positions of the lighthouse sweeps on the unit plane,
	// stored in member variable pos2D_ based on whatever is in the current
	// clockTicks_ member. Before calling this function make sure to check
	// that all the photodiodes are up-to-date.
	void update2DPositions();

	// Unit test checking that the update2DPositions() function is correct.
	void unitTest_update2DPositions();
};

#endif