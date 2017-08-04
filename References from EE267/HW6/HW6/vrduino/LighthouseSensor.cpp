#include "LighthouseSensor.h"

#if defined(KINETISK)
//#define CLOCKS_PER_MICROSECOND ((double)F_BUS / 1000000.0)
#define CLOCKS_PER_MICROSECOND (F_BUS / 1000000)
#elif defined(KINETISL)
// PLL is 48 Mhz, which is 24 clocks per microsecond, but
// there is a divide by two for some reason.
#define CLOCKS_PER_MICROSECOND (F_PLL / 2000000)
#endif


void
LighthouseSensor::begin(int id, int icp0, int icp1)
{
	this->id = id;
	this->icp_rising.begin(icp0, RISING);
	this->icp_falling.begin(icp1, FALLING);
}


int
LighthouseSensor::sweep_pulse(
    unsigned val,
    unsigned len,
    unsigned duty
)
{

	// Sweep! The 0 degree mark is when the rotor
	// that was not skipped sent its high pulse.
	// midpoint of the pulse is what we'll use
	unsigned now = val - len / 2;
	unsigned delta = now - this->zero_time;
	//ind depends on which lighthouse the observation was made from. Axis offsets us to the proper h or v sweep.
	const int ind = this->lighthouse*2 + this->axis; 

	// todo: filter if we don't know the axis
	int valid = this->lighthouse != 9 && delta < 8000 * CLOCKS_PER_MICROSECOND;

	if (debug) {
		Serial.print(val);
		Serial.print(", ID: ");
		Serial.print(this->id);
		Serial.print(", Sweep, ind: ");
		Serial.print(ind);
    	Serial.print(", lighthouse: ");
    	Serial.print(lighthouse, HEX);
		Serial.print(", axis: ");
		Serial.print(this->axis);
		Serial.print(", delta: ");
		Serial.print(delta);
		Serial.print(", valid: ");
		Serial.print(valid);
		Serial.print(", ticks:");
		Serial.print(len / CLOCKS_PER_MICROSECOND);
		Serial.println();
#if 0
	} else if (valid) {
		Serial.print(val);
		Serial.print(",");
		Serial.print(i);
		Serial.print(",");
		Serial.print(axis[i]);
		Serial.print(",");
		Serial.print(delta);
		Serial.println();
#endif
	}

	// flag that we have the sweep for this one already
	this->got_sweep = 1;
	this->got_skip = this->got_not_skip = 0;
	this->lighthouse = 9; 

	if (!valid) return -1;

	// update our angle measurement (raw and floating point)
	this->raw[ind] = delta;

	// let the caller know that we have a new valid measurement
	// returns 0, 1, 2, or 3 for Bx By Cx Cy
	return ind;
}


int LighthouseSensor::poll() {
	uint32_t val;
	int rc = this->icp_falling.read(&val);
	if (rc != 0)
	{
		// we have a falling edge pulse, store the time stamp
		this->last_falling = val;
	}

	rc = this->icp_rising.read(&val);
	if (rc <= 0)
		return -1;

	// We have a rising edge pulse, process it
	const uint32_t len = val - this->last_falling;
	const uint32_t duty = val - this->last_rising;
	this->last_rising = val;

	// short pulse means sweep by the laser.
	if (len < 15 * CLOCKS_PER_MICROSECOND)
		return this->sweep_pulse(val, len, duty);


	// this is our first non-sweep pulse,
	// reset our parameters to wait for our next sync.
	if (this->got_sweep || duty > 800 * CLOCKS_PER_MICROSECOND) {
		this->lighthouse = 9;  // invalid
		this->got_sweep = this->got_skip = this->got_not_skip = 0;
	}

	int skip = 9;
	int rotor = 9;
	int data = 9;
	const char * name = "??";

	const unsigned window = 4 * CLOCKS_PER_MICROSECOND;

	static const unsigned midpoints[] = {
		(unsigned) (62.5 * CLOCKS_PER_MICROSECOND),
		(unsigned) (83.3 * CLOCKS_PER_MICROSECOND),
		(unsigned) (72.9 * CLOCKS_PER_MICROSECOND),
		(unsigned) (93.8 * CLOCKS_PER_MICROSECOND),
		(unsigned) (104.0 * CLOCKS_PER_MICROSECOND),
		(unsigned) (125.0 * CLOCKS_PER_MICROSECOND),
		(unsigned) (115.0 * CLOCKS_PER_MICROSECOND),
		(unsigned) (135.0 * CLOCKS_PER_MICROSECOND),
	};

	static const char * const names[] = {"j0", "j1", "k0", "k1", "j2", "j3", "k2", "k3",};


	for (int i = 0 ; i < 8 ; i++)
	{

		if (len < midpoints[i] - window)
			continue;
		if (len > midpoints[i] + window)
			continue;

		skip = (i >> 2) & 1;
		rotor = (i >> 1) & 1;
		data = (i >> 0) & 1;
		name = names[i];
		break;
	}

	if (skip == 0)
	{
		// store the time of the rising edge of this pulse
		// and the rotor that is being sent
		this->zero_time = last_falling;
		this->axis = rotor;
		this->got_not_skip = 1; //commented out for b testing

		// if we have already seen the skip sync pluse,
		// then this is lighthouse 0,
		if (this->got_skip) {
			this->lighthouse = 0;
			//Serial.println((unsigned) data);
		}
	} else {
		this->got_skip = 1;
		// if we have already seen the not-skip sync pulse,
		// then this is lighthouse 1
		if (this->got_not_skip)
			this->lighthouse = 1;
	}



	if (debug) {
		Serial.print(val);
		Serial.print(", ID: ");
		Serial.print(this->id);
		Serial.print(", X broadband, name: ");
		Serial.print(name);
    	Serial.print(", skip: ");
		Serial.print(skip);
		Serial.print(", rotor: ");
		Serial.print(rotor);
		Serial.print(", data: ");
		Serial.print(data);
		Serial.print(", ticks: ");
		Serial.print(len / CLOCKS_PER_MICROSECOND);
		Serial.println();
	}

	// no new samples
	return -1;
}
