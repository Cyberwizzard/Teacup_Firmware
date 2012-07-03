#include	"dda.h"

/** \file
	\brief Digital differential analyser - this is where we figure out which steppers need to move, and when they need to move
*/

#include	<string.h>
#include	<stdlib.h>
#include	<math.h>
#include	<avr/interrupt.h>

#include	"timer.h"
#include	"delay.h"
#include	"serial.h"
#include	"sermsg.h"
#include	"gcode_parse.h"
#include	"dda_queue.h"
#include	"debug.h"
#include	"sersendf.h"
#include	"pinio.h"
#include	"config.h"
//#include "graycode.c"

#ifndef ACCELERATION_RAMPING
// Only enable the lookahead bits if ramping acceleration is enabled
#undef LOOKAHEAD
#else
// Note: the floating point bit is optimized away during compilation
#define ACCELERATE_RAMP_LEN(speed) (((speed)*(speed)) / (uint32_t)((7200000.0f * ACCELERATION) / (float)STEPS_PER_M_X))
// This is the same to ACCELERATE_RAMP_LEN but now the steps per m can be switched.
// Note: use this with a macro so the float is removed by the preprocessor
#define ACCELERATE_RAMP_SCALER(spm) (uint32_t)((7200000.0f * ACCELERATION) / (float)spm)
#define ACCELERATE_RAMP_LEN2(speed, scaler) (((speed)*(speed)) / (scaler))

// Pre-calculated factors to determine ramp lengths for all axis
#define ACCELERATE_SCALER_X ACCELERATE_RAMP_SCALER(STEPS_PER_M_X)
#define ACCELERATE_SCALER_Y ACCELERATE_RAMP_SCALER(STEPS_PER_M_Y)
#define ACCELERATE_SCALER_Z ACCELERATE_RAMP_SCALER(STEPS_PER_M_Z)
#define ACCELERATE_SCALER_E ACCELERATE_RAMP_SCALER(STEPS_PER_M_E)
#endif

#ifdef LOOKAHEAD
// Used for look-ahead debugging
#ifdef LOOKAHEAD_DEBUG_VERBOSE
	#define serprintf(...) sersendf_P(__VA_ARGS__)
#else
	#define serprintf(...)
#endif
#endif

#ifdef	DC_EXTRUDER
	#include	"heater.h"
#endif

#ifdef STEPS_PER_MM_Y
#error STEPS_PER_MM_Y is gone, review your config.h and use STEPS_PER_M_Y
#endif

/// step timeout
volatile uint8_t	steptimeout = 0;

/*
	position tracking
*/

/// \var startpoint
/// \brief target position of last move in queue
TARGET startpoint __attribute__ ((__section__ (".bss")));

/// \var startpoint_steps
/// \brief target position of last move in queue, expressed in steps
TARGET startpoint_steps __attribute__ ((__section__ (".bss")));

/// \var current_position
/// \brief actual position of extruder head
/// \todo make current_position = real_position (from endstops) + offset from G28 and friends
TARGET current_position __attribute__ ((__section__ (".bss")));

/// \var move_state
/// \brief numbers for tracking the current state of movement
MOVE_STATE move_state __attribute__ ((__section__ (".bss")));

/*
	utility functions
*/

// courtesy of http://www.flipcode.com/archives/Fast_Approximate_Distance_Functions.shtml
/*! linear approximation 2d distance formula
	\param dx distance in X plane
	\param dy distance in Y plane
	\return 3-part linear approximation of \f$\sqrt{\Delta x^2 + \Delta y^2}\f$

	see http://www.flipcode.com/archives/Fast_Approximate_Distance_Functions.shtml
*/
uint32_t approx_distance( uint32_t dx, uint32_t dy )
{
	uint32_t min, max, approx;

	if ( dx < dy )
	{
		min = dx;
		max = dy;
	} else {
		min = dy;
		max = dx;
	}

	approx = ( max * 1007 ) + ( min * 441 );
	if ( max < ( min << 4 ))
		approx -= ( max * 40 );

	// add 512 for proper rounding
	return (( approx + 512 ) >> 10 );
}

// courtesy of http://www.oroboro.com/rafael/docserv.php/index/programming/article/distance
/*! linear approximation 3d distance formula
	\param dx distance in X plane
	\param dy distance in Y plane
	\param dz distance in Z plane
	\return 3-part linear approximation of \f$\sqrt{\Delta x^2 + \Delta y^2 + \Delta z^2}\f$

	see http://www.oroboro.com/rafael/docserv.php/index/programming/article/distance
*/
uint32_t approx_distance_3( uint32_t dx, uint32_t dy, uint32_t dz )
{
	uint32_t min, med, max, approx;

	if ( dx < dy )
	{
		min = dy;
		med = dx;
	} else {
		min = dx;
		med = dy;
	}

	if ( dz < min )
	{
		max = med;
		med = min;
		min = dz;
	} else if ( dz < med ) {
		max = med;
		med = dz;
	} else {
		max = dz;
	}

	approx = ( max * 860 ) + ( med * 851 ) + ( min * 520 );
	if ( max < ( med << 1 )) approx -= ( max * 294 );
	if ( max < ( min << 2 )) approx -= ( max * 113 );
	if ( med < ( min << 2 )) approx -= ( med *  40 );

	// add 512 for proper rounding
	return (( approx + 512 ) >> 10 );
}

/*!
	integer square root algorithm
	\param a find square root of this number
	\return sqrt(a - 1) < returnvalue <= sqrt(a)

	see http://www.embedded-systems.com/98/9802fe2.htm
*/
// courtesy of http://www.embedded-systems.com/98/9802fe2.htm
uint16_t int_sqrt(uint32_t a) {
	uint32_t rem = 0;
	uint32_t root = 0;
	uint16_t i;

	for (i = 0; i < 16; i++) {
		root <<= 1;
		rem = ((rem << 2) + (a >> 30));
		a <<= 2;
		root++;
		if (root <= rem) {
			rem -= root;
			root++;
		}
		else
			root--;
	}
	return (uint16_t) ((root >> 1) & 0xFFFFL);
}

// this is an ultra-crude pseudo-logarithm routine, such that:
// 2 ^ msbloc(v) >= v
/*! crude logarithm algorithm
	\param v value to find \f$log_2\f$ of
	\return floor(log(v) / log(2))
*/
const uint8_t	msbloc (uint32_t v) {
	uint8_t i;
	uint32_t c;
	for (i = 31, c = 0x80000000; i; i--) {
		if (v & c)
			return i;
		c >>= 1;
	}
	return 0;
}

#ifdef LOOKAHEAD
// We also need the inverse: given a ramp length, determine the expected speed
// Note: the calculation is scaled by a factor 10000 to obtain an answer with a smaller
// rounding error.
// Warning: this is an expensive function as it requires a square root to get the result.
//
uint32_t dda_steps_to_velocity(uint32_t steps) {
	// v(t) = a*t, with v in mm/s and a = acceleration in mm/s²
	// s(t) = 1/2*a*t² with s (displacement) in mm
	// Rewriting yields v(s) = sqrt(2*a*s)
	// Rewriting into steps and seperation in constant part and dynamic part:
	// F_steps = sqrt((2000*a)/STEPS_PER_M_X) * 60 * sqrt(steps)
	static uint32_t part = 0;
	if(part == 0)
		part = int_sqrt((uint32_t)((2000.0f*ACCELERATION*3600.0f*10000.0f)/(float)STEPS_PER_M_X));
	uint32_t res = int_sqrt((steps) * 10000) * part;
	return res / 10000;
}

/**
 * Determine the 'jerk' between 2 vectors and their speeds. The jerk can be used to obtain an
 * acceptable speed for changing directions between moves.
 * @param x1 x component of first vector
 * @param y1 y component of first vector
 * @param F1 feed rate of first move
 * @param x2 x component of second vector
 * @param y2 y component of second vector
 * @param F2 feed rate of second move
 */
int dda_jerk_size_2d(int32_t x1, int32_t y1, uint32_t F1, int32_t x2, int32_t y2, uint32_t F2) {
	const int maxlen = 10000;
	// Normalize vectors so their length will be fixed
	// Note: approx_distance is not precise enough and may result in violent direction changes
	//sersendf_P(PSTR("Input vectors: (%ld, %ld) and (%ld, %ld)\r\n"),x1,y1,x2,y2);
	int32_t len = int_sqrt(x1*x1+y1*y1);
	x1 = (x1 * maxlen) / len;
	y1 = (y1 * maxlen) / len;
	len = int_sqrt(x2*x2+y2*y2);
	x2 = (x2 * maxlen) / len;
	y2 = (y2 * maxlen) / len;

	//sersendf_P(PSTR("Normalized vectors: (%ld, %ld) and (%ld, %ld)\r\n"),x1,y1,x2,y2);

	// Now scale the normalized vectors by their speeds
	x1 *= F1; y1 *= F1; x2 *= F2; y2 *= F2;

	//sersendf_P(PSTR("Speed vectors: (%ld, %ld) and (%ld, %ld)\r\n"),x1,y1,x2,y2);

	// The difference between the vectors actually depicts the jerk
	x1 -= x2; y1 -= y2;

	//sersendf_P(PSTR("Jerk vector: (%ld, %ld)\r\n"),x1,y1);

	return approx_distance(x1,y1) / maxlen;
}
#endif

/**
 * Safety procedure: if something goes wrong, for example an opto is triggered during normal movement,
 * we shut down the entire machine.
 * @param msg The reason why the machine did an emergency stop
 */
void dda_emergency_shutdown(PGM_P msg) {
	// Todo: is it smart to enable all interrupts again? e.g. can we create concurrent executions?
	sei();	// Enable interrupts to print the message
	serial_writestr_P(PSTR("error: emergency stop:"));
	if(msg!=NULL) serial_writestr_P(msg);
	serial_writestr_P(PSTR("\r\n"));
	delay(20000);		// Delay so the buffer can be flushed - otherwise the message is never sent
	timer_stop();
	queue_flush();
	power_off();
	cli();
	for (;;) { }
}

/*! Inititalise DDA movement structures
*/
void dda_init(void) {
	// set up default feedrate
	if (startpoint.F == 0)
		startpoint.F = next_target.target.F = SEARCH_FEEDRATE_Z;

	#ifdef ACCELERATION_RAMPING
		move_state.n = 1;
		move_state.c = ((uint32_t)((double)F_CPU / sqrt((double)(STEPS_PER_M_X * ACCELERATION / 1000.)))) << 8;
	#endif
}

/*! Distribute a new startpoint to DDA's internal structures without any movement.

	This is needed for example after homing or a G92. The new location must be in startpoint already.
*/
void dda_new_startpoint(void) {
	um_to_steps_x(startpoint_steps.X, startpoint.X);
	um_to_steps_y(startpoint_steps.Y, startpoint.Y);
	um_to_steps_z(startpoint_steps.Z, startpoint.Z);
	um_to_steps_e(startpoint_steps.E, startpoint.E);
}

/*! CREATE a dda given current_position and a target, save to passed location so we can write directly into the queue
	\param *dda pointer to a dda_queue entry to overwrite
	\param *target the target position of this move

	\ref startpoint the beginning position of this move

	This function does a /lot/ of math. It works out directions for each axis, distance travelled, the time between the first and second step

	It also pre-fills any data that the selected accleration algorithm needs, and can be pre-computed for the whole move.

	This algorithm is probably the main limiting factor to print speed in terms of firmware limitations
*/
void dda_create(DDA *dda, TARGET *target, DDA *prev_dda) {
	uint32_t	steps, x_delta_um, y_delta_um, z_delta_um, e_delta_um;
	uint32_t	distance, c_limit, c_limit_calc;
	enum axis_e {X,Y,Z,E} leading = X;	// Used to keep track of the leading axis (= axis with most steps)
	#ifdef ACCELERATION_RAMPING
	uint32_t ramp_scaler = 1;			// Used in the calculation for ramp length - calculated based on leading axis
	#endif
	#ifdef LOOKAHEAD
		// All values in um
		static int32_t x_delta_old = 0,y_delta_old = 0,z_delta_old = 0;
		int32_t x_delta,y_delta,z_delta;
		int32_t jerk = -1;					// Negative jerk means no lookahead
		static uint32_t moveno = 1;			// Used for debugging the look ahead code (by numbering moves)
		static uint32_t la_cnt = 0;			// Counter: how many moves did we join?

		// Calculating the look-ahead settings can take a while; before modifying
		// the previous move, we need to locally store any values and write them
		// when we are done (and the previous move is not already active).
		uint32_t prev_F, prev_F_start, prev_F_end, prev_rampup, prev_rampdown, prev_total_steps;
		uint32_t this_F_start, this_rampup, this_rampdown;
	#endif

	// initialize DDA to a known state
	dda->allflags = 0;

	if (DEBUG_DDA && (debug_flags & DEBUG_DDA))
		serial_writestr_P(PSTR("\n{DDA_CREATE: ["));

	// we end at the passed target
	memcpy(&(dda->endpoint), target, sizeof(TARGET));

	#ifdef LOOKAHEAD
	// Set the start and stop speeds to zero for this move (will be updated later by look-ahead)
	// Note: original behavior - full stops between moves; if the lookahead fails to finish in time
	dda->F_start = 0;
	dda->F_end = 0;
	#endif

// TODO TODO: We should really make up a loop for all axes.
//            Think of what happens when a sixth axis (multi colour extruder)
//            appears?
	x_delta_um = (uint32_t)labs(target->X - startpoint.X);
	y_delta_um = (uint32_t)labs(target->Y - startpoint.Y);
	z_delta_um = (uint32_t)labs(target->Z - startpoint.Z);

	#ifdef LOOKAHEAD
	// Look ahead vectorization: determine the vectors for this move
	x_delta = target->X - startpoint.X;
	y_delta = target->Y - startpoint.Y;
	z_delta = target->Z - startpoint.Z;
	#endif

	um_to_steps_x(steps, target->X);
	dda->x_delta = labs(steps - startpoint_steps.X);
	startpoint_steps.X = steps;
	um_to_steps_y(steps, target->Y);
	dda->y_delta = labs(steps - startpoint_steps.Y);
	startpoint_steps.Y = steps;
	um_to_steps_z(steps, target->Z);
	dda->z_delta = labs(steps - startpoint_steps.Z);
	startpoint_steps.Z = steps;

	dda->x_direction = (target->X >= startpoint.X)?1:0;
	dda->y_direction = (target->Y >= startpoint.Y)?1:0;
	dda->z_direction = (target->Z >= startpoint.Z)?1:0;

	if (target->e_relative) {
		e_delta_um = labs(target->E);
		um_to_steps_e(dda->e_delta, e_delta_um);
		dda->e_direction = (target->E >= 0)?1:0;
	}
	else {
		e_delta_um = (uint32_t)labs(target->E - startpoint.E);
		um_to_steps_e(steps, target->E);
		dda->e_delta = labs(steps - startpoint_steps.E);
		startpoint_steps.E = steps;
		dda->e_direction = (target->E >= startpoint.E)?1:0;
	}

	if (DEBUG_DDA && (debug_flags & DEBUG_DDA))
		sersendf_P(PSTR("%ld,%ld,%ld,%ld] ["), target->X - startpoint.X, target->Y - startpoint.Y, target->Z - startpoint.Z, target->E - startpoint.E);

	dda->total_steps = dda->x_delta;
	if (dda->y_delta > dda->total_steps) {
		dda->total_steps = dda->y_delta;
		leading = Y;
	}
	if (dda->z_delta > dda->total_steps) {
		dda->total_steps = dda->z_delta;
		leading = Z;
	}
	if (dda->e_delta > dda->total_steps) {
		dda->total_steps = dda->e_delta;
		leading = Y;
	}

	if (DEBUG_DDA && (debug_flags & DEBUG_DDA))
		sersendf_P(PSTR("ts:%lu"), dda->total_steps);

	if (dda->total_steps == 0) {
		dda->nullmove = 1;
	}
	else {
		// get steppers ready to go
		steptimeout = 0;
		power_on();
		stepper_enable();
		x_enable();
		y_enable();
		// Z is enabled in dda_start()
		e_enable();

		// since it's unusual to combine X, Y and Z changes in a single move on reprap, check if we can use simpler approximations before trying the full 3d approximation.
		if (z_delta_um == 0)
			distance = approx_distance(x_delta_um, y_delta_um);
		else if (x_delta_um == 0 && y_delta_um == 0)
			distance = z_delta_um;
		else
			distance = approx_distance_3(x_delta_um, y_delta_um, z_delta_um);

		if (distance < 2)
			distance = e_delta_um;

		if (DEBUG_DDA && (debug_flags & DEBUG_DDA))
			sersendf_P(PSTR(",ds:%lu"), distance);

		#ifdef	ACCELERATION_TEMPORAL
			// bracket part of this equation in an attempt to avoid overflow: 60 * 16MHz * 5mm is >32 bits
			uint32_t move_duration, md_candidate;

			move_duration = distance * ((60 * F_CPU) / (target->F * 1000UL));
			md_candidate = dda->x_delta * ((60 * F_CPU) / (MAXIMUM_FEEDRATE_X * 1000UL));
			if (md_candidate > move_duration)
				move_duration = md_candidate;
			md_candidate = dda->y_delta * ((60 * F_CPU) / (MAXIMUM_FEEDRATE_Y * 1000UL));
			if (md_candidate > move_duration)
				move_duration = md_candidate;
			md_candidate = dda->z_delta * ((60 * F_CPU) / (MAXIMUM_FEEDRATE_Z * 1000UL));
			if (md_candidate > move_duration)
				move_duration = md_candidate;
			md_candidate = dda->e_delta * ((60 * F_CPU) / (MAXIMUM_FEEDRATE_E * 1000UL));
			if (md_candidate > move_duration)
				move_duration = md_candidate;
		#else
			// pre-calculate move speed in millimeter microseconds per step minute for less math in interrupt context
			// mm (distance) * 60000000 us/min / step (total_steps) = mm.us per step.min
			//   note: um (distance) * 60000 == mm * 60000000
			// so in the interrupt we must simply calculate
			// mm.us per step.min / mm per min (F) = us per step

			// break this calculation up a bit and lose some precision because 300,000um * 60000 is too big for a uint32
			// calculate this with a uint64 if you need the precision, but it'll take longer so routines with lots of short moves may suffer
			// 2^32/6000 is about 715mm which should be plenty

			// changed * 10 to * (F_CPU / 100000) so we can work in cpu_ticks rather than microseconds.
			// timer.c setTimer() routine altered for same reason

			// changed distance * 6000 .. * F_CPU / 100000 to
			//         distance * 2400 .. * F_CPU / 40000 so we can move a distance of up to 1800mm without overflowing
			uint32_t move_duration = ((distance * 2400) / dda->total_steps) * (F_CPU / 40000);
		#endif

		// similarly, find out how fast we can run our axes.
		// do this for each axis individually, as the combined speed of two or more axes can be higher than the capabilities of a single one.
		c_limit = 0;
		// check X axis
		c_limit_calc = ((x_delta_um * 2400L) / dda->total_steps * (F_CPU / 40000) / MAXIMUM_FEEDRATE_X) << 8;
		if (c_limit_calc > c_limit)
			c_limit = c_limit_calc;
		// check Y axis
		c_limit_calc = ((y_delta_um * 2400L) / dda->total_steps * (F_CPU / 40000) / MAXIMUM_FEEDRATE_Y) << 8;
		if (c_limit_calc > c_limit)
			c_limit = c_limit_calc;
		// check Z axis
		c_limit_calc = ((z_delta_um * 2400L) / dda->total_steps * (F_CPU / 40000) / MAXIMUM_FEEDRATE_Z) << 8;
		if (c_limit_calc > c_limit)
			c_limit = c_limit_calc;
		// check E axis
		c_limit_calc = ((e_delta_um * 2400L) / dda->total_steps * (F_CPU / 40000) / MAXIMUM_FEEDRATE_E) << 8;
		if (c_limit_calc > c_limit)
			c_limit = c_limit_calc;

		#ifdef ACCELERATION_REPRAP
		// c is initial step time in IOclk ticks
		dda->c = (move_duration / startpoint.F) << 8;
		if (dda->c < c_limit)
			dda->c = c_limit;
		dda->end_c = (move_duration / target->F) << 8;
		if (dda->end_c < c_limit)
			dda->end_c = c_limit;

		if (DEBUG_DDA && (debug_flags & DEBUG_DDA))
			sersendf_P(PSTR(",md:%lu,c:%lu"), move_duration, dda->c >> 8);

		if (dda->c != dda->end_c) {
			uint32_t stF = startpoint.F / 4;
			uint32_t enF = target->F / 4;
			// now some constant acceleration stuff, courtesy of http://www.embedded.com/columns/technicalinsights/56800129?printable=true
			uint32_t ssq = (stF * stF);
			uint32_t esq = (enF * enF);
			int32_t dsq = (int32_t) (esq - ssq) / 4;

			uint8_t msb_ssq = msbloc(ssq);
			uint8_t msb_tot = msbloc(dda->total_steps);

			// the raw equation WILL overflow at high step rates, but 64 bit math routines take waay too much space
			// at 65536 mm/min (1092mm/s), ssq/esq overflows, and dsq is also close to overflowing if esq/ssq is small
			// but if ssq-esq is small, ssq/dsq is only a few bits
			// we'll have to do it a few different ways depending on the msb locations of each
			if ((msb_tot + msb_ssq) <= 30) {
				// we have room to do all the multiplies first
				if (DEBUG_DDA && (debug_flags & DEBUG_DDA))
					serial_writechar('A');
				dda->n = ((int32_t) (dda->total_steps * ssq) / dsq) + 1;
			}
			else if (msb_tot >= msb_ssq) {
				// total steps has more precision
				if (DEBUG_DDA && (debug_flags & DEBUG_DDA))
					serial_writechar('B');
				dda->n = (((int32_t) dda->total_steps / dsq) * (int32_t) ssq) + 1;
			}
			else {
				// otherwise
				if (DEBUG_DDA && (debug_flags & DEBUG_DDA))
					serial_writechar('C');
				dda->n = (((int32_t) ssq / dsq) * (int32_t) dda->total_steps) + 1;
			}

			if (DEBUG_DDA && (debug_flags & DEBUG_DDA))
				sersendf_P(PSTR("\n{DDA:CA end_c:%lu, n:%ld, md:%lu, ssq:%lu, esq:%lu, dsq:%lu, msbssq:%u, msbtot:%u}\n"), dda->end_c >> 8, dda->n, move_duration, ssq, esq, dsq, msb_ssq, msb_tot);

			dda->accel = 1;
		}
		else
			dda->accel = 0;
		#elif defined ACCELERATION_RAMPING
// remove this when people have swallowed the new config item
#ifdef ACCELERATION_STEEPNESS
#error ACCELERATION_STEEPNESS is gone, review your config.h and use ACCELERATION
#endif
			// yes, this assumes always the x axis as the critical one regarding acceleration. If we want to implement per-axis acceleration, things get tricky ...
			dda->c_min = (move_duration / target->F) << 8;
			if (dda->c_min < c_limit)
				dda->c_min = c_limit;
			// This section is plain wrong, like in it's only half of what we need. This factor 960000 is dependant on STEPS_PER_MM.
			// overflows at target->F > 65535; factor 16. found by try-and-error; will overshoot target speed a bit
			//dda->rampup_steps = target->F * target->F / (uint32_t)(STEPS_PER_M_X * ACCELERATION / 960000.);
			//sersendf_P(PSTR("rampup calc %lu\n"), dda->rampup_steps);
			//dda->rampup_steps = 100000; // replace mis-calculation by a safe value
			// End of wrong section.

			// Since we use the leading axis multiple times, assign it to a variable
			switch(leading) {
				case X:	ramp_scaler = ACCELERATE_SCALER_X; break;
				case Y:	ramp_scaler = ACCELERATE_SCALER_Y; break;
				case Z:	ramp_scaler = ACCELERATE_SCALER_Z; break;
				case E:	ramp_scaler = ACCELERATE_SCALER_E; break;
			}

			/**
			 * Assuming: F is in mm/min, STEPS_PER_M_X is in steps/m, ACCELERATION is in mm/s²
			 * Given:
			 * - Velocity v at time t given acceleration a: v(t) = a*t
			 * - Displacement s at time t given acceleration a: s(t) = 1/2 * a * t²
			 * - Displacement until reaching target velocity v: s = 1/2 * (v² / a)
			 * - Final result: steps needed to reach velocity v given acceleration a:
			 * steps = (STEPS_PER_M_X * F^2) / (7200000 * ACCELERATION)
			 * To keep precision, break up in floating point and integer part: F^2 * (int)(STEPS_PER_M_X / (7200000 * ACCELERATION))
			 * Note: the floating point part is static so its calculated during compilation.
			 * Note 2: the floating point part will be smaller than one, invert it:
			 * steps = F^2 / (int)((7200000 * ACCELERATION) / STEPS_PER_M_X)
			 * Note 3: As mentioned, setting F to 65535 or larger will overflow the calculation. Make sure this does not happen.
			 * Note 4: General remark, anyone trying to run their machine at 65535 mm/min > 1m/s is nuts
			 *
			 * ((speed*speed) / (uint32_t)((7200000.0f * ACCELERATION) / (float)STEPS_PER_M_X))
			 */
			if(target->F > 65534) target->F = 65534;
			dda->rampup_steps = ACCELERATE_RAMP_LEN2(target->F, ramp_scaler);
			if (dda->rampup_steps > dda->total_steps / 2)
				dda->rampup_steps = dda->total_steps / 2;
			// Mirror the ramp up
			dda->rampdown_steps = dda->total_steps - dda->rampup_steps;

			#ifdef LOOKAHEAD
			// Look-ahead: attempt to join moves into smooth movements
			// Note: moves are only modified after the calculations are complete.
			// Only prepare for look-ahead if we have 2 available moves to
			// join and the Z axis is unused (for now, Z axis moves are NOT joined).
			if(prev_dda!=NULL && prev_dda->live==0 && z_delta_old==z_delta) {
				// Calculate the jerk if the previous move and this move would be joined
				// together at full speed.
				// Todo: since we regard the deltas as unitless vectors, perhaps we could
				// switch to the steps instead? Would that be useful (less chance on overflow)?
				jerk = dda_jerk_size_2d(x_delta_old, y_delta_old, prev_dda->endpoint.F,
						x_delta, y_delta, target->F);
				serprintf(PSTR("Jerk: %lu\r\n"), jerk);
			} else
				serprintf(PSTR("Move already active - no LA\r\n"));

			// Note: jerk will be -1 if we do not have 2 moves to join
			if(prev_dda!=NULL && prev_dda->live==0 && jerk >= 0) {
				// Copy the values to preserve them during the calculations
				prev_F = prev_dda->endpoint.F;
				prev_F_start = prev_dda->F_start;
				prev_F_end = prev_dda->F_end;
				prev_rampup = prev_dda->rampup_steps;
				prev_rampdown = prev_dda->rampdown_steps;
				prev_total_steps = prev_dda->total_steps;

				// The initial crossing speed is the minimum between both target speeds
				// Note: this is a given: the start speed and end speed can NEVER be
				// higher than the target speed in a move!
				// Note 2: this provides an upper limit, if needed, the speed is lowered.
				uint32_t crossF = prev_F;
				if(crossF > target->F) crossF = target->F;

				// If the jerk is too big, scale the proposed cross speed
				if(jerk > LOOKAHEAD_MAX_JERK) {
					serprintf(PSTR("Jerk too big: scale cross speed between moves\r\n"));
					// Get the highest speed between both moves
					if(crossF < prev_F)
						crossF = prev_F;

					// Perform an exponential scaling
					uint32_t ujerk = (uint32_t)jerk;	// Use unsigned to double the range before overflowing
					crossF = (crossF*LOOKAHEAD_MAX_JERK*LOOKAHEAD_MAX_JERK)/(ujerk*ujerk);

					// Safety: make sure we never exceed the maximum speed of a move
					if(crossF > target->F) crossF = target->F;
					if(crossF > prev_F) crossF = prev_F;
				}

				// Show the proposed crossing speed - this might get adjusted below
				serprintf(PSTR("Initial crossing speed: %lu\r\n"), crossF);

				// Forward check: test if we can actually reach the target speed in the previous move
				// If not: we need to determine the obtainable speed and adjust crossF accordingly.
				// Note: these ramps can be longer than the move: if so we can not reach top speed.
				uint32_t up = ACCELERATE_RAMP_LEN2(prev_F, ramp_scaler) - ACCELERATE_RAMP_LEN2(prev_F_start, ramp_scaler);
				uint32_t down = ACCELERATE_RAMP_LEN2(prev_F, ramp_scaler) - ACCELERATE_RAMP_LEN2(crossF, ramp_scaler);
				// Test if both the ramp up and ramp down fit within the move
				if(up+down > prev_total_steps) {
					// Test if we can reach the crossF rate: if the difference between both ramps is larger
					// than the move itself, there is no ramp up or down from F_start to crossF...
					uint32_t diff = (up>down) ? up-down : down-up;
					if(diff > prev_total_steps) {
						// Cannot reach crossF from F_start, lower crossF and adjust both ramp-up and down
						down = 0;
						// Before we can determine how fast we can go in this move, we need the number of
						// steps needed to reach the entry speed.
						uint32_t prestep = ACCELERATE_RAMP_LEN2(prev_F_start, ramp_scaler);
						// Calculate what feed rate we can reach during this move
						crossF = dda_steps_to_velocity(prestep+prev_total_steps);
						// Make sure we do not exceed the target speeds
						if(crossF > prev_F) crossF = prev_F;
						if(crossF > target->F) crossF = target->F;
						// The problem with the 'dda_steps_to_velocity' is that it will produce a
						// rounded result. Use it to obtain an exact amount of steps needed to reach
						// that speed and set that as the ramp up; we might stop accelerating for a
						// couple of steps but that is better than introducing errors in the moves.
						up = ACCELERATE_RAMP_LEN2(crossF, ramp_scaler) - prestep;

						#ifdef LOOKAHEAD_DEBUG
						// Sanity check: the ramp up should never exceed the move length
						if(up > prev_total_steps) {
							sersendf_P(PSTR("FATAL ERROR during prev ramp scale, ramp is too long: up:%lu ; len:%lu ; target speed: %lu\r\n"),
								up, prev_total_steps, crossF);
							sersendf_P(PSTR("F_start:%lu ; F:%lu ; crossF:%lu\r\n"),
								prev_F_start, prev_F, crossF);
							dda_emergency_shutdown(PSTR("LA prev ramp scale, ramp is too long"));
						}
						#endif
						// Show the result on the speed on the clipping of the ramp
						serprintf(PSTR("Prev speed & crossing speed truncated to: %lu\r\n"), crossF);
					} else {
						// Can reach crossF; determine the apex between ramp up and ramp down
						// In other words: calculate how long we can accelerate before decelerating to exit at crossF
						// Note: while the number of steps is exponentially proportional to the velocity,
						// the acceleration is linear: we can simply remove the same number of steps of both ramps.
						uint32_t diff = (up + down - prev_total_steps) / 2;
						up -= diff;
						down -= diff;
					}

					#ifdef LOOKAHEAD_DEBUG
					// Sanity check: make sure the speed limits are maintained
					if(prev_F_start > prev_F || crossF > prev_F) {
						serprintf(PSTR("Prev target speed exceeded!: prev_F_start:%lu ; prev_F:%lu ; prev_F_end:%lu\r\n"), prev_F_start, prev_F, crossF);
						dda_emergency_shutdown(PSTR("Prev target speed exceeded"));
					}
					#endif
				}
				// Save the results
				prev_rampup = up;
				prev_rampdown = prev_total_steps - down;
				prev_F_end = crossF;

				#ifdef LOOKAHEAD_DEBUG
				// Sanity check: make sure the speed limits are maintained
				if(crossF > target->F) {
					serprintf(PSTR("This target speed exceeded!: F_start:%lu ; F:%lu ; prev_F_end:%lu\r\n"), crossF, target->F);
					dda_emergency_shutdown(PSTR("This target speed exceeded"));
				}
				#endif

				// Forward check 2: test if we can actually reach the target speed in this move.
				// If not: determine obtainable speed and adjust crossF accordingly. If that
				// happens, a third (reverse) pass is needed to lower the speeds in the previous move...
				up = ACCELERATE_RAMP_LEN2(target->F, ramp_scaler) - ACCELERATE_RAMP_LEN2(crossF, ramp_scaler);
				down = ACCELERATE_RAMP_LEN2(target->F, ramp_scaler);
				// Test if both the ramp up and ramp down fit within the move
				if(up+down > dda->total_steps) {
					// Test if we can reach the crossF rate
					// Note: this is the inverse of the previous move: we need to exit at 0 speed as
					// this is the last move in the queue. Implies that down >= up
					if(down-up > dda->total_steps) {
						serprintf(PSTR("This move can not reach crossF - lower it\r\n"));
						// Cannot reach crossF, lower it and adjust ramps
						// Note: after this, the previous move needs to be modified to match crossF.
						up = 0;
						// Calculate what crossing rate we can reach: total/down * F
						crossF = dda_steps_to_velocity(dda->total_steps);
						// Speed limit: never exceed the target rate
						if(crossF > target->F) crossF = target->F;
						// crossF will be conservative: calculate the actual ramp down length
						down = ACCELERATE_RAMP_LEN2(crossF, ramp_scaler);

						#ifdef LOOKAHEAD_DEBUG
						// Make sure we can break to a full stop before the move ends
						if(down > dda->total_steps) {
							sersendf_P(PSTR("FATAL ERROR during ramp scale, ramp is too long: down:%lu ; len:%lu ; target speed: %lu\r\n"),
								down, dda->total_steps, crossF);
							dda_emergency_shutdown(PSTR("LA current ramp scale, ramp is too long"));
						}
						#endif
					} else {
						serprintf(PSTR("This: crossF is usable but we will not reach Fmax\r\n"));
						// Can reach crossF; determine the apex between ramp up and ramp down
						// In other words: calculate how long we can accelerate before decelerating to start at crossF
						// and end at F = 0
						uint32_t diff = (down + up - dda->total_steps) / 2;
						up -= diff;
						down -= diff;
						serprintf(PSTR("Apex: %lu - new up: %lu - new down: %lu\r\n"), diff, up, down);

						// sanity stuff: calculate the speeds for these ramps
						serprintf(PSTR("Ramp up speed: %lu mm/s\r\n"), dda_steps_to_velocity(up+prev_dda->rampup_steps));
						serprintf(PSTR("Ramp down speed: %lu mm/s\r\n"), dda_steps_to_velocity(down));
					}
				}
				// Save the results
				this_rampup = up;
				this_rampdown = dda->total_steps - down;
				this_F_start = crossF;
				serprintf(PSTR("Actual crossing speed: %lu\r\n"), crossF);

				// Potential reverse processing:
				// Make sure the crossing speed is the same, if its not, we need to slow the previous move to
				// the current crossing speed (note: the crossing speed could only be lowered).
				// This can happen when this move is a short move and the previous move was a larger or faster move:
				// since we need to be able to stop if this is the last move, we lowered the crossing speed
				// between this move and the previous move...
				if(prev_F_end != crossF) {
					// Third reverse pass: slow the previous move to end at the target crossing speed.
					// Note: use signed values so we  can check if results go below zero
					// Note 2: when up2 and/or down2 are below zero from the start, you found a bug in the logic above.
					int32_t up2 = ACCELERATE_RAMP_LEN2(prev_F, ramp_scaler) - ACCELERATE_RAMP_LEN2(prev_F_start, ramp_scaler);
					int32_t down2 = ACCELERATE_RAMP_LEN2(prev_F, ramp_scaler) - ACCELERATE_RAMP_LEN2(crossF, ramp_scaler);

					// Test if both the ramp up and ramp down fit within the move
					if(up2+down2 > prev_total_steps) {
						int32_t diff = (up2 + down2 - (int32_t)prev_total_steps) / 2;
						up2 -= diff;
						down2 -= diff;

						#ifdef LOOKAHEAD_DEBUG
						if(up2 < 0 || down2 < 0) {
							// Cannot reach crossF from prev_F_start - this should not happen!
							sersendf_P(PSTR("FATAL ERROR during reverse pass ramp scale, ramps are too long: up:%ld ; down:%ld; len:%lu ; F_start: %lu ; crossF: %lu\r\n"),
												up2, down2, prev_total_steps, prev_F_start, crossF);
							sersendf_P(PSTR("Original up: %ld - down %ld (diff=%ld)\r\n"),up2+diff,down2+diff,diff);
							dda_emergency_shutdown(PSTR("reverse pass ramp scale, can not reach F_end from F_start"));
						}
						#endif
					}
					// Assign the results
					prev_rampup = up2;
					prev_rampdown = prev_total_steps - down2;
					prev_F_end = crossF;
				}

				#ifdef LOOKAHEAD_DEBUG
				if(crossF > target->F || crossF > prev_F)
					dda_emergency_shutdown(PSTR("Lookahead exceeded speed limits in crossing!"));

				// When debugging, print the 2 moves we joined
				// Legenda: Fs=F_start, len=# of steps, up/down=# steps in ramping, Fe=F_end
				serprintf(PSTR("LA: (%lu) Fs=%lu, len=%lu, up=%lu, down=%lu, Fe=%lu <=> (%lu) Fs=%lu, len=%lu, up=%lu, down=%lu, Fe=0\r\n\r\n"),
					moveno-1, prev_dda->F_start, prev_dda->total_steps, prev_dda->rampup_steps,
					prev_dda->total_steps-prev_dda->rampdown_steps, prev_dda->F_end,
					moveno, dda->F_start, dda->total_steps, dda->rampup_steps,
					dda->total_steps - this_rampdown);
				#endif

				// Determine if we are fast enough - if not, just leave the moves
				if(prev_dda->live == 0) {
					// Do an 'atomic' copy here to apply the values to the moves.
					// TODO: Make this truly atomic
					prev_dda->F_end = prev_F_end;
					prev_dda->rampup_steps = prev_rampup;
					prev_dda->rampdown_steps = prev_rampdown;
					dda->rampup_steps = this_rampup;
					dda->rampdown_steps = this_rampdown;
					dda->F_end = 0;
					dda->F_start = this_F_start;
					la_cnt++;
				} else
					sersendf_P(PSTR("Error: look ahead not fast enough\r\n"));
			}

			// Store the deltas for the next iteration
			x_delta_old = x_delta;
			y_delta_old = y_delta;
			z_delta_old = z_delta;

			// End of lookahead logic
			#endif
		#elif defined ACCELERATION_TEMPORAL
			// TODO: limit speed of individual axes to MAXIMUM_FEEDRATE
			// TODO: calculate acceleration/deceleration for each axis
			dda->x_step_interval = dda->y_step_interval = \
				dda->z_step_interval = dda->e_step_interval = 0xFFFFFFFF;
			if (dda->x_delta)
				dda->x_step_interval = move_duration / dda->x_delta;
			if (dda->y_delta)
				dda->y_step_interval = move_duration / dda->y_delta;
			if (dda->z_delta)
				dda->z_step_interval = move_duration / dda->z_delta;
			if (dda->e_delta)
				dda->e_step_interval = move_duration / dda->e_delta;

			dda->axis_to_step = 'x';
			dda->c = dda->x_step_interval;
			if (dda->y_step_interval < dda->c) {
				dda->axis_to_step = 'y';
				dda->c = dda->y_step_interval;
			}
			if (dda->z_step_interval < dda->c) {
				dda->axis_to_step = 'z';
				dda->c = dda->z_step_interval;
			}
			if (dda->e_step_interval < dda->c) {
				dda->axis_to_step = 'e';
				dda->c = dda->e_step_interval;
			}

			dda->c <<= 8;
		#else
			dda->c = (move_duration / target->F) << 8;
			if (dda->c < c_limit)
				dda->c = c_limit;
		#endif
	}

	if (DEBUG_DDA && (debug_flags & DEBUG_DDA))
		serial_writestr_P(PSTR("] }\n"));

	#ifdef LOOKAHEAD
	moveno++;
	#endif

	// next dda starts where we finish
	memcpy(&startpoint, target, sizeof(TARGET));
}

/*! Start a prepared DDA
	\param *dda pointer to entry in dda_queue to start

	This function actually begins the move described by the passed DDA entry.

	We set direction and enable outputs, and set the timer for the first step from the precalculated value.

	We also mark this DDA as running, so other parts of the firmware know that something is happening

	Called both inside and outside of interrupts.
*/
void dda_start(DDA *dda) {
	// called from interrupt context: keep it simple!
	if ( ! dda->nullmove) {
		// get ready to go
		steptimeout = 0;
		if (dda->z_delta)
			z_enable();

		// set direction outputs
		x_direction(dda->x_direction);
		y_direction(dda->y_direction);
		z_direction(dda->z_direction);
		e_direction(dda->e_direction);

		#ifdef	DC_EXTRUDER
		if (dda->e_delta)
			heater_set(DC_EXTRUDER, DC_EXTRUDER_PWM);
		#endif

		// initialise state variable
		move_state.x_counter = move_state.y_counter = move_state.z_counter = \
			move_state.e_counter = -(dda->total_steps >> 1);
		memcpy(&move_state.x_steps, &dda->x_delta, sizeof(uint32_t) * 4);
		#ifdef ACCELERATION_RAMPING
			move_state.step_no = 0;
		#endif
		#ifdef ACCELERATION_TEMPORAL
		move_state.x_time = move_state.y_time = \
			move_state.z_time = move_state.e_time = 0UL;
		#endif

		// ensure this dda starts
		dda->live = 1;

		// set timeout for first step
		#ifdef ACCELERATION_RAMPING
		if (dda->c_min > move_state.c) // can be true when look-ahead removed all deceleration steps
			setTimer(dda->c_min >> 8);
		else
			setTimer(move_state.c >> 8);
		#else
		setTimer(dda->c >> 8);
		#endif
	}
	// else just a speed change, keep dda->live = 0

	current_position.F = dda->endpoint.F;
}

/*! STEP
	\param *dda the current move

	This is called from our timer interrupt every time a step needs to occur. Keep it as simple as possible!
	We first work out which axes need to step, and generate step pulses for them
	Then we re-enable global interrupts so serial data reception and other important things can occur while we do some math.
	Next, we work out how long until our next step using the selected acceleration algorithm and set the timer.
	Then we decide if this was the last step for this move, and if so mark this dda as dead so next timer interrupt we can start a new one.
	Finally we de-assert any asserted step pins.
*/
void dda_step(DDA *dda) {
	uint8_t endstop_stop; ///< Stop due to endstop trigger
	uint8_t endstop_not_done = 0; ///< Which axes haven't finished homing

#if defined X_MIN_PIN || defined X_MAX_PIN
	if (dda->endstop_check & 0x1) {
#if defined X_MIN_PIN
		if (x_min() == dda->endstop_stop_cond)
			move_state.debounce_count_xmin++;
		else
			move_state.debounce_count_xmin = 0;
#endif

#if defined X_MAX_PIN
		if (x_max() == dda->endstop_stop_cond)
			move_state.debounce_count_xmax++;
		else
			move_state.debounce_count_xmax = 0;
#endif

		endstop_stop = move_state.debounce_count_xmin >= ENDSTOP_STEPS ||
		               move_state.debounce_count_xmax >= ENDSTOP_STEPS;
		if (!endstop_stop)
			endstop_not_done |= 0x1;
	} else
#endif
		endstop_stop = 0;

#if ! defined ACCELERATION_TEMPORAL
	if ((move_state.x_steps) && ! endstop_stop) {
		move_state.x_counter -= dda->x_delta;
		if (move_state.x_counter < 0) {
			x_step();
			move_state.x_steps--;
			move_state.x_counter += dda->total_steps;

#if (defined X_MIN_PIN || defined X_MAX_PIN) && defined ENDSTOP_ALWAYS_CHECK
			// Check if we want to search for the end stops, if not and they are toggled do an emergency stop
			if ((dda->endstop_check & 0x1) == 0) {
#if defined X_MIN_PIN
				if (x_min() == 1) dda_emergency_shutdown(PSTR("x-axis min end stop triggered"));
#endif
#if defined X_MAX_PIN
				if (x_max() == 1) dda_emergency_shutdown(PSTR("x-axis max end stop triggered"));
#endif
			}
#endif
		}
	}
#else	// ACCELERATION_TEMPORAL
	if ((dda->axis_to_step == 'x') && ! endstop_stop) {
		x_step();
		move_state.x_steps--;
		move_state.x_time += dda->x_step_interval;
		move_state.all_time = move_state.x_time;

#if (defined X_MIN_PIN || defined X_MAX_PIN) && defined ENDSTOP_ALWAYS_CHECK
		// Check if we want to search for the end stops, if not and they are toggled do an emergency stop
		if ((dda->endstop_check & 0x1) == 0) {
#if defined X_MIN_PIN
			if (x_min() == 1) dda_emergency_shutdown(PSTR("x-axis min end stop triggered"));
#endif
#if defined X_MAX_PIN
			if (x_max() == 1) dda_emergency_shutdown(PSTR("x-axis max end stop triggered"));
#endif
		}
#endif
	}
#endif

#if defined Y_MIN_PIN || defined Y_MAX_PIN
	if (dda->endstop_check & 0x2) {
#if defined Y_MIN_PIN
		if (y_min() == dda->endstop_stop_cond)
			move_state.debounce_count_ymin++;
		else
			move_state.debounce_count_ymin = 0;
#endif

#if defined Y_MAX_PIN
		if (y_max() == dda->endstop_stop_cond)
			move_state.debounce_count_ymax++;
		else
			move_state.debounce_count_ymax = 0;
#endif

		endstop_stop = move_state.debounce_count_ymin >= ENDSTOP_STEPS ||
		               move_state.debounce_count_ymax >= ENDSTOP_STEPS;
		if (!endstop_stop)
			endstop_not_done |= 0x2;
	} else
#endif
		endstop_stop = 0;

#if ! defined ACCELERATION_TEMPORAL
	if ((move_state.y_steps) && ! endstop_stop) {
		move_state.y_counter -= dda->y_delta;
		if (move_state.y_counter < 0) {
			y_step();
			move_state.y_steps--;
			move_state.y_counter += dda->total_steps;

#if (defined Y_MIN_PIN || defined Y_MAX_PIN) && defined ENDSTOP_ALWAYS_CHECK
			// Check if we want to search for the end stops, if not and they are toggled do an emergency stop
			if ((dda->endstop_check & 0x2) == 0) {
#if defined Y_MIN_PIN
				if (y_min() == 1) dda_emergency_shutdown(PSTR("y-axis min end stop triggered"));
#endif
#if defined Y_MAX_PIN
				if (y_max() == 1) dda_emergency_shutdown(PSTR("y-axis max end stop triggered"));
#endif
			}
#endif
		}
	}
#else	// ACCELERATION_TEMPORAL
	if ((dda->axis_to_step == 'y') && ! endstop_stop) {
		y_step();
		move_state.y_steps--;
		move_state.y_time += dda->y_step_interval;
		move_state.all_time = move_state.y_time;

#if (defined Y_MIN_PIN || defined Y_MAX_PIN) && defined ENDSTOP_ALWAYS_CHECK
		// Check if we want to search for the end stops, if not and they are toggled do an emergency stop
		if ((dda->endstop_check & 0x2) == 0) {
#if defined Y_MIN_PIN
			if (y_min() == 1) dda_emergency_shutdown(PSTR("y-axis min end stop triggered"));
#endif
#if defined Y_MAX_PIN
			if (y_max() == 1) dda_emergency_shutdown(PSTR("y-axis max end stop triggered"));
#endif
		}
#endif
	}
#endif

#if defined Z_MIN_PIN || defined Z_MAX_PIN
	if (dda->endstop_check & 0x4) {
#if defined Z_MIN_PIN
		if (z_min() == dda->endstop_stop_cond)
			move_state.debounce_count_zmin++;
		else
			move_state.debounce_count_zmin = 0;
#endif

#if defined Z_MAX_PIN
		if (z_max() == dda->endstop_stop_cond)
			move_state.debounce_count_zmax++;
		else
			move_state.debounce_count_zmax = 0;
#endif

		endstop_stop = move_state.debounce_count_zmin >= ENDSTOP_STEPS ||
		               move_state.debounce_count_zmax >= ENDSTOP_STEPS;
		if (!endstop_stop)
			endstop_not_done |= 0x4;
	} else 
#endif
		endstop_stop = 0;

#if ! defined ACCELERATION_TEMPORAL
	if ((move_state.z_steps) && ! endstop_stop) {
		move_state.z_counter -= dda->z_delta;
		if (move_state.z_counter < 0) {
			z_step();
			move_state.z_steps--;
			move_state.z_counter += dda->total_steps;

#if (defined Z_MIN_PIN || defined Z_MAX_PIN) && defined ENDSTOP_ALWAYS_CHECK
			// Check if we want to search for the end stops, if not and they are toggled do an emergency stop
			if ((dda->endstop_check & 0x4) == 0) {
#if defined Z_MIN_PIN
				if (z_min() == 1) dda_emergency_shutdown(PSTR("z-axis min end stop triggered"));
#endif
#if defined Z_MAX_PIN
				if (z_max() == 1) dda_emergency_shutdown(PSTR("z-axis max end stop triggered"));
#endif
			}
#endif
		}
	}
#else	// ACCELERATION_TEMPORAL
	if ((dda->axis_to_step == 'z') && ! endstop_stop) {
		z_step();
		move_state.z_steps--;
		move_state.z_time += dda->z_step_interval;
		move_state.all_time = move_state.z_time;

#if (defined Z_MIN_PIN || defined Z_MAX_PIN) && defined ENDSTOP_ALWAYS_CHECK
		// Check if we want to search for the end stops, if not and they are toggled do an emergency stop
		if ((dda->endstop_check & 0x4) == 0) {
#if defined Z_MIN_PIN
			if (z_min() == 1) dda_emergency_shutdown(PSTR("z-axis min end stop triggered"));
#endif
#if defined Z_MAX_PIN
			if (z_max() == 1) dda_emergency_shutdown(PSTR("z-axis max end stop triggered"));
#endif
		}
#endif
	}
#endif

#if ! defined ACCELERATION_TEMPORAL
	if (move_state.e_steps) {
		move_state.e_counter -= dda->e_delta;
		if (move_state.e_counter < 0) {
			e_step();
			move_state.e_steps--;
			move_state.e_counter += dda->total_steps;
		}
	}
#else	// ACCELERATION_TEMPORAL
	if (dda->axis_to_step == 'e') {
		e_step();
		move_state.e_steps--;
		move_state.e_time += dda->e_step_interval;
		move_state.all_time = move_state.e_time;
	}
#endif

	#if STEP_INTERRUPT_INTERRUPTIBLE
		// Since we have sent steps to all the motors that will be stepping
		// and the rest of this function isn't so time critical, this interrupt
		// can now be interruptible by other interrupts.
		// The step interrupt is disabled before entering dda_step() to ensure
		// that we don't step again while computing the below.
		sei();
	#endif

	#ifdef ACCELERATION_REPRAP
		// linear acceleration magic, courtesy of http://www.embedded.com/columns/technicalinsights/56800129?printable=true
		if (dda->accel) {
			if ((dda->c > dda->end_c) && (dda->n > 0)) {
				uint32_t new_c = dda->c - (dda->c * 2) / dda->n;
				if (new_c <= dda->c && new_c > dda->end_c) {
					dda->c = new_c;
					dda->n += 4;
				}
				else
					dda->c = dda->end_c;
			}
			else if ((dda->c < dda->end_c) && (dda->n < 0)) {
				uint32_t new_c = dda->c + ((dda->c * 2) / -dda->n);
				if (new_c >= dda->c && new_c < dda->end_c) {
					dda->c = new_c;
					dda->n += 4;
				}
				else
					dda->c = dda->end_c;
			}
			else if (dda->c != dda->end_c) {
				dda->c = dda->end_c;
			}
			// else we are already at target speed
		}
	#endif
	#ifdef ACCELERATION_RAMPING
		// - algorithm courtesy of http://www.embedded.com/columns/technicalinsights/56800129?printable=true
		// - precalculate ramp lengths instead of counting them, see AVR446 tech note
		uint8_t recalc_speed;

		// debug ramping algorithm
		//if (move_state.step_no == 0) {
		//	sersendf_P(PSTR("\r\nc %lu  c_min %lu  n %d"), dda->c, dda->c_min, move_state.n);
		//}

		recalc_speed = 0;
		if (move_state.step_no < dda->rampup_steps) {
			if (move_state.n < 0) // wrong ramp direction
				move_state.n = -((int32_t)2) - move_state.n;
			recalc_speed = 1;
			//serial_writestr((uint8_t*)"Up:  ");
		}
		else if (move_state.step_no >= dda->rampdown_steps) {
			if (move_state.n > 0) // wrong ramp direction
				move_state.n = -((int32_t)2) - move_state.n;
			recalc_speed = 1;
			//serial_writestr((uint8_t*)"Down:");
		}
		if (recalc_speed) {
			move_state.n += 4;
			// be careful of signedness! note: equation 13:
			move_state.c = (int32_t)move_state.c - ((int32_t)(move_state.c * 2) / (int32_t)move_state.n);
			//sersendf_P(PSTR("n:%ld ; c:%ld ; steps: %ld / %lu\r\n"), move_state.n, move_state.c, move_state.step_no, move_state.y_steps);
		}
		move_state.step_no++;
// Print the number of steps actually needed for ramping up
// Needed for comparing the number with the one calculated in dda_create()
//static char printed = 0;
//if (printed == 0 && dda->c_min >= move_state.c) {
//  sersendf_P(PSTR("speedup %lu steps\n"), move_state.step_no);
//  printed = 1;
//}
//if (move_state.step_no < 3) printed = 0;

		// debug ramping algorithm
		// raise this 10 for higher speeds to avoid flooding the serial line
		//if (move_state.step_no % 10 /* 10, 50, 100, ...*/ == 0)
		//	sersendf_P(PSTR("\r\nc %lu  c_min %lu  n %ld"),
		//	           move_state.c, dda->c_min, move_state.n);
	#endif

	// TODO: If we stop axes individually, could we home two or more axes at the same time?
	if (dda->endstop_check != 0x0 && endstop_not_done == 0x0) {
		move_state.x_steps = move_state.y_steps = move_state.z_steps = move_state.e_steps = 0;
		// as we stop without ramping down, we have to re-init our ramping here
		dda_init();
	}
	#ifdef ACCELERATION_TEMPORAL
		/** How is this ACCELERATION TEMPORAL expected to work?

			All axes work independently of each other, as if they were on four different, synchronized timers. As we have not enough suitable timers, we have to share one for all axes.

			To do this, each axis maintains the time of its last step in move_state.{xyze}_time. This time is updated as the step is done, see early in dda_step(). To find out which axis is the next one to step, the time of each axis' next step is compared to the time of the step just done. Zero means this actually is the axis just stepped, the smallest value > 0 wins.

			One problem undoubtly arising is, steps should sometimes be done at {almost,exactly} the same time. We trust the timer to deal properly with very short or even zero periods. If a step can't be done in time, the timer shall do the step as soon as possible and compensate for the delay later. In turn we promise here to send a maximum of four such short-delays consecutively and to give sufficient time on average.
		*/
		uint32_t c_candidate;

		dda->c = 0xFFFFFFFF;
		if (move_state.x_steps) {
			c_candidate = move_state.x_time + dda->x_step_interval - move_state.all_time;
			dda->axis_to_step = 'x';
			dda->c = c_candidate;
		}
		if (move_state.y_steps) {
			c_candidate = move_state.y_time + dda->y_step_interval - move_state.all_time;
			if (c_candidate < dda->c) {
				dda->axis_to_step = 'y';
				dda->c = c_candidate;
			}
		}
		if (move_state.z_steps) {
			c_candidate = move_state.z_time + dda->z_step_interval - move_state.all_time;
			if (c_candidate < dda->c) {
				dda->axis_to_step = 'z';
				dda->c = c_candidate;
			}
		}
		if (move_state.e_steps) {
			c_candidate = move_state.e_time + dda->e_step_interval - move_state.all_time;
			if (c_candidate < dda->c) {
				dda->axis_to_step = 'e';
				dda->c = c_candidate;
			}
		}
		dda->c <<= 8;
	#endif

	// If there are no steps left, we have finished.
	if (move_state.x_steps == 0 && move_state.y_steps == 0 &&
	    move_state.z_steps == 0 && move_state.e_steps == 0) {
		dda->live = 0;
		#ifdef	DC_EXTRUDER
			heater_set(DC_EXTRUDER, 0);
		#endif
		// z stepper is only enabled while moving
		z_disable();
	}
	else
		steptimeout = 0;

	#ifdef ACCELERATION_RAMPING
		// we don't hit maximum speed exactly with acceleration calculation, so limit it here
		// the nice thing about _not_ setting dda->c to dda->c_min is, the move stops at the exact same c as it started, so we have to calculate c only once for the time being
		// TODO: set timer only if dda->c has changed
		if (dda->c_min > move_state.c)
			setTimer(dda->c_min >> 8);
		else
			setTimer(move_state.c >> 8);
	#else
		setTimer(dda->c >> 8);
	#endif

	// turn off step outputs, hopefully they've been on long enough by now to register with the drivers
	// if not, too bad. or insert a (very!) small delay here, or fire up a spare timer or something.
	// we also hope that we don't step before the drivers register the low- limit maximum speed if you think this is a problem.
	unstep();
}

/// update global current_position struct
void update_current_position() {
	DDA *dda = &movebuffer[mb_tail];

	if (queue_empty()) {
		current_position.X = startpoint.X;
		current_position.Y = startpoint.Y;
		current_position.Z = startpoint.Z;
		current_position.E = startpoint.E;
	}
	else if (dda->live) {
		if (dda->x_direction)
			// (STEPS_PER_M_X / 1000) is a bit inaccurate for low STEPS_PER_M numbers
			current_position.X = dda->endpoint.X -
			                     // should be: move_state.x_steps * 1000000 / STEPS_PER_M_X)
			                     // but x_steps can be like 1000000 already, so we'd overflow
			                     move_state.x_steps * 1000 / ((STEPS_PER_M_X + 500) / 1000);
		else
			current_position.X = dda->endpoint.X +
			                     move_state.x_steps * 1000 / ((STEPS_PER_M_X + 500) / 1000);

		if (dda->y_direction)
			current_position.Y = dda->endpoint.Y -
			                     move_state.y_steps * 1000 / ((STEPS_PER_M_Y + 500) / 1000);
		else
			current_position.Y = dda->endpoint.Y +
			                     move_state.y_steps * 1000 / ((STEPS_PER_M_Y + 500) / 1000);

		if (dda->z_direction)
			current_position.Z = dda->endpoint.Z -
			                     move_state.z_steps * 1000 / ((STEPS_PER_M_Z + 500) / 1000);
		else
			current_position.Z = dda->endpoint.Z +
			                     move_state.z_steps * 1000 / ((STEPS_PER_M_Z + 500) / 1000);

		if (dda->endpoint.e_relative) {
			current_position.E = move_state.e_steps * 1000 / ((STEPS_PER_M_E + 500) / 1000);
		}
		else {
			if (dda->e_direction)
				current_position.E = dda->endpoint.E -
				                     move_state.e_steps * 1000 / ((STEPS_PER_M_E + 500) / 1000);
			else
				current_position.E = dda->endpoint.E +
				                     move_state.e_steps * 1000 / ((STEPS_PER_M_E + 500) / 1000);
		}

		// current_position.F is updated in dda_start()
	}
}
