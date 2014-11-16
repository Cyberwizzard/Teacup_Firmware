#!/bin/bash
# *** WHAT DOES IT DO ***
# Script to run simulavr for teacup with the fake host script
# sender.php to spool gcode scripts with more than MOVEBUFFER_SIZE
# moves through the simulator.
#
# *** HOW TO USE ***
# compile simulavr as instructed on the Teacup wiki and then
# run this script. It will wait for 'sender.php' to act as gcode host before
# actually starting the simulation. Then run 'sender.php my.gcode' to begin
# the test. When all gcode has been executed (or 60 virtual seconds elapse),
# the script will stop the simulator and generate the output files.
#
# *** GENERATED RESULTS ***
# This script takes no arguments and will generate the following files
# after 60 seconds of simulated time (taking much longer in real life)
# or when sender.php is done and closes the connection.
#  * simulation.data	Data file for gnuplot to graph the speed and 
#						position of the X and Y axis
#  * simulation.png		The plotted results from the .data file
#  * simulation.vcd		VCD file with raw traces from pins of the ATmega
#						being simulated
#  * simulation.processed.vcd
#						VCD file with inferred axis speeds and positions
#						based on simulation.vcd
#

FIFO_IN=host_to_firmware.fifo
FIFO_OUT=firmware_to_host.fifo

# This should point to a SIMINFO-enabled SimulAVR executable:
SIMULAVR="../../simulavr/src/simulavr"
if [ ! -x "${SIMULAVR}" ]; then
  echo "SimulAVR executable not found, please adjust"
  echo "variables at the top of ${0}."
  exit 1;
fi

echo "$0 runs each G-code file in the simulator, limited to 60 virtual seconds"
#echo "simulation time (can take much more wall clock time) and processes the"
#echo "results into a PNG picture and a .vcd file with calculated velocities."
#echo
echo "Should be used with sender.php to act as a host to send a gcode script"
echo "using the fifos '$FIFO_OUT' and '$FIFO_IN'."

# General preparation.
TRACEIN_FILE=$(mktemp)
STATISTICS_FILE=$(mktemp)

trap 'cat '${STATISTICS_FILE}'; rm -f '${TRACEIN_FILE}' '${STATISTICS_FILE} 0

# Prepare statistics.
echo                             > ${STATISTICS_FILE}
(cd .. && make size) | tail -4  >> ${STATISTICS_FILE}


# Prepare a pin tracing file, assuming a Gen7-v1.4 configuration. See
# http://reprap.org/wiki/SimulAVR#Putting_things_together:_an_example
#
#   #define X_DIR_PIN        DIO28
#   #define X_STEP_PIN       DIO29
#   #define Y_DIR_PIN        DIO26
#   #define Y_STEP_PIN       DIO27
#   #define DEBUG_LED_PIN    DIO21
#   #define Z_STEP_PIN       DIO23
#   #define Z_DIR_PIN        DIO22
echo "# X Dir"              > ${TRACEIN_FILE}
echo "+ PORTA.A3-Out"      >> ${TRACEIN_FILE}
echo "# X Step"            >> ${TRACEIN_FILE}
echo "+ PORTA.A2-Out"      >> ${TRACEIN_FILE}
echo "# Y Dir"             >> ${TRACEIN_FILE}
echo "+ PORTA.A5-Out"      >> ${TRACEIN_FILE}
echo "# Y Step"            >> ${TRACEIN_FILE}
echo "+ PORTA.A4-Out"      >> ${TRACEIN_FILE}
echo "# DEBUG LED"         >> ${TRACEIN_FILE}
echo "+ PORTC.C5-Out"      >> ${TRACEIN_FILE}
echo "# Z Dir"             >> ${TRACEIN_FILE}
echo "+ PORTC.C6-Out"      >> ${TRACEIN_FILE}
echo "# Z Step"            >> ${TRACEIN_FILE}
echo "+ PORTC.C7-Out"      >> ${TRACEIN_FILE}
echo "# E Dir"             >> ${TRACEIN_FILE}
echo "+ PORTC.C2-Out"      >> ${TRACEIN_FILE}
echo "# E Step"            >> ${TRACEIN_FILE}
echo "+ PORTC.C3-Out"      >> ${TRACEIN_FILE}
echo "Assuming pin configuration for a Gen7-v1.4 + debug LED on DIO21."

STEPS_PER_M_X=$(grep STEPS_PER_M_X ../config.h | \
                grep -v ^// | awk '{ print $3; }')
if [ "${STEPS_PER_M_X}"0 -eq 0 ]; then
  echo "STEPS_PER_M_X not found, assuming 80'000."
  STEPS_PER_M_X=80000
fi
STEPS_PER_M_Y=$(grep STEPS_PER_M_Y ../config.h | \
                grep -v ^// | awk '{ print $3; }')
if [ "${STEPS_PER_M_Y}"0 -eq 0 ]; then
  echo "STEPS_PER_M_Y not found, assuming 80'000."
  STEPS_PER_M_Y=80000
fi
STEPS_PER_M_Z=$(grep STEPS_PER_M_Z ../config.h | \
                grep -v ^// | awk '{ print $3; }')
if [ "${STEPS_PER_M_Z}"0 -eq 0 ]; then
  echo "STEPS_PER_M_Z not found, assuming 2'000'000."
  STEPS_PER_M_Z=2000000
fi
STEPS_PER_M_E=$(grep STEPS_PER_M_E ../config.h | \
                grep -v ^// | awk '{ print $3; }')
if [ "${STEPS_PER_M_E}"0 -eq 0 ]; then
  echo "STEPS_PER_M_E not found, assuming 300'000."
  STEPS_PER_M_E=300000
fi

echo "Taking STEPS_PER_M_X = ${STEPS_PER_M_X} and"
echo "       STEPS_PER_M_Y = ${STEPS_PER_M_Y} and"
echo "       STEPS_PER_M_Z = ${STEPS_PER_M_Z} and"
echo "       STEPS_PER_M_E = ${STEPS_PER_M_E} for mm/min calculation."

if [ ! -p "$FIFO_IN" ]; then
	echo "Creating FIFO $FIFO_IN"
	mkfifo $FIFO_IN
fi
if [ ! -p "$FIFO_OUT" ]; then
	echo "Creating FIFO $FIFO_OUT"
	mkfifo $FIFO_OUT
fi

# Write statistics header
echo                      >> ${STATISTICS_FILE}
echo "gcode statistics:"  >> ${STATISTICS_FILE}

FILE=simulation
VCD_FILE="${FILE}.vcd"
DATA_FILE="${FILE}.data"
VEL_FILE="${FILE}.processed.vcd"

echo "Starting simulator... (start sender.php if you have not done so)"
# Run the simulator against the fifos for sender.php.
# Note that the closing of the fifo at the end of the script
# kills the simulator as well.
"${SIMULAVR}" -c vcd:${TRACEIN_FILE}:"${VCD_FILE}" \
              -f ../build/teacup.elf \
              -m 60000000000 -v < "${FIFO_IN}" > "${FIFO_OUT}" 

# Make plottable files from VCD files.
# This is very rudimentary and does a lot of assumptions.
# For examble, pin data is assumed to always be b0/b1, pin naming
# is assumed to match the order in ${TRACEIN_FILE} and starting at "0".
awk '
BEGIN {
	print "0 0 0 0 0 0 0 0";
	xDir = yDir = zDir = eDir = 0;
	xPos = yPos = zPos = ePos = 0;
	xVel = yVel = zVel = eVel = 0;
	xAcc = yAcc = zAcc = eAcc = 0;
	lastxTime = lastyTime = lastztime = lastetime = 0;
}
/^#/ {
	time = substr($0, 2);
	next;
}
{
	bit = substr($1, 2);
	if ($2 == "0") { # X Dir
		if (bit == 0) xDir = -1;
		else if (bit == 1) xDir = 1;
		else xDir = 0;
	}
	if ($2 == "2") { # Y Dir
		if (bit == 0) yDir = -1;
		else if (bit == 1) yDir = 1;
		else yDir = 0;
	}
	if ($2 == "5") { # Z Dir
		if (bit == 0) zDir = -1;
		else if (bit == 1) zDir = 1;
		else zDir = 0;
	}
	if ($2 == "7") { # E Dir
		if (bit == 0) eDir = -1;
		else if (bit == 1) eDir = 1;
		else eDir = 0;
	}
	if ($2 == "1") { # X Step, count only raising flanges
		if (bit == 1) {
			xPos += xDir;
			xVel = 1000000000 / (time - lastxTime);
			print time " " xPos " " yPos " " zPos " " ePos " " xVel " " yVel " " zVel " " eVel;
			lastxTime = time;
		}
	}
	if ($2 == "3") { # Y Step, count only raising flanges
		if (bit == 1) {
			yPos += yDir;
			yVel = 1000000000 / (time - lastyTime);
			print time " " xPos " " yPos " " zPos " " ePos " " xVel " " yVel " " zVel " " eVel;
			lastyTime = time;
		}
	}
	if ($2 == "6") { # Z Step, count only raising flanges
		if (bit == 1) {
			zPos += zDir;
			zVel = 1000000000 / (time - lastzTime);
			print time " " xPos " " yPos " " zPos " " ePos " " xVel " " yVel " " zVel " " eVel;
			lastzTime = time;
		}
	}
	if ($2 == "8") { # E Step, count only raising flanges
		if (bit == 1) {
			ePos += eDir;
			eVel = 1000000000 / (time - lasteTime);
			print time " " xPos " " yPos " " zPos " " ePos " " xVel " " yVel " " zVel " " eVel;
			lasteTime = time;
		}
	}
	# No usage for the LED pin signal ($2 == "4") so far.
}
' < "${VCD_FILE}" > "${DATA_FILE}"


# Create a plot.
gnuplot << EOF
	set terminal png size 1600,1080
	set output "${FILE}.png"

	set title "${GCODE_FILE}"

	set xlabel "X steps"
	set ylabel "Y steps"
	set y2label "feedrate [steps/s]"

	#set origin 10,10;
	plot "${FILE}.data" using (\$2):(\$3) with dots title "XY position", \
			"${FILE}.data" using (\$2):(\$6) with lines title "X feedrate", \
			"${FILE}.data" using (\$3):(\$7) with lines title "Y feedrate", \
			"${FILE}.data" using (\$4):(\$8) with lines title "Z feedrate", \
			"${FILE}.data" using (\$5):(\$9) with lines title "E feedrate"
EOF


# Next task: rewrite the VCD file to add speed values.
#
# This is a bit tricky, as VCD files demand timestamps in ascending order.
# Strategy taken: write out all timestamped data with one line per event,
# then run it through 'sort -g' and reformat it yet again to be a properly
# formatted VCD file.
awk '
function print_binary(n, e) {  # n = number; e = number of bits
	string = "";
	for (i = 0; i < e; i++) {
	if (n >= (2 ^ (e - 1))) # top bit set
		string = string "1";
	else
		string = string "0";
	n *= 2;
	if (n >= (2 ^ e))
		n -= (2 ^ e);
	}
	return string;
}
BEGIN {
	# These lines must match the ones after the sort.
	intLen = 20;
	xStepID = "0"; xPosID = "1"; xUmID = "2"; xVelID = "3"; xMmmID = "4";
	yStepID = "5"; yPosID = "6"; yUmID = "7"; yVelID = "8"; yMmmID = "9";
	ledID = "10"; ledTimeID = "11";
	zStepID = "12"; zPosID = "13"; zUmID = "14"; zVelID = "15"; zMmmID = "16";
	eStepID = "17"; ePosID = "18"; eUmID = "19"; eVelID = "20"; eMmmID = "21";
	
	xDir = yDir = zDir = eDir = 0;
	xPos = yPos = zPos = ePos = 0;
	lastxTime = lastyTime = lastzTime = lasteTime = 0;
	ledOnTime = 0;

	ledTimeMin = ledTimeMax = 0;
	ledTimeCount = 0;
	ledTimeSum = 0;
}
/^#/ {
	time = substr($0, 2);
	if (time == 0) {
	do {
		getline;
	} while ($0 != "$end");
	}
	next;
}
{
	bit = substr($1, 2);
	if ($2 == "0") { # X Dir
		if (bit == 0) xDir = -1;
		else if (bit == 1) xDir = 1;
		else xDir = 0;
	}
	if ($2 == "2") { # Y Dir
		if (bit == 0) yDir = -1;
		else if (bit == 1) yDir = 1;
		else yDir = 0;
	}
	if ($2 == "5") { # Z Dir
		if (bit == 0) zDir = -1;
		else if (bit == 1) zDir = 1;
		else zDir = 0;
	}
	if ($2 == "7") { # E Dir
		if (bit == 0) eDir = -1;
		else if (bit == 1) eDir = 1;
		else eDir = 0;
	}
	if ($2 == "1") { # X Step
		if (bit == 1) { # raising flange
			xPos += xDir;
			print time " b" print_binary(xPos, intLen) " " xPosID;
			# TODO: it might be better to output mm as real value, but ...
			#       ... does the VCD file format support this? If yes, how?
			print time " b" print_binary(xPos * 1000000 / '"${STEPS_PER_M_X}"', intLen) " " xUmID;
			vel = 1000000000 / (time - lastxTime);
			print lastxTime " b" print_binary(vel, intLen) " " xVelID;
			vel = vel * 60000 / '"${STEPS_PER_M_X}"';
			print lastxTime " b" print_binary(vel, intLen) " " xMmmID;
			print time " b" bit " " xStepID;
			lastxTime = time;
		} else { # falling flange
			print time " b" bit " " xStepID;
		}
	}
	if ($2 == "3") { # Y Step
		if (bit == 1) { # raising flange
			yPos += yDir;
			print time " b" print_binary(yPos, intLen) " " yPosID;
			print time " b" print_binary(yPos * 1000000 / '"${STEPS_PER_M_Y}"', intLen) " " yUmID;
			vel = 1000000000 / (time - lastyTime);
			print lastyTime " b" print_binary(vel, intLen) " " yVelID;
			vel = vel * 60000 / '"${STEPS_PER_M_Y}"';
			print lastyTime " b" print_binary(vel, intLen) " " yMmmID;
			print time " b" bit " " yStepID;
			lastyTime = time;
		} else { # falling flange
			print time " b" bit " " yStepID;
		}
	}
	if ($2 == "4") { # LED signal
		if (bit == 1) { # raising flange
			print time " b" bit " " ledID;
			ledOnTime = time;
		} else { # falling flange
			print time " b" bit " " ledID;
			if (ledOnTime != 0) {
			ledTime = time - ledOnTime;
			print ledOnTime " b" print_binary(ledTime, 32) " " ledTimeID;
			ledTime /= 50; # Convert from nanoseconds to clock cycles.
			if (ledTimeMin == 0 || ledTime < ledTimeMin) {
				ledTimeMin = ledTime;
			}
			if (ledTime > ledTimeMax) {
				ledTimeMax = ledTime;
			}
			ledTimeCount++;
			ledTimeSum += ledTime;
			}
		}
	}
	if ($2 == "6") { # Z Step
		if (bit == 1) { # raising flange
			zPos += zDir;
			print time " b" print_binary(zPos, intLen) " " zPosID;
			print time " b" print_binary(zPos * 1000000 / '"${STEPS_PER_M_Z}"', intLen) " " zUmID;
			vel = 1000000000 / (time - lastzTime);
			print lastzTime " b" print_binary(vel, intLen) " " zVelID;
			vel = vel * 60000 / '"${STEPS_PER_M_Z}"';
			print lastzTime " b" print_binary(vel, intLen) " " zMmmID;
			print time " b" bit " " zStepID;
			lastzTime = time;
		} else { # falling flange
			print time " b" bit " " zStepID;
		}
	}
	if ($2 == "8") { # E Step
		if (bit == 1) { # raising flange
			ePos += eDir;
			print time " b" print_binary(ePos, intLen) " " ePosID;
			print time " b" print_binary(ePos * 1000000 / '"${STEPS_PER_M_E}"', intLen) " " eUmID;
			vel = 1000000000 / (time - lasteTime);
			print lasteTime " b" print_binary(vel, intLen) " " eVelID;
			vel = vel * 60000 / '"${STEPS_PER_M_E}"';
			print lasteTime " b" print_binary(vel, intLen) " " eMmmID;
			print time " b" bit " " eStepID;
			lasteTime = time;
		} else { # falling flange
			print time " b" bit " " eStepID;
		}
	}
}
END {
	if (ledTimeCount > 0) {
		print "LED on occurences: " ledTimeCount "." >> "'${STATISTICS_FILE}'";
		print "LED on time minimum: " ledTimeMin " clock cycles." \
			>> "'${STATISTICS_FILE}'";
		print "LED on time maximum: " ledTimeMax " clock cycles." \
			>> "'${STATISTICS_FILE}'";
		print "LED on time average: " ledTimeSum / ledTimeCount " clock cycles." \
			>> "'${STATISTICS_FILE}'";
	} else {
		print "Debug LED apparently unused." > "'${STATISTICS_FILE}'";
	}
}
' < "${VCD_FILE}" | \
sort -g | \
awk '
BEGIN {
	# These lines must match the ones before the sort.
	intLen = 20;
	xStepID = "0"; xPosID = "1"; xUmID = "2"; xVelID = "3"; xMmmID = "4";
	yStepID = "5"; yPosID = "6"; yUmID = "7"; yVelID = "8"; yMmmID = "9";
	ledID = "10"; ledTimeID = "11";
	zStepID = "12"; zPosID = "13"; zUmID = "14"; zVelID = "15"; zMmmID = "16";
	eStepID = "17"; ePosID = "18"; eUmID = "19"; eVelID = "20"; eMmmID = "21";

	lastTime = "";

	print "$timescale 1ns $end";
	print "$scope module Steppers $end";
	print "$var wire 1 " xStepID " X_step $end";
	print "$var integer " intLen " " xPosID " X_pos_steps $end";
	print "$var integer " intLen " " xUmID " X_pos_um $end";
	print "$var integer " intLen " " xVelID " X_steps/s $end";
	print "$var integer " intLen " " xMmmID " X_mm/min $end";
	print "$var wire 1 " yStepID " Y_step $end";
	print "$var integer " intLen " " yPosID " Y_pos_steps $end";
	print "$var integer " intLen " " yUmID " Y_pos_um $end";
	print "$var integer " intLen " " yVelID " Y_steps/s $end";
	print "$var integer " intLen " " yMmmID " Y_mm/min $end";
	print "$var wire 1 " zStepID " Z_step $end";
	print "$var integer " intLen " " zPosID " Z_pos_steps $end";
	print "$var integer " intLen " " zUmID " Z_pos_um $end";
	print "$var integer " intLen " " zVelID " Z_steps/s $end";
	print "$var integer " intLen " " zMmmID " Z_mm/min $end";
	print "$var wire 1 " eStepID " E_step $end";
	print "$var integer " intLen " " ePosID " E_pos_steps $end";
	print "$var integer " intLen " " eUmID " E_pos_um $end";
	print "$var integer " intLen " " eVelID " E_steps/s $end";
	print "$var integer " intLen " " eMmmID " E_mm/min $end";
	print "$upscope $end";
	print "$scope module Timings $end";
	print "$var wire 1 " ledID " Debug_LED $end";
	print "$var integer " 32 " " ledTimeID " LED_on_time $end";
	print "$upscope $end";
	print "$enddefinitions $end";
	print "#0";
	print "$dumpvars";
	print "bz " xStepID;
	print "bz " xPosID;
	print "bz " xUmID;
	print "bz " xVelID;
	print "bz " xMmmID;
	print "bz " yStepID;
	print "bz " yPosID;
	print "bz " yUmID;
	print "bz " yVelID;
	print "bz " yMmmID;
	print "bz " ledID;
	print "bz " ledTimeID;
	print "bz " zStepID;
	print "bz " zPosID;
	print "bz " zUmID;
	print "bz " zVelID;
	print "bz " zMmmID;
	print "bz " eStepID;
	print "bz " ePosID;
	print "bz " eUmID;
	print "bz " eVelID;
	print "bz " eMmmID;
	print "$end";
}
{
	if ($1 != lastTime) {
	print "#" $1;
	lastTime = $1;
	}
	print $2 " " $3;
}
' > "${VEL_FILE}"

