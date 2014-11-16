#!/usr/bin/php
<?php
/*
	Generate a number of circles to test lookahead in g-code by Cyberwizzard
	
	This script generates gcode to plot a number of circles of increasing speed to demonstrate
	the functionality of (multi-pass) look-ahead.
	
	Note: this is also useful to calibrate look-ahead and the jerk parameters: if the jerk is
	too high, the abuse of the machine will by visible or audible. Use this script to tweak all
	look-ahead parameters until the machine can move through curves without slowing down.
*/
// Settings:
$seglen = 1.0;					// Segment length
$ccount = 3;					// Number of loops (circles) to do per run
$cdia = 50;						// Circle diameter
$cdelta = -10;					// Number of mm to make the next circle bigger (can be negative)
$cruns = 3;						// Number of tests to generate: with cdia=50 and cdelta=5 and cruns=3 you get a test at 50mm, 55mm and 60mm
$F_start = 1000;				// Start speed
$F_end = 4000;					// End speed

$F_return = 8000;				// Return to origin speed after an arc
$xoffset = 30;					// Always start at this X coordinate
$yoffset = 30;					// Always start at this Y coordinate

// Utility settings - do not alter these
$two_pi = 6.28318530718;		// Used for degrees to radians
$deg_to_rad = $two_pi / 360.0;	// Fixed multiplier to go to radians
$cur_arc = 0;					// Current arc angle: start at 0 (straight line)

// Start code
echo "G21\nG92\n";
echo "G28 X0 Y0\nG01 X0 Y0 F2000\n";

// Maximum angle in radians for $ccount loops
$maxrad = $ccount * $two_pi;

// Generate multiple tests at various sizes
for($i=0;$i<$cruns;$i++) {
	// Calculate the angle we need to step at to generate $seglen sized line segments
	$rad = 2 * atan( ($seglen / 2) / ($cdia / 2) );
	$acc = 0;
	
	// Move to start
	$x = $xoffset + sin($acc) * $cdia / 2;
	$y = $yoffset + cos($acc) * $cdia / 2;
	echo sprintf("G1 X%.2f Y%.2f F%.1f\n", $x, $y, $F_return);
	
	while($acc < $maxrad) {
		$x = $xoffset + sin($acc) * $cdia / 2;
		$y = $yoffset + cos($acc) * $cdia / 2;
		
		$f = $F_start + (($F_end - $F_start) * ($acc / $maxrad));
		echo sprintf("G1 X%.2f Y%.2f F%.1f\n", $x, $y, $f);

		// Increase the angle
		$acc += $rad;
	}

	$cdia += $cdelta;
}

// End code
echo "M2\n";

?>
