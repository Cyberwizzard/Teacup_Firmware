#!/usr/bin/php
<?php
/*
	Generate a number of curves to test lookahead in g-code by Cyberwizzard
	
	This script generates gcode to plot a number of curves of increasing angle in the X-Y plane to 
	test (multi-pass) look-ahead.
*/
// Settings:
$seglen = 0.5;					// Segment length
$segcount = 14;					// Number of segments to generate
$F_start = 1000;				// Start speed
$F_end = 1000;					// End speed
$F_return = 8000;				// Return to origin speed after an arc
$arc = 2;						// Step size per arc
$runs = 3;						// Generate this number of arcs each made of $segcount lines
$x_offset = 0;					// Always start at this X coordinate
$y_offset = 0;					// Always start at this Y coordinate

// Utility settings - do not alter these
$two_pi = 6.28318530718;		// Used for degrees to radians
$deg_to_rad = $two_pi / 360.0;	// Fixed multiplier to go to radians
$cur_arc = 0;					// Current arc angle: start at 0 (straight line)

// Start code
echo "G21\nG92\n";

// Set the start position
$x = $x_offset;	// x does accumulate across moves
$y = $y_offset;
for($r = 0; $r < $runs; $r++) {
	$c = 0;				// Number of degrees in this corner
	
	// Comment this out to do not return to origin
	//$x = $x_offset;
	//$y = $y_offset;		// Keep track of the location of the arc
	//echo sprintf("G1 X%.1f Y%.1f F%.1f\n", $x, $y, $F_return);

	for($i = 0; $i < $segcount; $i++) {
		// Radians
		$rad = $c * $deg_to_rad;

		$x += sin($rad) * $seglen;
		$y += cos($rad) * $seglen;
		
		$f = $F_start + ( ($F_end - $F_start) * $i / ($segcount-1) );

		echo sprintf("G1 X%.2f Y%.2f F%.1f\n", $x, $y, $f);
	
		// Increase the angle
		$c += $cur_arc;
	}

	// Increase the steepness of the angle
	$cur_arc += $arc;
}

// End code
echo "M2\n";

?>
