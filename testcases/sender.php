#!/usr/bin/php
<?php
$in = 'firmware_to_host.fifo';
$out = 'host_to_firmware.fifo';
$log = 'simulation.log';

# This should point to a SIMINFO-enabled SimulAVR executable:
$SIMULAVR="../../simulavr/src/simulavr";

if(count($argv)!=2) {
	echo "Usage: " . $argv[0] . " your.gcode\n";
	
	echo "	This script creates two fifos, named '$in' and '$out'.\n";
	echo "	The gcode file is simply passed line by line as long as\n";
	echo "	the printer replies with 'ok'.\n";
	exit(1);
}

if(!file_exists($argv[1])) {
	echo "Gcode " . $argv[1] . " file does not exist\n";
	exit(1);
}
echo "Using '".$argv[1]."' as input script\n";
// Open the gcode file
$gc = fopen($argv[1], "r");
if($gc===false) {
	echo "Could not open gcode file '" . $argv[1] . "'\n";
	exit(1);
}

$lh = fopen($log, 'w');
if($lh===false) die("could not open log file '$log'\n");

// Logging function to show the results on screen and capture them in a file
function wlog($s) {
	global $lh;
	list($usec, $sec) = explode(" ", microtime());
	$usec = substr($usec, 2, 3);
	$ls = date("[H:i:s") . ".$usec] $s\n";
	echo $ls;
	fwrite($lh,$ls);
}

echo "Checking and creating fifos\n";
// Create the fifos
if(!file_exists($in))
	if(exec("mkfifo $in")!=0)
		echo "Error creating fifo '$in'\n";
if(!file_exists($out))
	if(exec("mkfifo $out")!=0)
		echo "Error creating fifo '$out'\n";

echo "Opening fifos - start simulavr now\n";
$fi = fopen($in, "r+");		// r+ uses this handle as a writer as well, preventing a block until the simulator connects
//echo "Input open";
$fo = fopen($out, "w");
//echo "Output open";
// Check if the files were opened correctly

if($fi===false) {
	echo "Could not open '$fi'\n";
	exit(1);
}
if($fo===false) {
	echo "Could not open '$fo'\n";
	exit(1);
}

echo "Sender ready, starting now...\n";

// Use stream_select to flush anything still in the fifo
// as it will confuse the sender (for example the simulavr
// startup blurp)
$null = NULL;
$rs = array($fi);
while(($s = stream_select($rs, $null, $null, 1)) !== false && $s > 0) {
	// There is data, read until the simulator is waiting for input
	$r = fgets($fi);
	if($r !== false) {
		$r = trim($r);
		wlog("< $r");
	}
}
echo "Flush complete, starting gcode script\n";

// Now start sending commands
while(($line = fgets($gc))!==false) {
	// Filter lines
	$arr = explode(';', $line);
	if(trim($arr[0]) == '') {
		// Line only contains a comment, skip
		continue;
	}
	
	// Grab the content before the semi-colon
	$line = trim($arr[0]);

	// Write to printer
	wlog("> $line");
	fwrite($fo, $line."\n");
	
	// Read until 'ok' is received
	while(1) {
		$r = fgets($fi);
		if($r !== false) {
			$r = trim($r);
			wlog("< $r");
			
			// Parse response: an M2 signals the end of the script and
			// will cause the firmware to reply with 'stop'
			if(substr($r,0,4)=="stop") {
				// Kill the simulator
				wlog("Received stop, killing simulator");
				exec("kill -INT $(pidof -x \"$SIMULAVR\")");
				exit(0);
			}
			// Parse response: a timeout is a reason to stop
			if(strpos($r, "Ran too long") !== false) {
				wlog("Simulator ended");
				exit(0);
			}
			// Parse response: an 'ok' at the start of a line means
			// the command was accepted and we can send another
			if(substr($r,0,2)=="ok")
				break;
		}
	}
}

sleep(1);

// Use stream_select to flush anything still in the fifo
// as it will confuse the sender (for example the simulavr
// startup blurp)
$null = NULL;
$rs = array($fi);
while(($s = stream_select($rs, $null, $null, 1)) !== false && $s > 0) {
	// There is data, read until the simulator is waiting for input
	$r = fgets($fi);
	if($r !== false) {
		$r = trim($r);
		wlog("<< $r");
		
		// Parse response: an M2 signals the end of the script and
		// will cause the firmware to reply with 'stop'
		if(substr($r,0,4)=="stop") {
			// Kill the simulator
			wlog("Received stop, killing simulator");
			exec("kill -INT $(pidof -x \"$SIMULAVR\")");
			exit(0);
		}
	}
}

// Kill the simulator if its still running
echo "Done\n";

