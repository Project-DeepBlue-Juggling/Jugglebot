This script's purpose is to get familiar with using SteamVR with my Tundra Trackers to get accurate real-time measurements of relative pose.

There are some fairly cool things that will become possible with this tech:
 - Accurate "external" pose measurements 
 - Getting more data of me juggling (tracking both/either the balls and my body)
 - Using the trackers to control the robot by syncing robot pose with tracker pose. Would be a great way to test the safety procedures (eg. not exceeding limits, not getting into structurally weak poses etc.)
 
THIS SCRIPT IS A WORK IN PROGRESS!

Note that this script will only work on Windows (and Mac?) machines due to its dependence on SteamVR. To use this script without having to connect a HMD, you'll have to modify your steamvr settings (Program Files (x86) > Steam > config > steamvr.vrsettings) by adding:

	"forcedHmd": "0",
	"requireHmd": false
	
to the "steamvr" section. An example steamvr.vrsettings file is included in this directory.