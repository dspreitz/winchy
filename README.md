# winchy
Winchs is a project which creates a glider winch rope force and advice system.
It consists of two electronic units. The rope unit is attached to the winch rope at the end the glider is connected, close to, but before (seen from the winch) the weak link.
The ground semgent is located at the winch and has a display which is visible to the winch operator. Both units exchange information via radio link.
Both units are ESP32 based. The rope unit uses a lilygo T-Beam supreme board. This board is is extended with a breakout board containing an ADS1232 chip. A weak link instrumented with strain gauges is introduced into the rope near the glider. The instrumented weak link is used to measure rope tension. The sensors of the T-Beam supreme are used to identify the rope angle (wrt. ground) at the weak link position.
A Kalman filter in the rope segment is used to fuse information from 3 axis acceleration, magnetometer, pressure sensor and gps if available.
System time is derived by the rope segment from GPS at startup. RTC is set accordingly. System time is transmitted to the winch segment at initial start-up.
Rope tension and postion of T-Beam in space is transmitted to a ground receiver, which is located at the winch.
The ground receiver displays the rope tension information to the winch operator and provides guidance information on weather to increase or decrease throttle.
Throttle advice changes over the course of the winch tow.

The transmission between rope segment and winch segment happens via radio transmition. The device is meant to be used under radio amateur regulations.

During the initial acceleration phase of the glider from standstill the weight of the glider is aproximated, from measuring force (rope tension) and acceleration.
The approximated glider mass is transfered to the winch segment and displayed there and used for throttle advice.
Wind is initially ignored, but wind compensation shall later be implemented in future project revisions.
Microphython is uses as SW on both segments.
Measurement data and debug information shall be stored within the rope and winch segment as appropriate.
If Wifi is available, measurement data shall be uploaded to a server for analysis. 
