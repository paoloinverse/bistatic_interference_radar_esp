# Bistatic interference radar fully based on the ESP32 DevKit onboard wifi radio.

Do you really need to connect your ESP32 SoC to external movement detector sensors such as infrared PIR, ultrasonic or dedicated radar sensors, when the ESP32 itself can work as a passive radar *without any additional hardware* ?


Use the ESP32 wifi 802.11 radio as a bistatic radar based on multipath interference. Will detect human intruders when they cross the path between the ESP32 and other access points or wifi clients or freely move inside rooms / buildings at a reasonable distance from the ESP32.

Refer to the included .ino example, the basic usage is pretty simple. Feel free to experiment, this code works pretty well inside buildings. 

You will notice the strongest variations are when an intruder crosses the line-of-sight path between the ESP32 and the other access point or client used as transmitter. However, appreciable variations will be detected when indirect / reflected paths are crossed!

The example code outputs the signal over the serial port, Arduino IDE can be used to produce a plot in order to test the capabilities of the radar code and to 
determine the optimum parameters. 
Most of the code is fully automated and preconfigured, although the core parameters can be modified by editing the .h file. 

Functions are provided to tweak the most impost important parameters, including the RSSI threshold, alarm threshold and the debugging level. 

Commented example plot follows:

![example_plot_01](https://user-images.githubusercontent.com/62485162/146927658-b540635e-16f6-4b56-b713-32469a1c8256.png)

The plot is in arbitrary units derived from the received signal variance data, it is contructed by a relatively complex digital filter entirely built in integer math, with a low computational expense.

Please note the code is mostly self-configuring and can autonomously take care of the common problems and failures typically encountered in a wifi based infrastructure, incuding faults affecting the nearby access points and stations, depending on the ESP32 operating mode. 

That being said, feel free to mess with the library internal parameters (such as buffer and filter sizes): if you find anything interesting and worth of notice, I'd be pleased to discuss it with you. 
