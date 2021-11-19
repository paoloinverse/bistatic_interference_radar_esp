# bistatic_interference_radar_esp
Use the ESP32 wifi 802.11 radio as a bistatic radar based on multipath interference. Will detect human intruders when they cross the path between the ESP32 and other access points or wifi clients or freely move inside rooms / buildings at a reasonable distance from the ESP32.

Refer to the included .ino example, the basic usage is pretty simple. Feel free to experiment, this code works pretty well inside buildings. 

You will notice the strongest variations are when an intruder crosses the line-of-sight path between the ESP32 and the other access point or client used as transmitter. However, appreciable variations will be detected when indirect / reflected paths are crossed!
