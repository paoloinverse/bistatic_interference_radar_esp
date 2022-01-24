// implementation of the bistatic / multistatic interference radar concept: any object moving inside the field between two wifi radios modifies the propagation paths and therefore induces a variation in the RSSI values of each. The purpose of this library is to filter and detect such variations. 
// warning: the bistatic version requires wifi supplicant and access point to be connected. 
// the multistatic version, requires the presence of one or more access point in range, but no connection is required. 


// WARNING: YOU'RE SUPPOSED TO HAVE ALREADY INITIALIZED WIFI BEFORE USING THIS LIBRARY (i.e. WiFi.begin() and mode (WIFI_STA or WIFI_STA_AP) have already been set, and WiFi is already connected to an external AP or has one client already connected

// Note: the library internally uses the standard wifi.h library but can also be made to work without such library if the rssi value is passed as a parameter. 



#define MINIMUM_RSSI -80  // in dBm   // minimum acceptable RSSI to consider a transmitter valid for processing (too low the RSSI, the results might become meaningless)

#define ABSOLUTE_RSSI_LIMIT -128  // in dBm  // normaly you should never need to touch this value, it's already lower than the physical limits of most radios.


#define RADAR_SCAN_INTERVAL 2000 // userland define


int bistatic_interference_radar_init();  // initializes the storage arrays in internal RAM


int bistatic_interference_radar_init_PSRAM();  // initializes the storage arrays in PSRAM (only for ESP32-CAM modules and ESP32-WROVER modules with the onboard SPI RAM chip)


int bistatic_interference_radar_deinit();  // deinitializes the storage arrays


int bistatic_interference_radar_config(int, int, int, int, bool); // reconfigure the library with new parameters: sample buffer depth (how many samples to store in the circular buffer, mobile average filter size, variance threshold ( >= 0, how much the interference signal deviates from the norm before triggering a detection result, in dBm), variance integrator limit (how many variance samples we cumulate before evaluating the variance threshold level), finally bolean var set to true -> enable autoregressive filtering (default is false -> disable autoregressive filtering)


int bistatic_interference_radar_process(int); // receives RSSI signal as parameter, returns the detection level ( < 0 -> error (see ERROR LEVELS section), == 0 -> no detection, > 0 -> detection level in dBm)


int bistatic_interference_radar(); // generic version, only works in STA mode; request the RSSI level internally, then process the signal and return the detection level in dBm




// bistatic version ESP32 ONLY

int bistatic_interference_radar_esp(); // ESP32 specific version, uses the ESP API directly and also works in SoftAP mode; request the RSSI level internally, then process the signal and return the detection level in dBm




// runtime configuration functions


int bistatic_interference_radar_debug_via_serial(int);  // parameter is debug level, set it to at least >= 1; the highest the level, the more messages you enjoy


int bistatic_interference_radar_set_debug_level(int);  // parameter is debug level, set it to at least >= 1; the highest the level, the more messages you enjoy



// current status: IMPLEMENTED
int bistatic_interference_radar_enable_serial_CSV_graph_data(int); // [ 0 = disabled, >=1 = enabled (default is 0) ] completely disable the verbose output and only print the variance over the serial console, so that any program listening to the serial can graph the relevant data. 


// current status: IMPLEMENTED
int bistatic_interference_radar_set_minimum_RSSI(int);

// current status: IMPLEMENTED
int bistatic_interference_radar_enable_alarm(int);


// current status: IMPLEMENTED
int bistatic_interference_radar_set_alarm_threshold(int);


// ERROR LEVELS

#define RADAR_RSSI_TOO_LOW -6
#define WIFI_UNINITIALIZED -5
#define WIFI_MODEINVALID -4
#define RADAR_INOPERABLE -3
#define RADAR_UNINITIALIZED -2
#define RADAR_BOOTING -1
