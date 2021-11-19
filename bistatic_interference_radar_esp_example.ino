// Bistatic interference radar, basic example for the ESP32.

#include "bistatic_interference_radar.h"

#include <WiFi.h>


const char* ssid     = "your-ssid";
const char* password = "your-password";


void setup()
{
    Serial.begin(115200);
    delay(10);

    // We start by connecting to a WiFi network

    Serial.println();
    Serial.println();
    Serial.print("Connecting to ");
    Serial.println(ssid);

    WiFi.begin(ssid, password);

    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }

    Serial.println("");
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());


  
    // setting up the wifi radar

    bistatic_interference_radar_init();  // initializes the storage arrays in internal RAM

    // reconfigure the library with new parameters: 
    // sample buffer depth (how many samples to store in the circular buffer),  = 64
    // mobile average filter size, = 16 
    // variance threshold ( >= 0, how much the interference signal deviates from the norm before triggering a detection result, in dBm), = 3
    // variance integrator limit (how many variance samples we cumulate before evaluating the variance threshold level),  = 3
    // finally bolean var set to true -> enable autoregressive filtering (default is false -> disable autoregressive filtering)  = false

    // look into bistatic_interference_radar.cpp for more details and to eventually modify any hard configuration limits in the #define lines.

    bistatic_interference_radar_config(64, 16, 3, 3, false); 

    

    bistatic_interference_radar_debug_via_serial(1); // show debugging information, at the simplest debugging level. Level 0 means no output. 


    bistatic_interference_radar_set_debug_level(3); // set a very verbose level for operating the radar.

    
}



int wifiRadarLevel = RADAR_BOOTING; // initial value = -1, any values < 0 are errors, see bistatic_interference_radar.h , ERROR LEVELS sections for details on how to intepret any errors.




void loop()
{
    delay(500);

    
    wifiRadarLevel = bistatic_interference_radar_esp();  // if the connection fails, the radar will automatically try to switch to different operating modes by using ESP32 specific calls. 
    
    Serial.print("wifiRadarLevel: ");
    Serial.println(wifiRadarLevel);
    
}
