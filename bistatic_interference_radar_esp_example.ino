// Bistatic interference radar, basic example for the ESP32.

#include "bistatic_interference_radar.h"

#include <WiFi.h>


int enableCSVgraphOutput = 1; // 0 disable, 1 enable // if enabled, you may use Tools-> Serial Plotter to plot the variance output for each transmitter. 

int scanInterval = 500; // in milliseconds

// PLEASE NOTE: by default configuration, it takes 64 iterations to collect enough data to produce a meaningful output. PLEASE BE PATIENT AND WAIT about 32 seconds for that. 
// While initializing, the output is tipically < 0 and meaningless. 

// My suggestion is to use Tools->Serial plotter to see the actual graph, also don't forget you can change most runtime parameters for this library by sending simple serial commands 
// (see the manageSerialCommands() function down below)


// set the access point and password to connect to. THIS IS MANDATORY FOR THE MOMENT.
const char* ssid     = "my access point name";
const char* password = "my wifi password";

// also configure the onboard soft AP
const char *soft_ap_ssid = "MyESP32AP";
const char *soft_ap_password = "testpassword";


int wifiRadarLevel = RADAR_BOOTING; // initial value = -1, any values < 0 are errors, see bistatic_interference_radar.h , ERROR LEVELS sections for details on how to intepret any errors.



void setup()
{
    Serial.begin(115200);
    delay(10);

    // We start by connecting to a WiFi network

    Serial.println();
    Serial.println();
    Serial.print("Connecting to ");
    Serial.println(ssid);

    WiFi.mode(WIFI_MODE_APSTA);

    WiFi.softAP(soft_ap_ssid, soft_ap_password);

    // uncomment this block if you prefer connecting the ESP32 to an external access point
    /*
    WiFi.begin(ssid, password);

    int connCounter = 0;

    while ((WiFi.status() != WL_CONNECTED) && (connCounter < 20)) {
        delay(500);
        Serial.print(".");
        connCounter++;
    }

    Serial.println("");
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());

    */



  
    // setting up the wifi radar

    bistatic_interference_radar_init();  // initializes the storage arrays in internal RAM

    // reconfigure the library with new parameters: 
    // sample buffer depth (how many samples to store in the circular buffer),  = 64
    // mobile average filter size, = 16 
    // variance threshold ( >= 0, how much the interference signal deviates from the norm before triggering a detection result, in variance arbitrary units), = 3
    // variance integrator limit (how many variance samples we cumulate before evaluating the variance threshold level),  = 3
    // finally bolean var set to true -> enable autoregressive filtering (default is false -> disable autoregressive filtering)  = false

    // look into bistatic_interference_radar.cpp for more details and to eventually modify any hard configuration limits in the #define lines.

    bistatic_interference_radar_config(64, 16, 3, 3, false); 

    

    bistatic_interference_radar_debug_via_serial(1); // show debugging information, at the simplest debugging level. Level 0 means no output. 


    bistatic_interference_radar_set_debug_level(3); // set a very verbose level for operating the radar.



    if (enableCSVgraphOutput > 0) {      // USE THE Tools->Serial Plotter to graph the live data!
      bistatic_interference_radar_enable_serial_CSV_graph_data(enableCSVgraphOutput); // output CSV data only
    }
    
    //// trying to reduce overall power usage

    setCpuFrequencyMhz(80);

    Serial.setTimeout(1000);

    
}




void manageSerialCommands() { // receives simple commands via serial port in the form CommandParamenter where command is a single character and Parameter is an ascii string representing a number, for example: d1 sets command d (debug level) to 1. I expect the string received to be null terminated.

// for example, to set the minimum acceptable RSSI threshold to -80 dBm, send via serial the string m-80
// to enable debugging messages at level 3, send via serial the string d3 
// and so on. 


  char serBuf[32] = {0};  // don't make these global: character buffers NEED to be reinitialized to zero after each use. Just leave this to the function code itself.
  char serCom = 0;
  char serPar[64] = {0};
  int serBytesAvail = 0;
  int serParVal = 0;


    serBytesAvail = Serial.available();
    if (serBytesAvail >=32 ) {
      serBytesAvail = 0;
      Serial.flush();
    }
    if (serBytesAvail >= 2) {
      Serial.readBytes(serBuf, serBytesAvail);
      Serial.flush();
      serCom = serBuf[0];
      for (int paridx = 1; paridx < serBytesAvail; paridx++) {
        serPar[paridx -1] = serBuf[paridx];
        if (serBuf[paridx] == 0) {
          break;
        }
      }
      serPar[serBytesAvail] = 0;
      serParVal = atoi(serPar);
      if (serCom == 'd') {
        bistatic_interference_radar_set_debug_level(serParVal);
      }
      
      if (serCom == 'g') {
        bistatic_interference_radar_enable_serial_CSV_graph_data(serParVal);
      }
      if (serCom == 's') {
        bistatic_interference_radar_debug_via_serial(serParVal);
      }
      
      if (serCom == 'm') {
        bistatic_interference_radar_set_minimum_RSSI(serParVal);
      }
      if (serCom == 'a') {
        bistatic_interference_radar_enable_alarm(serParVal);
      }
      if (serCom == 't') {
        bistatic_interference_radar_set_alarm_threshold(serParVal);
      }
      if (serCom == 'r') {
        Serial.println("REBOOT REQUESTED");
        ESP.restart();
      }
      if (serCom == 'q') {
        Serial.println("SERIAL FLUSH REQUESTED");
        Serial.flush();
      }
      if (serCom == 'i') { // set the scan interval in milliseconds
        scanInterval = serParVal;
      }
    }

}



void loop()
{
    delay(scanInterval);

    manageSerialCommands();
      
    wifiRadarLevel = bistatic_interference_radar_esp();  // if the connection fails, the radar will automatically try to switch to different operating modes by using ESP32 specific calls. 
    if (enableCSVgraphOutput == 0) {  // for the graph via serial, we just let the library do the job. 
      Serial.print("wifiRadarLevel: ");
      Serial.println(wifiRadarLevel);
    }

  
    
    
}
