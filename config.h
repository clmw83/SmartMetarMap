// Site-specific configuration

/************************* WiFi Setup *********************************/

#define WLAN_SSID       "<CHANGE ME>"             // Your SSID
#define WLAN_PASS       "<CHANGE ME>"        // Your password

//#define USE_SSL_FINGERPRINT

/************************* MQTT Setup *********************************/

const PROGMEM char* MQTT_SERVER = "<CHANGE ME>";  // Can use a local MQTT server (for e.g. homeassistant, or one like Adafruit.io that can tie in to IFTTT)
const PROGMEM uint16_t MQTT_PORT = 1883;  // Use 8883 for SSL
const PROGMEM char* MQTT_CLIENT_ID = "<CHANGE ME>";
const PROGMEM char* MQTT_USER = "<CHANGE ME>";
const PROGMEM char* MQTT_PASSWORD = "<CHANGE ME>"; // If using Adafruit.io, should be your key

// The configuration of states/topics below is an example for setting it like an MQTT Light (e.g. in HomeAssistant)
const PROGMEM char* MQTT_STATE_TOPIC = "METARMap/status";
const PROGMEM char* MQTT_COMMAND_TOPIC = "METARMap/switch";
const PROGMEM char* MQTT_BRIGHTNESS_STATE_TOPIC = "METARMap/brightness";
const PROGMEM char* MQTT_BRIGHTNESS_COMMAND_TOPIC = "METARMap/brightness/set";
const PROGMEM char* MAP_ON = "ON";
const PROGMEM char* MAP_OFF = "OFF";

// Uncomment below if you want to use SSL to connect to the MQTT broker
//#define USE_MQTT_SSL  
// Define USE_SSL_FINGERPRINT if you want a truy secure connection.  Downside is that you need to update the fingerprint
// every time the remote SSL certificate is updated.
//#define USE_SSL_FINGERPRINT
//#define SSL_FINGERPRINT "7700542ddae7d80327312399eb27dbcba54c5718"
