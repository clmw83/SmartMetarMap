

// This project is based on https://github.com/ironcannibal/led-sectional
// I added some smart-home bits, letting it be controlled by AdafruitIO (and thus by google home)

#include <ESP8266WiFi.h>
#include <PubSubClient.h>

#define FASTLED_ESP8266_RAW_PIN_ORDER
#include <FastLED.h>
#include <vector>
using namespace std;
#include "config.h"

#define CYCLE_TIME 50
/*--------------------------- METAR MAP CONFIG -------------------*/

#define NUM_AIRPORTS 2 //
#define WIND_THRESHOLD 25 // Maximum windspeed for green
#define LIGHTNING_INTERVAL 5000 // ms - how often should lightning strike; not precise because we sleep in-between
#define LIGHTNING_DURATION 1000 // what length of time do lightning clusters take
#define LIGHTNING_BLINK_TIME 50 //ms - how long a lightning strike is

#define DO_LIGHTNING true // Lightning uses more power, but is cool.
#define DO_WINDS true // color LEDs for high winds

#define SERVER "www.aviationweather.gov"
#define BASE_URI "/adds/dataserver_current/httpparam?dataSource=metars&requestType=retrieve&format=xml&hoursBeforeNow=3&mostRecentForEachStation=true&stationString="

#define DEBUG false
boolean ledStatus = true; // used so leds only indicate connection status on first boot, or after failure
int status = WL_IDLE_STATUS;

#define READ_TIMEOUT 15 // Cancel query if no data received (seconds)
#define WIFI_TIMEOUT 60 // in seconds
#define RETRY_TIMEOUT 15000 // in ms
#define UPDATE_TIME 900000 // in ms (15 min is 900000)

// Define the array of leds
CRGB leds[NUM_AIRPORTS];
#define DATA_PIN    D2
#define LED_TYPE    WS2812B
#define COLOR_ORDER GRB
#define MAX_BRIGHTNESS 175

std::vector<unsigned short int> lightningLeds;
std::vector<CRGB> lightningColors;
std::vector<String> airports({
  "KBED", // 0
  "KACK" // 1
});

/*--------------------------- METAR MAP CONFIG -------------------*/

// Create an ESP8266 WiFiClientSECURE for getting METAR information
WiFiClientSecure client;

// Set up a Wifi client for MQTT
#ifdef USE_MQTT_SSL
  WiFiClientSecure mclient;
#else
  WiFiClient mclient;
#endif

PubSubClient mqtt(mclient);

// Set up some variables for holding
unsigned long wait_ticks;
unsigned long last_updated;
unsigned long last_status;

// Set up some state variables for what our map is currently doing
bool map_on = true;
int map_brightness = 100;
bool first_mqtt = true;  // This is here to enable our map to return to its last state at power-on


// Function prototypes
void setMapState(); 
void set_map_off();
void set_map_brightness(int brightness);
void publishMapState();  // Used to publish the current state to our MQTT broker
void callback(char* p_topic, byte* p_payload, unsigned int p_length);  // Callback function when an MQTT message is received

bool getMetars();  // gets the metars...
void doColor(String identifier, unsigned short int led, int wind, int gusts, String condition, String wxstring); // Used to set the appropriate LED colors


void setup() {
  
  Serial.begin(115200);

  // Connect to WiFi access point.
  Serial.println(); Serial.println();
  Serial.print("Connecting to ");
  Serial.println(WLAN_SSID);
  WiFi.begin(WLAN_SSID, WLAN_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();

  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  Serial.println("Using SSL _without_ fingerprint verification for getting METARS");
  client.setInsecure();

  #ifdef USE_MQTT_SSL
    #ifdef USE_SSL_FINGERPRINT
      Serial.println("Using SSL fingerprint for MQTT connection");
      mclient.setFingerprint(SSL_FINGERPRINT);
    #else
      Serial.println("Using SSL _without_ fingerprint verification for MQTT");
      mclient.setInsecure();
    #endif
  #endif


  // Initialize LEDs
  FastLED.addLeds<LED_TYPE, DATA_PIN, COLOR_ORDER>(leds, NUM_AIRPORTS).setCorrection(TypicalLEDStrip);
  FastLED.setBrightness(MAX_BRIGHTNESS);

  mqtt.setServer(MQTT_SERVER, MQTT_PORT);
  mqtt.setCallback(callback);

  last_updated = millis();
  last_status = last_updated;
  wait_ticks = 0;
  MQTT_connect();
}


void loop() {

  const unsigned long millis_now = millis();

  // If our timer runs out, it is time to refresh METARS
  if (millis_now - last_updated >= wait_ticks) {
    Serial.println("Getting METARs ...");
    last_updated=millis_now;
    if (getMetars()) {
      Serial.println("Refreshing LEDs.");
      FastLED.show();
      wait_ticks = UPDATE_TIME;
    } else { // Getting METARS Failed
      Serial.print("Getting METARs Failed; Waiting for: ");
      Serial.println(RETRY_TIMEOUT);
      wait_ticks = RETRY_TIMEOUT; // try again if unsuccessful
    }
  }
  if (DO_LIGHTNING && lightningLeds.size() > 0) {
      for (unsigned short int i = 0; i < lightningLeds.size(); ++i) {
        unsigned short int currentLed = lightningLeds[i];
        leds[currentLed] = lightningColors[i];
        if (millis_now % LIGHTNING_INTERVAL < LIGHTNING_DURATION){
          if (millis_now % (LIGHTNING_BLINK_TIME * 2) < LIGHTNING_BLINK_TIME){
          leds[currentLed] = CRGB::White;
          }
        }
        FastLED.show();
    }
  }
  

  if ((millis_now - last_status)> 5000){
    Serial.println("Waiting for MQTT Updates");
    last_status = millis_now;
  }
  if (!mqtt.connected()) {
    MQTT_connect();
  }
  mqtt.loop();
  yield();
  delay(10);
}

void MQTT_connect() {
  if (!mqtt.connected()) {
    Serial.println("INFO: Attempting MQTT connection...");
    // Attempt to connect
    if (mqtt.connect(MQTT_CLIENT_ID, MQTT_USER, MQTT_PASSWORD)) {
      Serial.println("INFO: connected");
      //publishMapState();  // Uncomment this line if you want the server to reflect the current state.  Leaving it commented will set the LEDs to the last commanded state.
      // ... and resubscribe
      mqtt.subscribe(MQTT_COMMAND_TOPIC);
      mqtt.subscribe(MQTT_BRIGHTNESS_COMMAND_TOPIC);
    } else {
      Serial.print("ERROR: failed, rc=");
      Serial.print(mqtt.state());
      Serial.println("DEBUG: try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

// function called to publish the state of the map (on/off)
void publishMapState() {
  char mb [3];
  if (map_on) {
    mqtt.publish(MQTT_STATE_TOPIC, MAP_ON, true);
    sprintf (mb, "%03i", map_brightness);
    mqtt.publish(MQTT_BRIGHTNESS_STATE_TOPIC, mb, true);
  } else {
    mqtt.publish(MQTT_STATE_TOPIC, MAP_OFF, true);
  }
}

// function called to turn on/off the map s
void setMapState() {
  if (map_on) {
    Serial.print("INFO: Map ON, Brightness: ");
    Serial.println(map_brightness);
    FastLED.setBrightness((MAX_BRIGHTNESS * map_brightness)/100);
    FastLED.show();
    
  } else {
    Serial.println("INFO: Map OFF");
    FastLED.setBrightness(0);
    FastLED.show();
  }
}

void set_map_off() {
  if (map_on != false) {
    map_on = false;
    setMapState();
    publishMapState();
  }
}
void set_map_brightness(int brightness) {
  map_brightness = constrain(brightness, 0, 100);
  if (map_brightness == 0) {
    set_map_off();
  }
  else {
    if (! first_mqtt){  // Prevents getting a retained brightness level from turning on the map
      map_on = true;
    } else{
      first_mqtt=false;
    }
    setMapState();
    publishMapState();
  }
}

// function called when a MQTT message arrived
void callback(char* p_topic, byte* p_payload, unsigned int p_length) {

  // concat the payload into a string
  String payload;
  for (uint8_t i = 0; i < p_length; i++) {
    payload.concat((char)p_payload[i]);
  }
  // handle message topic
  if (String(MQTT_COMMAND_TOPIC).equals(p_topic)) {
    // test if the payload is equal to "ON" or "OFF"
    if (payload.equals(String(MAP_ON))) {
      set_map_brightness(map_brightness);
    } else if (payload.equals(String(MAP_OFF))) {
      set_map_off();
    }
  } else if (String(MQTT_BRIGHTNESS_COMMAND_TOPIC).equals(p_topic)) {
    set_map_brightness(constrain(payload.toInt(), 0, 100));
  }
}


bool getMetars() {
  lightningLeds.clear(); // clear out existing lightning LEDs since they're global
  lightningColors.clear();  // clear out existing lightning colors
  
  fill_solid(leds, NUM_AIRPORTS, CRGB::Black); // Set everything to black just in case there is no report
  uint32_t t;
  char c;
  boolean readingAirport = false;
  boolean readingCondition = false;
  boolean readingWind = false;
  boolean readingGusts = false;
  boolean readingWxstring = false;

  unsigned short int led = 99;
  String currentAirport = "";
  String currentCondition = "";
  String currentLine = "";
  String currentWind = "";
  String currentGusts = "";
  String currentWxstring = "";
  String airportString = "";
  bool firstAirport = true;
  for (int i = 0; i < (NUM_AIRPORTS); i++) {
    if (airports[i] != "NULL" && airports[i] != "VFR" && airports[i] != "MVFR" && airports[i] != "WVFR" && airports[i] != "IFR" && airports[i] != "LIFR") {
      if (firstAirport) {
        firstAirport = false;
        airportString = airports[i];
      } else airportString = airportString + "," + airports[i];
    }
  }


  Serial.println("\nStarting connection to server...");
  // if you get a connection, report back via serial:
  if (!client.connect(SERVER, 443)) {
    Serial.println("Connection failed!");
    client.stop();
    return false;
  } else {
    Serial.println("Connected ...");
    Serial.print("GET ");
    Serial.print(BASE_URI);
    Serial.print(airportString);
    Serial.println(" HTTP/1.1");
    Serial.print("Host: ");
    Serial.println(SERVER);
    Serial.println("Connection: close");
    Serial.println();
    // Make a HTTP request, and print it to console:
    client.print("GET ");
    client.print(BASE_URI);
    client.print(airportString);
    client.println(" HTTP/1.1");
    client.print("Host: ");
    client.println(SERVER);
    client.println("Connection: close");
    client.println();
    client.flush();
    t = millis(); // start time
    FastLED.clear();

    Serial.print("Getting data");

    while (!client.connected()) {
      if ((millis() - t) >= (READ_TIMEOUT * 1000)) {
        Serial.println("---Timeout---");
        client.stop();
        return false;
      }
      Serial.print(".");
      delay(1000);
    }

    Serial.println();

    while (client.connected()) {
      if ((c = client.read()) >= 0) {
        yield(); // Otherwise the WiFi stack can crash
        currentLine += c;
        if (c == '\n') currentLine = "";
        if (currentLine.endsWith("<station_id>")) { // start paying attention
          if (led != 99) { // we assume we are recording results at each change in airport; 99 means no airport
            doColor(currentAirport, led, currentWind.toInt(), currentGusts.toInt(), currentCondition, currentWxstring);
          }
          currentAirport = ""; // Reset everything when the airport changes
          readingAirport = true;
          currentCondition = "";
          currentWind = "";
          currentGusts = "";
          currentWxstring = "";
        } else if (readingAirport) {
          if (!currentLine.endsWith("<")) {
            currentAirport += c;
          } else {
            readingAirport = false;
            for (unsigned short int i = 0; i < NUM_AIRPORTS; i++) {
              if (airports[i] == currentAirport) {
                led = i;
              }
            }
          }
        } else if (currentLine.endsWith("<wind_speed_kt>")) {
          readingWind = true;
        } else if (readingWind) {
          if (!currentLine.endsWith("<")) {
            currentWind += c;
          } else {
            readingWind = false;
          }
        } else if (currentLine.endsWith("<wind_gust_kt>")) {
          readingGusts = true;
        } else if (readingGusts) {
          if (!currentLine.endsWith("<")) {
            currentGusts += c;
          } else {
            readingGusts = false;
          }
        } else if (currentLine.endsWith("<flight_category>")) {
          readingCondition = true;
        } else if (readingCondition) {
          if (!currentLine.endsWith("<")) {
            currentCondition += c;
          } else {
            readingCondition = false;
          }
        } else if (currentLine.endsWith("<wx_string>")) {
          readingWxstring = true;
        } else if (readingWxstring) {
          if (!currentLine.endsWith("<")) {
            currentWxstring += c;
          } else {
            readingWxstring = false;
          }
        }
        t = millis(); // Reset timeout clock
      } else if ((millis() - t) >= (READ_TIMEOUT * 1000)) {
        Serial.println("---Timeout---");
        fill_solid(leds, NUM_AIRPORTS, CRGB::Cyan); // indicate status with LEDs
        FastLED.show();
        ledStatus = true;
        client.stop();
        return false;
      }
    }
  }
  // need to doColor this for the last airport
  doColor(currentAirport, led, currentWind.toInt(), currentGusts.toInt(), currentCondition, currentWxstring);

  // Do the key LEDs now if they exist
  for (int i = 0; i < (NUM_AIRPORTS); i++) {
    // Use this opportunity to set colors for LEDs in our key then build the request string
    if (airports[i] == "VFR") leds[i] = CRGB::Green;
    else if (airports[i] == "WVFR") leds[i] = CRGB::Yellow;
    else if (airports[i] == "MVFR") leds[i] = CRGB::Blue;
    else if (airports[i] == "IFR") leds[i] = CRGB::Red;
    else if (airports[i] == "LIFR") leds[i] = CRGB::Magenta;
  }

  client.stop();
  return true;
}


void doColor(String identifier, unsigned short int led, int wind, int gusts, String condition, String wxstring) {
  CRGB color;
  Serial.print(identifier);
  Serial.print(": ");
  Serial.print(condition);
  Serial.print(" ");
  Serial.print(wind);
  Serial.print("G");
  Serial.print(gusts);
  Serial.print("kts LED ");
  Serial.print(led);
  Serial.print(" WX: ");
  Serial.println(wxstring);

  if (condition == "LIFR" || identifier == "LIFR") color = CRGB::Magenta;
  else if (condition == "IFR") color = CRGB::Red;
  else if (condition == "MVFR") color = CRGB::Blue;
  else if (condition == "VFR") {
    if ((wind > WIND_THRESHOLD || gusts > WIND_THRESHOLD) && DO_WINDS) {
      color = CRGB::Yellow;
    } else {
      color = CRGB::Green;
    }
  } else color = CRGB::Black;

  leds[led] = color;
  
  if (wxstring.indexOf("TS") != -1) {
    Serial.println("... found lightning!");
    lightningLeds.push_back(led);
    lightningColors.push_back(color);
  }
  
}
