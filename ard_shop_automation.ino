#include <dhtnew.h>
#include <SPI.h>
#include <Ethernet.h>
#include <PubSubClient.h>
#include <Adafruit_MCP23X17.h>
#include <ArduinoJson.h>
#define MCP_HEAT 0
#define MCP_COOL 1
#define MCP_LIGHTS 2
#define MCP_COMP 3
#define PIN_EXSW_LIGHTS 23
#define PIN_LIGHTS_SENSE 24
#define PIN_EXSW_COMP 25
#define PIN_COMP_SENSE 26


Adafruit_MCP23X17 mcp;
DHTNEW mySensor(22);

// Update these with values suitable for your hardware/network.
byte mac[]    = {  0xDE, 0xED, 0xBA, 0xFE, 0xFE, 0xBE };
IPAddress server(10, 150, 86, 199);
IPAddress ip(192, 168, 0, 29);
IPAddress myDns(192, 168, 0, 1);

EthernetClient eClient;
PubSubClient client(eClient);

long lastReconnectAttempt = 0;

int setTemp = 0;
String setMode = "";
float tempF = 0.00;
float humidityRH = 0.00;
unsigned long lastTimer = 0UL;
bool stateHeating = false;
bool stateCooling = false;
bool stateLights1 = false;
bool stateComp1 = false;
unsigned long coolTimer = 0UL;
String setLights1 = "";
String setComp1 = "";

boolean reconnect() {
  if (client.connect("arduinoClient", "mqtt_devices", "10994036")) {
    client.subscribe("hvac/mode/set");
    client.subscribe("hvac/temperature/set");
    client.subscribe("shop/switch/lights1/set");
    client.subscribe("shop/switch/comp1");

  }
  return client.connected();
}

void callback(char* topic, byte* payload, unsigned int length) {
  String tmpTopic = topic;
  char tmpStr[length+1];
  for (int x=0; x<length; x++) {
    tmpStr[x] = (char)payload[x]; // payload is a stream, so we need to chop it by length.
  }
  tmpStr[length] = 0x00; // terminate the char string with a null

  if (tmpTopic == "hvac/mode/set") {setMode = tmpStr; }
  else if (tmpTopic == "hvac/temperature/set") { setTemp = atoi(tmpStr); }
  else if (tmpTopic == "shop/switch/lights1/set") { setLights1 = tmpStr; }
  else if (tmpTopic == "shop/switch/comp1/set") { setComp1 = tmpStr; }

}

void setup() {
  client.setServer(server, 1883);
  client.setCallback(callback);
  mcp.begin_I2C();
 //Serial.println("Initialize Ethernet with DHCP:");
  if (Ethernet.begin(mac) == 0) {
    //Serial.println("Failed to configure Ethernet using DHCP");
    // Check for Ethernet hardware present
    if (Ethernet.hardwareStatus() == EthernetNoHardware) {
      //Serial.println("Ethernet shield was not found.  Sorry, can't run without hardware. :(");
      //errorProc(1);
    }
    if (Ethernet.linkStatus() == LinkOFF) {
      //Serial.println("Ethernet cable is not connected.");
      //errorProc(2);
    }
    // try to congifure using IP address instead of DHCP:
    Ethernet.begin(mac, ip, myDns);
  } else {
    //Serial.print("  DHCP assigned IP ");
    //Serial.println(Ethernet.localIP());
  }
  delay(1500);
  lastReconnectAttempt = 0;

  for (int i=0; i<= 15; i++) {
    mcp.pinMode(i, OUTPUT);
    mcp.digitalWrite(i, LOW);
  }

}

void loop() {
  if (!client.connected()) {
    long now = millis();
    if (now - lastReconnectAttempt > 5000) {
      lastReconnectAttempt = now;
      // Attempt to reconnect
      if (reconnect()) {
        lastReconnectAttempt = 0;
      }
    }
  } else {
    // Client connected

    client.loop();
  }

  if (millis() - lastTimer >= 500) {
    /** \brief setup current values
     *
     *
     */
    lastTimer = millis();
    int chk = mySensor.read();
    humidityRH = mySensor.getHumidity();
    tempF = ((mySensor.getTemperature() * 1.80) + 32.00);


    /** \brief handle switches
     *
     *
     */
    if (setLights1 == "ON" && digitalRead(PIN_LIGHTS_SENSE) == LOW) {
        stateLights1 = !stateLights1;
        mcp.digitalWrite(MCP_LIGHTS, stateLights1);
    } else if (setLights1 == "OFF" && digitalRead(PIN_LIGHTS_SENSE) == HIGH) {
        stateLights1 = !stateLights1;
        mcp.digitalWrite(MCP_LIGHTS, stateLights1);
    }

    if (setComp1 == "ON" && digitalRead(PIN_COMP_SENSE) == LOW) {
        stateComp1 = !stateComp1;
        mcp.digitalWrite(MCP_COMP, stateComp1);
    } else if (setComp1 == "OFF" && digitalRead(PIN_COMP_SENSE) == HIGH) {
        stateComp1 = !stateComp1;
        mcp.digitalWrite(MCP_COMP, stateComp1);
    }


    /** \brief handle thermostat
     *
     *
     */
    if (setMode == "off") {
        stateHeating = false;
        stateCooling = false;
    } else if (setMode == "heat") {
        stateCooling = false;
        if (tempF >= setTemp + 1) {stateHeating = false;}
        else if (tempF <= setTemp - 2) {stateHeating = true;}
    } else if (setMode == "cool") {
        stateHeating = false;
        if (tempF <= setTemp - 1) {stateCooling = false;}
        else if (tempF >= setTemp + 2) {stateCooling = true;}
    }
    if (stateHeating == true) {mcp.digitalWrite(MCP_HEAT, HIGH);}
    else {mcp.digitalWrite(MCP_HEAT, LOW);}
    if (stateCooling == true && millis() - coolTimer > 10000) {coolTimer = millis(); mcp.digitalWrite(MCP_COOL, HIGH);}
    else {mcp.digitalWrite(MCP_COOL, LOW);}

    /** \brief setup and send values and states
     *
     *
     */
    char sz[32];
    String strAction = "";
    if (stateHeating == true) {strAction = "heating";}
    else if (stateCooling == true) {strAction = "cooling";}
    else {strAction = "off";}
    strAction.toCharArray(sz, 32);
    client.publish("hvac/action",sz);
    dtostrf(tempF, 4, 2, sz);
    client.publish("hvac/temperature/current",sz);
    dtostrf(humidityRH, 4, 2, sz);
    client.publish("hvac/humidity/current",sz);
    if (digitalRead(PIN_LIGHTS_SENSE) == HIGH) {
        client.publish("shop/switch/lights1","ON");
    } else {
        client.publish("shop/switch/lights1","OFF");
    }
    if (digitalRead(PIN_COMP_SENSE) == HIGH) {
        client.publish("shop/switch/comp1","ON");
    } else {
        client.publish("shop/switch/comp1","OFF");
    }
  }
}
