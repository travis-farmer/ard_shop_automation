#include <dhtnew.h>
#include <SPI.h>
#include <Ethernet.h>
#include <PubSubClient.h>
#include <Adafruit_MCP23X17.h>
#include <ArduinoJson.h>
#include <PZEM004T.h>

#define MCP_HEAT 0
#define MCP_COOL 1
#define MCP_LIGHTS 2
#define MCP_COMP 3
#define ADC_COMP_PSI A0

Adafruit_MCP23X17 mcp;
DHTNEW mySensor(48);
PZEM004T* pzemA;
PZEM004T* pzemB;

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

  pzemA = new PZEM004T(&Serial1);
  pzemA->setAddress(ip);
  pzemB = new PZEM004T(&Serial2);
  pzemB->setAddress(ip);
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
    float adcCompPsiVolts = ((5.00/1024)*analogRead(ADC_COMP_PSI));
    float adcCompPsi = map(adcCompPsiVolts,0.50,4.50,0,150);

    float vA = pzemA->voltage(ip);
    float iA = pzemA->current(ip);
    float pA = pzemA->power(ip);
    float eA = pzemA->energy(ip);
    float vB = pzemB->voltage(ip);
    float iB = pzemB->current(ip);
    float pB = pzemB->power(ip);
    float eB = pzemB->energy(ip);

    /** \brief handle switches
     *
     *
     */
    if (setLights1 == "ON") {
        mcp.digitalWrite(MCP_LIGHTS, HIGH);
    } else if (setLights1 == "OFF") {
        mcp.digitalWrite(MCP_LIGHTS, LOW);
    }

    if (setComp1 == "ON") {
        mcp.digitalWrite(MCP_COMP, HIGH);
    } else if (setComp1 == "OFF") {
        mcp.digitalWrite(MCP_COMP, LOW);
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
    if (setLights1 == "ON") {
        client.publish("shop/switch/lights1","ON");
    } else {
        client.publish("shop/switch/lights1","OFF");
    }
    if (setComp1 == "ON") {
        client.publish("shop/switch/comp1","ON");
    } else {
        client.publish("shop/switch/comp1","OFF");
    }
    dtostrf(adcCompPsi, 4, 2, sz);
    client.publish("shop/sensor/comppsi1",sz);
    dtostrf(vA, 4, 2, sz);
    client.publish("shop/sensor/vA",sz);
    dtostrf(iA, 4, 2, sz);
    client.publish("shop/sensor/iA",sz);
    dtostrf(pA, 4, 2, sz);
    client.publish("shop/sensor/pA",sz);
    dtostrf(eA, 4, 2, sz);
    client.publish("shop/sensor/eA",sz);
    dtostrf(vB, 4, 2, sz);
    client.publish("shop/sensor/vB",sz);
    dtostrf(iB, 4, 2, sz);
    client.publish("shop/sensor/iB",sz);
    dtostrf(pB, 4, 2, sz);
    client.publish("shop/sensor/pB",sz);
    dtostrf(eB, 4, 2, sz);
    client.publish("shop/sensor/eB",sz);
  }
}