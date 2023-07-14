#include <dhtnew.h>
#include <SPI.h>
#include <Ethernet.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <PZEM004T.h>

#define RELAY_HEAT 30
#define RELAY_COOL 31
#define RELAY_FAN 32
#define RELAY_LIGHTS 33
#define RELAY_COMP 34

#define ADC_COMP_PSI A0
#define STATE_LIGHTS 22
#define STATE_COMP 23

DHTNEW mySensor(48);
PZEM004T* pzemA;
PZEM004T* pzemB;

// Update these with values suitable for your hardware/network.
byte mac[]    = {  0xDE, 0xED, 0xBA, 0xFE, 0xFE, 0xBE };
IPAddress server(10, 150, 86, 196);
IPAddress ip(192, 168, 0, 29);
IPAddress myDns(192, 168, 0, 1);

EthernetClient eClient;
PubSubClient client(eClient);

long lastReconnectAttempt = 0;

int setTemp = 0;
String setMode = "";
float tempF = 0.00;
float humidityRH = 0.00;
float thermostatHysteresis = 3.00;
unsigned long lastTimer = 0UL;
bool stateHeating = false;
bool stateCooling = false;
bool stateFan = false;
bool stateCool = false;
bool stateLights1 = false;
bool stateComp1 = false;
bool stateLights1SW = false;
bool stateComp1SW = false;
bool state_act_lights1 = false;
bool state_act_comp1 = false;
unsigned long coolTimer = 0UL;
String setLights1 = "";
String setComp1 = "";

boolean reconnect() {
  if (client.connect("arduinoClient")) {
    client.subscribe("hvac/mode/set");
    client.subscribe("hvac/temperature/set");
    client.subscribe("shop/switch/lights1/set");
    client.subscribe("shop/switch/comp1/set");

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

  /*for (int i=30; i<= 46; i++) {
    pinMode(i, OUTPUT);
    digitalWrite(i, HIGH); // Relays are active with a LOW signal
  }*/

  for (int i = 30; i <= 34; i++) {
    pinMode(i, OUTPUT);
    digitalWrite(i,HIGH);
  }

  pzemA = new PZEM004T(&Serial1);
  pzemA->setAddress(ip);
  pzemB = new PZEM004T(&Serial2);
  pzemB->setAddress(ip);

  pinMode(STATE_LIGHTS,INPUT);
  pinMode(STATE_COMP,INPUT);
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

    /*float vA = pzemA->voltage(ip);
    float iA = pzemA->current(ip);
    float pA = pzemA->power(ip);
    float eA = pzemA->energy(ip);
    float vB = pzemB->voltage(ip);
    float iB = pzemB->current(ip);
    float pB = pzemB->power(ip);
    float eB = pzemB->energy(ip);*/

    /** \brief handle switches
     *
     *
     */
    if (setLights1 == "ON" && state_act_lights1 == false) {
        stateLights1 = true;
    } else if (setLights1 == "OFF" && state_act_lights1 == true) {
        stateLights1 = false;
    }
    if (stateLights1 == true) {
        digitalWrite(RELAY_LIGHTS,LOW);
        state_act_lights1 = true;
    } else {
        digitalWrite(RELAY_LIGHTS,HIGH);
        state_act_lights1 = false;
    }

    if (setComp1 == "ON" && state_act_comp1 == false) {
        stateComp1 = true;
    } else if (setComp1 == "OFF" && state_act_comp1 == true) {
        stateComp1 = false;
    }
    if (stateComp1 == true) {
        digitalWrite(RELAY_COMP,LOW);
        state_act_comp1 = true;
    } else {
        digitalWrite(RELAY_COMP,HIGH);
        state_act_comp1 = false;
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
        if (tempF >= setTemp) {stateHeating = false;}
        else if (tempF <= setTemp - thermostatHysteresis) {stateHeating = true;}
    } else if (setMode == "cool") {
        stateHeating = false;
        if (tempF <= setTemp) {stateCooling = false;}
        else if (tempF >= setTemp + thermostatHysteresis) {stateCooling = true;}
    } else if (setMode == "auto") {
        if (tempF >= setTemp + thermostatHysteresis && stateCooling == false) {
            stateCooling = true;
            stateHeating = false;
        } else if (tempF <= setTemp - thermostatHysteresis && stateHeating == false) {
            stateCooling = false;
            stateHeating = true;
        } else if ((tempF >= setTemp && stateHeating == true) || (tempF <= setTemp && stateCooling == true)) {
            stateCooling = false;
            stateHeating = false;
        }

    }
    if (stateHeating == true) {digitalWrite(RELAY_HEAT, LOW);}
    else {digitalWrite(RELAY_HEAT, HIGH);}
    if (stateCooling == true && stateFan == false && stateCool == false) { digitalWrite(RELAY_FAN, LOW); stateFan = true; }
    else if (stateCooling == true && millis() - coolTimer >= 10000 && stateFan == true && stateCool == false) {stateCool = true; coolTimer = millis(); digitalWrite(RELAY_COOL, LOW);}
    else if(stateCooling == false) {digitalWrite(RELAY_COOL, HIGH); digitalWrite(RELAY_FAN, HIGH); stateFan = false; stateCool = false;}

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
    if (state_act_lights1 == true) {
        client.publish("shop/switch/lights1","ON");
    } else {
        client.publish("shop/switch/lights1","OFF");
    }
    if (state_act_comp1 == true) {
        client.publish("shop/switch/comp1","ON");
    } else {
        client.publish("shop/switch/comp1","OFF");
    }
    if (digitalRead(STATE_LIGHTS) == HIGH) {
        client.publish("shop/switch/lights1/state","ON");
    } else {
        client.publish("shop/switch/lights1/state","OFF");
    }
    if (digitalRead(STATE_COMP) == HIGH) {
        client.publish("shop/switch/comp1/state","ON");
    } else {
        client.publish("shop/switch/comp1/state","OFF");
    }
    dtostrf(adcCompPsi, 4, 2, sz);
    client.publish("shop/sensor/comppsi1",sz);
    /*dtostrf(vA, 4, 2, sz);
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
    client.publish("shop/sensor/eB",sz);*/
  }
}
