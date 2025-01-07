#include <ESP8266MQTTClient.h>
#include <ESP8266WiFi.h>
//#include <ESP8266HTTPUpdateServer.h>
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>
#include <EEPROM.h>
#include <fauxmoESP.h>
#include "Linear_c.hpp"
/*************************** MQTT ***************************/
const String topicNyulketrecCtl = "/emoke/1/nyulajtonyitctl";
const String topicNyulketrecStat = "/emoke/1/nyulajtonyitstat";
const String topicTestMoveTo = "/test/moveto";      // param: position
const String topicTestFreeRun = "/test/freerun";    // params: speed(-1023..0..1023): speed can be + or -, changes direction, 0 stop

const String mqttBoolTrue = "1";
const String mqttBoolFalse = "0";

bool mqttConnected;
MQTTClient mqtt;

/*************************** WIFI ***************************/
const char gWifiConfiguredMarker[] = { "ZSIR" };
const char host[] = "nyuszi";

ESP8266WebServer server (80);
String wifi_ssid;
String wifi_passwd;
bool ghaveWifiCredentials;
bool apActive;                      // AP should be active
bool wifiStarted;                   // true if connection was initiated
bool wifiPending;                   // wifi connection initiated but not settled in a connected state
unsigned long wifiPendingTimeout;   // time by that wifi should have a final state - connected, fail...
unsigned long wifiSTAStoppedmsec;   // time STA connection gets broken / disconnected
unsigned long apActiveTimeout;      // time for AP to switch off and check if STA connection can be successful
bool apSwithOffEnabled;             // AP could be deactivated on timeout to try STA mode
uint8_t wifiReconnectCtr;           // number of trials still have

const uint16_t apActiveTimeoutBeforeSTATrial = 60 * 1000;
const uint8_t delaySecsBtwReconnectAttempts[] = { 5, 10 };// , 20, 20, 30, 60};
const uint8_t reconnectTrials = sizeof (delaySecsBtwReconnectAttempts);


/*************************** Motor ***************************/
const uint8_t pinHomeSensor = GPIO_ID_PIN (0);
const uint8_t pinEncoder = GPIO_ID_PIN (12);
const uint8_t pinMotorCtrlPos = GPIO_ID_PIN (4);
const uint8_t pinMotorCtrlNeg = GPIO_ID_PIN (5);

std::unique_ptr<LinearUnitWithEncoderAndHomeSensor> door;

/*************************** Alexa ***************************/
fauxmoESP alexa;

void returnOK()
{
    server.send (200, "text/plain", "OK");
}
void returnFail (String msg)
{
    server.send (500, "text/plain", msg + "\r\n");
}
void handleRoot()
{
    auto htmlConfigPage = F ("\
	<!DOCTYPE html>\
	<html>\
	<head>\
	</head>\
	<body>\
	<H1>Device Configuration<br></H1>\
	<H2>Please configure your device named : Nyuszivezerlo v1.0</H2>\
	<form method = 'post'>\
	<table cols = '2' width = '10em'>\
	<tr><td align = 'right'>SSID:</td><td><input name = 'ssid' type = 'text' value = 'ssid'/></td></tr>\
	<tr><td align = 'right'>Password:</td><td><input name = 'passwd' type = 'password'/></td></tr>\
	</table>\
	<input name = 'submit' type = 'submit'/>\
	</form>\
	</body>\
	</html>\
	");

    server.send (200, "text/html", htmlConfigPage);
}
void saveCredentials (const String &ssid, const String &pw)
{
    bool b;
    EEPROM.begin (128);
    String cred = ssid + '\t' + pw;
    EEPROM.put (sizeof (gWifiConfiguredMarker), cred);
    //b = EEPROM.commit();
    //if (!b)
    //{
    //    Serial.println (F ("Could not write credentials to EEPROM"));
    //}
    EEPROM.put (0, gWifiConfiguredMarker);
    b = EEPROM.commit();
    if (!b)
    {
        Serial.println (F ("Could not write magic string to EEPROM"));
    }
    EEPROM.end();   // +commit
    Serial.print (F ("Saved wifi credentials: "));
    Serial.println (cred);
}
void handleRootPost()
{
    if (server.args() < 2)
    {
        return returnFail ("BAD ARGS");
    }
    String ssid = server.arg (0);
    String pw = server.arg (1);
    Serial.print ("Wifi config received: ");
    Serial.print (ssid);
    Serial.print (":");
    Serial.println (pw);

    returnOK();

    wifi_passwd = pw;
    wifi_ssid = ssid;
    ghaveWifiCredentials = true;
    saveCredentials (ssid, pw);
    //Serial.println ("Rebooting...");

    //ESP.restart();
    //ESP.reset();
    WiFi.disconnect (true);
    delay (50);
    wifiStarted = false;
    apActive = false;
}
void handleNotFound()
{
    String message = "File Not Found\n\n";
    message += "URI: ";
    message += server.uri();
    //message += "\nMethod: ";
    //message += (server.method() == HTTP_GET) ? "GET" : "POST";
    //message += "\nArguments: ";
    //message += server.args();
    //message += "\n";
    //for (uint8_t i = 0; i<server.args(); i++) {
    //  message += " " + server.argName(i) + ": " + server.arg(i) + "\n";
    //}
    server.send (404, "text/plain", message);
}
// gets called when door stops moving
void onMotionStopped (float pos, int32_t error)
{
    String nyulajtoState = pos < 10 ? mqttBoolFalse : mqttBoolTrue;
    if (mqttConnected)
    {
        mqtt.publish (topicNyulketrecStat, nyulajtoState, 2, 1);
    }

    Serial.print (F ("Door movement has finished. Pos: "));
    Serial.print (pos);
    Serial.print (" state: ");
    Serial.println (nyulajtoState);
}
void onMQTTDisconnect()
{
    mqttConnected = false;
    Serial.println (F ("MQTT Disconnected"));
}
void onMQTTConnect()
{
    Serial.println (F ("MQTT: Connected"));
    mqttConnected = true;
    mqtt.subscribe (topicNyulketrecCtl, 2);
    mqtt.subscribe (topicTestMoveTo, 0);
    mqtt.subscribe (topicTestFreeRun, 0);
}
void onMQTTData (String topic, String data, bool cont)
{
    Serial.printf ("Topic: %s, data: %s\r\n", topic.c_str(), data.c_str());
    if (topic == topicNyulketrecCtl)
    {
        Serial.print (F ("Door should move: "));
        Serial.println (data);

        auto nyulajtoState = data == mqttBoolTrue ? true : false;
        auto pos = door->getPos();
        if (pos < 10 && nyulajtoState)
        {
            // open door
            door->moveTo (door->getEndPos());
        }
        else if (pos > 10 && !nyulajtoState)
        {
            // close
            door->moveTo (0);
        }
    }
    else if (topic == topicTestMoveTo)
    {
        float pos = static_cast<float> (atof (data.c_str()));
        Serial.print (F ("Motor was triggered to move to "));
        Serial.println (pos);
        door->moveTo (pos);
    }
    else if (topic == topicTestFreeRun)
    {
        int i = atoi (data.c_str());
        door->freeRun (i != 0, i > 0, abs (i));
        Serial.print (F ("Motor was triggered to run free or stop: "));
        Serial.println (i);
    }
}

void wifiEventCb (WiFiEvent_t e)
{
    bool apChanged = false;
    bool stopServer = false;

    Serial.print ("WiFi event: ");
    switch (e)
    {

        case WIFI_EVENT_STAMODE_CONNECTED:
            Serial.println ("EV:STA Connected");
            mqtt.begin ("mqtt://broker.hivemq.com:1883#Dolu5639_2");
            //configTime (2 * 3600, 0, "pool.ntp.org", "time.nist.gov");
            break;
        case WIFI_EVENT_STAMODE_DISCONNECTED:
            Serial.println ("EV:STA Disconnected");
            if (mqttConnected)
            {
                onMQTTDisconnect();
            }
            wifiSTAStoppedmsec = millis();
            apSwithOffEnabled = ghaveWifiCredentials;
            break;
        case WIFI_EVENT_SOFTAPMODE_STACONNECTED:
            Serial.println ("EV:AP Connected");
            apChanged = true;
            apSwithOffEnabled = false;
            break;
        case WIFI_EVENT_SOFTAPMODE_STADISCONNECTED:
            Serial.println ("EV:AP Disconnected");
            apChanged = true;
            break;
        case WIFI_EVENT_SOFTAPMODE_PROBEREQRECVED:
            Serial.println ("EV:AP Probed");
            break;

        default:
            Serial.print ("EV:Unhandled: ");
            Serial.println (static_cast<uint16_t> (e));
            break;
    }

    if (apChanged)
    {
        MDNS.notifyAPChange();
        MDNS.update();
    }

    if (stopServer)
    {
        server.close();
        Serial.println ("Server stopped");
    }
}
void setup()
{
    char tmpBuf[128];
    auto eeLen = sizeof (tmpBuf); //EEPROM.length();

    PIN_FUNC_SELECT (PERIPHS_IO_MUX_MTDI_U, FUNC_GPIO12);
    pinMode (12, FUNCTION_3);
    WiFi.persistent (false);

    //Serial.begin(74880);
    Serial.begin (115200);
    delay (10);
    Serial.println ("Booting...");

    /* read wifi access credentials */
    EEPROM.begin (eeLen);
    EEPROM.get (0, tmpBuf);
    EEPROM.end();

    if (tmpBuf[sizeof (gWifiConfiguredMarker)] != 0)
    {
        tmpBuf[sizeof (gWifiConfiguredMarker)] = 0;
    }

    Serial.print ("Marker: ");
    Serial.println (tmpBuf);

    if (0 == strcmp (tmpBuf, gWifiConfiguredMarker))
    {
        // wifi configured
        String m (tmpBuf + sizeof (gWifiConfiguredMarker));
        Serial.print (F ("WiFi config: "));
        Serial.println (m);
        auto sep = m.indexOf ('\t');

        wifi_ssid = m.substring (0, sep);
        wifi_passwd = "";
        if (sep >= 0)
        {
            // there is passwd
            wifi_passwd = m.substring (sep + 1);
        }
        Serial.printf ("Using wifi network: %s (%s)\n", wifi_ssid.c_str(), wifi_passwd.c_str());
        ghaveWifiCredentials = true;
    }
    else
    {
        ghaveWifiCredentials = false;
    }

    apActive = !ghaveWifiCredentials;
    apSwithOffEnabled = ghaveWifiCredentials;
    wifiStarted = false;
    wifiPending = false;
    wifiSTAStoppedmsec = std::numeric_limits<uint64_t>::max();  // force start wifi
    WiFi.onEvent (wifiEventCb);

    server.on ("/", HTTP_GET, handleRoot);
    server.on ("/", HTTP_POST, handleRootPost);
    server.onNotFound (handleNotFound);

    mqttConnected = false;
    mqtt.onConnect (&onMQTTConnect);
    //mqtt.onDisconnect (&onMQTTDisconnect);
    mqtt.onData (&onMQTTData);

    MDNS.begin (host);
    MDNS.addService ("http", "tcp", 80);

    server.begin();

    LinearConfig units;
    units.pulsePerRevolution = 1;
    units.distanceMmPerRevolution = 0.7f; // M4 screw
    units.decelerationDistanceMm = 10;   // start deceleration before the end
    units.totalLinearLengthMm = 400;   // limit
    units.motorMinSpeedPwm = 600;
    door.reset (new LinearUnitWithEncoderAndHomeSensor (pinMotorCtrlPos, pinMotorCtrlNeg, pinEncoder, pinHomeSensor)); // create motor instance
    attachInterrupt (digitalPinToInterrupt (pinEncoder), []()
    {
        door->onPulse();
    }, FALLING);
    attachInterrupt (digitalPinToInterrupt (pinHomeSensor), []()
    {
        door->onSensor (0, digitalRead (pinHomeSensor));
    }, FALLING);

    door->setStoppedHandler (onMotionStopped);
    door->init (units); // init

    alexa.addDevice ("Freedom");
    alexa.onSetState ([] (unsigned char device_id, const char *device_name, bool state)
    {
        Serial.printf ("[MAIN] Device #%d (%s) state: %s\n", device_id, device_name, state ? "ON" : "OFF");
        onMQTTData (topicNyulketrecCtl, state ? "1" : "0", false);
    });
    alexa.onGetState ([] (unsigned char device_id, const char *device_name)
    {
        return door->getPos() >= (door->getEndPos() - 2) ; // return if door is open
    });
    alexa.enable (true);
}

void loop()
{
    auto ms = millis();
    if (wifiStarted)
    {
        // connection should be present and handle if not
        // if AP mode is actual, check for go to STA
        if (WiFi.getMode() == WIFI_AP)
        {
            if (apSwithOffEnabled && (ms > apActiveTimeout))
            {
                // time has come to switch to STA mode
                apActive = false;
                WiFi.disconnect (true); // disconnect
                delay (333);
                Serial.println ("AP mode switched off");
                wifiStarted = false;
                wifiPending = false;
            }
        }
    }
    else
    {
        if (wifiPending)
        {
            bool failed = false;
            bool done = true;

            // already started and should have result in some secs
            auto stat = WiFi.status();
            if (stat == WL_CONNECTED)
            {
                Serial.println (F ("Pending sais CONNECTED"));
            }
            else if (stat == WL_CONNECT_FAILED || stat == WL_CONNECTION_LOST || stat == WL_NO_SSID_AVAIL)
            {
                Serial.print (F ("Connection error at trial "));
                Serial.println (wifiReconnectCtr);
                failed = true;
            }
            //else if (stat == WL_DISCONNECTED)
            //{
            //    Serial.println (F ("Pending Disconnected..."));
            //    done = false;
            //}
            else
            {
                // check for timeout 15secs
                auto elapsedms = (ms - wifiPendingTimeout);
                if (elapsedms > 1000 * 15)
                {
                    Serial.println (F ("Pending Timeout..."));
                    failed = true;
                }
                else
                {
                    done = false;   // not ready, keep waiting
                    if (0 == elapsedms % 1000)
                    {
                        Serial.print (static_cast<uint16_t> (stat));
                        Serial.print (".");
                    }
                }
            }

            if (failed)
            {
                if (wifiReconnectCtr < reconnectTrials)
                {
                    // next trial...
                    delay (1000 * delaySecsBtwReconnectAttempts[wifiReconnectCtr]);
                    ++wifiReconnectCtr;
                    Serial.println (F ("Try again..."));
                }
                else
                {
                    Serial.println (F ("No more trial, start AP"));
                    // out of trials, lets start AP
                    apActive = true;
                }

                wifiStarted = false;
                wifiPending = false;
                done = false;
            }

            if (done)
            {
                // success
                wifiStarted = true;
                wifiPending = false;
                wifiReconnectCtr = 0;
                Serial.println (F ("Connection...Success"));
            }
        }   //pending
        else
        {
            // need to start wifi, either AP or STA
            if (apActive)
            {
                // start AP mode
                // init AP with webserver for configuration
                WiFi.mode (WIFI_AP);
                String ssid = "Nyuszi_";
                ssid.concat (ESP.getChipId());
                Serial.print (F ("Starting AP name "));
                Serial.println (ssid);

                // start AP
                WiFi.softAP (ssid.c_str());
                apActiveTimeout = apActiveTimeoutBeforeSTATrial + millis(); // AP should be switched off latest that time
                wifiStarted = true; // shortcut to open state
            }
            else
            {
                // station mode to be started with MQTT client
                wifiPendingTimeout = millis();
                wifiPending = true;
                WiFi.mode (WIFI_STA);
                Serial.println (F ("STA mode Connecting..."));
                WiFi.begin (wifi_ssid.c_str(), wifi_passwd.length() ? wifi_passwd.c_str() : nullptr);
            }
        }
    }
    door->doTasks();
    server.handleClient();
    mqtt.handle();
    alexa.handle();
}
