#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <WiFiManager.h>
#include <Ticker.h>
#include <AsyncMqttClient.h>
#include <EEPROM.h>
#include "bsec.h"

// eeprom
#define MQTT_IP_OFFSET         0
#define MQTT_IP_LENGTH        16
#define MQTT_USER_OFFSET      16
#define MQTT_USER_LENGTH      32
#define MQTT_PASSWORD_OFFSET  48
#define MQTT_PASSWORD_LENGTH  32
#define ALTITUDE_OFFSET       80
#define ALTITUDE_LENGTH        4
#define TEMP_OFFSET_OFFSET    84
#define TEMP_OFFSET_LENGTH     4
#define BSEC_STATE_OFFSET     88
#define BSEC_STATE_SIZE       (BSEC_MAX_STATE_BLOB_SIZE + 1)   // 140

// pins
#define BUTTON_GPIO  14

// access point
#define AP_NAME "MqttIndoorClimate"
#define AP_TIMEOUT 300
#define MQTT_PORT 1883

// other
#define PUBLISH_CYCLE_SEC   600ul
#define STATE_SAVE_PERIOD   86400ul  // 24 hours

// bsec config
const uint8_t bsec_config_iaq[] = {
#include "config/generic_33v_3s_4d/bsec_iaq.txt"
};

// topics
char topic_temperature[40] = "/";
char topic_humidity[40] = "/";
char topic_pressure[40] = "/";
char topic_staticiaq[40] = "/";
char topic_staticiaqaccuracy[40] = "/";

// channel values
float altitude = 0.0F;
float temp_offset = 0.0F;
float temperature = 0.0F;
float humidity = 0.0F;
float pressure = 0.0F;
float staticiaq = 0.0F;
uint8_t staticiaqaccuracy = 0;
uint32_t cycle = 0ul;
uint32_t oldtime = 0ul;

// bsec
Bsec iaqSensor;
uint8_t bsecState[BSEC_MAX_STATE_BLOB_SIZE] = {0};
uint32_t time_now;
uint16_t stateUpdateCounter = 0;

// mqtt
IPAddress mqtt_server;
AsyncMqttClient mqttClient;
Ticker mqttReconnectTimer;

char mqtt_ip_pre[MQTT_IP_LENGTH] = "";
char mqtt_user_pre[MQTT_USER_LENGTH] = "";
char mqtt_password_pre[MQTT_PASSWORD_LENGTH] = "";
char altitude_str_pre[10] = "";
char temp_offset_str_pre[10] = "";

char mqtt_ip[MQTT_IP_LENGTH] = "";
char mqtt_user[MQTT_USER_LENGTH] = "";
char mqtt_password[MQTT_PASSWORD_LENGTH] = "";
char altitude_str[10] = "";
char temp_offset_str[10] = "";

boolean mqtt_connected = false;

char temperature_str[10];
char humidity_str[10];
char pressure_str[10];
char staticIaq_str[10];
char staticIaqAccuracy_str[2];

// wifi
WiFiManager wifiManager;
WiFiEventHandler wifiConnectHandler;
WiFiEventHandler wifiDisconnectHandler;
Ticker wifiReconnectTimer;
char mac_str[13];

// watchdog
Ticker secondTick;
volatile uint8_t watchdogCount = 0;

void ISRwatchdog(void)
{
  watchdogCount++;
  if (watchdogCount > 60)
  {
     ESP.restart();
  }
}

float readEEPROMfloat(unsigned int addr)
{
	union
	{
		byte b[4];
		float f;
	} data;
	for(int i = 0; i < 4; i++)
	{
		data.b[i] = EEPROM.read(addr+i);
	}
	return data.f;
}

void writeEEPROMfloat(unsigned int addr, float x)
{
	union
	{
		byte b[4];
		float f;
	} data;
	data.f = x;
	for(int i = 0; i < 4; i++)
	{
		EEPROM.write(addr+i, data.b[i]);
	}
}

String readEEPROM(int offset, int len)
{
  String res = "";
  for (int i = 0; i < len; ++i)
  {
    res += char(EEPROM.read(i + offset));
  }
  return res;
}

void writeEEPROM(int offset, int len, String value)
{
  for (int i = 0; i < len; ++i)
  {
    if (i < value.length())
    {
      EEPROM.write(i + offset, value[i]);
    }
    else
    {
      EEPROM.write(i + offset, 0x00);
    }
  }
}

void connectToWifi()
{
  Serial.println("Re-Connecting to Wi-Fi...");
  WiFi.setSleepMode(WIFI_NONE_SLEEP);
  WiFi.mode(WIFI_STA);
  WiFi.begin();
}

void onWifiConnect(const WiFiEventStationModeGotIP& event)
{
  Serial.println("Connected to Wi-Fi.");
  connectToMqtt();
}

void onWifiDisconnect(const WiFiEventStationModeDisconnected& event)
{
  Serial.println("Disconnected from Wi-Fi.");
  mqttReconnectTimer.detach(); // ensure we don't reconnect to MQTT while reconnecting to Wi-Fi
  wifiReconnectTimer.once(2, connectToWifi);
}

void connectToMqtt()
{
  Serial.println("Connecting to MQTT...");
  mqttClient.connect();
}

void onMqttConnect(bool sessionPresent)
{
  Serial.println("Connected to MQTT.");
  mqtt_connected = true;
  digitalWrite(LED_BUILTIN, HIGH);
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason)
{
  digitalWrite(LED_BUILTIN, LOW);
  mqtt_connected = false;
  Serial.println("Disconnected from MQTT.");

  if (WiFi.isConnected())
  {
    mqttReconnectTimer.once(2, connectToMqtt);
  }
}

float seaLevelForAltitude(float altitude, float atmospheric, float temp)
{
  return atmospheric / pow(1.0F - ((0.0065F * altitude) / (273.15F + temp)), 5.255F);
}

void setup(void)
{
  uint8_t mac[6];

  // init UART
  Serial.begin(115200);
  Serial.println();

  // LED
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  
  // init EEPROM
  EEPROM.begin(256);

  // init button
  pinMode(BUTTON_GPIO, INPUT);

  // check if button is pressed
  if (LOW == digitalRead(BUTTON_GPIO))
  {
    Serial.println("reset wifi settings and restart.");
    wifiManager.resetSettings();
    delay(1000);
    ESP.restart();
  }

  // init WIFI
  readEEPROM(MQTT_IP_OFFSET, MQTT_IP_LENGTH).toCharArray(mqtt_ip_pre, MQTT_IP_LENGTH);
  readEEPROM(MQTT_USER_OFFSET, MQTT_USER_LENGTH).toCharArray(mqtt_user_pre, MQTT_USER_LENGTH);
  readEEPROM(MQTT_PASSWORD_OFFSET, MQTT_PASSWORD_LENGTH).toCharArray(mqtt_password_pre, MQTT_PASSWORD_LENGTH);
  altitude = readEEPROMfloat(ALTITUDE_OFFSET);
  strcpy(altitude_str_pre, String(altitude).c_str());
  temp_offset = readEEPROMfloat(TEMP_OFFSET_OFFSET);
  strcpy(temp_offset_str_pre, String(temp_offset).c_str());

  WiFiManagerParameter custom_mqtt_ip("ip", "MQTT ip", mqtt_ip_pre, MQTT_IP_LENGTH);
  WiFiManagerParameter custom_mqtt_user("user", "MQTT user", mqtt_user_pre, MQTT_USER_LENGTH);
  WiFiManagerParameter custom_mqtt_password("password", "MQTT password", mqtt_password_pre, MQTT_PASSWORD_LENGTH, "type=\"password\"");
  WiFiManagerParameter custom_mqtt_altitude("altitude", "Altitude [m]", altitude_str_pre, 10);
  WiFiManagerParameter custom_mqtt_tempoffset("tempoffset", "Temp. Offset [°C]", temp_offset_str_pre, 10);
  
  wifiManager.addParameter(&custom_mqtt_ip);
  wifiManager.addParameter(&custom_mqtt_user);
  wifiManager.addParameter(&custom_mqtt_password);
  wifiManager.addParameter(&custom_mqtt_altitude);
  wifiManager.addParameter(&custom_mqtt_tempoffset);

  WiFi.setSleepMode(WIFI_NONE_SLEEP);
  wifiManager.setConfigPortalTimeout(AP_TIMEOUT);
  wifiManager.setAPStaticIPConfig(IPAddress(192,168,1,1), IPAddress(192,168,1,1), IPAddress(255,255,255,0));
  if (!wifiManager.autoConnect(AP_NAME))
  {
    Serial.println("failed to connect and restart.");
    delay(1000);
    // restart and try again
    ESP.restart();
  }
 
  strcpy(mqtt_ip, custom_mqtt_ip.getValue());
  strcpy(mqtt_user, custom_mqtt_user.getValue());
  strcpy(mqtt_password, custom_mqtt_password.getValue());
  strcpy(altitude_str, custom_mqtt_altitude.getValue());
  strcpy(temp_offset_str, custom_mqtt_tempoffset.getValue());

  if ((0 != strcmp(mqtt_ip, mqtt_ip_pre)) || 
      (0 != strcmp(mqtt_user, mqtt_user_pre)) || 
      (0 != strcmp(mqtt_password, mqtt_password_pre)) || 
      (0 != strcmp(altitude_str, altitude_str_pre)) || 
      (0 != strcmp(temp_offset_str, temp_offset_str_pre)))
  {
    Serial.println("Parameters changed, need to update EEPROM.");
    writeEEPROM(MQTT_IP_OFFSET, MQTT_IP_LENGTH, mqtt_ip);
    writeEEPROM(MQTT_USER_OFFSET, MQTT_USER_LENGTH, mqtt_user);
    writeEEPROM(MQTT_PASSWORD_OFFSET, MQTT_PASSWORD_LENGTH, mqtt_password);
    altitude = String(altitude_str).toFloat();
    writeEEPROMfloat(ALTITUDE_OFFSET, altitude);
    temp_offset = String(temp_offset_str).toFloat();
    writeEEPROMfloat(TEMP_OFFSET_OFFSET, temp_offset);

    EEPROM.commit();
  }

  wifiConnectHandler = WiFi.onStationModeGotIP(onWifiConnect);
  wifiDisconnectHandler = WiFi.onStationModeDisconnected(onWifiDisconnect);

  // construct MQTT topics with MAC
  WiFi.macAddress(mac);
  sprintf(mac_str, "%02x%02x%02x%02x%02x%02x", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

  Serial.printf("MAC: %s\n", mac_str);

  strcat(topic_temperature, mac_str);
  strcat(topic_temperature, "/temperature");
  strcat(topic_humidity, mac_str);
  strcat(topic_humidity, "/humidity");
  strcat(topic_pressure, mac_str);
  strcat(topic_pressure, "/pressure");
  strcat(topic_staticiaq, mac_str);
  strcat(topic_staticiaq, "/staticiaq");
  strcat(topic_staticiaqaccuracy, mac_str);
  strcat(topic_staticiaqaccuracy, "/staticiaqaccuracy");
  
  if (mqtt_server.fromString(mqtt_ip))
  {
    char mqtt_id[40] = AP_NAME;

    strcat(mqtt_id, "-");
    strcat(mqtt_id, mac_str);
    mqttClient.onConnect(onMqttConnect);
    mqttClient.onDisconnect(onMqttDisconnect);
    mqttClient.setServer(mqtt_server, MQTT_PORT);
    mqttClient.setCredentials(mqtt_user, mqtt_password);
    mqttClient.setClientId(mqtt_id);

    connectToMqtt();
  }
  else
  {
    Serial.println("invalid MQTT Broker IP.");
  }

  Wire.begin();

  // set address
  iaqSensor.begin(BME680_I2C_ADDR_SECONDARY, Wire);
  // set offset
  iaqSensor.setTemperatureOffset(temp_offset);

  // useless info
  Serial.println("\nBSEC library version " + String(iaqSensor.version.major) + "." + String(iaqSensor.version.minor) + "." + String(iaqSensor.version.major_bugfix) + "." + String(iaqSensor.version.minor_bugfix));

  checkIaqSensorStatus();

  // set proprietary BOSCH config
  iaqSensor.setConfig(bsec_config_iaq);
  checkIaqSensorStatus();

  // check EEPROM
  loadState();

  // subscribe to data from sensor
  bsec_virtual_sensor_t sensorList[10] =
  {
    BSEC_OUTPUT_RAW_TEMPERATURE,
    BSEC_OUTPUT_RAW_PRESSURE,
    BSEC_OUTPUT_RAW_HUMIDITY,
    BSEC_OUTPUT_RAW_GAS,
    BSEC_OUTPUT_IAQ,
    BSEC_OUTPUT_STATIC_IAQ,
    BSEC_OUTPUT_CO2_EQUIVALENT,
    BSEC_OUTPUT_BREATH_VOC_EQUIVALENT,
    BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE,
    BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY,
  };

  iaqSensor.updateSubscription(sensorList, 10, BSEC_SAMPLE_RATE_LP);
  checkIaqSensorStatus();

  // watchdog
  secondTick.attach(3, ISRwatchdog);
  oldtime = millis();
}

void loop(void)
{
  if (mqtt_connected == true)
  {
	  // If new data is available
    if (iaqSensor.run())
    {
      cycle ++;
      updateState();
      checkIaqSensorStatus();

      temperature += iaqSensor.temperature;
      pressure += seaLevelForAltitude(altitude, (iaqSensor.pressure / 100.0F), temperature);
      humidity += iaqSensor.humidity;
      staticiaq += iaqSensor.staticIaq;
      staticiaqaccuracy = iaqSensor.staticIaqAccuracy;

      if ((millis() - oldtime) > (PUBLISH_CYCLE_SEC * 1000))
      {
        temperature /= ((float)cycle);
        pressure /= ((float)cycle);
        humidity /= ((float)cycle);
        staticiaq /= ((float)cycle);

        snprintf(temperature_str, 10, "%.1f", temperature);
        snprintf(pressure_str, 10, "%.1f", pressure);
        snprintf(humidity_str, 10, "%.1f", humidity);
        snprintf(staticIaq_str, 10, "%.1f", staticiaq);

        Serial.printf("Temperature: %.1f°C Humidity: %.1f%% Pressure: %.1fhPa IAQ: %.1f IAQaccuracy: %d\n", temperature, humidity, pressure, staticiaq, staticiaqaccuracy);

        mqttClient.publish(topic_temperature, 0, true, temperature_str);
        mqttClient.publish(topic_humidity, 0, true, humidity_str);
        mqttClient.publish(topic_pressure, 0, true, pressure_str);
        mqttClient.publish(topic_staticiaq, 0, true, staticIaq_str);

        oldtime = millis();
        cycle = 0ul;
        temperature = 0.0F;
        pressure = 0.0F;
        humidity = 0.0F;
        staticiaq = 0.0F;
      }

      snprintf(staticIaqAccuracy_str, 2, "%d", staticiaqaccuracy);
      mqttClient.publish(topic_staticiaqaccuracy, 0, true, staticIaqAccuracy_str);

      // trigger watchdog
      watchdogCount = 0;
    }
  }
}

void checkIaqSensorStatus(void)
{
  if (iaqSensor.status != BSEC_OK)
  {
    if (iaqSensor.status < BSEC_OK)
    {
      Serial.println("BSEC error code : " + String(iaqSensor.status));
      for (;;)
        errLeds(); /* Halt in case of failure */
    }
    else
    {
      Serial.println("BSEC warning code : " + String(iaqSensor.status));
    }
  }

  if (iaqSensor.bme680Status != BME680_OK)
  {
    if (iaqSensor.bme680Status < BME680_OK)
    {
      Serial.println("BME680 error code : " + String(iaqSensor.bme680Status));
      for (;;)
        errLeds(); /* Halt in case of failure */
    }
    else
    {
      Serial.println("BME680 warning code : " + String(iaqSensor.bme680Status));
    }
  }
}

void errLeds(void)
{
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(100);
  digitalWrite(LED_BUILTIN, LOW);
  delay(100);
}

void loadState(void)
{
  if (EEPROM.read(BSEC_STATE_OFFSET) == BSEC_MAX_STATE_BLOB_SIZE)
  {
    // Existing state in EEPROM
    Serial.println("Reading state from EEPROM");

    for (uint8_t i = 0; i < BSEC_MAX_STATE_BLOB_SIZE; i++)
    {
      bsecState[i] = EEPROM.read(BSEC_STATE_OFFSET + i + 1);
    }

    iaqSensor.setState(bsecState);
    checkIaqSensorStatus();
  }
  else
  {
    // Erase the EEPROM with zeroes
    Serial.println("Erasing EEPROM");

    for (uint8_t i = 0; i < BSEC_STATE_SIZE; i++)
    {
      EEPROM.write(BSEC_STATE_OFFSET + i, 0);
    }

    EEPROM.commit();
  }
}

void updateState(void)
{
  bool update = false;

  if (stateUpdateCounter == 0) {
    /* First state update when IAQ accuracy is >= 3 */
    if (iaqSensor.staticIaqAccuracy >= 3)
    {
      time_now = millis();
      update = true;
      stateUpdateCounter++;
    }
  }
  else
  {
    /* Update every STATE_SAVE_PERIOD minutes */
    if ((millis() - time_now) > STATE_SAVE_PERIOD)
    {
      time_now = millis();
      update = true;
      stateUpdateCounter++;
    }
  }

  if (update == true)
  {
    iaqSensor.getState(bsecState);
    checkIaqSensorStatus();

    Serial.println("Writing state to EEPROM");

    for (uint8_t i = 0; i < BSEC_MAX_STATE_BLOB_SIZE ; i++)
    {
      EEPROM.write(BSEC_STATE_OFFSET + i + 1, bsecState[i]);
    }

    EEPROM.write(BSEC_STATE_OFFSET, BSEC_MAX_STATE_BLOB_SIZE);
    EEPROM.commit();
  }
}
