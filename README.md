# MQTT Indoor Climate
## Overview
In this project you will see how to build a MQTT client on base of an ESP8266 to measure high accurate indoor climate data. The features for my needs are to measure typical (indoor) climate data like temperature, humidity, pressure and air quality with high accuracy.

## Hardware design
### Schematic
![Schematic](/hardware/Schematic.png)

### ESP8266 controller board
My descision fell on the `Wemos D1 mini` board.
![WemosD1mini](/hardware/WemosD1mini.png)

### Sensor
My descision was to take the Bosch BME680 all-in-one sensor, which incorporates a temperature, humidity, pressure and gas sensor in one housing. The sensor is connected via I²C interface. The Bosch Sensortec Environmental Cluster (BSEC) Software library is used to talk with the BME680. This library has been conceptualized to provide higher-level signal processing and fusion for the BME680. The library receives compensated sensor values from the sensor API. It processes the BME680 signals to provide the requested sensor outputs.
![BME680](/hardware/BME680.png)

## SW-update
SW-update is done via USB cable.

## SW installation and SW build
Following steps need to be done first:
1. install Arduino IDE 1.8.1x
2. download and install the ESP8266 board supporting libraries with this URL: http://arduino.esp8266.com/stable/package_esp8266com_index.json
3. select the `Lolin(Wemos) D1 R2 & mini` board
4. install the `BSEC Software` library
5. modify the platform.txt file, see how at https://github.com/BoschSensortec/BSEC-Arduino-library/blob/master/README.md
6. install the `Async MQTT client` library: https://github.com/marvinroger/async-mqtt-client/archive/master.zip
7. install the `Async TCP` library: https://github.com/me-no-dev/ESPAsyncTCP/archive/master.zip
8. config (see next chapter), compile and flash

## SW configuration
The configuration is completely done in the web frontend of the WifiManager. At first startup, the software boots up in access point mode. In this mode you can configure parameters like
* Wifi SSID
* Wifi password
* MQTT broker IP address
* MQTT user
* MQTT password
* your altitude (above sea level) in meters
* your temperature compensation value in degree Celsius

## SW normal operation
The BSEC library collects every 3 seconds new sensor data. The software publishes averaged values every 10 minutes to 5 MQTT topics: temperature [°C], humidity [rel. %], pressure [hPa], air quality [0..500] and air quality accuracy [0..3]. Also the software supports re-connection to Wifi and to the MQTT broker in case of power loss, Wifi loss or MQTT broker unavailability. The MQTT topics begin with the device specific MAC-address string (in the following "A020A600F73A" as an example). This is useful when having multiple controllers in your MQTT cloud to avoid collisions.

Publish topics:
* Topic: "/A020A600F73A/temperature"            Payload example: "22.5"
* Topic: "/A020A600F73A/humidity"               Payload example: "43.2"
* Topic: "/A020A600F73A/pressure"               Payload example: "1023.7"
* Topic: "/A020A600F73A/staticiaq"              Payload example: "145.3"
* Topic: "/A020A600F73A/staticiaqaccuracy"      Payload example: "3"

## Builtin LED
In case of no MQTT connection the blue builtin LED is permanently on. In case of severe errors of the bme680- or BSEC driver the builtin LED begins to flash for 3 minutes and the watchdog will finally reset the ESP8266. During normal operation without errors the builtin LED is off.

## Correction of pressure value
As the sensor is calibrated at sea level (0m) it will show a very low pressure value at your location, which is way below the real pressure at your location and which would falsely indicate something like a hurricane. This value needs to be corrected with the international barometric formula (see https://en.wikipedia.org/wiki/Pressure_altitude). This is done in function `seaLevelForAltitude()` which has the parameters for actual outdoor temperature, the athmospheric pressure (the raw sensor pressure value) and the altitude at your location.
