// #############################################
/*
 * This file is part of the HA_smarkant project
 *
 * (c) 2021 Leon Schneider, www.leon-schneider.de
 *
 * Based on the Smarkant project from Dirk Grappendorf
 * https://github.com/grappendorf/smarkant
 */
#ifndef _CONFIG_H
#define _CONFIG_H

char* WLAN_SSID = "***";
char* WLAN_PASSPHRASE = "***";
const IPAddress local_IP(192, 168, 1, 10);
const IPAddress gateway(192, 168, 1, 1);
const IPAddress subnet(255, 255, 255, 0);
const IPAddress primaryDNS(192, 168, 1, 1);
const char* MQTT_SERVER = "192.168.1.5";
const int MQTT_PORT = 1883;
const char* MQTT_USER = "***";
const char* MQTT_PASS = "***";

#endif
