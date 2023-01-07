#include <Arduino.h>
#include <WiFi.h>

void initWifi() {
    const char* ssid           = "ESP32-Access-Point";                // SSID Name
    const char* password       = "12345678";   // SSID Password - Set to NULL to have an open AP
    const int   channel        = 10;                        // WiFi Channel number between 1 and 13
    const bool  hide_SSID      = false;                     // To disable SSID broadcast -> SSID will not appear in a basic WiFi scan
    const int   max_connection = 3;  

    Serial.println("\n[*] Creating AP");
    WiFi.mode(WIFI_AP);
    WiFi.softAP(ssid, password, channel, hide_SSID, max_connection);
    Serial.print("[+] AP Created with IP Gateway ");
    Serial.println(WiFi.softAPIP());
}