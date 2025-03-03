/*
 * WiFi.cpp
 * Copyright (C) 2019-2020 Linar Yusupov
 *
 * Bug fixes and improvements
 * Copyright (C) 2020 Manuel Roesel
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "SoC.h"

#if defined(EXCLUDE_WIFI)
void WiFi_setup()
{}
void WiFi_loop()
{}
void WiFi_fini()
{}
#else

#include <FS.h>
#include <TimeLib.h>
#include <WiFiMulti.h>

#include "OTA.h"
#include "GNSS.h"
#include "EEPROM.h"
#include "OLED.h"
#include "Time.h"
#include "WiFi.h"
#include "Traffic.h"
#include "RF.h"
#include "Web.h"
//#include "NMEA.h"
#include "Battery.h"

#include "config.h"
#include "global.h"
#include "logos.h"
#ifdef TBEAM
#include "Platform_ESP32.h"
#endif

WiFiMulti *wifiMulti;
String host_name = HOSTNAME;

IPAddress local_IP(192, 168, 1, 1);
IPAddress gateway(192, 168, 1, 1);
IPAddress subnet(255, 255, 255, 0);

/**
 * Default WiFi connection information.
 *
 */

String station_ssid = "ognbase";
String station_psk  = "123456789";

const char* ap_default_psk = "987654321"; ///< Default PSK.

#if defined(USE_DNS_SERVER)
#include <DNSServer.h>

const byte DNS_PORT = 53;
DNSServer  dnsServer;
bool       dns_active = false;
#endif

// A UDP instance to let us send and receive packets over UDP
WiFiUDP Uni_Udp;

unsigned int RFlocalPort = RELAY_SRC_PORT;      // local port to listen for UDP packets

char UDPpacketBuffer[256]; // buffer to hold incoming and outgoing packets

#if defined(POWER_SAVING_WIFI_TIMEOUT)
static time_t WiFi_No_Clients_Time = 0;
#endif

size_t Raw_Receive_UDP(uint8_t* buf)
{
    int noBytes = Uni_Udp.parsePacket();
    if (noBytes)
    {
        if (noBytes > MAX_PKT_SIZE)
            noBytes = MAX_PKT_SIZE;

        // We've received a packet, read the data from it
        Uni_Udp.read(buf, noBytes); // read the packet into the buffer

        return (size_t) noBytes;
    }
    else
        return 0;
}

void Raw_Transmit_UDP()
{
    size_t rx_size = RF_Payload_Size(ogn_protocol_1);
    rx_size = rx_size > sizeof(fo.raw) ? sizeof(fo.raw) : rx_size;
    String str = Bin2Hex(fo.raw, rx_size);
    size_t len = str.length();
    // ASSERT(sizeof(UDPpacketBuffer) > 2 * PKT_SIZE + 1)
    str.toCharArray(UDPpacketBuffer, sizeof(UDPpacketBuffer));
    UDPpacketBuffer[len] = '\n';
    //SoC->WiFi_transmit_UDP(RELAY_DST_PORT, (byte *)UDPpacketBuffer, len + 1);
}

/**
 * @brief Arduino setup function.
 */
void WiFi_setup()
{
    char buf[32];
    static bool configged = false;
    //static bool reconnecting = false;

      if (WiFi.getMode() != WIFI_STA)
      {
          WiFi.mode(WIFI_STA);
          delay(500);
      }

    if (! configged) {
      configged = OGN_read_config();
      Serial.println(F("...Read configuration"));   /* ensure compiler does not skip the OGN_read_config() */
    }

    if (configged) {

        Serial.println(F("Using WiFi config..."));

        WiFi.mode(WIFI_OFF);
        delay(1000);

        //if (reconnecting)
        //    delete wifiMulti;
        wifiMulti = new WiFiMulti();
        OLED_draw_Bitmap(85, 20, 1, true);
        for (int i=0; i < 5; i++) {
            if (ogn_ssid[i] != "") {
                snprintf(buf, sizeof(buf), "%d: %s", i, ogn_ssid[i].c_str());
                OLED_write(buf, 0, i * 9, false);
                Serial.println(buf);
                if (i == 0 || ogn_ssid[i] != "xxxxxxx")
                  wifiMulti->addAP(ogn_ssid[i].c_str(), ogn_wpass[i].c_str());
            }
        }
        // Check connection
        for (int n = 0; n < 20; n++) { /* retry */
            if (wifiMulti->run() == WL_CONNECTED) {
                // ... print IP Address
                Serial.print("Connect "); Serial.print(WiFi.SSID());
                Serial.print(" at "); Serial.println(WiFi.localIP());
                snprintf(buf, sizeof(buf), "Connect %s",  WiFi.SSID().c_str());
                OLED_write(buf, 0, 45, false);
                snprintf(buf, sizeof(buf), " at %s",
                WiFi.localIP().toString().c_str());
                OLED_write(buf, 0, 54, false);
                host_name += String((SoC->getChipId() & 0xFFFFFF), HEX);
                SoC->WiFi_hostname(host_name);

                // Print hostname.
                Serial.println("Hostname: " + host_name);
                break;
            }
            Serial.write('.');
            Serial.print(WiFi.status());
            delay(500);
        }
        if (wifiMulti->run() != WL_CONNECTED) {
            Serial.println();
            Serial.println(F("Can not connect to WiFi station"));
            snprintf(buf, sizeof(buf), "failed..");
            OLED_write(buf, 0, 45, false);
            //DebugLogWrite("WiFi_setup: cannot connect");
        }

    } else {  /* not configged */

        station_ssid = MY_ACCESSPOINT_SSID;
        station_psk  = MY_ACCESSPOINT_PSK;
        WiFi.begin();
        delay(1000);
    }

    if (WiFi.status() != WL_CONNECTED)
    {
        //if (reconnecting)
        //    return;      // do not go into AP mode if temporarily disconnected

        host_name += String((SoC->getChipId() & 0xFFFFFF), HEX);
        SoC->WiFi_hostname(host_name);

        // Print hostname.
        Serial.println("Hostname: " + host_name);
        Serial.println(F("Wait for WiFi connection."));

        snprintf(buf, sizeof(buf), "Starting AP");
        OLED_write(buf, 0, 54, false);

        WiFi.mode(WIFI_AP);
        SoC->WiFi_setOutputPower(WIFI_TX_POWER_MED); // 10 dB
        // WiFi.setOutputPower(0); // 0 dB
        //system_phy_set_max_tpw(4 * 0); // 0 dB
        delay(10);

        Serial.print(F("Setting soft-AP configuration ... "));
        Serial.println(WiFi.softAPConfig(local_IP, gateway, subnet) ?
                       F("Ready") : F("Failed!"));

        Serial.print(F("Setting soft-AP ... "));
        Serial.println(WiFi.softAP(host_name.c_str(), ap_default_psk) ?
                       F("Ready") : F("Failed!"));
        Serial.print(F("IP address: "));
        Serial.println(WiFi.softAPIP());
    }

    //reconnecting = true;

    Uni_Udp.begin(RFlocalPort);
    Serial.print(F("UDP server has started at port: "));
    Serial.println(RFlocalPort);

#if defined(POWER_SAVING_WIFI_TIMEOUT)
    WiFi_No_Clients_Time = 0;   /* until our clock is stable */
#endif
}

bool Wifi_connected()
{
    if (WiFi.status() != WL_CONNECTED)
        return false;
    else
        return true;
}

void WiFi_loop()
{

#if defined(USE_DNS_SERVER)
    if (dns_active)
        dnsServer.processNextRequest();

#endif

#ifdef TBEAM
    if (WiFi.getMode() == WIFI_STA) {
      static time_t prevtime = 0;
      if (ThisAircraft.timestamp != prevtime) {
        prevtime = ThisAircraft.timestamp;
        if (prevtime & 0x01)
          turn_LED_on();
        else
          turn_LED_off();     /* blink blue LED at 0.5 Hz if time is advancing */
      }
    }
#endif

#if defined(POWER_SAVING_WIFI_TIMEOUT)
    if ((settings->power_save & POWER_SAVE_WIFI) && WiFi.getMode() == WIFI_AP)
    {
        if (SoC->WiFi_clients_count() == 0) {

          if (ThisAircraft.timestamp != 0 && WiFi_No_Clients_Time != 0
            && (ThisAircraft.timestamp > WiFi_No_Clients_Time + POWER_SAVING_WIFI_TIMEOUT)) {

#ifdef TBEAM
              turn_LED_off();   /* turn off bright blue LED to save power and signal end of WiFi */
              OLED_disable();
#endif
              if (settings->nmea_p)
                  StdOut.println(F("$PSRFS,WIFI_OFF"));
              //NMEA_fini();
              Web_fini();
              WiFi_fini();
              Serial.println(F("[cpp] shutting down WiFI & LED..."));
          }

        } else if (ThisAircraft.timestamp != 0 && ThisAircraft.timestamp > WiFi_No_Clients_Time) {

            WiFi_No_Clients_Time = ThisAircraft.timestamp;
            // Serial.print(F("postponing WiFi timeout..."));

        }
    }
#endif
}

void WiFi_fini()
{
    Uni_Udp.stop();

    WiFi.mode(WIFI_OFF);
    delete wifiMulti;
}

#endif /* EXCLUDE_WIFI */
