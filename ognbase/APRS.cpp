/*
   OGN.cpp
   Copyright (C) 2020 Manuel Rösel

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "APRS.h"
#include "Time.h"
#include "PVALID.h"
#include "SoftRF.h"
#include "Battery.h"
#include "Traffic.h"
#include "RF.h"
#include "EEPROM.h"
#include "WiFi.h"
#include "global.h"
#include "Log.h"
#include "OLED.h"
#include "global.h"


#include <TimeLib.h>

#define kKey 0x73e2

int MIN_SPEED = 0;

int  aprs_registred   = 0;
bool aprs_connected   = false;
int  last_packet_time = 0; // seconds
int  ap_uptime        = 0;

char ogn_server_name_buf[12];
const char *ogn_server_name = "GLIDERN0";
bool got_ogn_server_name = false;

#define seconds() (millis() / 1000)

// const unsigned long seventyYears = 2208988800UL;

static String zeroPadding(String data, int len)
{
    if (data.charAt(2) == '.')
        data.remove(len, data.length());

    if (data.charAt(1) == '.')
        data.remove(len - 1, data.length());
    String tmp = "";
    for (int i = data.length(); i < len; i++)
        tmp += "0";
    tmp += data;
    return tmp;
}

static String getWW(String data)
{
    String tmp = data;
    int    len = tmp.length();
    tmp.remove(0, len - 1);
    return tmp;
}

static float SnrCalc(float rssi)
{
    float noise = -108.0;
    return rssi - (noise);
}

static short AprsPasscode(const char* theCall)
{
    char  rootCall[10];
    char* p1 = rootCall;

    while ((*theCall != '-') && (*theCall != 0) && (p1 < rootCall + 9)) {
        *p1++ = toupper(*theCall++);
    }

    *p1 = 0;

    short hash = kKey;
    short i    = 0;
    short len  = strlen(rootCall);
    char* ptr  = rootCall;

    while (i < len) {
        hash ^= (*ptr++) << 8;
        hash ^= (*ptr++);
        i    += 2;
    }

    return hash & 0x7fff;
}

static bool OGN_APRS_Connect(void)
{
    if (SoC->WiFi_connect_TCP(ogn_server.c_str(), ogn_port))
        return true;
    else
        return false;
}

static bool OGN_APRS_DisConnect(void)
{
    if (!SoC->WiFi_disconnect_TCP())
        return true;
    return false;
}

static int OGN_APRS_check_reg(char* cp) // 0 = unverified // 1 = verified // -1 = wrong message
{
    String msg = cp;   // convert char array to String

    if (msg.indexOf("verified") > -1)
        return 1;
    if (msg.indexOf("unverified") > -1)
        return 0;
    if (msg.indexOf("invalid") > -1)
        return 2;

    return -1;
}

bool OGN_APRS_check_Wifi(void)
{
    if (WiFi.status() != WL_CONNECTED && WiFi.getMode() == WIFI_STA)
    {
        WiFi.disconnect();
        WiFi.mode(WIFI_OFF);
        WiFi.mode(WIFI_STA);
        //WiFi.begin(ogn_ssid_1.c_str(), ogn_wpass_1.c_str());
        delay(100);
    }
    if (WiFi.status() == WL_CONNECTED)
        return true;
    else if (! ognrelay_enable)
    {
        Serial.println("10 minutes in AP mode - resetting");
        SoC->reset();
    }
    return false;
}

int OGN_APRS_check_messages(void)
{
    static char RXbuffer[512];  // was malloc()ed

    int    recStatus = SoC->WiFi_receive_TCP(RXbuffer, 512);
    String msg;

    if (recStatus > 0)
    {
        last_packet_time = seconds();
        if (recStatus < 0)    recStatus = 0; 
        if (recStatus > 511)  recStatus = 511; 
        RXbuffer[recStatus] = '\0';
        if (recStatus > 0) {
            Serial.println("");
            Serial.print(RXbuffer);
        }
        int reg = OGN_APRS_check_reg(RXbuffer);
        if (reg == 1)
            msg = "APRS login successful";
        else if (reg == 0)
            msg = "APRS login unsuccessful";
        else if (reg == 2)
            msg = "APRS login invalid";
        else
            msg = "";
        if (reg >= 0) {
          Serial.println(msg.c_str());
          Serial.println("");
          Logger_send_udp(&msg);
        }

      if (reg == 1 && recStatus > 41) {
        if (strncmp(RXbuffer,"# logresp",9) == 0) {
Serial.println("getting server name from logresp...");
            char *p;
            if (p = strstr(RXbuffer,"server GLIDERN")) {
                if (strlen(p) > 14) {
                    p += 7;
                    p[9] = '\0';
                    (void) strcpy(ogn_server_name_buf, p);
                    //ogn_server_name = ogn_server_name_buf;  - uncomment when we know how to get non-GLIDERN0 packets through
                    got_ogn_server_name = true;
//Serial.print("server name from logresp: ");
//Serial.println(ogn_server_name);
                }
            }
        }
      } else if (reg < 0 && recStatus > 57) {   /* not a registration message */
        if (strncmp(RXbuffer,"# aprsc",7) == 0) {
            char *p;
            if (p = strstr(RXbuffer,"GMT GLIDERN")) {
                if (strlen(p) > 11) {
                    p += 4;
                    if (p[8] == ' ')
                        p[8] = '\0';
                    else
                        p[9] = '\0';
                    (void) strcpy(ogn_server_name_buf, p);
                    //ogn_server_name = ogn_server_name_buf;  - uncomment when we know how to get non-GLIDERN0 packets through
                    got_ogn_server_name = true;
//Serial.print("server name from aprsc: ");
//Serial.println(ogn_server_name);
                }
            }
        }
      }

    }

    if (seconds() - last_packet_time > 60)
    {
        msg = "no packet since > 60 seconds...reconnecting";
        Logger_send_udp(&msg);
        SoC->WiFi_disconnect_TCP();
        aprs_registred = 0;
    }

    // free(RXbuffer);
    return aprs_registred;
}

static const char* symbol_table[16] =
    {"/", "/", "\\", "/", "\\", "\\", "/", "/",
     "\\", "J", "/", "/", "M", "/", "\\", "\\"};
    // 0x79 -> aircraft type 1110 dec 14 & 0x51 -> aircraft type 4
static const char* symbol[16] =
    {"z", "^", "^", "X", " ", "^", "g", "g", "^", "^", "^", "O", "^", "\'", " ", "n"};

void OGN_APRS_Export(void)
{
    struct aprs_airc_packet APRS_AIRC;

//    if (OurTime == 0)  /* no GNSS time available yet */
//      return;

    for (int i = 0; i < MAX_TRACKING_OBJECTS; i++) {

        if (Container[i].addr && Container[i].waiting) {

            if ((ThisAircraft.timestamp - Container[i].timestamp) > EXPORT_EXPIRATION_TIME) {
                Container[i].waiting = false;
                // Container[i].timereported = OurTime;
                Serial.println("... skipping expired packet");
                continue;
            }

            if ((Container[i].stealth && !ogn_istealthbit) || (Container[i].no_track && !ogn_itrackbit)) {
                Container[i].waiting = false;
                Container[i].timereported = OurTime;
                Serial.println("... skipping stealth packet");
                continue;
            }

            uint16_t distance = (uint16_t) (Container[i].distance * 0.001);

//Serial.printf("export? [%d] t=%d d=%d\r\n", i, Container[i].timestamp, distance);

            if (distance > ogn_range) {
                Container[i].waiting = false;
                Container[i].timereported = OurTime;
                Serial.println("... skipping too-far packet");
                continue;
            }

            if (!isPacketValid(&Container[i])) {
                // Container[i].waiting = false;
                // Container[i].timereported = OurTime;
                Serial.println("... skipping invalid packet");
                continue;
            }

            if (distance > largest_range)  largest_range = distance;

            float LAT = fabs(Container[i].latitude);
            float LON = fabs(Container[i].longitude);

            APRS_AIRC.callsign = zeroPadding(String(Container[i].addr, HEX), 6);
            APRS_AIRC.callsign.toUpperCase();
            APRS_AIRC.rec_callsign = ogn_callsign;

            // TBD need to use Container[i].timestamp instead of hour(), minute(), second()
            // actually, need to make sure Container[i].timestamp is based on SlotTime not current time due slot-2 time extension
            //converting Container[i].timestamp to hour minutes seconds

            /* seconds since 1970 for APRS */
            // time_t APRStime = Container[i].timestamp; // - seventyYears;
            APRS_AIRC.timestamp = zeroPadding(String(Container[i].hour), 2)
                                + zeroPadding(String(Container[i].minute), 2)
                                + zeroPadding(String(Container[i].second), 2) + "h";

            /*Latitude*/
            APRS_AIRC.lat_deg = String(int(LAT));
            APRS_AIRC.lat_min = zeroPadding(String((LAT - int(LAT)) * 60, 3), 5);

            /*Longitude*/
            APRS_AIRC.lon_deg = zeroPadding(String(int(LON)), 3);
            APRS_AIRC.lon_min = zeroPadding(String((LON - int(LON)) * 60, 3), 5);

            /*Altitude*/
            APRS_AIRC.alt = zeroPadding(String(int(Container[i].altitude * 3.28084)), 6);

            APRS_AIRC.heading      = zeroPadding(String(int(Container[i].course)), 3);
            APRS_AIRC.ground_speed = zeroPadding(String(int(Container[i].speed)), 3);


            APRS_AIRC.sender_details = zeroPadding(String((Container[i].aircraft_type << 2)
                                                     | (Container[i].stealth << 7)
                                                     | (Container[i].no_track << 6)
                                                     | (Container[i].addr_type), HEX), 2);

            APRS_AIRC.symbol_table = String(symbol_table[Container[i].aircraft_type]);
            APRS_AIRC.symbol       = String(symbol[Container[i].aircraft_type]);

            APRS_AIRC.snr = String(SnrCalc(Container[i].rssi), 1);


            String W_lat = String((LAT - int(LAT)) * 60, 3);
            String W_lon = String((LON - int(LON)) * 60, 3);


            APRS_AIRC.pos_precision = getWW(W_lat) + getWW(W_lon);

            if (Container[i].vs >= 0)
                APRS_AIRC.climbrate = "+" + zeroPadding(String(int(Container[i].vs)), 3);
            else
                APRS_AIRC.climbrate = zeroPadding(String(int(Container[i].vs)), 3);

            String AircraftPacket;

            if (Container[i].addr_type == 1)
                AircraftPacket = "ICA";
            else if (Container[i].addr_type == 2)
                AircraftPacket = "FLR";
            else if (Container[i].addr_type == 3)
                AircraftPacket = "OGN";
            else if (Container[i].addr_type == 4)
                AircraftPacket = "P3I";
            else if (Container[i].addr_type == 5)
                AircraftPacket = "FNT";
            else // (Container[i].addr_type == 0 || Container[i].addr_type > 5)
                AircraftPacket = "RANDOM";

            AircraftPacket += APRS_AIRC.callsign;
            APRS_AIRC.sender_details.toUpperCase();

            AircraftPacket += ">OGFLR,qAS,";
            AircraftPacket += APRS_AIRC.rec_callsign;
            AircraftPacket += ":/";
            AircraftPacket += APRS_AIRC.timestamp;
            AircraftPacket += APRS_AIRC.lat_deg;
            AircraftPacket += APRS_AIRC.lat_min;
            if (Container[i].latitude < 0)
                AircraftPacket += "S";
            else
                AircraftPacket += "N";
            AircraftPacket += APRS_AIRC.symbol_table;
            AircraftPacket += APRS_AIRC.lon_deg;
            AircraftPacket += APRS_AIRC.lon_min;
            if (Container[i].longitude < 0)
                AircraftPacket += "W";
            else
                AircraftPacket += "E";
            AircraftPacket += APRS_AIRC.symbol;
            AircraftPacket += APRS_AIRC.heading;
            AircraftPacket += "/";
            AircraftPacket += APRS_AIRC.ground_speed;
            AircraftPacket += "/A=";
            AircraftPacket += APRS_AIRC.alt;
            AircraftPacket += " !W";
            AircraftPacket += APRS_AIRC.pos_precision;
            AircraftPacket += "! id";
            AircraftPacket += APRS_AIRC.sender_details;
            AircraftPacket += APRS_AIRC.callsign;
            AircraftPacket += " ";
            AircraftPacket += APRS_AIRC.climbrate;
            AircraftPacket += "fpm +0.0rot ";
            AircraftPacket += APRS_AIRC.snr;
            AircraftPacket += "dB 0e -0.0kHz";
            AircraftPacket += "\r\n";

            Serial.println("");
            Serial.println(AircraftPacket.c_str());
            Serial.println("");

            Logger_send_udp(&AircraftPacket);
            Logger_send_udp(&APRS_AIRC.pos_precision);

            SoC->WiFi_transmit_TCP(AircraftPacket);

            Container[i].waiting = false;
            Container[i].timereported = OurTime;
            ++traffic_packets_reported;
        }
    }
}

int OGN_APRS_Register(ufo_t* this_aircraft)
{
//    Serial.println("OGN_APRS_Registering...");

//    if (OurTime == 0)  /* no GNSS time available yet */
//      return aprs_registred;

    if (OGN_APRS_Connect())
    {

        struct aprs_login_packet APRS_LOGIN;

        APRS_LOGIN.user    = String(this_aircraft->addr, HEX);
        APRS_LOGIN.pass    = String(AprsPasscode(APRS_LOGIN.user.c_str()));
        APRS_LOGIN.appname = "OGNbase";
        APRS_LOGIN.version = SOFTRF_FIRMWARE_VERSION;

        String LoginPacket = "user ";
        LoginPacket += APRS_LOGIN.user;
        LoginPacket += " pass ";
        LoginPacket += APRS_LOGIN.pass;
        LoginPacket += " vers ESP32-";
        LoginPacket += APRS_LOGIN.appname;
        LoginPacket += " ";
        LoginPacket += APRS_LOGIN.version;
        LoginPacket += " filter m/";
        LoginPacket += String(ogn_range);
        LoginPacket += "\n";

        Serial.println("");
        Serial.println(LoginPacket.c_str());
        Serial.println("");
        Logger_send_udp(&LoginPacket);

        SoC->WiFi_transmit_TCP(LoginPacket);

        aprs_registred = 1;
        return 1;
    }

    else
    {
        Serial.println("");
        Serial.println("OGN connection failed");
        Serial.println("");
        aprs_registred = 0;
        return -1;
    }

}

bool OGN_APRS_Location(ufo_t* this_aircraft)
{
    if (aprs_registred != 1 || ! got_ogn_server_name)
        return false;

        /* RUSSIA>APRS,TCPIP*,qAC,GLIDERN2:/220757h626.56NI09353.92E&/A=000446 */

        struct  aprs_reg_packet APRS_REG;
        float   LAT = fabs(this_aircraft->latitude);
        float   LON = fabs(this_aircraft->longitude);
        // time_t  APRStime = OurTime; // - seventyYears;

        APRS_REG.origin   = ogn_callsign;
        APRS_REG.callsign = String(ogn_server_name);
        //APRS_REG.callsign.toUpperCase();
        APRS_REG.alt       = zeroPadding(String(int(this_aircraft->altitude * 3.28084)), 6);
        if (OurTime == 0) {
            APRS_REG.timestamp = "000000h";
        } else {
            APRS_REG.timestamp = zeroPadding(String(this_aircraft->hour), 2)
                                 + zeroPadding(String(this_aircraft->minute), 2)
                                 + zeroPadding(String(this_aircraft->second), 2) + "h";
        }
        APRS_REG.lat_deg = zeroPadding(String(int(LAT)), 2);
        APRS_REG.lat_min = zeroPadding(String((LAT - int(LAT)) * 60, 3), 5);

        APRS_REG.lon_deg = zeroPadding(String(int(LON)), 3);
        APRS_REG.lon_min = zeroPadding(String((LON - int(LON)) * 60, 3), 5);

        String RegisterPacket = "";
        RegisterPacket += APRS_REG.origin;
        RegisterPacket += ">OGNSDR,TCPIP*,qAC,";
        RegisterPacket += APRS_REG.callsign;
        RegisterPacket += ":/";
        RegisterPacket += APRS_REG.timestamp;
        RegisterPacket += APRS_REG.lat_deg + APRS_REG.lat_min;

        if (this_aircraft->latitude < 0)
            RegisterPacket += "S";
        else
            RegisterPacket += "N";
        RegisterPacket += "I";
        RegisterPacket += APRS_REG.lon_deg + APRS_REG.lon_min;
        if (this_aircraft->longitude < 0)
            RegisterPacket += "W";
        else
            RegisterPacket += "E";
        RegisterPacket += "&/A=";
        RegisterPacket += APRS_REG.alt;
        RegisterPacket += "\r\n";

        Serial.println("");
        Serial.println(RegisterPacket.c_str());
        Serial.println("");
        Logger_send_udp(&RegisterPacket);

        SoC->WiFi_transmit_TCP(RegisterPacket);

        return true;
}

void OGN_APRS_KeepAlive(void)
{
    String KeepAlivePacket = "#keepalive\n";
    Serial.println(KeepAlivePacket.c_str());
    Logger_send_udp(&KeepAlivePacket);
    SoC->WiFi_transmit_TCP(KeepAlivePacket);
}

// LKHS>APRS,TCPIP*,qAC,GLIDERN2:>211635h v0.2.6.ARM CPU:0.2 RAM:777.7/972.2MB NTP:3.1ms/-3.8ppm 4.902V 0.583A +33.6C

bool OGN_APRS_Status(ufo_t* this_aircraft)
{
    if (OurTime == 0) {      /* no GNSS time available yet */
      // return false;
      Serial.println("sending status packet without time stamp");
    }

    // time_t APRStime = OurTime; // - seventyYears;
    struct aprs_stat_packet APRS_STAT;
    APRS_STAT.origin   = ogn_callsign;
    if (! got_ogn_server_name)
        return false;
    APRS_STAT.callsign = String(ogn_server_name);
    //APRS_STAT.callsign.toUpperCase();
    if (OurTime == 0) {
        APRS_STAT.timestamp = "000000h";
    } else {
        APRS_STAT.timestamp = zeroPadding(String(this_aircraft->hour), 2)
                            + zeroPadding(String(this_aircraft->minute), 2)
                            + zeroPadding(String(this_aircraft->second), 2) + "h";
    }

    /*issue17*/ /*v0.1.0.20-ESP32*/
    APRS_STAT.platform      = "v";
    APRS_STAT.platform      += SOFTRF_FIRMWARE_VERSION;
    APRS_STAT.platform      += "-ESP32-OGNbase";

    //APRS_STAT.realtime_clock = String(0.0);

    // standalone station reports local voltage
    if (!ognrelay_time)
        remote_voltage = Battery_voltage();
    // else remote_voltage is already holding voltage reported from remote station
    int v = (int)(10.0 * remote_voltage + 0.5);
    APRS_STAT.board_voltage  = String(v/10) + "." + String(v%10) + "V";
    /* remote_voltage may be wrong if time not synched yet */

    String StatusPacket = APRS_STAT.origin;
    StatusPacket += ">OGNSDR,TCPIP*,qAC,";
    StatusPacket += APRS_STAT.callsign;
    StatusPacket += ":>";
    StatusPacket += APRS_STAT.timestamp;
    StatusPacket += " ";
    StatusPacket += APRS_STAT.platform;
    StatusPacket += " ";
    StatusPacket += APRS_STAT.board_voltage;
    // added packets per minute and IDs seen in last hour
    StatusPacket += " ";
    StatusPacket += String(packets_per_minute) + "/min";
    StatusPacket += " ";
    StatusPacket += String(numvisible) + "/" + String(numseen_1hr) + "Acfts[1h]";
    StatusPacket += " ";
    if (ognrelay_time) {
        StatusPacket += String(remote_sats);
        StatusPacket += "sat ";
        StatusPacket += time_synched? "time_synched " : "time_not_synched ";
#if defined(TBEAM)
    } else if (ogn_gnsstime) {
        StatusPacket += String(gnss.satellites.value());
        StatusPacket += "sat ";
#endif
    }
    StatusPacket += String(ognrelay_time ? remote_uptime : uptime);
    StatusPacket += "_m_uptime\r\n";
    Serial.println("");
    Serial.println(StatusPacket.c_str());
    Serial.println("");
    Logger_send_udp(&StatusPacket);
    SoC->WiFi_transmit_TCP(StatusPacket);
    return true;
}
