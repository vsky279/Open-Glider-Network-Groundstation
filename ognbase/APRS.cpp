/*
   APRS.cpp
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
#include "SoC.h"
#include "global.h"

#include <TimeLib.h>

#define kKey 0x73e2

int MIN_SPEED = 0;

int  aprs_registred   = 0;
bool aprs_connected   = false;
bool wifi_reconnected = false;
uint32_t wifi_outage  = 0;
int  last_packet_time = 0; // seconds
int  ap_uptime        = 0;

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
    if (WiFi.getMode() != WIFI_STA)
        return false;
    if (WiFi.status() != WL_CONNECTED) {
        WiFi.disconnect();
        delay(150);
        //WiFi.mode(WIFI_OFF);
        //WiFi.mode(WIFI_STA);
        //WiFi.begin(ogn_ssid_1.c_str(), ogn_wpass_1.c_str());
        WiFi.reconnect();
        delay(350);
        if (WiFi.status() == WL_CONNECTED) {
            wifi_reconnected = true;
            Serial.println("WiFi reconnected");
        }
    }
    if (WiFi.status() == WL_CONNECTED) {
        return true;
    }

    //Serial.println(F("APRS_check_Wifi: not connected"));
    //DebugLogWrite("APRS_check_Wifi: not connected");
    if (wifi_outage == 0)
        wifi_outage = millis();
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

/* original OGNbase code, not correct
static const char* symbol_table[16] =
    {"/", "/", "\\", "/", "\\", "\\", "/", "/",
     "\\", "J", "/", "/", "M", "/", "\\", "\\"};
    // 0x79 -> aircraft type 1110 dec 14 & 0x51 -> aircraft type 4
static const char* symbol[16] =
    {"z", "^", "^", "X", " ", "^", "g", "g", "^", "^", "^", "O", "^", "\'", " ", "n"};
*/

/*
From http://wiki.glidernet.org/wiki:ogn-flavoured-aprs
  "/z",  //  0 = ?
  "/'",  //  1 = (moto-)glider    (most frequent)
  "/'",  //  2 = tow plane        (often)
  "/X",  //  3 = helicopter       (often)
  "/g" , //  4 = parachute        (rare but seen - often mixed with drop plane)
  "\\^", //  5 = drop plane       (seen)
  "/g" , //  6 = hang-glider      (rare but seen)  <<< Angel's code had "/_"
  "/g" , //  7 = para-glider      (rare but seen)
  "\\^", //  8 = powered aircraft (often)
  "/^",  //  9 = jet aircraft     (rare but seen)
  "/z",  //  A = UFO              (people set for fun)
  "/O",  //  B = balloon          (seen once)
  "/O",  //  C = airship          (seen once)
  "/'",  //  D = UAV              (drones, can become very common)
  "/z",  //  E = ground support   (ground vehicles at airfields)
  "\\n"  //  F = static object    (ground relay ?)
*/

static const char* symbol_table[16] =
    {"/", "/", "/", "/",   "/", "\\", "/", "/",
     "\\", "/", "/", "/",  "/", "/", "/", "\\"};
static const char* symbol[16] =
    {"z", "'", "'", "X",   "g", "^", "_", "g",
     "^", "^", "z", "O",   "O", "'", "z", "n"};

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
                Serial.println(F("... skipping expired packet"));
                continue;
            }

            // if ((Container[i].stealth && !ogn_istealthbit) || (Container[i].no_track && !ogn_itrackbit)) {
            if (Container[i].no_track && !ogn_itrackbit) {
                Container[i].waiting = false;
                Container[i].timereported = OurTime;
                Serial.println(F("... skipping no-track packet"));
                continue;
            }

            uint16_t distance = (uint16_t) (Container[i].distance * 0.001);

//Serial.printf("export? [%d] t=%d d=%d\r\n", i, Container[i].timestamp, distance);

            if (distance > ogn_range) {
                Container[i].waiting = false;
                Container[i].timereported = OurTime;
                Serial.println(F("... skipping too-far packet"));
                continue;
            }

            if (!isPacketValid(&Container[i])) {
                // Container[i].waiting = false;
                // Container[i].timereported = OurTime;
                Serial.println(F("... skipping invalid packet"));
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
            uint8_t addr_type = Container[i].addr_type;

            bool relayed = (Container[i].protocol == RF_PROTOCOL_LEGACY && addr_type > 3);
              // assume these are messages relayed by SoftRF

            if (relayed) {
                // restore the original address type
                if (addr_type == 4) {
                    AircraftPacket = "ICA";             // was ICAO
                    addr_type = 1;
                } else if (addr_type == 7) {
                    AircraftPacket = "FLR";             // was FLARM
                    addr_type = 2;
                } else if (addr_type == 6) {
                    AircraftPacket = "OGN";             // was anonymous
                    addr_type = 3;
                } else {
                    AircraftPacket = "RND";
                    addr_type = 0;
                }
            } else /* not relayed */ {
                if (addr_type == 1)
                    AircraftPacket = "ICA";
                else if (addr_type == 2)
                    AircraftPacket = "FLR";
                else if (addr_type == 3)
                    AircraftPacket = "OGN";
                else if (addr_type == 4)
                    AircraftPacket = "P3I";
                else if (addr_type == 5)
                    AircraftPacket = "FNT";
                else {
                    AircraftPacket = "RND";
                    addr_type = 0;
                }
            }

            APRS_AIRC.sender_details = zeroPadding(String((Container[i].aircraft_type << 2)
                                                     | (Container[i].stealth << 7)
                                                     | (Container[i].no_track << 6)
                                                     | (addr_type & 0x3), HEX), 2);
            APRS_AIRC.sender_details.toUpperCase();

            AircraftPacket += APRS_AIRC.callsign;
            // AircraftPacket += ">OGFLR,qOR:/";
            if (relayed) {
                AircraftPacket += ">OGFLR,qAS,relayed:/";
                // disassociate this packet from this station, so
                // as not to mess up the station range measurement
            } else {
                AircraftPacket += ">OGFLR,qOR:/";
            }
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
            // AircraftPacket += "fpm +0.0rot ";
            AircraftPacket += "fpm ";
            if (relayed)
              AircraftPacket += "99.0";   // marks as relayed, when SNR is meaningless anyway
            else
              AircraftPacket += APRS_AIRC.snr;
            // AircraftPacket += "dB 0e -0.0kHz";
            // AircraftPacket += "\r\n";
            //if (relayed)
            //  AircraftPacket += "dB relayed\r\n";
            //else
            AircraftPacket += "dB\r\n";

            Serial.println("");
            Serial.println(AircraftPacket.c_str());
            Serial.println("");

            Logger_send_udp(&AircraftPacket);
            Logger_send_udp(&APRS_AIRC.pos_precision);

            if (SoC->WiFi_transmit_TCP(AircraftPacket) == 0)  // <<<
                Serial.println(F("transmit traffic to TCP failed"));

            Container[i].waiting = false;
            Container[i].timereported = OurTime;
            ++traffic_packets_reported;

            yield();
        }
    }
}

int OGN_APRS_Register(ufo_t* this_aircraft)
{
//    Serial.println(F("OGN_APRS_Registering..."));

//    if (OurTime == 0)  /* no GNSS time available yet */
//      return aprs_registred;

    if (OGN_APRS_Connect())
    {

        struct aprs_login_packet APRS_LOGIN;

        //APRS_LOGIN.user  = String(this_aircraft->addr, HEX);
        APRS_LOGIN.user    = ogn_callsign;
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
        LoginPacket += " filter m/20\r\n";
//      LoginPacket += " filter m/";
//      LoginPacket += String(ogn_range);     // no need to receive data from other stations
//      LoginPacket += "\r\n";

        Serial.println("");
        Serial.println(LoginPacket.c_str());
        Serial.println("");
        Logger_send_udp(&LoginPacket);

        if (SoC->WiFi_transmit_TCP(LoginPacket)) {   // <<<
            aprs_registred = 1;
            return 1;
        }
        Serial.println("");
        Serial.println(F("transmit to OGN failed"));    // <<<
        Serial.println("");
        aprs_registred = 0;
        return -1;
    }

    else
    {
        Serial.println("");
        Serial.println(F("OGN connection failed"));
        Serial.println("");
        aprs_registred = 0;
        return -1;
    }

}


/* RUSSIA>APRS,TCPIP*,qAC,GLIDERN2:/220757h626.56NI09353.92E&/A=000446 - wrong format */
// correct format: K2B9>OGNSXR:/220757h626.56NI09353.92E&/A=000446

bool OGN_APRS_Location(ufo_t* this_aircraft)
{
    if (aprs_registred != 1)
        return false;

    struct  aprs_reg_packet APRS_REG;
    float   LAT = fabs(this_aircraft->latitude);
    float   LON = fabs(this_aircraft->longitude);
    // time_t  APRStime = OurTime; // - seventyYears;

    APRS_REG.origin   = ogn_callsign;
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
    RegisterPacket += ">";
    RegisterPacket += TOCALL;
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
    Serial.print(RegisterPacket.c_str());
    Serial.println("");
    Logger_send_udp(&RegisterPacket);

    if(SoC->WiFi_transmit_TCP(RegisterPacket))  // <<<
       return true;

    Serial.println(F("transmit location to TCP failed"));
    return false;
}

void OGN_APRS_KeepAlive(void)
{
    String KeepAlivePacket = "#keepalive\r\n";
    Serial.println(KeepAlivePacket.c_str());
    Logger_send_udp(&KeepAlivePacket);
    if (SoC->WiFi_transmit_TCP(KeepAlivePacket) == 0)
        Serial.println(F("transmit keepalive to TCP failed"));
}

// LKHS>APRS,TCPIP*,qAC,GLIDERN2:>211635h v0.2.6.ARM CPU:0.2 RAM:777.7/972.2MB NTP:3.1ms/-3.8ppm 4.902V 0.583A +33.6C
// LKHS>OGNSXR,TCPIP*,qAC,GLIDERN0:>211635h vMB101-ESP32-OGNbase 3.9V 55/min 2/2Acfts[1h] 8sat time_synched 120_m_r_uptime

// correct format:
// K2B9>OGNSXR:>183602h vMB101-ESP32-OGNbase 3.8V 55/min 2/3Acfts[1h] 10sat time_synched 180_m_r_uptime

bool OGN_APRS_Status(ufo_t* this_aircraft, bool first_time, String resetReason)
{
    //char buf[32];
    //buf[0] = '\0';

    // time_t APRStime = OurTime; // - seventyYears;
    struct aprs_stat_packet APRS_STAT;
    APRS_STAT.origin   = ogn_callsign;
    if (OurTime == 0) {
        APRS_STAT.timestamp = "000000h";
    } else {
        APRS_STAT.timestamp = zeroPadding(String(this_aircraft->hour), 2)
                            + zeroPadding(String(this_aircraft->minute), 2)
                            + zeroPadding(String(this_aircraft->second), 2) + "h";
    }

    /*issue17*/ /*v0.1.0.20-ESP32*/
    APRS_STAT.platform = "v";
    APRS_STAT.platform += SOFTRF_FIRMWARE_VERSION;
#if defined(ESP32)
    APRS_STAT.platform += "-ESP32";
#endif
//  APRS_STAT.platform += rf_chip->name;
//   - not useful, should report the RF chip in REMOTE station.
//     - but no bits left in radio packet to send that info.
    APRS_STAT.platform += "-OGNbase";

    //APRS_STAT.realtime_clock = String(0.0);

    String StatusPacket = APRS_STAT.origin;
    StatusPacket += ">";
    StatusPacket += TOCALL;
    StatusPacket += ":>";
    StatusPacket += APRS_STAT.timestamp;
    StatusPacket += " ";
    StatusPacket += APRS_STAT.platform;
    if ((! ognrelay_time) || time_synched) {
        int v;
        if (time_synched) {
            v = (int)(10.0 * remote_voltage + 0.5);
        } else {
#if defined(TBEAM)
            if (on_ext_power())   // USB powered
                v = 0;
            else
#endif
            v = (int)(10.0 * Battery_voltage() + 0.5);
            // standalone station reports local voltage
        }
        if (v > 20)
            StatusPacket += " " + String(v/10) + "." + String(v%10) + "V";
        if (sleep_length == 0 && remote_sleep_length == 0) {
            StatusPacket += " ";
            // added packets per minute and IDs seen in last hour
            StatusPacket += String(packets_per_minute) + "/min ";
            StatusPacket += String(numvisible) + "/" + String(numseen_1hr) + "Acfts[1h]";
        }
    }
    StatusPacket += " ";
    if (ognrelay_time) {
        if (time_synched) {
            StatusPacket += String(remote_sats);
            StatusPacket += "sat time_synched ";
        } else {
            StatusPacket += "time_not_synched ";
        }
#if defined(TBEAM)
    } else if (ogn_gnsstime) {
        StatusPacket += String(gnss.satellites.value());
        StatusPacket += "sat ";
#endif
    }
    if (first_time) {
        StatusPacket += "reset_reason_";
        StatusPacket += resetReason;
        StatusPacket += "\r\n";
    } else if (wifi_outage && millis() > wifi_outage + 360000) {
         //snprintf (buf, sizeof(buf), "%d_m_WiFi_outage_ended\r\n",
         //          (millis() - wifi_outage) / 60000);
         StatusPacket += ((millis() - wifi_outage) / 60000);
         StatusPacket += "_m_WiFi_outage_ended\r\n";
    } else if (wifi_reconnected) {
        StatusPacket += "wifi_reconnected\r\n";
    } else if (ognrelay_time && time_synched) {
        if (remote_sleep_length == 0) {
            StatusPacket += String(remote_uptime);
            StatusPacket += "_m_r_uptime\r\n";
        } else {
            StatusPacket += String(remote_sleep_length);
            StatusPacket += "_m_r_sleep\r\n";
        }
    } else {
        if (sleep_length == 0) {
            StatusPacket += String(uptime);
            StatusPacket += "_m_uptime\r\n";
        } else {
            StatusPacket += String(sleep_length/60);
            StatusPacket += "_m_sleep\r\n";
        }
    }
    Serial.println("");
    Serial.print(StatusPacket.c_str());
    Serial.println("");
    Logger_send_udp(&StatusPacket);
    if (SoC->WiFi_transmit_TCP(StatusPacket) == 0) {
        Serial.println(F("transmit status to TCP failed"));   // <<<
        return false;
    }

    if (OurTime == 0) {      /* no GNSS time available yet */
      Serial.println(F("sent status packet without time stamp"));
      return false;
    }

    // reset wifi_reconnected and wifi_outage here,
    // after sending packet out has succeeded.
    if (wifi_reconnected) {
        DebugLogWrite("WiFi reconnected");
        wifi_reconnected = false;
    }
    wifi_outage = 0;

    return true;
}
