/*
 * Time.cpp
 * Copyright (C) 2019-2020 Linar Yusupov
 *
 * Time-relay code by Moshe Braner, 2022
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
#include "GNSS.h"
#include "Time.h"
#include "RF.h"
#include "Log.h"
#include "OLED.h"
#include "global.h"

#include <TimeLib.h>

#if defined(TBEAM)
extern const gnss_chip_ops_t *gnss_chip;
#endif

time_t  OurTime = 0;           /* UTC time in seconds since start of 1970 */
uint32_t base_time_ms = 0;     /* this device millis() at last verified PPS */
uint32_t ref_time_ms = 0;      /* assumed local millis() at last PPS */

#define TIME_TO_TRYSYNC 2700
#define TIME_TO_ACKSYNC 10000
#define TIME_TO_RE_SYNC 185000
#define ADJ_FOR_TRANSMISSION_DELAY 10
#define TIME_TO_NTP_AGAIN 600000

int uptime = 0;
uint32_t last_hour = 0;

bool time_synched = false;
uint32_t when_synched = 0;
uint32_t when_sync_sent = 0;

uint8_t remote_sats=0;
uint8_t remote_uptime=0;
uint32_t remote_traffic = 0, remote_other=0;
uint16_t remote_timesent=0, remote_bad=0;
uint8_t remote_pctrel=0, remote_noack=0, remote_restarts=0, remote_round=0;

uint32_t traffic_packets_recvd = 0;
uint32_t traffic_packets_relayed = 0;
uint32_t traffic_packets_reported = 0;
uint16_t time_packets_sent = 0;
uint16_t ack_packets_recvd = 0;
uint32_t total_delays = 0;
uint16_t sync_restarts = 0;
uint16_t bad_packets_recvd = 0;
uint32_t other_packets_recvd = 0;


/********************* GNSS-time relay code ************************/

/* borrow the hash function used in freqplan.h */
uint32_t TimeHash(uint32_t Time) {
     Time  = (Time<<15) + (~Time);
     Time ^= Time>>12;
     Time += Time<<2;
     Time ^= Time>>4;
     Time *= 2057;
     return Time ^ (Time>>16);
}

/* this is called by both remote & base stations, to */
/*  send time data, and to acknowledge, respectively */
bool send_time(void)
{
    if (! ognrelay_time)
      return false;

    uint32_t satellites;
#if defined(TBEAM)
    if (ognrelay_enable && (! isValidGNSStime()))   /* time data source needs GPS time */
      return false;
    if (ognrelay_enable) {
      satellites = gnss.satellites.value();
      if (satellites > 0x0F)  satellites = 0x0F;
    }
#else
    if (ognrelay_enable)   /* relay station must have GPS to relay time */
      return false;
#endif

    legacy_packet_t* pkt = (legacy_packet_t *) TxBuffer;
    pkt->addr = 0xACACAC;                 /* marks time-relay packets */
    pkt->_unk0 = 0xC;                     /* marks time-relay packets */
    uint32_t* p = ( uint32_t *) TxBuffer; 
    p[1] = (uint32_t) OurTime;

    if (ognrelay_enable) {

      p[2] = ((millis() - ref_time_ms) & 0x0FFF);

      /* send stats */
      p[2] |= (satellites << 12);
      p[2] |= ((other_packets_recvd & 0x0FFFF) << 16);
      // pkt->addr_type = (((total_delays/(ack_packets_recvd+1)) >> 2) & 0x07);  /* 3 bits: avg roundtrip ms / 4 */
      pkt->addr_type = (uptime & 0x07);
      pkt->_unk1 = (uptime & 0x08) >> 3;
      int pctrel = 100 * traffic_packets_relayed;
      pctrel /= (traffic_packets_recvd+1);    /* percent relayed */
      p[3] = (traffic_packets_recvd & 0x01FFFFFF) | (pctrel<<25);
/* p[4]:
      0000 0000 0000 0000 0000 0000 0000 0000
           |       |    | ^^^^ ^^^^ ^^^^ ^^^^ - time_packets_sent
           |       | ^^^^ 00 - %!acked
           |^^^ ^^^^ 000 - bad packets
      ^^^^ ^ - restarts
*/
      int pctnoak = 100 * (time_packets_sent - ack_packets_recvd);
      pctnoak /= (time_packets_sent+1);    /* percent not acked */
      p[4] = (time_packets_sent & 0x0000FFFF)      /* 16 bits */
            | ((pctnoak & 0x7C) << 14)             /* 5 bits: %!acked/4 */
            | ((bad_packets_recvd & 0x3F8) << 17)  /* 7 bits: bad packets / 8 */
            | ((sync_restarts & 0x01F) << 27);

      p[3] ^= 0xACACACAC;                          /* rudimentary whitening */
      p[4] ^= 0xACACACAC;

    } else {   /* base station sending ack - blank fields */

      p[2] = 0;
      p[3] = 0xACACACAC;
      p[4] = 0xACACACAC;

    }

    /* also send hashed combination of time and relay ID for error check & security */
    /* >>> note: the same secret ognrelay_key needs to be in config of both stations */
    p[5] = TimeHash((p[1] ^ p[2]) ^ ognrelay_key);
    p[1] ^= 0xACACACAC;
    p[2] ^= 0xACACACAC;
    size_t size = LEGACY_PAYLOAD_SIZE;
    bool wait = true;  // false (no wait) often seemed to fail to transmit // 
    bool success = RF_Transmit(size, wait);
    if (success) {
      ++time_packets_sent;
      when_sync_sent = millis();
      if (ognrelay_enable)
        Serial.println(F("sent time & stats..."));
      else
        Serial.println(F("sent time ack"));
    }
    return success;
}

/* just check whether the packet is a time-sync one */
bool time_sync_pkt(uint8_t *raw)
{
    if (! ognrelay_time)        return false;
    legacy_packet_t* pkt = (legacy_packet_t *) raw;
    if (pkt->_unk0 != 0xC)      return false;    /* marks time-relay packets */
    if (pkt->addr != 0xACACAC)  return false;    /* marks time-sync packets */
    return true;
}

/* this is called by base station to process incoming time data */
void set_our_clock(uint8_t *raw)
{
    if (! ognrelay_base || ! ognrelay_time)  return;

    uint32_t* p = (uint32_t *) raw; 
    p[1] ^= 0xACACACAC;
    p[2] ^= 0xACACACAC;                      /* undo rudimentary whitening */
    if (p[5] != TimeHash((p[1] ^ p[2]) ^ ognrelay_key)) {
      ++bad_packets_recvd;                   /* this statistic collected within base */
      return;                                /* failed security check */
    }

    OurTime = (time_t) p[1];
    uint32_t timeOffset = (p[2] & 0x0FFF);
    timeOffset += ADJ_FOR_TRANSMISSION_DELAY;   /* ms */
    if (timeOffset >= 1000) {
      OurTime += 1;
      timeOffset -= 1000;
    }
    when_synched = millis();
    ref_time_ms = when_synched - timeOffset;   /* time of last PPS in remote station */

    if (send_time())           /* sent an ack successfully */
      time_synched = true;

    String msg;
    if (time_synched) {
      Serial.printf("time_synched at %d ms\r\n", when_synched);
      msg = "time_synched";
    } else {
      Serial.println(F("time-sync ack failed"));
      msg = "time-sync ack failed";
    }
    Logger_send_udp(&msg);

    /* show stats sent from remote station */
    legacy_packet_t* pkt = (legacy_packet_t *) raw;
    p[3] ^= 0xACACACAC;
    p[4] ^= 0xACACACAC;
    remote_sats = ((p[2]>>12) & 0x0F);
    remote_traffic = (p[3] & 0x01FFFFFF);
    remote_pctrel = (p[3]>>25) & 0x7F;
    remote_timesent = (p[4] & 0x0000FFFF);
    remote_noack = (p[4]>>14) & 0x7C;
    remote_bad = (p[4]>>17) & 0x3F8;
    remote_other = p[2]>>16;
    remote_restarts = (p[4]>>27) & 0x1F;
    // remote_round = (pkt->addr_type <<2);
    remote_uptime = (pkt->addr_type & 0x07) | (pkt->_unk1 & 0x01) << 3;
    Serial.println(F("Remote Stats:"));
    Serial.printf(
    "  traffic %d, \%relayed %d, timesent %d, \%noack %d, bad %d, other %d, restarts %d, round %d\r\n",
        remote_traffic, remote_pctrel, remote_timesent, remote_noack,
        remote_bad, remote_other, remote_restarts, remote_round);
    Serial.println(F("Local Stats:"));
    Serial.printf("  traffic: %d,  bad: %d,  restarts: %d\r\n",
        traffic_packets_recvd, bad_packets_recvd, sync_restarts);
    /* these stats also displayed in the statistics web page */
    // should also send these stats out via UDP.
}

/* this is called by remote station to process incoming ack packets */
void sync_alive_pkt(uint8_t *raw)
{
    if (! ognrelay_enable)  return;

    uint32_t* p = (uint32_t *) raw; 
    p[1] ^= 0xACACACAC;                         /* undo rudimentary whitening */
    p[2] ^= 0xACACACAC;
    if (p[5] != TimeHash((p[1] ^ p[2]) ^ ognrelay_key)) {
      ++bad_packets_recvd;                      /* this statistic collected within remote */
      return;                                   /* failed security check */
    }
    time_synched = true;    /* remote station switches mode only after ack received */
    ++ack_packets_recvd;
    when_synched = millis();
    total_delays += (when_synched - when_sync_sent);
    Serial.println(F("received timesync ack..."));
}

/* this is called by base station to tell the remote station to reboot */
bool reboot_remote(void)
{
    if (! ognrelay_base)  return false;

    legacy_packet_t* pkt = (legacy_packet_t *) TxBuffer;
    pkt->addr = 0xCACACA;                 /* marks remote_reboot packets */
    pkt->_unk0 = 0xA;

    uint32_t* p = ( uint32_t *) TxBuffer; 

    p[1] = 0xACACACAC;
    p[2] = millis();  /* just something unpredictable to hash with */
    p[3] = 0xACACACAC;
    p[4] = 0xACACACAC;

    /* also send hashed combination of time and relay ID for error check & security */
    /* >>> note: the same secret ognrelay_key needs to be in config of both stations */
    p[5] = TimeHash(p[2] ^ ognrelay_key);
    p[2] ^= 0xACACACAC;
    size_t size = LEGACY_PAYLOAD_SIZE;
    bool wait = true;
    bool success = RF_Transmit(size, wait);
    if (success)
        Serial.println(F("sent remote reboot packet"));
    return success;
}

/* check whether the packet is telling the remote station to reboot */
bool maybe_remote_reboot(uint8_t *raw) {
    if (! ognrelay_enable)      return false;
    legacy_packet_t* pkt = (legacy_packet_t *) raw;
    if (pkt->_unk0 != 0xA)      return false;
    if (pkt->addr != 0xCACACA)  return false;    /* not a reboot packet */

    uint32_t* p = (uint32_t *) raw; 
    p[2] ^= 0xACACACAC;                         /* undo rudimentary whitening */
    if (p[5] != TimeHash(p[2] ^ ognrelay_key)) {
      ++bad_packets_recvd;                      /* this statistic collected within remote */
      return true;                             /* failed security check, but not traffic packet */
    }

    Serial.println("got signal to reboot...");

    if (millis() > 600000                  /* don't reboot _again_ for a while */
        && WiFi.getMode() == WIFI_OFF) {   /* don't reboot unless WIFI has gone to sleep */
            Serial.println("rebooting in 3 sec...");
            delay(3000);
            SoC->reset();
    }

    return true;                       /* did not reboot, but not traffic packet */
}

/********************* time loop code ************************/

void Time_loop()
{
    uint32_t now_ms = millis();

    if (ognrelay_base && ognrelay_time && (!time_synched || OurTime==0)) {
    /* initial time-sync when relaying time */

        // OurTime = 0;
        time_synched = false;
        return;
        /* wait for time data from remote station */
        /* it will be handled by set_our_clock()  */

    }

    if (ognrelay_base && ognrelay_time && time_synched && OurTime != 0) {
    /* when base is getting time data periodically from remote station */

        /* use free-running clock between time sync messages */
        if (now_ms >= ref_time_ms + 1000) {
          OurTime += 1;
          ref_time_ms += 1000;
        }

    } else if (!ognrelay_enable && !ognrelay_time && !ogn_gnsstime) {
    /* when base (or standalone) is getting time data from NTP */

        if (OurTime == 0 || ! time_synched) {
            //Serial.println(F("waiting for NTP..."));  
            return;
        }

        /* get fresh NTP time data periodically */
        static uint32_t time_to_call_ntp = 0;
        if (now_ms > time_to_call_ntp + TIME_TO_NTP_AGAIN) {
            Time_setup();
            time_to_call_ntp = millis();
        }

        /* use free-running clock between those times */
        if (now_ms >= ref_time_ms + 1000) {
          OurTime += 1;
          ref_time_ms += 1000;
        }

    } else if (ognrelay_enable && ! ognrelay_time && ! ogn_gnsstime) {
    /* when remote station is not using GNSS time */

        if (OurTime == 0) {    /* starting up */
          OurTime = 1;         /* arbitrary time, used for packet aging only */
          ref_time_ms = now_ms;
          time_synched = true;
        }

        /* use free-running clock */
        if (now_ms >= ref_time_ms + 1000) {
          OurTime += 1;
          ref_time_ms += 1000;
        }

#if defined(TBEAM)
    } else if ((!ognrelay_base || !ognrelay_time) && OurTime != 0
               && (now_ms < base_time_ms + 2000
                   || (now_ms < base_time_ms + 60000 && !isValidGNSStime()))) {

        /* use free-running clock for short periods or during GPS dropouts */
        if (now_ms >= ref_time_ms + 1000) {
          OurTime += 1;
          ref_time_ms += 1000;
        }
#endif

    } else { /* get fresh GNSS time data */

#if !defined(TBEAM)
      //Serial.println(F("should use TBEAM with GNSS"));
#endif

#if defined(TBEAM)

    if (ognrelay_enable && ! isValidGNSStime()) {     /* even after a minute of free-running */

      // OurTime = 0;
      if (time_synched) {
        time_synched = false;
        ++sync_restarts;
      }
//Serial.println(F("waiting for GNSS time..."));  

    } else {
//Serial.println(F("processing GNSS time..."));  

      uint32_t pps_btime_ms = SoC->get_PPS_TimeMarker();
      if (now_ms > pps_btime_ms + 1010)
        pps_btime_ms += 1000;
      uint32_t time_corr_neg;

      if (pps_btime_ms) {
        uint32_t last_Commit_Time = now_ms - gnss.time.age();
        if (pps_btime_ms <= last_Commit_Time) {
          time_corr_neg = (last_Commit_Time - pps_btime_ms) % 1000;
        } else {
          time_corr_neg = 1000 - ((pps_btime_ms - last_Commit_Time) % 1000);
        }
        ref_time_ms = base_time_ms = pps_btime_ms;
      } else {
        uint32_t last_RMC_Commit = now_ms - gnss.date.age();
        time_corr_neg = gnss_chip ? gnss_chip->rmc_ms : 100;
        ref_time_ms = base_time_ms = last_RMC_Commit - time_corr_neg;
      }

      int yr    = gnss.date.year();
      if( yr > 99)
          yr    = yr - 1970;
      else
          yr    += 30;
      tmElements_t tm;
      tm.Year   = yr;
      tm.Month  = gnss.date.month();
      tm.Day    = gnss.date.day();
      tm.Hour   = gnss.time.hour();
      tm.Minute = gnss.time.minute();
      tm.Second = gnss.time.second();

//    OurTime = makeTime(tm) + (gnss.time.age() - time_corr_neg) / 1000;
      OurTime = makeTime(tm) + (gnss.time.age() + time_corr_neg) / 1000;

    }
#endif   /* TBEAM */

    }  /* got time data */


    if (ThisAircraft.timestamp != OurTime) {      /* do this only once per second */

//Serial.println(F("updating our timestamp..."));  

      ThisAircraft.timestamp = OurTime;
#if defined(TBEAM)
      if (ogn_gnsstime && isValidGNSStime()) {
        ThisAircraft.hour = gnss.time.hour();
        ThisAircraft.minute = gnss.time.minute();
        ThisAircraft.second = gnss.time.second();
      } else {
        ThisAircraft.hour = hour(OurTime);
        ThisAircraft.minute = minute(OurTime);
        ThisAircraft.second = second(OurTime);
      }
#else
        ThisAircraft.hour = hour(OurTime);
        ThisAircraft.minute = minute(OurTime);
        ThisAircraft.second = second(OurTime);
#endif

    }

    if (now_ms > last_hour + 3600000) {
        ++uptime;    
        last_hour = now_ms;
    }

    if ((ognrelay_enable || ognrelay_base) && ognrelay_time) {

       /* attempting initial time-sync from remote station */
       if (ognrelay_enable && !time_synched
             && now_ms > when_sync_sent + TIME_TO_TRYSYNC
             && now_ms > TxTimeMarker) {
         if (isValidGNSStime())
           (void) send_time();        /* may or may not succeed */
         when_sync_sent = now_ms;   /* even if no success, do not try too often */
         return;
       }

       /* send fresh time data every 10 seconds */
       if (ognrelay_enable && time_synched
             && now_ms > when_sync_sent + TIME_TO_ACKSYNC
             && now_ms > TxTimeMarker) {
         if (isValidGNSStime()) {
           if (send_time())           /* success (which also updated when_sync_sent) */
              return;
         }
         /* base sation will send an ack back, so both stations will update when_synched */
       }

       /* if haven't heard from other station in a while, start over */
       if (time_synched && (now_ms > when_synched + TIME_TO_RE_SYNC)) {
          time_synched = false;
          ++sync_restarts;
          if (ognrelay_base) {
            remote_sats = 0;
            String msg = "restarting time sync";
            Logger_send_udp(&msg);
          }
          Serial.printf("restarting time sync %d ms - syncd @ %d\r\n", now_ms, when_synched);
       }
    }
}


/****************************** NTP code ******************************/

#if !defined(EXCLUDE_NTP)

unsigned int localPort = 2390;      // local port to listen for UDP packets

/* Don't hardwire the IP address or we won't get the benefits of the pool.
 *  Lookup the IP address for the host name instead */
//IPAddress timeServer(129, 6, 15, 28); // time.nist.gov NTP server
IPAddress timeServerIP; // time.nist.gov NTP server address
//const char* ntpServerName = "time.nist.gov";
const String ntpServerName_suffix = ".pool.ntp.org";

const int NTP_PACKET_SIZE = 48; // NTP time stamp is in the first 48 bytes of the message

byte NTPPacketBuffer[ NTP_PACKET_SIZE]; //buffer to hold incoming and outgoing packets

// A UDP instance to let us send and receive packets over UDP
WiFiUDP NTP_udp;

// send an NTP request to the time server at the given address
void sendNTPpacket(IPAddress& address)
{
    Serial.println(F("sending NTP packet..."));
    // set all bytes in the buffer to 0
    memset(NTPPacketBuffer, 0, NTP_PACKET_SIZE);
    // Initialize values needed to form NTP request
    // (see URL above for details on the packets)
    NTPPacketBuffer[0] = 0b11100011; // LI, Version, Mode
    NTPPacketBuffer[1] = 0;          // Stratum, or type of clock
    NTPPacketBuffer[2] = 6;          // Polling Interval
    NTPPacketBuffer[3] = 0xEC;       // Peer Clock Precision
    // 8 bytes of zero for Root Delay & Root Dispersion
    NTPPacketBuffer[12] = 49;
    NTPPacketBuffer[13] = 0x4E;
    NTPPacketBuffer[14] = 49;
    NTPPacketBuffer[15] = 52;

    // all NTP fields have been given values, now
    // you can send a packet requesting a timestamp:
    NTP_udp.beginPacket(address, 123); //NTP requests are to port 123
    NTP_udp.write(NTPPacketBuffer, NTP_PACKET_SIZE);
    NTP_udp.endPacket();
}

#endif /* EXCLUDE_NTP */


void Time_setup()
{
    char buf[32];

    if (last_hour == 0)
        last_hour = millis();

    /*
     * only use NTP in base station
     * and if not getting time from remote station
     * and if not using local GNSS time
     */
    if (ognrelay_enable || ognrelay_time || ogn_gnsstime)
        return;

    /* rest is only for base (or single) station that is not using GNSS time nor relay time */
    /*   - i.e., it needs to get the UTC time from NTP to timestamp the OGN reports */

#if !defined(EXCLUDE_WIFI)
#if !defined(EXCLUDE_NTP)

    int    cb = 0;
    String ntpServerName;

    // Do not attempt to timesync in Soft AP mode
    if (WiFi.getMode() == WIFI_AP) {
        Serial.println(F("Cannot get NTP time without internet"));
        snprintf(buf, sizeof(buf), "no internet no NTP time");
        OLED_write(buf, 0, 54, false);
        return;
    }

    Serial.println(F("Starting NTP UDP"));
    NTP_udp.begin(localPort);
    Serial.print(F("Local port: "));
    Serial.println(localPort);

    for (int attempt = 1; attempt <= 4; attempt++) {
        //get a random server from the pool
        ntpServerName = String(attempt - 1) + ntpServerName_suffix;
        WiFi.hostByName(ntpServerName.c_str(), timeServerIP);

        Serial.print('#');
        Serial.print(attempt);
        Serial.print(F(" NTP server's IP address: "));
        Serial.println(timeServerIP);

        sendNTPpacket(timeServerIP); // send an NTP packet to a time server

        // wait to see if a reply is available
        delay(2000);

        cb = NTP_udp.parsePacket();
        if (!cb)
        {
            Serial.print(F("No response on request #"));
            Serial.println(attempt);
            continue;
        }
        else
        {
            Serial.print(F("Reply packet received, length="));
            Serial.println(cb);
            // We've received a packet, read the data from it
            NTP_udp.read(NTPPacketBuffer, NTP_PACKET_SIZE); // read the packet into the buffer
            break;
        }
    }

    NTP_udp.stop();

    if (!cb)
    {
        Serial.println(F("WARNING! Unable to sync time by NTP."));
        return;
    }

    //the timestamp starts at byte 40 of the received packet and is four bytes,
    // or two words, long. First, esxtract the two words:

    uint32_t highWord = word(NTPPacketBuffer[40], NTPPacketBuffer[41]);
    uint32_t lowWord  = word(NTPPacketBuffer[42], NTPPacketBuffer[43]);
    // combine the four bytes (two words) into a long integer
    // this is NTP time (seconds since Jan 1 1900):
    uint32_t secsSince1900 = highWord << 16 | lowWord;
    Serial.print(F("Seconds since Jan 1 1900 = "));
    Serial.println(secsSince1900);

    // now convert NTP time into everyday time:
    Serial.print(F("Unix time = "));
    // Unix time starts on Jan 1 1970. In seconds, that's 2208988800:
    const uint32_t seventyYears = 2208988800UL;
    // subtract seventy years:
    uint32_t epoch = secsSince1900 - seventyYears;
    // print Unix time:
    Serial.println(epoch);
    setTime((time_t) epoch);
    OurTime = (time_t) epoch;
    time_synched = true;
    ref_time_ms = millis();

#if 0
    // print the hour, minute and second:
    Serial.print(F("The UTC time is "));    // UTC is the time at Greenwich Meridian (GMT)
    Serial.print((epoch  % 86400L) / 3600); // print the hour (86400 equals secs per day)
    Serial.print(':');
    if (((epoch % 3600) / 60) < 10)
        // In the first 10 minutes of each hour, we'll want a leading '0'
        Serial.print('0');
    Serial.print((epoch  % 3600) / 60);// print the minute (3600 equals secs per minute)
    Serial.print(':');
    if ((epoch % 60) < 10)
        // In the first 10 seconds of each minute, we'll want a leading '0'
        Serial.print('0');
    Serial.println(epoch % 60); // print the second
#endif
    // print the hour, minute and second:
    Serial.print(F("UTC time (from NTP) is "));    // UTC is the time at Greenwich Meridian (GMT)
    Serial.printf("%02d:%02d:%02d\r\n", hour(epoch), minute(epoch), second(epoch));

#endif /* EXCLUDE_NTP */
#endif /* EXCLUDE_WIFI */

}
