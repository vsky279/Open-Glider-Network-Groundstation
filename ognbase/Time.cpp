/*
 * Time.cpp
 * Copyright (C) 2019-2020 Linar Yusupov
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
#include "RF.h"
#include "Log.h"
#include "global.h"

#include <TimeLib.h>

/********************* GNSS-time relay code ************************/

#if defined(TBEAM)
extern const gnss_chip_ops_t *gnss_chip;
#endif

time_t  OurTime = 0;
uint32_t ref_time_ms = 0;
uint32_t base_time_ms = 0;

#define TIME_TO_TRYSYNC 2700
#define TIME_TO_ACKSYNC 10000
#define TIME_TO_RE_SYNC 155000
#define ADJ_FOR_TRANSMISSION_DELAY 25

bool time_synched = false;
uint32_t when_synched = 0;
uint32_t when_sync_sent = 0;

uint8_t remote_sats = 0;

uint32_t traffic_packets_recvd = 1;      /* avoid division by 0 */
uint32_t traffic_packets_relayed = 1;
uint16_t time_packets_sent = 1;
uint16_t ack_packets_recvd = 1;
uint32_t total_delays = 0;
uint16_t bad_packets_recvd = 0;
uint16_t sync_restarts = 0;


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

#if defined(TBEAM)
    if (ognrelay_enable && (! isValidGNSStime()))   /* time data source needs GPS time */
      return false;
#else
    if (ognrelay_enable)   /* relay station must have GPS to relay time */
      return false;
#endif

    legacy_packet_t* pkt = (legacy_packet_t *) TxBuffer;
    pkt->addr = 0xACACAC;                 /* marks time-sync packets */
    pkt->_unk0 = 0xC;                     /* marks time-relay packets */
    uint32_t* p = ( uint32_t *) TxBuffer; 
    p[1] = (uint32_t) OurTime;
    uint32_t satellites = gnss.satellites.value();
    if (satellites > 0x0F)  satellites = 0x0F;
    uint32_t rightnow = millis();
    uint32_t RxOffset = ((rightnow - ref_time_ms) & 0x0FFF);
    p[2] = RxOffset | (satellites << 12);
         // higher-order 16 bits of p[2] are not used for now!
    if (ognrelay_enable) {
      /* send stats */
      pkt->addr_type = (((total_delays/ack_packets_recvd) >> 2) ^ 0x07);  /* 3 bits: avg roundtrip ms / 4 */
      // also available: pkt->_unk1 - 1 bit - not used for now.
      p[3] = (traffic_packets_recvd & 0x01FFFFFF)
            | ((100*(traffic_packets_recvd-traffic_packets_relayed)/traffic_packets_recvd)<<25);  /* % dropped */
      p[4] = (time_packets_sent & 0x0000FFFF)   /* 16 bits */
            | (((100*(time_packets_sent-ack_packets_recvd)/time_packets_sent)&0x3E)<<15)  /* 5 bits: %!acked/2 */
            | ((bad_packets_recvd & 0x1F8) << 18)  /* 6 bits: bad packets / 8 */
            | ((sync_restarts & 0x01F) << 27);
      p[3] ^= 0xACACACAC;
      p[4] ^= 0xACACACAC;                         /* rudimentary whitening */
    } else {   /* base station sending ack */
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
    bool success = RF_Transmit(size, wait);      /* transmit rightnow without waiting */
    if (success) {
      ++time_packets_sent;
      when_sync_sent = rightnow;
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
    if (! ognrelay_base)  return;

    uint32_t* p = (uint32_t *) raw; 
    p[1] ^= 0xACACACAC;
    p[2] ^= 0xACACACAC;                      /* undo rudimentary whitening */
    if (p[5] != TimeHash((p[1] ^ p[2]) ^ ognrelay_key)) {
      ++bad_packets_recvd;                   /* this statistic collected within base */
      return;                                /* failed security check */
    }
    OurTime = (time_t) p[1];
    uint32_t RxOffset = (p[2] & 0x0FFF);
    RxOffset += ADJ_FOR_TRANSMISSION_DELAY;   /* ms */
    if (RxOffset >= 1000) {
      OurTime += 1;
      RxOffset -= 1000;
    }
    when_synched = millis();
    ref_time_ms = when_synched - RxOffset;

    if (send_time())           /* sent an ack successfully */
      time_synched = true;

    if (time_synched) {
      Serial.printf("time_synched at %d ms\r\n", when_synched);
      String msg = "time_synched";
      Logger_send_udp(&msg);
    } else {
      String msg = "received time-pkt";
      Logger_send_udp(&msg);
    }

    /* show stats sent from remote station */
    legacy_packet_t* pkt = (legacy_packet_t *) raw;
    p[3] ^= 0xACACACAC;
    p[4] ^= 0xACACACAC;
    remote_sats = ((p[2]>>12) & 0x0F);
    Serial.printf("stats: %d, %d, %d  %08X  %08X\r\n",
       pkt->addr_type, pkt->_unk1, remote_sats, p[3], p[4]);
    // later will write code to de-cipher the other stats.
}

/* this is called by remote station to process incoming ack packets */
void sync_alive_pkt(uint8_t *raw)
{
    if (! ognrelay_enable)  return;

    uint32_t* p = (uint32_t *) raw; 
    uint32_t RxTime   = p[1] ^ 0xACACACAC;       /* undo rudimentary whitening */
    uint32_t RxOffset = p[2] ^ 0xACACACAC;
    if (p[5] != TimeHash((RxTime ^ RxOffset) ^ ognrelay_key)) {
      ++bad_packets_recvd;
      return;                              /* failed security check */
    }
    time_synched = true;    /* remote station switches mode only after ack received */
    ++ack_packets_recvd;
    when_synched = millis();
    total_delays += (when_synched - when_sync_sent);
    Serial.println(F("received timesync ack..."));
}

void Time_loop()
{
    uint32_t now_ms = millis();

//Serial.println("inside Time_loop...");

    if (ognrelay_base && ognrelay_time && (!time_synched || OurTime==0)) {

        // OurTime = 0;
        time_synched = false;
        return;
        /* wait for time data from remote station */
        /* it will be handled by set_our_clock()  */
//Serial.println(F("waiting for time data..."));  

    /* when base is getting time data periodically from remote station: */
    } else if (ognrelay_base && ognrelay_time && time_synched && OurTime != 0) {

        /* use free-running clock between time sync messages */
        if (now_ms >= ref_time_ms + 1000) {
          OurTime += 1;
          ref_time_ms += 1000;
//Serial.println(F("free-running clock..."));
        }

#if defined(TBEAM)
    } else if ((!ognrelay_base || !ognrelay_time) && OurTime != 0
               && (now_ms < base_time_ms + 2000
                   || (now_ms < base_time_ms + 60000 && !isValidGNSStime()))) {

        /* use free-running clock for short periods or during GPS dropouts */
        if (now_ms >= ref_time_ms + 1000) {
          OurTime += 1;
          ref_time_ms += 1000;
//Serial.println(F("temporary free-running clock..."));
        }
#endif

    } else { /* get fresh GNSS time data */

#if !defined(TBEAM)
      /* should use TBEAM with GNSS */
Serial.println(F("TTGO calling now()..."));
      OurTime = now();                /* will be UTC only if NTP used */
      ref_time_ms = base_time_ms = now_ms;
#endif

#if defined(TBEAM)

    if (! isValidGNSStime()) {     /* even after a minute of free-running */

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

//if (tm.Second < 3)
//Serial.println(F("seconds=0..."));  

    }
#endif

    }  /* got time data */


    if (ThisAircraft.timestamp != OurTime) {      /* do this only once per second */

//Serial.println(F("updating our timestamp..."));  

      ThisAircraft.timestamp = OurTime;
#if defined(TBEAM)
      if (isValidGNSStime()) {
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

    if ((ognrelay_enable || ognrelay_base) && ognrelay_time) {

       if (ognrelay_enable && !time_synched
             && now_ms > when_sync_sent + TIME_TO_TRYSYNC
             && now_ms > TxTimeMarker) {
         if (isValidGNSStime())
           (void) send_time();        /* may or may not succeed */
         when_sync_sent = now_ms;   /* even if no success, do not try too often */
         return;
       }

       /* send fresh time data every 15 seconds */
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

#if defined(EXCLUDE_WIFI)
void Time_setup()
{}
#else

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

void Time_setup()
{

#if !defined(EXCLUDE_NTP)

    int    cb = 0;
    String ntpServerName;

    // Do not attempt to timesync in Soft AP mode
    if (WiFi.getMode() == WIFI_AP)
        return;

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

    unsigned long highWord = word(NTPPacketBuffer[40], NTPPacketBuffer[41]);
    unsigned long lowWord  = word(NTPPacketBuffer[42], NTPPacketBuffer[43]);
    // combine the four bytes (two words) into a long integer
    // this is NTP time (seconds since Jan 1 1900):
    unsigned long secsSince1900 = highWord << 16 | lowWord;
    Serial.print(F("Seconds since Jan 1 1900 = "));
    Serial.println(secsSince1900);

    // now convert NTP time into everyday time:
    Serial.print(F("Unix time = "));
    // Unix time starts on Jan 1 1970. In seconds, that's 2208988800:
    const unsigned long seventyYears = 2208988800UL;
    // subtract seventy years:
    unsigned long epoch = secsSince1900 - seventyYears;
    // print Unix time:
    Serial.println(epoch);
    setTime((time_t) epoch);

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

#endif /* EXCLUDE_NTP */

}

#endif /* EXCLUDE_WIFI */
