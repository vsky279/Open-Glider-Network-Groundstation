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
#include "APRS.h"
#include "RF.h"
#include "Log.h"
#include "OLED.h"
#include "Battery.h"
#include "global.h"

#include <TimeLib.h>

#include "esp_wifi.h"
#include "esp_wifi_types.h"

#if defined(TBEAM)
extern const gnss_chip_ops_t *gnss_chip;
#endif

time_t   OurTime = 0;          /* UTC time in seconds since start of 1970 */
uint32_t base_time_ms = 0;     /* this device millis() at last verified PPS */
uint32_t ref_time_ms = 0;      /* assumed local millis() at last PPS */

#define TIME_TO_TRYSYNC 2700
#define TIME_TO_ACKSYNC 10300
#define TIME_TO_RE_SYNC 291000    // when to start over communicating with remote station
#define ADJ_FOR_FLARM_RECEPTION 0       /* 40-50 seems to receive FLARM packets better? */
#define ADJ_FOR_TRANSMISSION_DELAY 10
#define TIME_TO_GNSS_TIME 151000    // how often to check GNSS
#define TIME_TO_NTP_AGAIN 151000    // how often to check NTP

uint32_t time_to_call_ntp = 0;
int ntp_ms_adjust = 0;

uint32_t when_synched = 0;
uint32_t when_sync_tried = 0;
uint32_t when_sync_sent = 0;
uint32_t when_to_switch = 0;
uint32_t sleep_when = 0;
uint32_t sleep_length = 0;
uint16_t uptime = 0;
time_t last_hour = 0;

bool time_synched = false;
bool NTP_synched  = false;
bool got_time_ack = false;

uint32_t remote_traffic=0, remote_other=0;
uint16_t remote_uptime=0, remote_sleep_length=0;
uint16_t remote_timesent=0, remote_bad=0;
uint8_t remote_pctrel=0, remote_ack=0, remote_restarts=0, remote_round=0;
uint8_t remote_sats=0;
float remote_voltage=0.0;

uint32_t traffic_packets_recvd = 0;
uint32_t old_protocol_packets_recvd = 0;
uint32_t air_relayed_packets_recvd = 0;
uint32_t traffic_packets_relayed = 0;
uint32_t traffic_packets_reported = 0;
uint16_t time_packets_sent = 0;
uint16_t ack_packets_recvd = 0;
uint32_t total_delays = 0;
uint16_t sync_restarts = 0;
uint16_t bad_packets_recvd = 0;
uint32_t other_packets_recvd = 0;

#define TRAFFIC_MINUTES 8
uint32_t traffic_by_minute[TRAFFIC_MINUTES];
int traffic_minute_pointer = 0;
uint16_t packets_per_minute = 0;

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

// this is called by time_master to send time data, and by time_client to acknowledge
bool send_time(void)
{
    uint32_t satellites = 0;

    if (OurTime < 1000000)
        return false;

    if (! RF_TX_ready()) {   /* transmitting not allowed at this time */
//Serial.printf("cannot send time right now %d\r\n", millis());
        return false;
    }

#if defined(TBEAM)
    if (ognrelay_enable && time_master) {
      if (! isValidGNSStime())       /* remote needs GPS to be a time data source */
        return false;
      satellites = gnss.satellites.value();
      if (satellites > 0x0F)  satellites = 0x0F;
    }
#else
    if (ognrelay_enable && time_master)   /* relay station must have GPS to relay time */
      return false;
#endif

    legacy_packet_t* pkt = (legacy_packet_t *) TxBuffer;
    uint32_t* p = ( uint32_t *) TxBuffer;

    pkt->msg_type = (got_time_ack ? MSG_TYPE_ACK : MSG_TYPE_UNA);
    pkt->addr = 0xACACAC;            /* marks time-relay packets */

    uint32_t now_ms = millis();
    uint32_t offset = now_ms - ref_time_ms;
    if (offset > 1500) {    /* shouldn't happen */
Serial.printf("send_time: ref_time_ms %d << now %d ??\r\n", ref_time_ms, now_ms);
        return false;
    }

    /* send UTC time in seconds, and milliseconds within second */
    p[1] = (uint32_t) OurTime;
    p[2] = (offset & 0x0FFF);

    if (ognrelay_enable) {

      if (sleep_when > 0) {
          /* send intended sleep time (in minutes) instead of time stamp */
          p[1] = 0;
          p[2] = ((sleep_length / 60) & 0x0FFF);
      }

      /* send stats */
      p[2] |= (satellites << 12);
      p[2] |= ((other_packets_recvd & 0x01FFE0) << 11);
      remote_voltage = Battery_voltage();
      int volts = (int)(10.0 * remote_voltage + 0.5) - 28;
      if (volts < 0)  volts = 0;
      if (volts > 15) volts = 15;
      p[2] |= ((volts & 0x0F) << 28);
      if (uptime < 60) {
          pkt->addr_type = (uptime/10) & 0x07;
          pkt->_unk1 = 0;
      } else if (uptime < 300) {
          pkt->addr_type = ((uptime-60)/60) & 0x03;
          pkt->_unk1 = 1;
      } else {
          pkt->addr_type = (((uptime-300)/120) & 0x03) | 0x04;
          pkt->_unk1 = 1;
      }
Serial.printf("send_time: uptime %d sent as %d %X\r\n", uptime, pkt->_unk1, pkt->addr_type);
      int pctrel = 100 * traffic_packets_relayed;
      if (traffic_packets_recvd > 0)
        pctrel /= traffic_packets_recvd;    /* percent relayed */
      p[3] = (traffic_packets_recvd & 0x01FFFFFF) | ((pctrel & 0x7F) << 25);
/* p[4]:
      0000 0000 0000 0000 0000 0000 0000 0000
           |      |     | ^^^^ ^^^^ ^^^^ ^^^^ - time_packets_sent
           |      |^ ^^^^ 00 - %acked
           |^^^ ^^^0 00 - bad packets
      ^^^^ ^ - restarts
*/
      int pctack = 100 * ack_packets_recvd;
      if (time_packets_sent > 0)
        pctack /= time_packets_sent;               /* percent acked */
      p[4] = (time_packets_sent & 0x0000FFFF)      /* 16 bits */
            | ((pctack & 0x7C) << 14)              /* 5 bits: %acked/4 */
            | ((bad_packets_recvd & 0x1F8) << 18)  /* 6 bits: bad packets / 8 */
            | ((sync_restarts & 0x01F) << 27);

      p[3] ^= 0xACACACAC;                          /* rudimentary whitening */
      p[4] ^= 0xACACACAC;

    } else {   // base station sending time/ack - blank stats fields

      p[3] = 0xACACACAC;
      p[4] = 0xACACACAC;

    }

    if (time_client && ! time_synched) {
        /* client chooses when to switch to synched-time & tells the master */
        p[1] = (got_time_ack? (uint32_t) when_to_switch : 0);
        p[2] = p[1];   /* marks it as a time-to-switch */
    }

    /* also send hashed combination of time and relay ID for error check & security */
    /* >>> note: the same secret ognrelay_key needs to be in config of both stations */
    p[5] = TimeHash((p[1] ^ p[2]) ^ ognrelay_key);
    p[1] ^= 0xACACACAC;
    p[2] ^= 0xACACACAC;
    size_t size = LEGACY_PAYLOAD_SIZE;
    bool wait = true;
    bool success = RF_Transmit(size, wait);
    if (success) {
      ++time_packets_sent;
      when_sync_sent = millis();
      if (ognrelay_enable) {
        if (ognreverse_time)
          Serial.println(F("sent time-ack & stats..."));
        else
          Serial.println(F("sent time & stats..."));
//Serial.printf("send_time: ms=%d, ourt=%d, ofst=%d, reft=%d\r\n", now_ms, OurTime, offset, ref_time_ms);
      } else if (ognreverse_time) {
          Serial.println(F("base sent time to remote"));
      } else {
          Serial.println(F("sent time ack"));
//Serial.printf("send_time_ack: ms=%d, ourt=%d, reft=%d\r\n", now_ms, OurTime, ref_time_ms);
      }
    }
    if (time_master)
        got_time_ack = false;    /* check for ack again for each time-data packet */
//else
//Serial.println(F("send_time unsuccessful"));

    return success;
}

/* just check whether the packet is a time-sync one */
bool time_sync_pkt(uint8_t *raw)
{
//    if (!time_client && !time_master)
//        return false;
    legacy_packet_t* pkt = (legacy_packet_t *) raw;
    if (pkt->addr != 0xACACAC)     /* marks time-sync packets */
        return false;
    if (pkt->msg_type == MSG_TYPE_ACK)         /* marks acked time-relay packets */
        return true;
    if (pkt->msg_type == MSG_TYPE_UNA)         /* marks un-acked time-relay packets */
        return true;
    return false;
}

/* get stats sent from remote station */
void get_remote_stats(uint32_t* p)
{
    p[3] ^= 0xACACACAC;
    p[4] ^= 0xACACACAC;
    remote_sats = ((p[2]>>12) & 0x0F);
    remote_traffic = (p[3] & 0x01FFFFFF);
    remote_pctrel = (p[3]>>25) & 0x7F;
    remote_timesent = (p[4] & 0x0000FFFF);
    remote_ack = (p[4]>>14) & 0x7C;
    remote_bad = (p[4]>>18) & 0x1F8;
    remote_other = (p[2]>>11) & 0x01FFE0;
    uint8_t ivoltage = ((p[2]>>28) & 0x0F) + 28;
    if (ivoltage > 28)
        remote_voltage = (float)ivoltage * 0.1;
    else
        remote_voltage = 0.0;
    remote_restarts = (p[4]>>27) & 0x1F;
    // remote_round = (pkt->addr_type <<2);
    legacy_packet_t* pkt = (legacy_packet_t *) p;
    uint16_t rem_upt;
    if (pkt->_unk1 == 0)
        rem_upt = (pkt->addr_type & 0x07) * 10;
    else if (pkt->addr_type < 4)   // (& 0x04 == 0)
        rem_upt = pkt->addr_type * 60 + 60;
    else
        rem_upt = (pkt->addr_type - 4) * 120 + 300;  // (& 0x03) * 120 + 300;
    if (rem_upt < remote_uptime)
        Serial.printf("reported uptime %d (%d %d) but stored uptime %d?\r\n",
            rem_upt, pkt->_unk1, pkt->addr_type, remote_uptime);
    //if (rem_upt > remote_uptime)
        remote_uptime = rem_upt;

    Serial.println(F("Local Stats:"));
    Serial.printf("  traffic: %d,  reported: %d,  bad: %d,  other: %d,  restarts: %d\r\n",
        traffic_packets_recvd, traffic_packets_reported,
        bad_packets_recvd, other_packets_recvd, sync_restarts);
    Serial.println(F("Remote Stats:"));
    Serial.printf(
    "  traffic %d, pct rlyed %d, timesent %d, pct ack %d, bad %d, other %d, restarts %d, uptime %d (%d %d)\r\n",
        remote_traffic, remote_pctrel, remote_timesent, remote_ack,
        remote_bad, remote_other, remote_restarts, remote_uptime, pkt->_unk1, pkt->addr_type);
    if (remote_voltage > 0.0 && remote_voltage < 3.65)
        Serial.printf("Remote battery voltage: %.1f < 3.65\r\n", remote_voltage);
    else if (remote_voltage == 0.0)
        Serial.printf("Remote battery voltage: ----\r\n");
    else
        Serial.printf("Remote battery voltage: %.1f\r\n", remote_voltage);
    /* these stats also displayed in the statistics web page */
    // should also send these stats out via UDP.
}

/* this is called by time_client to process incoming time data */
void set_our_clock(uint8_t *raw)
{
    if (! time_client)  return;

    uint32_t* p = (uint32_t *) raw;
    p[1] ^= 0xACACACAC;
    p[2] ^= 0xACACACAC;                      /* undo rudimentary whitening */
    if (p[5] != TimeHash((p[1] ^ p[2]) ^ ognrelay_key)) {
      ++bad_packets_recvd;                   /* this statistic collected within base */
Serial.println(F("received time pkt failed check"));
      return;                                /* failed security check */
    }

    if (p[1] != 0) {
      when_synched = millis();
      OurTime = (time_t) p[1];
      uint32_t timeOffset = (p[2] & 0x0FFF);
      timeOffset += ADJ_FOR_TRANSMISSION_DELAY;
      if (timeOffset > 1000) {
        OurTime += 1;
        timeOffset -= 1000;
      }
      ref_time_ms = when_synched - timeOffset;   /* time of last PPS in time_master */
Serial.printf("set_our_clock: ms=%d, ourt=%d, ofst=%d, reft=%d\r\n",
              when_synched, OurTime, timeOffset, ref_time_ms);
      remote_sleep_length = 0;

    } else {   /* this packet reports intended sleep */

        remote_sleep_length = (uint16_t)(p[2] & 0x0FFF);   /* minutes */
    }

    legacy_packet_t* pkt = (legacy_packet_t *) raw;
    if (pkt->msg_type == MSG_TYPE_ACK)
        got_time_ack = true;    /* for time_client, this "sticks" */

    if (ognrelay_base)
        get_remote_stats(p);

    if (got_time_ack && when_to_switch == 0) {
        when_to_switch = ((OurTime + 24) & 0xFFFFFFF0) + 8;   /* 16-32 seconds in the future */
        Serial.printf(">>> will switch to time_synched at %d\r\n", when_to_switch);
    }       

    delay(40);
    (void) send_time();      /* attempt ack - may not succeed */
}

/* this is called by time_master to process incoming ack packets */
void sync_alive_pkt(uint8_t *raw)
{
    if (! time_master)  return;

    legacy_packet_t* pkt = (legacy_packet_t *) raw;

    uint32_t* p = (uint32_t *) raw; 
    p[1] ^= 0xACACACAC;                         /* undo rudimentary whitening */
    p[2] ^= 0xACACACAC;
    if (p[5] != TimeHash((p[1] ^ p[2]) ^ ognrelay_key)) {
      Serial.println(F("received timesync ack failed check"));
      ++bad_packets_recvd;                      /* this statistic collected within remote */
      return;                                   /* failed security check */
    }

    time_t ClientTime;
    uint32_t ClientOffset;
    int32_t timediff;

    if (p[1] == 0) {   /* this packet reports intended sleep */

        remote_sleep_length = (uint16_t)(p[2] & 0x0FFF);   /* minutes */

    } else {

        ClientTime = (time_t) p[1];
        ClientOffset = (p[2] & 0x0FFF);
        ClientOffset += ADJ_FOR_TRANSMISSION_DELAY;
        if (! ognreverse_time)  ClientOffset += 40;
        if (ClientOffset > 1000) {
          ClientTime += 1;
          ClientOffset -= 1000;
        }
        when_synched = millis();
        uint32_t client_ref_time = when_synched - ClientOffset;
        if (ClientTime == OurTime - 1) {
            ++ClientTime;
            client_ref_time += 1000;
        }
        timediff = (int32_t)client_ref_time - (int32_t)ref_time_ms;

        if (ognrelay_base)
            get_remote_stats(p);
    }

    uint32_t delay = when_synched - when_sync_sent;
    if (delay < 1000) {
        ++ack_packets_recvd;
        total_delays += delay;
        Serial.printf("received timesync ack: ms=%d, time=%d, diff=%d ms, avg round %d ms\r\n",
           when_synched, p[1], timediff, total_delays/ack_packets_recvd);
        if (ClientTime != OurTime && time_synched && p[1] != p[2])
            Serial.printf("client time: %d != master time: %d ??\r\n", ClientTime, OurTime);
    }

    bool client_ack = (pkt->msg_type == MSG_TYPE_ACK);        /* client says it got ack-ack */
    if (! time_synched && client_ack && p[1]==p[2]
          && (p[1] > OurTime + 8) && (p[1] < OurTime + 34)) {
        when_to_switch = p[1];                  /* client chose switch time */
        Serial.printf(">>> time_synched switch client chose %d\r\n", when_to_switch);
    }
    if (when_to_switch == 0) {
        /* even without ack-ack, choose a switch time */
        when_to_switch = ((OurTime + 24) & 0xFFFFFFF0) + 8;   /* 16-32 seconds in the future */
        Serial.printf(">>> will switch to time_synched at %d\r\n", when_to_switch);
    }
    if (! client_ack && when_to_switch > 0 && (OurTime - when_to_switch < 8)) {
        /* without ack-ack, postpone switch time if client still not ready */
        when_to_switch += 8;
        Serial.printf(">>> postpone time_synched switch to %d\r\n", when_to_switch);
    }
    got_time_ack = true;    /* affects next send_time() */
}

/* this is called by base station to tell the remote station to reboot */
bool reboot_remote(void)
{
    if (! ognrelay_base)  return false;

    legacy_packet_t* pkt = (legacy_packet_t *) TxBuffer;
    pkt->addr = 0xCACACA;                 /* marks remote_reboot packets */
    pkt->msg_type = MSG_TYPE_RBT;

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
    return success;
}

/* check whether the packet is telling this remote station to reboot */
bool maybe_remote_reboot(uint8_t *raw) {
    if (! ognrelay_enable)      return false;
    legacy_packet_t* pkt = (legacy_packet_t *) raw;
    if (pkt->msg_type != MSG_TYPE_RBT)      return false;
    if (pkt->addr != 0xCACACA)  return false;    /* not a reboot packet */

    uint32_t* p = (uint32_t *) raw; 
    p[2] ^= 0xACACACAC;                         /* undo rudimentary whitening */
    if (p[5] != TimeHash(p[2] ^ ognrelay_key)) {
      ++bad_packets_recvd;                      /* this statistic collected within remote */
      return true;                             /* failed security check, but is not a traffic packet */
    }

    Serial.println(">>>>>> got signal to reboot <<<<<<");

    if (millis() > 600000                  /* don't reboot _again_ for a while */
        && WiFi.getMode() == WIFI_OFF) {   /* don't reboot unless WIFI has gone to sleep */
            Serial.println("rebooting in 2 sec...");
            OLED_enable();
            delay(1000);
            OLED_write("rebooting...", 0, 27, true);
            DebugLogWrite("maybe_remote_reboot() rebooting");
            delay(1000);
            SoC->reset();
    }

    return true;                       /* did not reboot, but is not a traffic packet */
}

void Timesync_restart()
{
    time_synched = false;
    got_time_ack = false;
    when_synched = 0;
    when_sync_tried = 0;
    when_sync_sent = 0;
    when_to_switch = 0;
    remote_uptime = 0;
    // sleep_when = 0;
    // sleep_length = 0;
    // remote_sleep_length = 0;
    ++sync_restarts;
}

/********************* time loop code ************************/

void Poll_GNSS()
{
    uint32_t now_ms = millis();
#if defined(TBEAM)

    bool validgnss = isValidGNSStime();

    if ((!ognrelay_base || !ognrelay_time) && OurTime != 0
          && (now_ms < base_time_ms + (validgnss? TIME_TO_GNSS_TIME : TIME_TO_RE_SYNC))) {

        /* use free-running clock for short periods or during GPS dropouts */
        if (now_ms >= ref_time_ms + 1000) {
          OurTime += 1;
          ref_time_ms += 1000;
//if (validgnss)
//Serial.printf("free running clock - between gnss fixes -> %d\r\n", OurTime);
//else
//Serial.printf("free running clock - gnss dropout -> %d\r\n", OurTime);
        }
        return;
    } 

    if (! validgnss) {      /* even after free-running for a while */
        if (ognrelay_enable && ognrelay_time) {
            // OurTime = 0;
            if (time_synched && (sleep_when==0 || millis()>sleep_when+3*TIME_TO_RE_SYNC))
                Timesync_restart();
Serial.println(F("Lost GNSS time, will re-sync..."));
        }
        if (OurTime > 0 && now_ms >= ref_time_ms + 1000) {    // >>> added 5/2023: keep counting time
            OurTime += 1;
            ref_time_ms += 1000;
        }
//Serial.println(F("waiting for GNSS time..."));
        return;
    } 

    uint32_t gnss_age = gnss.time.age();
    uint32_t last_Commit_Time = now_ms - gnss_age;
    bool newfix = false;
    uint32_t pps_btime_ms = SoC->get_PPS_TimeMarker();
    uint32_t newtime;
    uint32_t time_corr_neg;
    if (pps_btime_ms > 0) {
      newtime = pps_btime_ms + ADJ_FOR_FLARM_RECEPTION;   /* seems to receive FLARM better */
    } else {   /* PPS not available */
      time_corr_neg = gnss_chip ? gnss_chip->rmc_ms : 100;
      newtime = last_Commit_Time - time_corr_neg;
      if (gnss_chip)  newtime -= 180;    // experimentally seen to be better
    }
    if (gnss_age < 2500 && newtime > base_time_ms) {
      static uint32_t lasttime_ms = 0;
      if (last_Commit_Time - lasttime_ms > 150) {     /* new data arrived from GNSS */
        newfix = true;
        lasttime_ms = last_Commit_Time;
      }
    }

    /* between fixes (but not before first fix): free-running clock */
    if (OurTime > 0 && !newfix) {
        if (now_ms >= ref_time_ms + 1000) {
          OurTime += 1;
          ref_time_ms += 1000;
//Serial.printf("free running clock - not a new fix -> %d\r\n", OurTime);
        }
        return;
    }

    if (pps_btime_ms > 0) {

        if (now_ms > newtime + 1500) {
Serial.printf("PPS=%d << now=%d ??\r\n", pps_btime_ms, now_ms);
            //return;
        }
        if (newtime < base_time_ms) {
Serial.printf("PPS=%d < base=%d ??\r\n", pps_btime_ms, base_time_ms);
            //return;
        }
        if (newtime + 200 < ref_time_ms) {
Serial.printf("PPS=%d < ref=%d ??\r\n", pps_btime_ms, ref_time_ms);
            //return;
        }

        // uint32_t last_Commit_Time = now_ms - gnss.time.age();
        if (pps_btime_ms <= last_Commit_Time) {
          time_corr_neg = (last_Commit_Time - pps_btime_ms) % 1000;
        } else {
          time_corr_neg = 1000 - ((pps_btime_ms - last_Commit_Time) % 1000);
        }
        ref_time_ms = base_time_ms = newtime;  // pps_btime_ms + ADJ_FOR_FLARM_RECEPTION

    } else {   /* PPS not available */

        // uint32_t last_RMC_Commit = now_ms - gnss.date.age();
        // time_corr_neg = gnss_chip ? gnss_chip->rmc_ms : 100;
        ref_time_ms = base_time_ms = newtime;
    }

//Serial.println(F("processing new GNSS time..."));

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

    OurTime = makeTime(tm) + (gnss_age + time_corr_neg) / 1000;

#else // !defined(TBEAM)
      //Serial.println(F("no GNSS, cannot set clock"));
#endif   /* TBEAM */
}

void Time_update()
{
    uint32_t now_ms = millis();

    if ((ognrelay_time || ognreverse_time) && ! time_synched
                && when_to_switch > 0 && OurTime > when_to_switch) {
        /* at a per-agreed time after initial time-sync contacts */
        time_synched = true;
        // when_to_switch = 0;
        Serial.printf(">>> time_synched at %d ms\r\n", now_ms);
        String msg = "time_synched";
        Logger_send_udp(&msg);
    }

    if (time_client && (!time_synched || OurTime==0)) {
    /* initial time-sync when relaying time */

        time_synched = false;
        /* use free-running clock while waiting for when_to_switch */
        if (OurTime != 0 && now_ms >= ref_time_ms + 1000) {
          OurTime += 1;
          ref_time_ms += 1000;
//Serial.printf("free running clock - client still waiting to sync -> %d\r\n", OurTime);
        }
        /* else wait for time data from remote station */
        return;
        /* new time packets will be handled by set_our_clock()  */

    }

    if (time_client && time_synched && OurTime > 1000000) {

        /* use free-running clock between time sync messages */
        if (now_ms >= ref_time_ms + 1000) {
          OurTime += 1;
          ref_time_ms += 1000;
//Serial.printf("free running clock - btwn time msgs -> %d\r\n", OurTime);
        }

        return;
    }

    if (!ognrelay_enable && !ognrelay_time && !ogn_gnsstime) {
    /* when base (or standalone) is getting time data from NTP */

        /* get fresh NTP time data periodically */
        if (now_ms > time_to_call_ntp) {
            Poll_NTP();
            return;
        }

        /* use free-running clock between those times */
        if (OurTime > 1000000 && now_ms >= ref_time_ms + 1000) {
            OurTime += 1;
            uint32_t increment;
            if (ntp_ms_adjust > 0) {    // NTP time is later than OurTime, speed up
                --ntp_ms_adjust;
                increment = 999;        // PPS was 1 ms earlier
            } else if (ntp_ms_adjust < 0) {
                ++ntp_ms_adjust;
                increment = 1001;
            } else { // if adjust=0
                increment = 1000;
            }
            ref_time_ms += increment;
//Serial.printf("free running clock - NTP -> %d\r\n", OurTime);
        }

        return;
    } 

    if (ognrelay_enable && ! ognrelay_time && ! ogn_gnsstime) {
    /* when remote station is not using GNSS time */

        if (OurTime == 0) {    /* starting up */
          OurTime = 1;         /* arbitrary time, used for packet aging only */
                               // >>> won't work for packet decrypting!
          ref_time_ms = now_ms;
          time_synched = true;
        }

        /* use free-running clock */
        if (now_ms >= ref_time_ms + 1000) {
          OurTime += 1;
          ref_time_ms += 1000;
        }

        return;
    }

    // none of the above, use GNSS time
    //if (ogn_gnsstime)
        Poll_GNSS();

#if 0
    // continue and do NTP too for comparison test
    if (now_ms > time_to_call_ntp) {
        Poll_NTP();
        //time_to_call_ntp = millis() + 30300;
    }
#endif
}

void Time_loop()
{
    Time_update();

    if (OurTime < 1000000)     // not UTC time (seconds since 1970)
        return;

/*
    // NTP test code
    static uint32_t when_to_NTP = 0;
    if (when_to_NTP == 0) {
        if (OurTime != 0)
            when_to_NTP = millis();
    } else if (millis() > when_to_NTP) {
        when_to_NTP = millis() + 20300;    // 60000;   // once a minute
        Poll_NTP();
    }
*/
    if (ThisAircraft.timestamp != OurTime) {      /* do this only once per second */

      ThisAircraft.timestamp = OurTime;
      int oldmin = ThisAircraft.minute;
#if defined(TBEAM)
      if (ogn_gnsstime && isValidGNSStime()) {
        ThisAircraft.second = gnss.time.second();
        if (ThisAircraft.second == 0)
            ThisAircraft.minute = gnss.time.minute();
        if (ThisAircraft.minute == 0)
            ThisAircraft.hour = gnss.time.hour();
      } else {
        ThisAircraft.second = second(OurTime);
        if (ThisAircraft.second == 0)
            ThisAircraft.minute = minute(OurTime);
        if (ThisAircraft.minute == 0)
            ThisAircraft.hour = hour(OurTime);
      }
#else
      ThisAircraft.second = second(OurTime);
      if (ThisAircraft.second == 0)
          ThisAircraft.minute = minute(OurTime);
      if (ThisAircraft.minute == 0)
          ThisAircraft.hour = hour(OurTime);
#endif

      if (last_hour == 0)
          last_hour = OurTime;                   /* seconds */
      else if (OurTime > last_hour + 3600)
          last_hour = OurTime;

      if (oldmin != ThisAircraft.minute) {

          /* restart the device when uptime is more than 47 days */
          if (millis() > (47 * 24 * 3600 * 1000UL))
              SoC->reset();

          /* minute changed, gather per-minute traffic stats */
          ++uptime;                        /* minutes */ 
//          if (! isValidGNSStime())
//              DebugLogWrite("time_loop: no gnss fix");
          static bool rounded = false;
          uint32_t cumul_traffic, prev_traffic;
          if (ognrelay_base)
            cumul_traffic = remote_traffic;  /* cumulative count of traffic packets */
          else
            cumul_traffic = traffic_packets_recvd;
          prev_traffic = traffic_by_minute[traffic_minute_pointer];
          traffic_by_minute[traffic_minute_pointer] = cumul_traffic;
          ++traffic_minute_pointer;
          if (traffic_minute_pointer >= TRAFFIC_MINUTES) {
            traffic_minute_pointer = 0;
            rounded = true;
          }
          // if (! rounded)   /* cannot yet compute rolling average */
          //  packets_per_minute = cumul_traffic
          //       / (traffic_minute_pointer > 0 ? traffic_minute_pointer : 1);
          // else
          if (rounded && OurTime > last_hour + 60*(TRAFFIC_MINUTES+1)
                      && cumul_traffic > prev_traffic)
            packets_per_minute = (cumul_traffic - prev_traffic) / TRAFFIC_MINUTES;
          else   /* first minutes after (re)start, or no traffic, or rollover */
            packets_per_minute = 0;
      }

#if defined(T3S3) || defined(TTGO)
      if (! ognrelay_enable) {
        if (ognrelay_time || ognreverse_time)
            green_LED(time_synched && (aprs_registred == 2));
        else
            green_LED(NTP_synched && (aprs_registred == 2));
      }
#endif

    }  /* done once-per-second chores */

    /* handle time-sync packets */

    uint32_t now_ms = millis();

    if (!time_master && !time_client)
        return;

    /* attempting initial time-sync from time_master */
    if (time_master && !time_synched && now_ms > when_sync_tried + TIME_TO_TRYSYNC) {
      if ((! ogn_gnsstime && OurTime > 0) || isValidGNSStime())
        (void) send_time();        /* may or may not succeed */
      when_sync_tried = now_ms;    /* even if no success, do not try too often */
      return;
    }

    /* send fresh time data about every 10 seconds */
    if (time_master && time_synched && now_ms > when_sync_tried + TIME_TO_ACKSYNC) {
      if ((! ogn_gnsstime && OurTime > 0) || isValidGNSStime()) {
        if (send_time())             /* success (which also updated when_sync_sent) */
          when_sync_tried = now_ms;
        else
          when_sync_tried += 300;    /* failed - try again in 300 ms */
      }
      /* client will send an ack back, so both stations will update when_synched */
    }

    /* if haven't heard from other station in a while, start over */
    if (time_synched && (now_ms > when_synched + 
         ((sleep_when==0 && remote_sleep_length==0)? TIME_TO_RE_SYNC : 3*TIME_TO_RE_SYNC))) {
       Serial.printf("restarting time sync @ %d ms - syncd @ %d\r\n", now_ms, when_synched);
       OLED_write("restart time sync", 0, 27, true);
       Timesync_restart();
       if (ognrelay_base) {
         remote_sats = 0;
         remote_voltage = 0;
         String msg = "restarting time sync";
         Logger_send_udp(&msg);
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
// other options: ntp.my-isp.net or (fastest) its IP address
// used here: 0.pool.ntp.org, 1.pool.ntp.org, etc - automatically finds close-by servers
const String ntpServerName_suffix = ".pool.ntp.org";

const int NTP_PACKET_SIZE = 48; // NTP time stamp is in the first 48 bytes of the message

byte NTPPacketBuffer[ NTP_PACKET_SIZE]; //buffer to hold incoming and outgoing packets

// A UDP instance to let us send and receive packets over UDP
WiFiUDP NTP_udp;

static uint32_t NTP_response_time = 0;
static uint32_t NTP_request_time  = 0;

// send an NTP request to the time server at the given address
bool sendNTPpacket(IPAddress& address)
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
    size_t r = NTP_udp.write(NTPPacketBuffer, NTP_PACKET_SIZE);
    NTP_udp.endPacket();
    if (r <= 0)
        return false;
    return true;
}

// https://www.lectrobox.com/projects/esp32-ntp/
// https://github.com/jelson/rulos/tree/main/src/lib/chip/esp32/periph/ntp
void sniffer(void *buf, wifi_promiscuous_pkt_type_t type) {
  if (type != WIFI_PKT_DATA) {
    return;
  }
  wifi_promiscuous_pkt_t *p = (wifi_promiscuous_pkt_t *)buf;
  // this is cheesy; in theory we should parse the whole packet and
  // make sure it's UDP, destined for our port number. but this
  // quick-and-dirty check for the right length is a quick hack which
  // works since sniffing is only turned on after we send a request
  if (p->rx_ctrl.sig_len != 130) {
    return;
  }
  //>>> should NTP_response_time be protected with a "mutex"?
  NTP_response_time = millis();
}

void Poll_NTP()
{
    char buf[32];
    int    cb = 0;
    String ntpServerName;

    NTP_synched = false;

    if (millis() < 20000)
        return;   // don't even try

    // Do not attempt to timesync in Soft AP mode
    if (WiFi.getMode() == WIFI_AP) {
        Serial.println(F("Cannot get NTP time without internet"));
        snprintf(buf, sizeof(buf), "no internet no NTP time");
        OLED_write(buf, 0, 54, false);
        time_to_call_ntp = millis() + TIME_TO_NTP_AGAIN;
        return;
    }

    if (WiFi.status() != WL_CONNECTED) {
        Serial.println(F("Cannot get NTP time: WiFi disconnected"));
        time_to_call_ntp = millis() + TIME_TO_NTP_AGAIN;
        return;
    }

    static int state = 0;

    if (state == 0) {   // need to start up UDP

        Serial.println(F("Starting NTP UDP"));
        NTP_udp.begin(localPort);
        Serial.print(F("Local port: "));
        Serial.println(localPort);
        state = 1;
        time_to_call_ntp = millis() + 200;
        return;
    }

    if (state == 1) {   // need to send a request to NTP server

        static int server_selector = 3;
        ++server_selector;
        if (server_selector > 3)
            server_selector = 0;
        //get a random server from the pool
        ntpServerName = String(server_selector) + ntpServerName_suffix;
        Serial.print(F(" NTP server's name: "));
        Serial.println(ntpServerName);
        WiFi.hostByName(ntpServerName.c_str(), timeServerIP);   // cache this?
        Serial.print('#');
        Serial.print(server_selector);
        Serial.print(F(" NTP server's IP address: "));
        Serial.println(timeServerIP);

        // send an NTP packet to a time server
        esp_wifi_set_promiscuous(true);
        NTP_request_time = millis();
        NTP_response_time = 0;    // will be filled in by sniffer()
        if (sendNTPpacket(timeServerIP) == false) { // send failed
            esp_wifi_set_promiscuous(false);
            Serial.println(F("sending to NTP failed"));
            NTP_udp.stop();
            time_to_call_ntp = millis() + 5100;   // try again in 5 seconds
            state = 0;
            return;
        }
        // sending succeeded, wait 100 ms for reply
        state = 2;
        time_to_call_ntp = millis() + 100;
        return;

    }
    
    // else if (state == 2)  // waiting for a reply from NTP server

    uint32_t now_ms = millis();
    cb = NTP_udp.parsePacket();
    if (!cb) {
        if (now_ms > NTP_request_time + 550) {
            esp_wifi_set_promiscuous(false);
            time_to_call_ntp = now_ms + 5100;   // try a new request in 5 seconds
            Serial.println(F("No response from NTP"));
            NTP_udp.stop();
            state = 0;
            return;
        } else {
            time_to_call_ntp = now_ms + 100;   // check again in 100 ms
            // state remains 2
            return;
        }
    } else if (NTP_response_time == 0) {
        esp_wifi_set_promiscuous(false);
        time_to_call_ntp = now_ms + 5100;   // try again in 5 seconds
        Serial.println(F("NTP response not caught by sniffer"));
        NTP_udp.stop();
        state = 0;
        return;
    } else {
        esp_wifi_set_promiscuous(false);
        Serial.print(F("Reply packet received, length="));
        Serial.println(cb);
        // We've received a packet, read the data from it
        NTP_udp.read(NTPPacketBuffer, NTP_PACKET_SIZE); // read the packet into the buffer
        NTP_udp.stop();
        state = 0;
    }

    // ready to set time
    // also prepare for the next NTP poll:
    time_to_call_ntp = now_ms + 30300;   // in case calcs below fail

    //the timestamp starts at byte 40 of the received packet and is four bytes,
    // or two words, long. First, esxtract the two words:
    uint32_t highWord = word(NTPPacketBuffer[40], NTPPacketBuffer[41]);
    uint32_t lowWord  = word(NTPPacketBuffer[42], NTPPacketBuffer[43]);
    // combine the four bytes (two words) into a long integer
    // this is NTP time (seconds since Jan 1 1900):
    uint32_t secsSince1900 = (highWord << 16) | lowWord;

    //the sub-second timestamp starts at byte 44 of the received packet.
    highWord = word(NTPPacketBuffer[44], NTPPacketBuffer[45]);
    lowWord  = word(NTPPacketBuffer[46], NTPPacketBuffer[47]);
    uint32_t frac_sec = (highWord << 16) | lowWord;
    // this is fraction of a second in units of 2^-32 sec
    //Serial.printf("frac_sec raw: %08X\r\n", frac_sec);
    frac_sec = ((((frac_sec >> 16) & 0x0FFFF) * 1000) >> 16);   // milliseconds (T3)
    //Serial.print("frac_sec: ");
    //Serial.println(frac_sec);

    // also need the received-timestamp from the NTP server (T2)
    highWord = word(NTPPacketBuffer[32], NTPPacketBuffer[33]);
    lowWord  = word(NTPPacketBuffer[34], NTPPacketBuffer[35]);
    uint32_t rec_sec = (highWord << 16) | lowWord;
    highWord = word(NTPPacketBuffer[36], NTPPacketBuffer[37]);
    lowWord  = word(NTPPacketBuffer[38], NTPPacketBuffer[39]);
    uint32_t rec_frac = (highWord << 16) | lowWord;
    rec_frac = ((((rec_frac >> 16) & 0x0FFFF) * 1000) >> 16);   // milliseconds (T2)

    uint32_t T1, T2, T3, T4;

    T1 = NTP_request_time;                             // these are millis() values
    T4 = NTP_response_time;
    T2 = (rec_sec       & 0x0FFFF)*1000 + rec_frac;    // different origin
    T3 = (secsSince1900 & 0x0FFFF)*1000 + frac_sec;    // but cancels out
    if (T3 < T2)  // shouldn't happen unless perhaps rolled over through 0xFFFF
        return;
    uint32_t serverlag = (T3 - T2);
    //Serial.print("server lag: ");  Serial.println(serverlag);
    if (T4 < T1) {  // may happen if received a late response to an earlier request
        Serial.printf("T1: %d  >  T4: %d\r\n", T1, T4);
        return;
    }
    if (T4 - T1 < serverlag) {
        Serial.printf("T1: %d   T4: %d  <  serverlag: %d\r\n", T1, T4, serverlag);
        return;
    }
    uint32_t netlag = (T4 - T1) - serverlag;
    Serial.print("net lag: ");
    Serial.println(netlag);

    // Time:= T3 + NetLag/2;
    uint32_t NTP_PPS_ms = T4 - (netlag>>1) - frac_sec;   // what millis() was at beginning of NTP second
  
    // now convert NTP time into everyday time:
    //Serial.print(F("Seconds since Jan 1 1900 = "));
    //Serial.println(secsSince1900);
    // Unix time starts on Jan 1 1970. In seconds, that's 2208988800:
    const uint32_t seventyYears = 2208988800UL;
    // subtract seventy years:
    uint32_t epoch = secsSince1900 - seventyYears;
    //Serial.print(F("Unix time = "));  Serial.println(epoch);
    // compare NTP time with our time
    //Serial.print(F("Our time = "));   Serial.println(OurTime);
    int sec_diff = (int)(epoch & 0x3FFFFFFF) - (int)(OurTime & 0x3FFFFFFF);
    int ms_diff = (int)ref_time_ms - (int)NTP_PPS_ms;   // time now is secs + (now-PPS) ms
    if (sec_diff < 0 && ms_diff > 600) {  // OurTime rolled over to next sec but NTP hasn't yet
        ++sec_diff;
        ms_diff -= 1000;
    } else if (sec_diff > 0 && ms_diff < -600) {    // vice versa
        --sec_diff;
        ms_diff += 1000;
    }
    Serial.printf("NTP time relative to OurTime: %d sec + %d ms\r\n", sec_diff, ms_diff);

#if 0
    // just for testing
    uint32_t pps_btime_ms = SoC->get_PPS_TimeMarker();
    Serial.printf("PPS_TimeMarker = %d   ref_time_ms = %d\r\n", pps_btime_ms, ref_time_ms);
#endif

    if (ogn_gnsstime)  // just testing
        return;

    NTP_synched = true;
    time_to_call_ntp = now_ms + TIME_TO_NTP_AGAIN;

    if (OurTime < 1000000) {        // time not set yet, set it all at once
        OurTime = epoch;
        ref_time_ms = NTP_PPS_ms;
        return;
    }
    // if second is wrong, fix that right away (rarely needed)
    if (sec_diff != 0)
        OurTime = epoch;
    //ref_time_ms -= ms_diff;
    // smooth out the ms changes over time, 1 ms each second
    if (netlag > 80)
        ms_diff /= 4;    // discount timestamps from long netlags, they are inaccurate
    else
        ms_diff /= 2;
    if (ms_diff > 0)     // NTP time is later than OurTime
        ntp_ms_adjust = (ms_diff >  30 ?  30 : ms_diff);
    else if (ms_diff < 0)
        ntp_ms_adjust = (ms_diff < -30 ? -30 : ms_diff);
    Serial.printf("NTP time adjustment: %d ms\r\n", ntp_ms_adjust);
}

#endif /* EXCLUDE_NTP */


void Time_setup()
{
#if 0
    // setup for NTP - up here just for testing, later move to bottom of this function
    const wifi_promiscuous_filter_t filt =
             {.filter_mask = WIFI_PROMIS_FILTER_MASK_DATA};
    esp_wifi_set_promiscuous_filter(&filt);
    esp_wifi_set_promiscuous_rx_cb(&sniffer);    
#endif

    /*
     * only use NTP in base (or single) station
     * and if not getting time from remote station
     * and if not using local GNSS time
     */
    if (ognrelay_enable || ognrelay_time || ogn_gnsstime)
        return;

#if 1
    /* base (or single) station that is not using GNSS time nor relay time */
    /*   - i.e., it needs to get the UTC time from NTP */
    // set up what's needed for NTP polling
    const wifi_promiscuous_filter_t filt =
             {.filter_mask = WIFI_PROMIS_FILTER_MASK_DATA};
    esp_wifi_set_promiscuous_filter(&filt);
    esp_wifi_set_promiscuous_rx_cb(&sniffer);        
#endif
}
