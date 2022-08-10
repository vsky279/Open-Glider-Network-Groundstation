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
#include "Battery.h"
#include "global.h"

#include <TimeLib.h>

#if defined(TBEAM)
extern const gnss_chip_ops_t *gnss_chip;
#endif

time_t   OurTime = 0;          /* UTC time in seconds since start of 1970 */
uint16_t have_approx_time = 0;
uint32_t base_time_ms = 0;     /* this device millis() at last verified PPS */
uint32_t ref_time_ms = 0;      /* assumed local millis() at last PPS */

#define TIME_TO_TRYSYNC 2700
#define TIME_TO_ACKSYNC 10300
#define TIME_TO_RE_SYNC 185000
#define ADJ_FOR_FLARM_RECEPTION 40       /* seems to receive FLARM packets better */
#define ADJ_FOR_TRANSMISSION_DELAY 10
#define TIME_TO_NTP_AGAIN 600000

uint16_t uptime = 0;
time_t last_hour = 0;

uint32_t when_synched = 0;
uint32_t when_sync_tried = 0;
uint32_t when_sync_sent = 0;
uint32_t when_to_switch = 0;
bool time_synched = false;
bool got_time_ack = false;
bool reverse_time_sync = false;

uint32_t remote_traffic = 0, remote_other=0;
uint16_t remote_uptime=0;
uint16_t remote_timesent=0, remote_bad=0;
uint8_t remote_pctrel=0, remote_ack=0, remote_restarts=0, remote_round=0;
uint8_t remote_sats=0;
float remote_voltage=0.0;

uint32_t traffic_packets_recvd = 0;
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

/* this is called by both remote & base stations, to */
/*  send time data, and to acknowledge, respectively */
bool send_time(void)
{
    uint32_t satellites = 0;
    uint8_t mark;

    if (! RF_TX_ready()) {   /* transmitting not allowed at this time */
//Serial.printf("cannot send time right now %d\r\n", millis());
        return false;
    }
    
    if (ognrelay_base && reverse_time_sync) {

        mark = 0xA;     /* marks reverse-time-sync packet */

    } else {

        if (! ognrelay_time)
          return false;

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
        mark = (got_time_ack ? 0xE : 0xC);
    }

    legacy_packet_t* pkt = (legacy_packet_t *) TxBuffer;
    pkt->addr = 0xACACAC;            /* marks time-relay packets */
    pkt->_unk0 = mark;               /* type of such packet */

    uint32_t* p = ( uint32_t *) TxBuffer; 

    uint32_t now_ms = millis();
    uint32_t offset = now_ms - ref_time_ms;
    if (offset > 1500) {    /* shouldn't happen */
Serial.printf("send_time: ref_time_ms %d << now %d ??\r\n", ref_time_ms, now_ms);
        return false;
    }

    if (ognrelay_enable) {

      p[1] = (uint32_t) OurTime;
      p[2] = (offset & 0x0FFF);

      /* send stats */
      p[2] |= (satellites << 12);
      p[2] |= ((other_packets_recvd & 0x01FFE0) << 11);
      remote_voltage = Battery_voltage();
      int volts = (int)(10.0 * remote_voltage + 0.5) - 28;
      if (volts < 0)  volts = 0;
      if (volts > 15) volts = 15;
      p[2] |= ((volts & 0x0F) << 28);
      // pkt->addr_type = (((total_delays/(ack_packets_recvd+1)) >> 2) & 0x07);  /* 3 bits: avg roundtrip ms / 4 */
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

    } else if (ognrelay_base) {   /* base station sending ack - blank stats fields */

      if (time_synched || reverse_time_sync) {
          /* send base time to remote station (NTP time if reverse_time_sync) */
          p[1] = (uint32_t) OurTime;
          p[2] = (offset & 0x0FFF);
      } else {
          /* base chooses when to switch to synched-time & tells the remote station */
          p[1] = (got_time_ack? (uint32_t) when_to_switch : 0);
          p[2] = p[1];   /* marks it as a time-to-switch */
      }
      p[3] = 0xACACACAC;
      p[4] = 0xACACACAC;

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
        Serial.println(F("sent time & stats..."));
//Serial.printf("send_time: ms=%d, ourt=%d, ofst=%d, reft=%d\r\n", now_ms, OurTime, offset, ref_time_ms);
        got_time_ack = false;    /* check for ack again for each time-data packet */
      } else if (reverse_time_sync) {
          Serial.println(F("base sent time to remote"));
      } else {
          Serial.println(F("sent time ack"));
//Serial.printf("send_time_ack: ms=%d, ourt=%d, reft=%d\r\n", now_ms, OurTime, ref_time_ms);
      }
    }
//else
//Serial.println(F("send_time unsuccessful"));

    return success;
}

/* just check whether the packet is a time-sync one */
bool time_sync_pkt(uint8_t *raw)
{
    if (! (ognrelay_time || (ognrelay_enable && reverse_time_sync)))
        return false;
    legacy_packet_t* pkt = (legacy_packet_t *) raw;
    if (pkt->addr != 0xACACAC)     /* marks time-sync packets */
        return false;
    if (pkt->_unk0 == 0xE)         /* marks acked time-relay packets */
        return true;
    if (pkt->_unk0 == 0xC)         /* marks un-acked time-relay packets */
        return true;
    if (pkt->_unk0 == 0xA)         /* marks reverse-time-relay packets */
        return true;
    return false;
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
Serial.println(F("received time pkt failed check"));
      return;                                /* failed security check */
    }

    OurTime = (time_t) p[1];
    uint32_t timeOffset = (p[2] & 0x0FFF);
    timeOffset += ADJ_FOR_TRANSMISSION_DELAY;
    if (timeOffset > 1000) {
      OurTime += 1;
      timeOffset -= 1000;
    }
    when_synched = millis();
    ref_time_ms = when_synched - timeOffset;   /* time of last PPS in remote station */

Serial.printf("set_our_clock: ms=%d, ourt=%d, ofst=%d, reft=%d\r\n",
              when_synched, OurTime, timeOffset, ref_time_ms);

    legacy_packet_t* pkt = (legacy_packet_t *) raw;
    if (pkt->_unk0 == 0xE)
        got_time_ack = true;    /* for base, this "sticks" */

    /* get stats sent from remote station */
    p[3] ^= 0xACACACAC;
    p[4] ^= 0xACACACAC;
    remote_sats = ((p[2]>>12) & 0x0F);
    remote_traffic = (p[3] & 0x01FFFFFF);
    remote_pctrel = (p[3]>>25) & 0x7F;
    remote_timesent = (p[4] & 0x0000FFFF);
    remote_ack = (p[4]>>14) & 0x7C;
    remote_bad = (p[4]>>18) & 0x1F8;
    remote_other = (p[2]>>11) & 0x01FFE0;
    remote_voltage = (float)(((p[2]>>28) & 0x0F) + 28) * 0.1;
    remote_restarts = (p[4]>>27) & 0x1F;
    // remote_round = (pkt->addr_type <<2);
    uint16_t rem_upt;
    if (pkt->_unk1 == 0)
        rem_upt = (pkt->addr_type & 0x07) * 10;
    else if (pkt->addr_type < 4)   // (& 0x04 == 0)
        rem_upt = pkt->addr_type * 60 + 60;
    else
        rem_upt = (pkt->addr_type - 4) * 120 + 300;  // (& 0x03) * 120 + 300;
    if (rem_upt > remote_uptime)
        remote_uptime = rem_upt;
    else if (rem_upt < remote_uptime)
        Serial.printf("reported uptime %d (%d %d) but stored uptime %d?\r\n",
            rem_upt, pkt->_unk1, pkt->addr_type, remote_uptime);

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
    else
        Serial.printf("Remote battery voltage: %.1f\r\n", remote_voltage);
    /* these stats also displayed in the statistics web page */
    // should also send these stats out via UDP.

    if (got_time_ack && when_to_switch == 0) {
        when_to_switch = ((OurTime + 24) & 0xFFFFFFF0) + 8;   /* 16-32 seconds in the future */
        Serial.printf(">>> will switch to time_synched at %d\r\n", when_to_switch);
    }       

    delay(40);
    (void) send_time();      /* may not succeed */
}

/* this is called by remote station to process incoming ack packets */
void sync_alive_pkt(uint8_t *raw)
{
    if (! ognrelay_enable)  return;

    legacy_packet_t* pkt = (legacy_packet_t *) raw;

    uint32_t* p = (uint32_t *) raw; 
    p[1] ^= 0xACACACAC;                         /* undo rudimentary whitening */
    p[2] ^= 0xACACACAC;
    if (p[5] != TimeHash((p[1] ^ p[2]) ^ ognrelay_key)) {
      Serial.println(F("received timesync ack failed check"));
      ++bad_packets_recvd;                      /* this statistic collected within remote */
      return;                                   /* failed security check */
    }

    when_synched = millis();

    /* if relay has no time source, base sends NTP time (only with 1 or 2 channels) */
    if (reverse_time_sync && pkt->_unk0 == 0xA) {

        OurTime = (time_t) p[1];
        have_approx_time = (TIME_TO_RE_SYNC/1000);  /* decrements once per second unless reset here */
        Serial.println(F("received NTP time from base"));
        return;

    } else if (ognrelay_time && time_synched && p[1] != p[2]) {

        time_t BaseTime = (time_t) p[1];
        uint32_t BaseOffset = (p[2] & 0x0FFF);
        BaseOffset += ADJ_FOR_TRANSMISSION_DELAY + 40;
        if (BaseOffset > 1000) {
          BaseTime += 1;
          BaseOffset -= 1000;
        }
        uint32_t base_ref_time = when_synched - BaseOffset;
        if (BaseTime == OurTime - 1) {
            ++BaseTime;
            base_ref_time += 1000;
        }
        if (BaseTime == OurTime) {
            int32_t timediff = (int32_t)base_ref_time - (int32_t)ref_time_ms;
            Serial.printf("base-remote timediff: %d ms\r\n", timediff);
        } else if (when_synched - when_sync_sent < 1000) {
            Serial.printf("base time: %d != remote time: %d ??\r\n", BaseTime, OurTime);
        }

    }

    uint32_t delay = when_synched - when_sync_sent;
    if (delay < 1000) {
        ++ack_packets_recvd;
        total_delays += delay;
        Serial.printf("received timesync ack: ms=%d, time=%d, ofst=%d, avg round %d ms\r\n",
           when_synched, p[1], p[2], total_delays/ack_packets_recvd);
    }

    bool base_ack = (pkt->_unk0 == 0xE);        /* base says it got ack-ack */
    if (! time_synched && base_ack && p[1]==p[2]
          && (p[1] > OurTime + 8) && (p[1] < OurTime + 34)) {
        when_to_switch = p[1];                  /* base chose switch time */
        Serial.printf(">>> time_synched switch base chose %d\r\n", when_to_switch);
    }
    if (when_to_switch == 0) {
        /* even without ack-ack, choose a switch time */
        when_to_switch = ((OurTime + 24) & 0xFFFFFFF0) + 8;   /* 16-32 seconds in the future */
        Serial.printf(">>> will switch to time_synched at %d\r\n", when_to_switch);
    }
    if (! base_ack && when_to_switch > 0 && (OurTime - when_to_switch < 8)) {
        /* without ack-ack, postpone switch time if base still not ready */
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
    return success;
}

/* check whether the packet is telling this remote station to reboot */
bool maybe_remote_reboot(uint8_t *raw) {
    if (! ognrelay_enable)      return false;
    legacy_packet_t* pkt = (legacy_packet_t *) raw;
    if (pkt->_unk0 != 0xA)      return false;
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
            Serial.println("rebooting in 3 sec...");
            delay(3000);
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
    ++sync_restarts;
}

/********************* time loop code ************************/

void Time_update()
{
    uint32_t now_ms = millis();

    if (ognrelay_time && ! time_synched && when_to_switch > 0 && OurTime > when_to_switch) {
        /* at a per-agreed time after initial time-sync contacts */
        time_synched = true;
        // when_to_switch = 0;
        Serial.printf(">>> time_synched at %d ms\r\n", now_ms);
        String msg = "time_synched";
        Logger_send_udp(&msg);
    }

    if (ognrelay_base && ognrelay_time && (!time_synched || OurTime==0)) {
    /* initial time-sync when relaying time */

        time_synched = false;
        /* use free-running clock while waiting for when_to_switch */
        if (OurTime > 0 && now_ms >= ref_time_ms + 1000) {
          OurTime += 1;
          ref_time_ms += 1000;
//Serial.printf("free running clock - base still waiting to sync -> %d\r\n", OurTime);
        }
        /* else wait for time data from remote station */
        return;
        /* new time packets will be handled by set_our_clock()  */

    }

    if (ognrelay_base && ognrelay_time && time_synched && OurTime != 0) {
    /* when base is getting time data periodically from remote station */

        /* use free-running clock between time sync messages */
        if (now_ms >= ref_time_ms + 1000) {
          OurTime += 1;
          ref_time_ms += 1000;
//Serial.printf("free running clock - base btwn time msgs-> %d\r\n", OurTime);
        }

        return;
    }

    if (!ognrelay_enable && !ognrelay_time && !ogn_gnsstime) {
    /* when base (or standalone) is getting time data from NTP */

        if (OurTime == 0 || ! time_synched) {
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
//Serial.printf("free running clock - NTP -> %d\r\n", OurTime);
        }

        return;
    } 

    if (ognrelay_enable && ! ognrelay_time && ! ogn_gnsstime) {
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
          if (have_approx_time == 0) {
//Serial.println(F("free running clock - remote, no gnss"));
          } else {
              --have_approx_time;
//Serial.println(F("have_approx_time - remote, no gnss"));
          }
        }

        return;
    } 

#if defined(TBEAM)

    bool validgnss = isValidGNSStime();

    if ((!ognrelay_base || !ognrelay_time) && OurTime != 0
          && (now_ms < base_time_ms + (validgnss? 1500 : TIME_TO_RE_SYNC))) {

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
#endif

#if !defined(TBEAM)
      //Serial.println(F("should use TBEAM with GNSS"));
#endif

#if defined(TBEAM)

    if (! validgnss) {      /* even after free-running for a while */
        if (ognrelay_enable && ognrelay_time) {
            // OurTime = 0;
            if (time_synched)
                Timesync_restart();
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
    }
    if (gnss_age < 2500 && newtime > base_time_ms) {
      static uint32_t lasttime_ms = 0;
      if (last_Commit_Time - lasttime_ms > 150) {     /* new data arrived from GNSS */
        newfix = true;
        lasttime_ms = last_Commit_Time;
      }
    }

    /* between fixes (but not before first fix): free-running clock */
    if (ref_time_ms > 0 && !newfix) {
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

#endif   /* TBEAM */
}

void Time_loop()
{
    Time_update();

    if (OurTime == 0)
        return;

    if (ThisAircraft.timestamp != OurTime) {      /* do this only once per second */

      if (! time_synched && ! ognrelay_time && ogn_gnsstime && ref_time_ms > 0)
            /* if using local (non-relay) GNSS time */
            time_synched = true;

      ThisAircraft.timestamp = OurTime;
      int oldmin = ThisAircraft.minute;
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

      if (last_hour == 0)
          last_hour = OurTime;                   /* seconds */
      else if (OurTime > last_hour + 3600)
          last_hour = OurTime;

      if (oldmin != ThisAircraft.minute) {
          /* minute changed, gather per-minute traffic stats */
          ++uptime;                        /* minutes */ 
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

    }  /* done once-per-second chores */

    /* handle time-sync packets */

    uint32_t now_ms = millis();

    if ((ognrelay_enable || ognrelay_base) && ognrelay_time) {

       /* attempting initial time-sync from remote station */
       if (ognrelay_enable && !time_synched
             && now_ms > when_sync_tried + TIME_TO_TRYSYNC) {
         if (isValidGNSStime())
           (void) send_time();        /* may or may not succeed */
         when_sync_tried = now_ms;    /* even if no success, do not try too often */
         return;
       }

       /* send fresh time data about every 10 seconds */
       if (ognrelay_enable && time_synched
             && now_ms > when_sync_tried + TIME_TO_ACKSYNC) {
         if (isValidGNSStime()) {
           if (send_time())             /* success (which also updated when_sync_sent) */
             when_sync_tried = now_ms;
           else
             when_sync_tried += 300;    /* failed - try again in 300 ms */
         }
         /* base sation will send an ack back, so both stations will update when_synched */
       }

       /* if haven't heard from other station in a while, start over */
       if (time_synched && (now_ms > when_synched + TIME_TO_RE_SYNC)) {
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
              /* done with ognrelay_time */

    /* if base gets NTP time & relay has no time source, send approximate time */
    } else if (ognrelay_base && reverse_time_sync && OurTime > 0
                && now_ms > when_sync_tried + TIME_TO_ACKSYNC) {
        (void) send_time();
        when_sync_tried = now_ms;
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

    /*
     * only use NTP in base station
     * and if not getting time from remote station
     * and if not using local GNSS time
     */
    if (ognrelay_time || ogn_gnsstime)
        return;

    if (ogn_band == RF_BAND_EU  || ogn_band > RF_BAND_AU)
          reverse_time_sync = true;

    if (ognrelay_enable)
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
