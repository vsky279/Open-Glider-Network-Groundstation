/*
 * Traffic.cpp
 * Copyright (C) 2018-2020 Linar Yusupov
 *
 * Traffic-relay prioritization code by Moshe Braner, 2022
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

#include "Traffic.h"
#include "EEPROM.h"
#include "RF.h"
#include "Time.h"
#include "GNSS.h"
#include "Web.h"
#include "Protocol_Legacy.h"
#include "OLED.h"
#include "global.h"
#include "Log.h"

#include <math.h>


uint32_t UpdateTrafficTimeMarker = 0;

ufo_t fo, Container[MAX_TRACKING_OBJECTS], EmptyFO;

static int8_t (* Alarm_Level)(ufo_t *, ufo_t *);

/*
 * No any alarms issued by the firmware.
 * Rely upon high-level flight management software.
 */
static int8_t Alarm_None(ufo_t* this_aircraft, ufo_t* fop)
{
    return ALARM_LEVEL_NONE;
}

/*
 * Simple, distance based alarm level assignment.
 */
static int8_t Alarm_Distance(ufo_t* this_aircraft, ufo_t* fop)
{
    int    distance = (int) fop->distance;
    int8_t rval     = ALARM_LEVEL_NONE;
    int    alt_diff = (int) (fop->altitude - this_aircraft->altitude);

    if (abs(alt_diff) < VERTICAL_SEPARATION) /* no warnings if too high or too low */
    {
        if (distance < ALARM_ZONE_URGENT)
            rval = ALARM_LEVEL_URGENT;
        else if (distance < ALARM_ZONE_IMPORTANT)
            rval = ALARM_LEVEL_IMPORTANT;
        else if (distance < ALARM_ZONE_LOW)
            rval = ALARM_LEVEL_LOW;
    }

    return rval;
}

/*
 * EXPERIMENTAL
 *
 * Linear, CoG and GS based collision prediction.
 */
static int8_t Alarm_Vector(ufo_t* this_aircraft, ufo_t* fop)
{
    int8_t rval = ALARM_LEVEL_NONE;

    /* TBD */

    return rval;
}

/*
 * "Legacy" method is based on short history of 2D velocity vectors (NS/EW)
 */
static int8_t Alarm_Legacy(ufo_t* this_aircraft, ufo_t* fop)
{
    int8_t rval = ALARM_LEVEL_NONE;

    /* TBD */

    return rval;
}

uint8_t numtracked = 0;

/* the following two CAN be different although for now are the same: */
#define relay_interval()   ((time_t)(numtracked <= 4 ? 4 : numtracked))
#define report_interval()  ((time_t)(numtracked <= 4 ? 4 : numtracked))

void calc_distance(ufo_t* fop)
{
//  fop->distance = gnss.distanceBetween(ThisAircraft.latitude,
//                                       ThisAircraft.longitude,
//                                       fop->latitude,
//                                       fop->longitude);
  static float coslat = 0;
  if (coslat == 0)  coslat = cosf(ThisAircraft.latitude * 0.0174533);
  float dy = 111300.0 * (fop->latitude - ThisAircraft.latitude);     /* meters */
  float dx = 111300.0 * (fop->longitude - ThisAircraft.longitude) * coslat;
  fop->distance = sqrtf(dx*dx + dy*dy);
}

void ParseData()
{
    size_t rx_size = RF_Payload_Size(ogn_protocol_1);
    if (rx_size > sizeof(fo.raw)) rx_size = sizeof(fo.raw);

    // fo = EmptyFO;   /* all zeros */
    // memset(fo.raw, 0, sizeof(fo.raw));
    memcpy(fo.raw, RxBuffer, rx_size);

    if (ognrelay_time) {

      if (time_sync_pkt(fo.raw)) {

        if (ognrelay_base)
            set_our_clock(fo.raw);    /* may set time_synched */

        if (ognrelay_enable)
            sync_alive_pkt(fo.raw);   /* may set time_synched */

        return;
      }
    }

    if (ognrelay_enable) {
        if (maybe_remote_reboot(fo.raw))
            /* a reboot packet, but did not reboot */
            return;
        /* else, it was not a remote-reset packet */
        /* fall through to process as a traffic packet */
    }

    /* until time-synched any received packets other than time-sync are ignored */
    if (OurTime == 0 || ! time_synched)
        return;

    /* otherwise fall through to process normal traffic packets */

//    ++traffic_packets_recvd;

    /*

    char buf[16];
    String msg;

    msg = "RXBuffer: ";

    Serial.print("RxBuffer: ");
    
    for(int i=0;i<rx_size;i++){
      Serial.print(RxBuffer[i], HEX); 
      msg += String(RxBuffer[i], HEX);
    }    

    Serial.println();
    Logger_send_udp(&msg);

    */

    if (protocol_decode && (*protocol_decode)((void *) fo.raw, &ThisAircraft, &fo))
    {
        int i, age, oldest, oldest_age;
        time_t timenow;
        ufo_t* cip;

        ++traffic_packets_recvd;

//Serial.println("parsing decoded packet...");

        /* timenow = now(); */
        timenow = OurTime;
        // fo.timestamp = timenow;    /* timestamp (and H,M,S) was set in legacy_decode() */

        if (! ognrelay_enable) {
            calc_distance(&fo);
            if (fo.distance /*m*/ > 1200 * ogn_range /*km*/)
                return;                            /* too far, or perhaps corrupted data */
        }

        /* if an already-tracked aircraft, update and report */
        /* - but if reported recently, ignore for now */

        for (i=0; i < MAX_TRACKING_OBJECTS; i++) {

            cip = &Container[i];
            if (cip->addr == fo.addr) {               /* this fo already tracked */

                 /* ignore duplicate FLARM packets (there are many!) */
                 if (fo.altitude == cip->altitude &&
                     fo.latitude == cip->latitude &&
                     fo.longitude == cip->longitude)
                               return;

              if (ognrelay_enable) {  /* remote station */

                  fo.timereported = cip->timereported;   /* last time it was relayed */
                  fo.waiting = (timenow >= fo.timereported + relay_interval());

              } else {  /* base station */

                  fo.prevtime = cip->timestamp;      /* keep some data from previous packet...   */
                  fo.prevlat = cip->latitude;        /* ... in order to check data in PVALID.cpp */
                  fo.prevlon = cip->longitude;
                  fo.timereported = cip->timereported;  /* last time it was reported to OGN */
                  fo.waiting = (timenow >= fo.timereported + report_interval());
//Serial.printf("[%d] received again\r\n", i);

              }
              *cip = fo;                      /* copy the whole struct, including new timestamp */
              return;
            }
        }

        /* new aircraft, not currently in the Container[] array */

        fo.prevtime = 0;              /* no previous data */
        fo.waiting = true;
        fo.timereported = 0;          /* never been relayed or reported */

        /* put it in an empty slot if available */
        /* if no empty slot, replace the oldest */

        oldest = 0;
        oldest_age = 0;
        for (i=0; i < MAX_TRACKING_OBJECTS; i++) {
            cip = &Container[i];
            if (cip->addr == 0) {                     /* empty slot */
                *cip = fo;
//if (ognrelay_base)
//Serial.printf("[%d] new\r\n", i);
                ++numtracked;
                return;
            }
            age = (int) (timenow - cip->timestamp);   /* not ->timereported */
            if (age < ENTRY_EXPIRATION_TIME && fo.timereported == 0)  /* not previously reported */
                age = 0;                              /* do not overwrite */
            if (! cip->waiting)                       /* treat non-waiting as older */
                age += ENTRY_EXPIRATION_TIME;
            if (! (ognrelay_enable && ! ogn_gnsstime)) {    /* remote could decode these fields */
                if (cip->stealth)
                    age += ENTRY_EXPIRATION_TIME;     /* treat stealth as older yet */
                if (cip->no_track)
                    age += 2 * ENTRY_EXPIRATION_TIME; /* treat no_track as very old */
            }
            if (age > oldest_age) {
                oldest_age = age;
                oldest = i;
            }
        }

        if (! (ognrelay_enable && ! ogn_gnsstime)) {    /* remote could decode these fields */
            if (oldest_age < ENTRY_EXPIRATION_TIME && fo.stealth)
                return;   /* drop the new traffic instead of replacing old traffic */
            if (oldest_age < 2*ENTRY_EXPIRATION_TIME && fo.no_track)
                return;   /* drop it */
        }

        if (oldest_age > 0) {
            Container[oldest] = fo;        /* overwrites older data */
//if (ognrelay_base)
//Serial.printf("[%d] overwritten\r\n", oldest);
        }
    }
}

void Traffic_setup()
{
    switch (settings->alarm)
    {
        case TRAFFIC_ALARM_NONE:
            Alarm_Level = &Alarm_None;
            break;
        case TRAFFIC_ALARM_VECTOR:
            Alarm_Level = &Alarm_Vector;
            break;
        case TRAFFIC_ALARM_LEGACY:
            Alarm_Level = &Alarm_Legacy;
            break;
        case TRAFFIC_ALARM_DISTANCE:
        default:
            Alarm_Level = &Alarm_Distance;
            break;
    }
}

void Traffic_Relay(ufo_t* fop)
{
    if (settings->no_track || settings->stealth || ogn_itrackbit || ogn_istealthbit) {
        fop->waiting = false;
        fop->timereported = OurTime;
        ++traffic_packets_relayed;      /* pretend relaying - for testing */
        return;
    }

    size_t size = relay_encode((void*) &fop->raw, fop);
    if (size > 0) {
      memcpy(TxBuffer, fop->raw, size);
      if (RF_Transmit(size, true)) {   /* success transmitting */
          fop->waiting = false;
          /* set time stamp for next update of same aircraft */
          fop->timereported = OurTime;
          ++traffic_packets_relayed;
      }
    }
}

void Traffic_loop()
{
    int i, waiting, oldest_waiting, age, oldest_age;
    time_t timenow;
    ufo_t* cip;

    timenow = OurTime;              /* based on GNSS time */
    numtracked = 0;
    waiting = 0;
    oldest_waiting = 0;
    oldest_age = -1;
    for (i=0; i < MAX_TRACKING_OBJECTS; i++) {
        cip = &Container[i];
        if (cip->addr) {
            ++numtracked;                      /* count tracked objects */
            if (cip->waiting) {
                ++waiting;                     /* count objects waiting to be reported */
                age = (int) (timenow - cip->timereported);   /* not ->timestamp */
                if (age > oldest_age) {
                   oldest_age = age;
                   oldest_waiting = i;
                }
            }
//if (ognrelay_base)
//Serial.printf("[%d] iswaiting: %d, age: %d\r\n", i, cip->waiting, age);
        }
    }
    if (ognrelay_enable) {
      if (oldest_age >= 0)
        Traffic_Relay(&Container[oldest_waiting]);
    }
    static uint32_t traffic_msg_time = 0;
    if (millis() > traffic_msg_time + 2000) {
        Serial.printf("%d recvd, %d trkd, %d waiting, oldest: %d\r\n",
              traffic_packets_recvd, numtracked, waiting, oldest_age);
        traffic_msg_time = millis();
    }
}

void ClearExpired()
{
    time_t timenow = OurTime;
    numtracked = 0;
    for (int i=0; i < MAX_TRACKING_OBJECTS; i++) {
        if (Container[i].addr) {
          if ((timenow - Container[i].timestamp) > ENTRY_EXPIRATION_TIME)
              Container[i].addr = 0;
          else
              ++numtracked;
        }
    }
}
