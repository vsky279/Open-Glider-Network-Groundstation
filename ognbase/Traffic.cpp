/*
 * Traffic.cpp
 * Copyright (C) 2018-2020 Linar Yusupov
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
    int8_t rval     = ALARM_LEVEL_NONE;
    int    alt_diff = (int) (fop->altitude - this_aircraft->altitude);

    if (abs(alt_diff) < VERTICAL_SEPARATION) /* no warnings if too high or too low */

    /* Subtract 2D velocity vector of traffic from 2D velocity vector of this aircraft */
    {
        float V_rel_x = this_aircraft->speed * cosf(radians(90.0 - this_aircraft->course)) -
                        fop->speed * cosf(radians(90.0 - fop->course));
        float V_rel_y = this_aircraft->speed * sinf(radians(90.0 - this_aircraft->course)) -
                        fop->speed * sinf(radians(90.0 - fop->course));

        float V_rel_magnitude = sqrtf(V_rel_x * V_rel_x + V_rel_y * V_rel_y) * _GPS_MPS_PER_KNOT;
        float V_rel_direction = atan2f(V_rel_y, V_rel_x) * 180.0 / PI; /* -180 ... 180 */

        /* convert from math angle into course relative to north */
        V_rel_direction = (V_rel_direction <= 90.0 ? 90.0 - V_rel_direction :
                           450.0 - V_rel_direction);

        /* +- 10 degrees tolerance for collision course */
        if (V_rel_magnitude > 0.1 && fabs(V_rel_direction - fop->bearing) < 10.0)
        {
            /* time is seconds prior to impact */
            float t = fop->distance / V_rel_magnitude;

            /* time limit values are compliant with FLARM data port specs */
            if (t < 9.0)
                rval = ALARM_LEVEL_URGENT;
            else if (t < 13.0)
                rval = ALARM_LEVEL_IMPORTANT;
            else if (t < 19.0)
                rval = ALARM_LEVEL_LOW;
        }
    }
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

uint8_t relay_interval(void)
{
    if (numtracked < 2*MIN_RELAY_INTERVAL)  return MIN_RELAY_INTERVAL;
    return ((numtracked + 1) / 2);
}

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

    /* memset(fo.raw, 0, sizeof(fo.raw)); */
    memcpy(fo.raw, RxBuffer, rx_size);

    if (ognrelay_time) {

      if (time_sync_pkt(fo.raw)) {

        if (ognrelay_base)
            set_our_clock(fo.raw);    /* may set time_synched */

        if (ognrelay_enable)
            sync_alive_pkt(fo.raw);   /* may set time_synched */

        return;
      }

      /* until time-synched any received packets other than time-sync are ignored */
      if (!time_synched)
        return;
    }

    /* otherwise fall through to process normal traffic packets */

    ++traffic_packets_recvd;

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

//Serial.println("parsing decoded packet...");

        /* timenow = now(); */
        timenow = OurTime;              /* based on local or relayed GNSS time */

        if (! ognrelay_enable) {
            fo.timestamp = timenow;
            fo.hour = ThisAircraft.hour;
            fo.minute = ThisAircraft.minute;
            fo.second = ThisAircraft.second;
            calc_distance(&fo);
        }

        fo.rssi = RF_last_rssi;

        /* if an already-tracked aircraft, update and report */
        /* - but if reported recently, ignore for now */

        for (i=0; i < MAX_TRACKING_OBJECTS; i++) {
            cip = &Container[i];
            if (cip->addr == fo.addr) {               /* this fo already tracked */
              if (ognrelay_enable) {
                  fo.timestamp = cip->timestamp;      /* last time it was relayed */
                  if (timenow >= fo.timestamp + (time_t) relay_interval())
                      fo.waiting = true;              /* time to relay this one again */
              } else {   /* base station */
                  fo.prevtime = cip->timestamp;      /* keep some data from previous packet...   */
                  fo.prevlat = cip->latitude;        /* ... in order to check data in PVALID.cpp */
                  fo.prevlon = cip->longitude;
                  fo.waiting = true;                 /* waiting to be reported to OGN */
//Serial.printf("[%d] received again\r\n", i);
              }
              *cip = fo;                      /* copy the whole struct, including new timestamp */
              return;
            }
        }

        /* new aircraft, not in the Container[] array yet */

        fo.waiting = true;
        if (ognrelay_enable)
            fo.timestamp = timenow - relay_interval();      /* to be relayed */

        /* put it in an empty slot if available */

        for (i=0; i < MAX_TRACKING_OBJECTS; i++) {
            if (Container[i].addr == 0) {
                Container[i] = fo;
//if (ognrelay_base)
//Serial.printf("[%d] new\r\n", i);
                ++numtracked;
                return;
            }
        }

        /* no empty slot, replace the oldest */

        oldest = 0;
        oldest_age = 0;
        for (i=0; i < MAX_TRACKING_OBJECTS; i++) {
            cip = &Container[i];
            age = (int) (timenow - cip->timestamp);
            if (! cip->waiting)                /* treat waiting as younger */
                age += ENTRY_EXPIRATION_TIME;
            if (cip->stealth)
                age += ENTRY_EXPIRATION_TIME;
            if (cip->no_track)
                age += 2 * ENTRY_EXPIRATION_TIME;
            if (age > oldest_age) {
                oldest_age = age;
                oldest = i;
            }
        }

        if (oldest_age < ENTRY_EXPIRATION_TIME && fo.stealth)
            return;   /* drop it */

        if (oldest_age < 2*ENTRY_EXPIRATION_TIME && fo.no_track)
            return;   /* drop it */

        Container[oldest] = fo;        /* overwrites */
//if (ognrelay_base)
//Serial.printf("[%d] overwritten\r\n", oldest);
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

/* >>> this assumes "Legacy" protocol, should be encapsulated into Legacy.cpp */
void Traffic_Relay(ufo_t* fop)
{
    legacy_packet_t* pkt = (legacy_packet_t *) fop->raw;

    if (pkt->_unk0 > 0) {  /* received relayed, or time, or special FLARM packet - do not relay */
      --traffic_packets_recvd;
      if (pkt->_unk0 >= 0xC)  ++bad_packets_recvd;  /* don't count special FLARM packets as "bad" */
      return;
    }

    if (settings->no_track || settings->stealth || ogn_itrackbit || ogn_istealthbit) {
        fop->waiting = false;
        fop->timestamp = OurTime;
        ++traffic_packets_relayed;      /* pretend relaying - for testing */
        return;
    }

    pkt->_unk0 = 0xE;      /* marks relayed packets */

    memcpy(TxBuffer, fop->raw, LEGACY_PAYLOAD_SIZE);

    if (RF_Transmit(LEGACY_PAYLOAD_SIZE, true)) {   /* success transmitting */
        fop->waiting = false;
        /* set time stamp for next update of same aircraft */
        fop->timestamp = OurTime;
        ++traffic_packets_relayed;
    }
}

void Traffic_loop()
{
    int i, waiting, oldest_waiting;
    time_t timenow, age, oldest_age;
    ufo_t* cip;

    timenow = OurTime;              /* based on GNSS time */
    numtracked = 0;
    waiting = 0;
    oldest_waiting = 0;
    oldest_age = 0;
    for (i=0; i < MAX_TRACKING_OBJECTS; i++) {
        cip = &Container[i];
        if (cip->addr) {
            ++numtracked;                     /* count tracked objects */
            age = timenow - cip->timestamp;
            if (cip->waiting) {
                ++waiting;
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
      if (oldest_age > 0)
        Traffic_Relay(&Container[oldest_waiting]);
    } else {
        static uint32_t traffic_msg_time = 0;
        if (millis() > traffic_msg_time + 2000) {
            Serial.printf("%d recvd, %d trkd, %d waiting, oldest: %d\r\n",
               traffic_packets_recvd, numtracked, waiting, oldest_age);
            traffic_msg_time = millis();
        }
    }
}

void ClearExpired()
{
    time_t timenow = OurTime;
    numtracked = 0;
    for (int i=0; i < MAX_TRACKING_OBJECTS; i++) {
        if (Container[i].addr) {
          if ((timenow - Container[i].timestamp) > ENTRY_EXPIRATION_TIME)
              Container[i] = EmptyFO;
          else
              ++numtracked;
        }
    }
}
