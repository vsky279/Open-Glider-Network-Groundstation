/*
 * RF.cpp
 * Copyright (C) 2016-2020 Linar Yusupov
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
#if defined(ARDUINO)
#include <SPI.h>
#endif /* ARDUINO */

#include "RF.h"
#include "Protocol_Legacy.h"
#include "Protocol_OGNTP.h"
#include "Protocol_P3I.h"
#include "Protocol_FANET.h"
#include "Protocol_UAT978.h"
#include "SoC.h"
#include "EEPROM.h"
#include "Web.h"
#include "global.h"
#include "Log.h"
#include "GNSS.h"
#include "PNET.h"
#include <fec.h>

#if LOGGER_IS_ENABLED
#include "LogHelper.h"
#endif /* LOGGER_IS_ENABLED */

byte RxBuffer[MAX_PKT_SIZE] __attribute__((aligned(sizeof(uint32_t))));

unsigned long TxTimeMarker = 0;
byte TxBuffer[MAX_PKT_SIZE] __attribute__((aligned(sizeof(uint32_t))));

uint32_t tx_packets_counter = 0;
uint32_t rx_packets_counter = 0;

int8_t RF_last_rssi = 0;

FreqPlan    RF_FreqPlan;
static bool RF_ready = false;

static size_t RF_tx_size    = 0;
// static long   TxRandomValue = 0;

const rfchip_ops_t *rf_chip = NULL;
bool RF_SX12XX_RST_is_connected = true;

size_t (* protocol_encode)(void *, ufo_t *);
bool   (* protocol_decode)(void *, ufo_t *, ufo_t *);

static Slots_descr_t Time_Slots, *ts;
static uint8_t       RF_timing = RF_TIMING_INTERVAL;

#if defined(TBEAM)
extern const gnss_chip_ops_t *gnss_chip;
#endif

static bool sx1276_probe(void);

static bool sx1262_probe(void);

static void sx12xx_setup(void);

static void sx12xx_channel(uint8_t);
static bool sx12xx_receive(void);

static void sx12xx_transmit(void);

static void sx12xx_shutdown(void);


const rfchip_ops_t sx1276_ops = {
    RF_IC_SX1276,
    "SX1276",
    sx1276_probe,
    sx12xx_setup,
    sx12xx_channel,
    sx12xx_receive,
    sx12xx_transmit,
    sx12xx_shutdown
};
#if defined(USE_BASICMAC)
const rfchip_ops_t sx1262_ops = {
    RF_IC_SX1262,
    "SX1262",
    sx1262_probe,
    sx12xx_setup,
    sx12xx_channel,
    sx12xx_receive,
    sx12xx_transmit,
    sx12xx_shutdown
};
#endif /* USE_BASICMAC */

String Bin2Hex(byte* buffer, size_t size)
{
    String str = "";
    for (int i=0; i < size; i++) {
        byte c = buffer[i];
        str += (c < 0x10 ? "0" : "") + String(c, HEX);
    }
    return str;
}

uint8_t parity(uint32_t x)
{
    uint8_t parity=0;
    while (x > 0) {
        if (x & 0x1)
            parity++;
        x >>= 1;
    }
    return parity % 2;
}

byte RF_setup(void)
{
    if (rf_chip == NULL)
    {
        if (sx1276_ops.probe())
        {
            rf_chip = &sx1276_ops;
            SX12XX_LL = &sx127x_ll_ops;
        }
        else if (sx1262_ops.probe())
        {
            rf_chip   = &sx1262_ops;
            SX12XX_LL = &sx126x_ll_ops;
        }

        if (rf_chip && rf_chip->name)
        {
            Serial.print(rf_chip->name);
            Serial.println(F(" RFIC is detected."));
        }
        else
            Serial.println(F("WARNING! None of supported RFICs is detected!"));
    }

    RF_FreqPlan.setPlan(ogn_band);

    if (rf_chip)
    {
        rf_chip->setup();

        const rf_proto_desc_t *p;

        switch (ogn_protocol_1)
        {
          case RF_PROTOCOL_OGNTP:     p = &ogntp_proto_desc;  break;
          case RF_PROTOCOL_P3I:       p = &p3i_proto_desc;    break;
          case RF_PROTOCOL_FANET:     p = &fanet_proto_desc;  break;
          case RF_PROTOCOL_ADSB_UAT:  p = &uat978_proto_desc; break;
          case RF_PROTOCOL_LEGACY:
          default:                    p = &legacy_proto_desc; break;
        }

        RF_timing         = p->tm_type;

        ts                = &Time_Slots;
        ts->air_time      = p->air_time;
        ts->interval_min  = p->tx_interval_min;
        ts->interval_max  = p->tx_interval_max;
        ts->interval_mid  = (p->tx_interval_max + p->tx_interval_min) / 2;
        ts->s0.begin      = p->slot0.begin;
        ts->s1.begin      = p->slot1.begin;
        ts->s0.duration   = p->slot0.end - p->slot0.begin;
        ts->s1.duration   = p->slot1.end - p->slot1.begin;

        uint16_t duration = ts->s0.duration + ts->s1.duration;
        ts->adj = duration > ts->interval_mid ? 0 : (ts->interval_mid - duration) / 2;

        return rf_chip->type;
    }
    else
        return RF_IC_NONE;
}

extern int status_LED;  // LEDHelper

#define FOR_RX 0
#define FOR_TX 1

#define TIME_TO_TRYSYNC 2700
#define TIME_TO_ACKSYNC 15000
#define TIME_TO_RE_SYNC 65000
#define ADJ_FOR_TRANSMISSION_DELAY 30

bool time_synched = false;
uint32_t when_synched = 0;
uint32_t when_sync_sent = 0;
time_t  Time;
uint32_t ref_time_ms;

/* borrow the hash function used in freqplan.h */
uint32_t TimeHash(uint32_t Time) {
     Time  = (Time<<15) + (~Time);
     Time ^= Time>>12;
     Time += Time<<2;
     Time ^= Time>>4;
     Time *= 2057;
     return Time ^ (Time>>16);
}

bool send_time(void)
{
    if (! ognrelay_time)
      return false;
#if defined(TBEAM)
    if (ognrelay_enable && (! isValidGNSStime()))   /* time data source needs GPS time */
      return false;
#else
    if (ognrelay_enable)   /* relay station must have GPS */
      return false;
#endif
    legacy_packet_t* pkt = (legacy_packet_t *) TxBuffer;
    pkt->addr = 0xAEAEAE;  /* marks time-sync packets */
    pkt->_unk0 = 0xE;      /* marks relay packets */
    uint32_t* p = ( uint32_t *) TxBuffer; 
    p[1] = (uint32_t) Time;
    p[2] = (uint32_t) (millis() - ref_time_ms);
    /* also send hashed combination of time and relay ID for error check & security */
    /* >>> note: the same secret ognrelay_key needs to be in config of both stations */
    p[3] = TimeHash((Time ^ p[2]) ^ ognrelay_key);
    p[4] = p[5] = 0xAEAEAEAE;
    size_t size = LEGACY_PAYLOAD_SIZE;
    bool wait = false;
    bool success = RF_Transmit(size, wait);
    if (success)
      when_sync_sent = millis();
    return success;
}


void set_our_clock(uint8_t *raw)
{
    if (! ognrelay_base)  return;
    if (! ognrelay_time)  return;
    legacy_packet_t* pkt = (legacy_packet_t *) raw;
    if (pkt->_unk0 != 0xE)      return;    /* marks relay packets */
    if (pkt->addr != 0xAEAEAE)  return;    /* marks time-sync packets */
    uint32_t* p = (uint32_t *) raw; 
    uint32_t RxTime = p[1];
    uint32_t RxOffset = p[2];
    if (p[3] != TimeHash((RxTime ^ RxOffset) ^ ognrelay_key))
      return;                              /* failed security check */
    Time = (time_t) RxTime;
    RxOffset += ADJ_FOR_TRANSMISSION_DELAY;  /* ms */
    if (RxOffset >= 1000) {
      Time += 1;
      RxOffset -= 1000;
    }
    when_synched = millis();
    ref_time_ms = when_synched - RxOffset;
    if (send_time())           /* sent an ack successfully */
      time_synched = true;
}

bool sync_alive_pkt(uint8_t *raw)
{
    if (! ognrelay_time)        return false;
    legacy_packet_t* pkt = (legacy_packet_t *) raw;
    if (pkt->_unk0 != 0xE)      return false;    /* marks relay packets */
    if (pkt->addr != 0xAEAEAE)  return false;    /* marks time-sync packets */
    uint32_t* p = (uint32_t *) raw; 
    uint32_t RxTime = p[1];
    uint32_t RxOffset = p[2];
    if (p[3] != TimeHash((RxTime ^ RxOffset) ^ ognrelay_key))
      return false;                              /* failed security check */
    time_synched = true;
    when_synched = millis();
    return true;
}

void RF_SetChannel(int tx=0)
{
    tmElements_t tm;
    uint8_t Slot;
    unsigned long pps_btime_ms;

    /* until time is synched use channel 0 to communicate between relay ends  */
    if ((ognrelay_enable || ognrelay_base) && ognrelay_time && !time_synched) {
      if (RF_ready && rf_chip)
          rf_chip->channel(0);
      return;
    }

    /* >>> assume base is getting time data from remote station */
    if (ognrelay_base && ognrelay_time && time_synched) {

    /* use free-running clock between time sync messages */
    if (millis() >= ref_time_ms + 1000) {
      Time += 1;
      ref_time_ms += 1000;
    }

    } else {    /* use GPS time data */

    switch (settings->mode)
    {
        case SOFTRF_MODE_GROUND:
        default:

#if defined(TBEAM)

// restored SoftRF code here, may want to modify later.

    pps_btime_ms = SoC->get_PPS_TimeMarker();
    unsigned long time_corr_neg;

    if (pps_btime_ms) {
      unsigned long last_Commit_Time = millis() - gnss.time.age();
      if (pps_btime_ms <= last_Commit_Time) {
        time_corr_neg = (last_Commit_Time - pps_btime_ms) % 1000;
      } else {
        time_corr_neg = 1000 - ((pps_btime_ms - last_Commit_Time) % 1000);
      }
      ref_time_ms = pps_btime_ms;
    } else {
      unsigned long last_RMC_Commit = millis() - gnss.date.age();
      time_corr_neg = gnss_chip ? gnss_chip->rmc_ms : 100;
      ref_time_ms = last_RMC_Commit - time_corr_neg;
    }

    int yr    = gnss.date.year();
    if( yr > 99)
        yr    = yr - 1970;
    else
        yr    += 30;
    tm.Year   = yr;
    tm.Month  = gnss.date.month();
    tm.Day    = gnss.date.day();
    tm.Hour   = gnss.time.hour();
    tm.Minute = gnss.time.minute();
    tm.Second = gnss.time.second();

    Time = makeTime(tm) + (gnss.time.age() - time_corr_neg) / 1000;

#else
      /* base station and not ognrelay_time - should use TBEAM */
      if (RF_ready && rf_chip)
          rf_chip->channel(0);
      return;
#endif

    break;
  }

  }  /* end of if not (ognrelay_base && time_synched) */

  switch (RF_timing)
  {
  case RF_TIMING_2SLOTS_PPS_SYNC:
    if ((millis() - ts->s0.tmarker) >= ts->interval_mid) {
      ts->s0.tmarker = ref_time_ms + ts->s0.begin - ts->adj;
      ts->current = 0;
    }
    if ((millis() - ts->s1.tmarker) >= ts->interval_mid) {
      ts->s1.tmarker = ref_time_ms + ts->s1.begin;
      ts->current = 1;
    }
    Slot = ts->current;
    break;
  case RF_TIMING_INTERVAL:
  default:
    Slot = 0;
    break;
  }

    /* use OGNTP frequencies for relay, otherwise FLARM frequencies */

    uint8_t OGN = 0;
    if (ognrelay_enable && tx)
        OGN = 1;
    else if (ognrelay_base && !tx)
        OGN = 1;

    uint8_t chan = RF_FreqPlan.getChannel(Time, Slot, OGN);

    // HOP Testing - time and channel
    //Serial.printf("Time: %d, %d\r\n", Time,chan);
#if DEBUG
    Serial.print("GSP Fix: ");
    Serial.println(isValidFix());
    Serial.print("Plan: ");
    Serial.println(RF_FreqPlan.Plan);
    Serial.print("Slot: ");
    Serial.println(Slot);
    Serial.print("OGN: ");
    Serial.println(OGN);
    Serial.print("Channel: ");
    Serial.println(chan);
#endif
 
    if (RF_ready && rf_chip)
      rf_chip->channel(chan);
}

void RF_loop()
{
    if (!RF_ready)
    {
        if (RF_FreqPlan.Plan == RF_BAND_AUTO)
        {
            if (ThisAircraft.latitude || ThisAircraft.longitude)
            {
                RF_FreqPlan.setPlan((int32_t)(ThisAircraft.latitude  * 600000),
                                    (int32_t)(ThisAircraft.longitude * 600000));
                RF_ready = true;
            }
        }
        else
            RF_ready = true;
    }

    if ((ognrelay_enable || ognrelay_base) && ognrelay_time) {

       if (ognrelay_enable && !time_synched && millis() - when_sync_sent > TIME_TO_TRYSYNC) {
         when_sync_sent = millis();   /* even if no success, do not try too often */
         (void) send_time();
         return;
       }

       /* send an ack every 15 seconds so other station will know synch is OK */
       if (time_synched && millis() - when_sync_sent > TIME_TO_ACKSYNC) {
         if (send_time())   /* success (which also updated when_sync_sent) */
            return;
       }

       /* if haven't heard from other station in a while, start over */
       if (time_synched && millis() - when_synched > TIME_TO_RE_SYNC)
          time_synched = false;
    }
}

size_t RF_Encode(ufo_t* fop)
{
    size_t size = 0;
    if (RF_ready && protocol_encode)
    {
        if (settings->txpower == RF_TX_POWER_OFF)
            return size;

      if (millis() > TxTimeMarker) {
        size = (*protocol_encode)((void *) &TxBuffer[0], fop);
      }
    }
    return size;
}

bool RF_Transmit(size_t size, bool wait)
{
    if (RF_ready && rf_chip && (size > 0))
    {
        RF_tx_size = size;

        if (settings->txpower == RF_TX_POWER_OFF)
            return true;

        if (!wait || millis() > TxTimeMarker) {

            // time_t timestamp = now();

            RF_SetChannel(FOR_TX);

            rf_chip->transmit();

            tx_packets_counter++;
            RF_tx_size = 0;

            RF_SetChannel(FOR_RX);

/* restored SoftRF code for TxTimeMarker: */

      Slot_descr_t *next;
      unsigned long adj;

      switch (RF_timing)
      {
      case RF_TIMING_2SLOTS_PPS_SYNC:
        next = RF_FreqPlan.Channels == 1 ? &(ts->s0) :
               ts->current          == 1 ? &(ts->s0) : &(ts->s1);
        adj  = ts->current ? ts->adj   : 0;
        TxTimeMarker = next->tmarker    +
                       ts->interval_mid +
                       SoC->random(adj, next->duration - ts->air_time);
        break;
      case RF_TIMING_INTERVAL:
      default:
        TxTimeMarker = millis() + SoC->random(ts->interval_min, ts->interval_max) - ts->air_time;
        break;
      }

            return true;
        }
    }
    return false;
}


bool RF_Receive(void)
{
    bool rval = false;

    if (RF_ready && rf_chip) {
        RF_SetChannel(FOR_RX);
        rval = rf_chip->receive();
    }
    return rval;
}

void RF_Shutdown(void)
{
    if (rf_chip)
        rf_chip->shutdown();
}

uint8_t RF_Payload_Size(uint8_t protocol)
{
    switch (protocol)
    {
        case RF_PROTOCOL_LEGACY:
            return legacy_proto_desc.payload_size;
        case RF_PROTOCOL_OGNTP:
            return ogntp_proto_desc.payload_size;
        case RF_PROTOCOL_P3I:
            return p3i_proto_desc.payload_size;
        case RF_PROTOCOL_FANET:
            return fanet_proto_desc.payload_size;
        case RF_PROTOCOL_ADSB_UAT:
            return uat978_proto_desc.payload_size;
        default:
            return 0;
    }
}


osjob_t sx12xx_txjob;
osjob_t sx12xx_timeoutjob;

static void sx12xx_tx_func(osjob_t* job);

static void sx12xx_rx_func(osjob_t* job);

static void sx12xx_rx(osjobcb_t func);

static bool sx12xx_receive_complete  = false;
bool        sx12xx_receive_active    = false;
static bool sx12xx_transmit_complete = false;

static uint8_t sx12xx_channel_prev = (uint8_t) -1;

#if defined(USE_BASICMAC)
void os_getDevEui(u1_t* buf)
{ }
u1_t os_getRegion(void)
{
    return REGCODE_EU868;
}

#else
#if !defined(DISABLE_INVERT_IQ_ON_RX)
#error This example requires DISABLE_INVERT_IQ_ON_RX to be set. Update \
    config.h in the lmic library to set it.
#endif
#endif

#define SX1276_RegVersion          0x42 // common

static u1_t sx1276_readReg(u1_t addr)
{
#if defined(USE_BASICMAC)
    hal_spi_select(1);
#else
    hal_pin_nss(0);
#endif
    hal_spi(addr & 0x7F);
    u1_t val = hal_spi(0x00);
#if defined(USE_BASICMAC)
    hal_spi_select(0);
#else
    hal_pin_nss(1);
#endif
    return val;
}

static bool sx1276_probe()
{
    u1_t v, v_reset;

    SoC->SPI_begin();

    hal_init_softrf(nullptr);

    // manually reset radio
    hal_pin_rst(0);                              // drive RST pin low
    hal_waitUntil(os_getTime() + ms2osticks(1)); // wait >100us

    v_reset = sx1276_readReg(SX1276_RegVersion);

    hal_pin_rst(2);                              // configure RST pin floating!
    hal_waitUntil(os_getTime() + ms2osticks(5)); // wait 5ms

    v = sx1276_readReg(SX1276_RegVersion);

    pinMode(lmic_pins.nss, INPUT);
    SPI.end();

    if (v == 0x12)
    {
        if (v_reset == 0x12)
            RF_SX12XX_RST_is_connected = false;

        return true;
    }
    else
        return false;
}

#if defined(USE_BASICMAC)

#define CMD_READREGISTER                0x1D
#define REG_LORASYNCWORDLSB         0x0741
#define SX126X_DEF_LORASYNCWORDLSB  0x24

static void sx1262_ReadRegs(uint16_t addr, uint8_t* data, uint8_t len)
{
    hal_spi_select(1);
    hal_pin_busy_wait();
    hal_spi(CMD_READREGISTER);
    hal_spi(addr >> 8);
    hal_spi(addr);
    hal_spi(0x00); // NOP
    for (uint8_t i = 0; i < len; i++)
        data[i] = hal_spi(0x00);
    hal_spi_select(0);
}

static uint8_t sx1262_ReadReg(uint16_t addr)
{
    uint8_t val;
    sx1262_ReadRegs(addr, &val, 1);
    return val;
}

static bool sx1262_probe()
{
    u1_t v, v_reset;

    SoC->SPI_begin();

    hal_init_softrf(nullptr);

    // manually reset radio
    hal_pin_rst(0);                              // drive RST pin low
    hal_waitUntil(os_getTime() + ms2osticks(1)); // wait >100us

    v_reset = sx1262_ReadReg(REG_LORASYNCWORDLSB);

    hal_pin_rst(2);                              // configure RST pin floating!
    hal_waitUntil(os_getTime() + ms2osticks(5)); // wait 5ms

    v = sx1262_ReadReg(REG_LORASYNCWORDLSB);

    pinMode(lmic_pins.nss, INPUT);
    SPI.end();

    u1_t fanet_sw_lsb = ((fanet_proto_desc.syncword[0]  & 0x0F) << 4) | 0x04;
    if (v == SX126X_DEF_LORASYNCWORDLSB || v == fanet_sw_lsb)
    {
        if (v_reset == SX126X_DEF_LORASYNCWORDLSB || v == fanet_sw_lsb)
            RF_SX12XX_RST_is_connected = false;

        return true;
    }
    else
        return false;
}

#endif

static void sx12xx_channel(uint8_t channel)
{
    if (channel != sx12xx_channel_prev)
    {
        uint32_t frequency = RF_FreqPlan.getChanFrequency(channel);
        int8_t   fc        = settings->freq_corr;

        //Serial.print("frequency: "); Serial.println(frequency);

        if (sx12xx_receive_active)
        {
            os_radio(RADIO_RST);
            sx12xx_receive_active = false;
        }

        if (rf_chip->type == RF_IC_SX1276)
        {
            /* correction of not more than 30 kHz is allowed */
            if (fc > 30)
                fc = 30;
            else if (fc < -30)
                fc = -30;
            ;
        }
        else
            /* Most of SX1262 designs use TCXO */
            fc = 0;

        /* Actual RF chip's channel registers will be updated before each Tx or Rx session */
        LMIC.freq = frequency + (fc * 1000);
        //LMIC.freq = 868200000UL;

        sx12xx_channel_prev = channel;
    }
}

static void sx12xx_setup()
{
    SoC->SPI_begin();

    // initialize runtime env
    os_init(nullptr);

    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();


    // range test.
    LMIC.agcref = 0x00;

    switch (ogn_protocol_1)
    {
        case RF_PROTOCOL_OGNTP:
            LMIC.protocol   = &ogntp_proto_desc;
            protocol_encode = &ogntp_encode;
            protocol_decode = &ogntp_decode;
            break;
        case RF_PROTOCOL_P3I:
            LMIC.protocol   = &p3i_proto_desc;
            protocol_encode = &p3i_encode;
            protocol_decode = &p3i_decode;
            break;
        case RF_PROTOCOL_FANET:
            LMIC.protocol   = &fanet_proto_desc;
            protocol_encode = &fanet_encode;
            protocol_decode = &fanet_decode;
            break;
        case RF_PROTOCOL_LEGACY:
        default:
            LMIC.protocol   = &legacy_proto_desc;
            protocol_encode = &legacy_encode;
            protocol_decode = &legacy_decode;
            /*
             * Enforce legacy protocol setting for SX1276
             * if other value (UAT) left in EEPROM from other (UATM) radio
             */
            ogn_protocol_1 = RF_PROTOCOL_LEGACY;
            break;
    }

    switch (settings->txpower)
    {
        case RF_TX_POWER_FULL:

            /* Load regional max. EIRP at first */
            LMIC.txpow = RF_FreqPlan.MaxTxPower;

            if (rf_chip->type == RF_IC_SX1262)
            {
                /* SX1262 is unable to give more than 22 dBm */
                if (LMIC.txpow > 22)
                    LMIC.txpow = 22;
            }
            else
            /* SX1276 is unable to give more than 20 dBm */
            if (LMIC.txpow > 20)
                LMIC.txpow = 20;

#if 1
            /*
             * Enforce Tx power limit until confirmation
             * that RFM95W is doing well
             * when antenna is not connected
             */
            if (LMIC.txpow > 17)
                LMIC.txpow = 17;
#endif
            break;
        case RF_TX_POWER_OFF:
        case RF_TX_POWER_LOW:
        default:
            LMIC.txpow = 2; /* 2 dBm is minimum for RFM95W on PA_BOOST pin */
            break;
    }
}

static void sx12xx_setvars()
{
    if (LMIC.protocol && LMIC.protocol->modulation_type == RF_MODULATION_TYPE_LORA)
    {
        LMIC.datarate = LMIC.protocol->bitrate;
        LMIC.syncword = LMIC.protocol->syncword[0];
    }
    else
        LMIC.datarate = DR_FSK;

#if defined(USE_BASICMAC)

#define updr2rps  LMIC_updr2rps

    // LMIC.rps = MAKERPS(sf, BW250, CR_4_5, 0, 0);

    LMIC.noRXIQinversion = true;
    LMIC.rxsyms          = 100;

#endif /* USE_BASICMAC */

    // This sets CR 4/5, BW125 (except for DR_SF7B, which uses BW250)
    LMIC.rps = updr2rps(LMIC.datarate);


    if (LMIC.protocol && LMIC.protocol->type == RF_PROTOCOL_FANET)
        /* for only a few nodes around, increase the coding rate to ensure a more robust transmission */
        LMIC.rps = setCr(LMIC.rps, CR_4_8);

    // set SX1276 AGC Reference
    /*if(settings->sxlna){
       LMIC.agcref = 0x13;
       // Serial.printf("setting agc ref to 0x%x\n", 0x13);
       }
       else{
       LMIC.agcref = 0x00;
       //Serial.printf("setting agc ref to 0x%x\n", 0x00);
       }*/
}

static bool sx12xx_receive()
{
    bool success = false;
    String msg;

    sx12xx_receive_complete = false;

    if (!sx12xx_receive_active)
    {
        msg = "activating receive...";
        Logger_send_udp(&msg);    
        sx12xx_setvars();
        sx12xx_rx(sx12xx_rx_func);
        sx12xx_receive_active = true;
    }

    if (sx12xx_receive_complete == false){
        // execute scheduled jobs and events        
        os_runstep();
    }

    if (sx12xx_receive_complete == true)
    {
        u1_t size = LMIC.dataLen - LMIC.protocol->payload_offset - LMIC.protocol->crc_size;

        if (size > sizeof(RxBuffer))
            size = sizeof(RxBuffer);

        for (u1_t i=0; i < size; i++){
            RxBuffer[i] = LMIC.frame[i + LMIC.protocol->payload_offset];
        }

        RF_last_rssi = LMIC.rssi;

        msg = "Receive complete...";
        Logger_send_udp(&msg);    

        /*decrypt payload for private network*/
        /*if packet is bigger , maybe its encryptedr*/
        /*V0.1.0-25*/
        if(false)
          if(size > RF_Payload_Size(ogn_protocol_1)){
            char *decrypted;
            size_t decrypted_len;
            switch (ogn_protocol_1)
              {
               case RF_PROTOCOL_FANET:
                    PNETdecrypt(RxBuffer, size, &decrypted, &decrypted_len);
                    if(decrypted_len == RF_Payload_Size(ogn_protocol_1)){      
                      for(size_t i=0; i<decrypted_len;i++){
                        RxBuffer[i] = decrypted[i];
                          }
                        }
                    break;
               case RF_PROTOCOL_LEGACY:
               case RF_PROTOCOL_P3I:
               case RF_PROTOCOL_OGNTP:
                    break;
              }
            free(decrypted);        
          }
        
        rx_packets_counter++;
        success = true;
    }
    return success;
}

static void sx12xx_transmit()
{
    sx12xx_transmit_complete = false;
    sx12xx_receive_active    = false;

    sx12xx_setvars();
    os_setCallback(&sx12xx_txjob, sx12xx_tx_func);

    while (sx12xx_transmit_complete == false) {
        // execute scheduled jobs and events
        os_runstep();

        yield();
    };
}

static void sx12xx_shutdown()
{
    LMIC_shutdown();
    SPI.end();
}

// Enable rx mode and call func when a packet is received
static void sx12xx_rx(osjobcb_t func)
{
    LMIC.osjob.func = func;
    LMIC.rxtime     = os_getTime(); // RX _now_
    // Enable "continuous" RX for LoRa only (e.g. without a timeout,
    // still stops after receiving a packet)
    os_radio(LMIC.protocol &&
             LMIC.protocol->modulation_type == RF_MODULATION_TYPE_LORA ?
             RADIO_RXON : RADIO_RX);
    //Serial.println("RX");
}

static void sx12xx_rx_func(osjob_t* job)
{
    u1_t crc8, pkt_crc8;
    u2_t crc16, pkt_crc16;
    u1_t i;

    // SX1276 is in SLEEP after IRQ handler, Force it to enter RX mode
    sx12xx_receive_active = false;

    /* FANET (LoRa) LMIC IRQ handler may deliver empty packets here when CRC is invalid. */
    if (LMIC.dataLen == 0)
        return;

    switch (LMIC.protocol->crc_type)
    {
        case RF_CHECKSUM_TYPE_GALLAGER:
        case RF_CHECKSUM_TYPE_NONE:
            /* crc16 left not initialized */
            break;
        case RF_CHECKSUM_TYPE_CRC8_107:
            crc8 = 0x71; /* seed value */
            break;
        case RF_CHECKSUM_TYPE_CCITT_0000:
            crc16 = 0x0000; /* seed value */
            break;
        case RF_CHECKSUM_TYPE_CCITT_FFFF:
        default:
            crc16 = 0xffff; /* seed value */
            break;
    }

    //Serial.print("Got ");
    //Serial.print(LMIC.dataLen);
    //Serial.println(" bytes");

    switch (LMIC.protocol->type)
    {
        case RF_PROTOCOL_LEGACY:
            /* take in account NRF905/FLARM "address" bytes */
            crc16 = update_crc_ccitt(crc16, 0x31);
            crc16 = update_crc_ccitt(crc16, 0xFA);
            crc16 = update_crc_ccitt(crc16, 0xB6);
            break;
        case RF_PROTOCOL_P3I:
        case RF_PROTOCOL_OGNTP:
        default:
            break;
    }

    for (i = LMIC.protocol->payload_offset;
         i < (LMIC.dataLen - LMIC.protocol->crc_size);
         i++)
    {
        switch (LMIC.protocol->crc_type)
        {
            case RF_CHECKSUM_TYPE_GALLAGER:
            case RF_CHECKSUM_TYPE_NONE:
                break;
            case RF_CHECKSUM_TYPE_CRC8_107:
                update_crc8(&crc8, (u1_t)(LMIC.frame[i]));
                break;
            case RF_CHECKSUM_TYPE_CCITT_FFFF:
            case RF_CHECKSUM_TYPE_CCITT_0000:
            default:
                crc16 = update_crc_ccitt(crc16, (u1_t)(LMIC.frame[i]));
                break;
        }

        switch (LMIC.protocol->whitening)
        {
            case RF_WHITENING_NICERF:
                LMIC.frame[i] ^= pgm_read_byte(&whitening_pattern[i - LMIC.protocol->payload_offset]);
                break;
            case RF_WHITENING_MANCHESTER:
            case RF_WHITENING_NONE:
            default:
                break;
        }
#if DEBUG
        Serial.printf("%02x", (u1_t)(LMIC.frame[i]));
#endif
    }

    switch (LMIC.protocol->crc_type)
    {
        case RF_CHECKSUM_TYPE_NONE:
            sx12xx_receive_complete = true;
            break;
        case RF_CHECKSUM_TYPE_GALLAGER:
            if (LDPC_Check((uint8_t *) &LMIC.frame[0]))
            {
#if DEBUG
                Serial.printf(" %02x%02x%02x%02x%02x%02x is wrong FEC",
                              LMIC.frame[i], LMIC.frame[i + 1], LMIC.frame[i + 2],
                              LMIC.frame[i + 3], LMIC.frame[i + 4], LMIC.frame[i + 5]);
#endif
                sx12xx_receive_complete = false;
            }
            else
                sx12xx_receive_complete = true;
            break;
        case RF_CHECKSUM_TYPE_CRC8_107:
            pkt_crc8 = LMIC.frame[i];
#if DEBUG
            if (crc8 == pkt_crc8)
                Serial.printf(" %02x is valid crc", pkt_crc8);
            else
                Serial.printf(" %02x is wrong crc", pkt_crc8);

#endif
            if (crc8 == pkt_crc8)
                sx12xx_receive_complete = true;
            else
                sx12xx_receive_complete = false;
            break;
        case RF_CHECKSUM_TYPE_CCITT_FFFF:
        case RF_CHECKSUM_TYPE_CCITT_0000:
        default:
            pkt_crc16 = (LMIC.frame[i] << 8 | LMIC.frame[i + 1]);
#if DEBUG
            if (crc16 == pkt_crc16)
                Serial.printf(" %04x is valid crc", pkt_crc16);
            else
                Serial.printf(" %04x is wrong crc", pkt_crc16);

#endif
            if (crc16 == pkt_crc16)
                sx12xx_receive_complete = true;
            else
                sx12xx_receive_complete = false;
            break;
    }

#if DEBUG
    Serial.println();
#endif
}

// Transmit the given string and call the given function afterwards
static void sx12xx_tx(unsigned char* buf, size_t size, osjobcb_t func)
{
    u1_t crc8;
    u2_t crc16;


    switch (LMIC.protocol->crc_type)
    {
        case RF_CHECKSUM_TYPE_GALLAGER:
        case RF_CHECKSUM_TYPE_NONE:
            /* crc16 left not initialized */
            break;
        case RF_CHECKSUM_TYPE_CRC8_107:
            crc8 = 0x71; /* seed value */
            break;
        case RF_CHECKSUM_TYPE_CCITT_0000:
            crc16 = 0x0000; /* seed value */
            break;
        case RF_CHECKSUM_TYPE_CCITT_FFFF:
        default:
            crc16 = 0xffff; /* seed value */
            break;
    }

    os_radio(RADIO_RST); // Stop RX first
    delay(1);            // Wait a bit, without this os_radio below asserts, apparently because the state hasn't changed yet

    LMIC.dataLen = 0;

    switch (LMIC.protocol->type)
    {
        case RF_PROTOCOL_LEGACY:
            /* take in account NRF905/FLARM "address" bytes */
            crc16 = update_crc_ccitt(crc16, 0x31);
            crc16 = update_crc_ccitt(crc16, 0xFA);
            crc16 = update_crc_ccitt(crc16, 0xB6);
            break;
        case RF_PROTOCOL_P3I:
            /* insert Net ID */
            LMIC.frame[LMIC.dataLen++] = (u1_t) ((LMIC.protocol->net_id >> 24) & 0x000000FF);
            LMIC.frame[LMIC.dataLen++] = (u1_t) ((LMIC.protocol->net_id >> 16) & 0x000000FF);
            LMIC.frame[LMIC.dataLen++] = (u1_t) ((LMIC.protocol->net_id >>  8) & 0x000000FF);
            LMIC.frame[LMIC.dataLen++] = (u1_t) ((LMIC.protocol->net_id >>  0) & 0x000000FF);
            /* insert byte with payload size */
            LMIC.frame[LMIC.dataLen++] = LMIC.protocol->payload_size;

            /* insert byte with CRC-8 seed value when necessary */
            if (LMIC.protocol->crc_type == RF_CHECKSUM_TYPE_CRC8_107)
                LMIC.frame[LMIC.dataLen++] = crc8;

            break;
        case RF_PROTOCOL_OGNTP:
        default:
            break;
    }

    for (u1_t i=0; i < size; i++) {
        switch (LMIC.protocol->whitening)
        {
            case RF_WHITENING_NICERF:
                LMIC.frame[LMIC.dataLen] = buf[i] ^ pgm_read_byte(&whitening_pattern[i]);
                break;
            case RF_WHITENING_MANCHESTER:
            case RF_WHITENING_NONE:
            default:
                LMIC.frame[LMIC.dataLen] = buf[i];
                break;
        }

        switch (LMIC.protocol->crc_type)
        {
            case RF_CHECKSUM_TYPE_GALLAGER:
            case RF_CHECKSUM_TYPE_NONE:
                break;
            case RF_CHECKSUM_TYPE_CRC8_107:
                update_crc8(&crc8, (u1_t)(LMIC.frame[LMIC.dataLen]));
                break;
            case RF_CHECKSUM_TYPE_CCITT_FFFF:
            case RF_CHECKSUM_TYPE_CCITT_0000:
            default:
                crc16 = update_crc_ccitt(crc16, (u1_t)(LMIC.frame[LMIC.dataLen]));
                break;
        }

        LMIC.dataLen++;
    }

    switch (LMIC.protocol->crc_type)
    {
        case RF_CHECKSUM_TYPE_GALLAGER:
        case RF_CHECKSUM_TYPE_NONE:
            break;
        case RF_CHECKSUM_TYPE_CRC8_107:
            LMIC.frame[LMIC.dataLen++] = crc8;
            break;
        case RF_CHECKSUM_TYPE_CCITT_FFFF:
        case RF_CHECKSUM_TYPE_CCITT_0000:
        default:
            LMIC.frame[LMIC.dataLen++] = (crc16 >>  8) & 0xFF;
            LMIC.frame[LMIC.dataLen++] = (crc16) & 0xFF;
            break;
    }

    LMIC.osjob.func = func;
    os_radio(RADIO_TX);
}

static void sx12xx_txdone_func(osjob_t* job)
{
    sx12xx_transmit_complete = true;
}

static void sx12xx_tx_func(osjob_t* job)
{
    if (RF_tx_size > 0)
        sx12xx_tx((unsigned char *) &TxBuffer[0], RF_tx_size, sx12xx_txdone_func);
}
