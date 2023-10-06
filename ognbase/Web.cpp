/*
 * Web.cpp
 * Copyright (C) 2020 Manuel Rösel
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

#include "WiFi.h"
#include "ESPAsyncWebServer.h"
#include "SPIFFS.h"
#include "SoC.h"
#include "OLED.h"
#include "EEPROM.h"
#include "Time.h"
#include "RF.h"
#include "global.h"
#include "Battery.h"
#include "Log.h"
#include "config.h"
#include "Traffic.h"

#include <string>

#include <ArduinoJson.h>

#include <ErriezCRC32.h>

#include <FS.h>   // Include the SPIFFS library

#define  U_PART U_FLASH
#define INDEX_CRC 3948812196

extern unsigned long ExportTimeWebRefresh;

File fsUploadFile;

#define countelems(a) (sizeof(a) / sizeof(a[0]))

// Create AsyncWebServer object on port 80
AsyncWebServer wserver(80);
AsyncWebSocket ws("/ws");

AsyncWebSocketClient* globalClient = NULL;

size_t content_len;

static const char upload_templ[] PROGMEM =
"<html>\
 <head>\
 <meta http-equiv='Content-Type' content='text/html; charset=utf-8'>\
 <meta http-equiv='cache-control' content='no-cache'>\
 </head>\
 <p>%s</p>\
 Files on device:<br>%s<br>\
 <div class = 'upload'>\
 <form method = 'POST' action = '/doUpload' enctype='multipart/form-data'>\
 <input type='file' name='data'/><input type='submit' name='upload' value='Upload' title = 'Upload Files'>\
 </form></div>\
 <p><a href='dnload' class='upload'>Download Config</a></p>\
 <p><a href='clear' class='clear'>Clear All Files</a></p>\
 <p><a href='reboot' class='clear'>Reboot</a></p>\
 </html>";

const char *ognopmode = "OGNbase";

static const char stats_templ[] PROGMEM =
"<html>\
 <head>\
 <meta http-equiv='Content-Type' content='text/html; charset=utf-8'>\
 <meta http-equiv='Refresh' content='23'>\
 </head>\
 <p>\r\nOperation mode: %s</p>\
 <br><p>\r\nBase Station Stats:</p>\
 <p>&nbsp;\r\n Uptime: %d minutes</p>\
 <p>&nbsp;\r\n Traffic packets received: %d</p>\
 <p>&nbsp; &nbsp; &nbsp;\r\n  per minute: %d</p>\
 <p>&nbsp;\r\n Traffic packets reported: %d</p>\
 <p>&nbsp;\r\n Aircraft seen ever: %d, today: %d</p>\
 <p>&nbsp;\r\n Largest Range: %d km</p>\
 <p>&nbsp;\r\n Corrupt packets: %d</p>\
 <p>&nbsp;\r\n Other packets: %d</p>\
 <p>&nbsp;\r\n Time-sync restarts: %d</p>\
 <br><p>\r\nRemote Station Stats:</p>\
 <p>&nbsp;\r\n Uptime: %d minutes</p>\
 <p>&nbsp;\r\n Traffic packets: %d</p>\
 <p>&nbsp; &nbsp; &nbsp;\r\n  per minute: %d</p>\
 <p>&nbsp; &nbsp; &nbsp;\r\n  pct relayed: %d</p>\
 <p>&nbsp;\r\n Time packets sent: %d</p>\
 <p>&nbsp; &nbsp; &nbsp;\r\n  pct acknowledged: %d</p>\
 <p>&nbsp;\r\n Time-sync restarts: %d</p>\
 <p>&nbsp;\r\n Corrupt packets: %d</p>\
 <p>&nbsp;\r\n Other packets: %d</p>\
 <p>&nbsp;\r\n Battery voltage: %.1f</p>\
 </html>";
                                  
void onWsEvent(AsyncWebSocket* server, AsyncWebSocketClient* client, AwsEventType type, void* arg, uint8_t* data, size_t len)
{
    if (type == WS_EVT_CONNECT)

        globalClient = client;

    else if (type == WS_EVT_DISCONNECT)

        globalClient = NULL;
}

// Replaces placeholder with LED state value
String processor(const String& var)
{
    Serial.println(var);
    return "ON";
}

void Web_fini()
{}

static const char update_html[] PROGMEM =
 "<form method='POST' action='/doUpdate' enctype='multipart/form-data'>\
  <input type='file' name='update'><input type='submit' value='Update'></form>";

void handleUpdate(AsyncWebServerRequest* request)
{
    request->send(200, "text/html", update_html);
}

#define UPLHTMSIZE 1150
#define FILELSTSIZ 600
void handleUpload(AsyncWebServerRequest* request)
{
  char *upload_html = (char *) malloc(UPLHTMSIZE);
  char *filelist = (char *) malloc(FILELSTSIZ);
  const char *msg;
  if (config_done < -4)
      msg = "could not open SPIFFS file system";
  else if (config_done == -4)
      msg = "config.json did not exist at boot";
  else if (config_done == -3)
      msg = "could not open config.json at boot";
  else if (config_done == -2)
      msg = "error parsing config.json at boot";
  else if (config_done == -1)
      msg = "config.json version not valid at boot";
  else if (config_done == 0)
      msg = "no wifi info in config.json at boot";
  else
      msg = "success parsing config.json at boot";
#if FILE_LOGGER
  if (! ogn_debug)
      SPIFFS.remove("/debuglog.txt");
#endif
  filelist[0] = '\0';
  int nfiles = 0;
  File root = SPIFFS.open("/");
  Serial.println(F("Files in SPIFFS:"));
  File file = root.openNextFile();
  while(file){
      Serial.print("... ");
      Serial.print(file.name());
      Serial.print("  [");
      Serial.print(file.size());
      Serial.println(" bytes]");
      int len = strlen(filelist);
      if (len < FILELSTSIZ-80) {
        snprintf(filelist+len, FILELSTSIZ-len,
           "&nbsp;&nbsp;%s&nbsp;&nbsp;[%d bytes]<br>", file.name(), file.size());
      } else {
        Serial.println("...");
        snprintf(filelist+len, FILELSTSIZ-len, "...<br>");
        break;
      }
      ++nfiles;
      file = root.openNextFile();
  }
  if (nfiles == 0) {
      Serial.println("... (none)");
      snprintf(filelist, FILELSTSIZ, "&nbsp;&nbsp;(none)<br>");
  }
  file.close();
  root.close();
  snprintf(upload_html, UPLHTMSIZE, upload_templ, msg, filelist);
  request->send(200, "text/html", upload_html);
  free(upload_html);
  free(filelist);
}

void DoUpload(AsyncWebServerRequest* request, String filename, size_t index, uint8_t* data, size_t len, bool final)
{
    bool was_open = false;
#if FILE_LOGGER
    if (strcmp(filename.c_str(),"debuglog.txt")==0) {
        if (DebugLogOpen) {
            DebugLog.close();
            was_open = true;
        }
    }
#endif
    if (!index)
        request->_tempFile = SPIFFS.open("/" + filename, "w");
    if (len)
        // stream the incoming chunk to the opened file
        request->_tempFile.write(data, len);
    if (final)
    {
        request->_tempFile.close();
        request->redirect("/");
    }
#if FILE_LOGGER
    if (was_open)
        DebugLog = SPIFFS.open("/debuglog.txt", FILE_APPEND);
#endif
}

void handleDnload(AsyncWebServerRequest* request)
{
  if (SPIFFS.exists("/config.json")) {
    request->send(SPIFFS, "/config.json", String(), true);
    Serial.println(F("Sent config.json file"));
  } else {
    request->send(404, "text/plain", "config.json not found");
    Serial.println(F("Config.json not found"));
  }
}

void handleDnldlog(AsyncWebServerRequest* request)
{
  bool was_open = false;
#if FILE_LOGGER
  if (DebugLogOpen) {
      DebugLog.close();
      was_open = true;
  }
#endif
  if (SPIFFS.exists("/debuglog.txt")) {
    request->send(SPIFFS, "/debuglog.txt", String(), true);
    Serial.println(F("Sent debuglog.txt file"));
  } else {
    request->send(404, "text/plain", "debuglog.txt not found");
    Serial.println(F("debuglog.txt not found"));
  }
#if FILE_LOGGER
  if (was_open)
      DebugLog = SPIFFS.open("/debuglog.txt", FILE_APPEND);
#endif
}

//void notFound(AsyncWebServerRequest *request) {
//    request->send(404, "text/plain", "file not found");
//}

void handleDoUpdate(AsyncWebServerRequest* request, const String& filename, size_t index, uint8_t* data, size_t len, bool final)
{
    String msg;

    if (!index)
    {
        msg = "updating firmware";
        Logger_send_udp(&msg);

       //SPIFFS.format();
        
        content_len = request->contentLength();
        // if filename includes spiffs, update the spiffs partition
        int cmd = (filename.indexOf("spiffs") > -1) ? U_PART : U_FLASH;

        if (!Update.begin(UPDATE_SIZE_UNKNOWN, cmd))
        {

            Update.printError(Serial);
        }
    }

    if (Update.write(data, len) != len)
    {
        Update.printError(Serial);
    }

    if (final)
    {
        AsyncWebServerResponse* response = request->beginResponse(302, "text/plain", "Please wait while the device reboots");
        response->addHeader("Refresh", "20");
        response->addHeader("Location", "/");
        request->send(response);
        if (!Update.end(true))
            Update.printError(Serial);
        else
        {
            DebugLogWrite("firmware update restart");
            delay(10000);
            // ESP.restart();
            RF_Shutdown();
            SoC->reset();
        }
    }
}

#define STATS_SIZE 1499
char stats_html[STATS_SIZE+1];

void update_stats()
{
  uint32_t zero = 0;
  if (ognrelay_enable && ! ogn_gnsstime && ! ognrelay_time) {
     if (have_approx_time > 0)
        ognopmode = "Remote station, NTP time from base";
     else
        ognopmode = "Remote station, no precise time";
  }
  if (ognrelay_base && ognrelay_time) {
     snprintf(stats_html, STATS_SIZE, stats_templ,
        ognopmode,
        uptime,
        traffic_packets_recvd, zero,
        traffic_packets_reported,
        (uint32_t) numseen_ever,
        (uint32_t) numseen_today,
        (uint32_t) largest_range,
        (uint32_t) bad_packets_recvd,
        other_packets_recvd,
        (uint32_t) sync_restarts,
        remote_uptime,
        remote_traffic,
        (uint32_t) packets_per_minute,
        (uint32_t) remote_pctrel,
        (uint32_t) remote_timesent,
        (uint32_t) remote_ack,
        (uint32_t) remote_restarts,
        (uint32_t) remote_bad,
        (uint32_t) remote_other,
        remote_voltage);
  } else if (ognrelay_base) {
     snprintf(stats_html, STATS_SIZE, stats_templ,
        ognopmode,
        uptime,
        traffic_packets_recvd, zero,
        traffic_packets_reported,
        (uint32_t) numseen_ever,
        (uint32_t) numseen_today,
        (uint32_t) largest_range,
        (uint32_t) bad_packets_recvd,
        other_packets_recvd,
        zero, zero,
        remote_traffic,
        zero,
        (uint32_t) remote_pctrel,
        zero, zero, zero, zero, zero,
        (float) 0.0);
  } else if (ognrelay_enable) {
     snprintf(stats_html, STATS_SIZE, stats_templ,
        ognopmode,
        zero,
        zero, zero, zero, zero, zero,
        zero, zero, zero, zero,
        uptime,
        traffic_packets_recvd,
        (uint32_t) packets_per_minute,
        (uint32_t) ((100 * traffic_packets_relayed)
                      / (traffic_packets_recvd+1)),
        (uint32_t) time_packets_sent,
        ((uint32_t) (100 * (ack_packets_recvd+1)) / (time_packets_sent+1)),
        (uint32_t) sync_restarts,
        (uint32_t) bad_packets_recvd,
        (uint32_t) other_packets_recvd,
        (float) Battery_voltage());
  } else {  /* single standalone station */
     snprintf(stats_html, STATS_SIZE, stats_templ,
        ognopmode,
        uptime,
        traffic_packets_recvd,
        (uint32_t) packets_per_minute,
        traffic_packets_reported,
        (uint32_t) numseen_ever,
        (uint32_t) numseen_today,
        (uint32_t) largest_range,
        (uint32_t) bad_packets_recvd,
        other_packets_recvd,
        zero, zero, zero, zero,
        zero, zero, zero, zero, zero, zero,
        (float) Battery_voltage());
  }
  stats_html[STATS_SIZE] = '\0';
//Serial.println(stats_html);
}

void Web_start()
{
    wserver.begin();
}

void Web_stop()
{
    wserver.end();
}

/* if no config, just offer to upload config */
void mini_server()
{
    // wserver.on("/", HTTP_GET, [upload_html](AsyncWebServerRequest* request){
    //wserver.on("/", HTTP_GET, [](AsyncWebServerRequest* request){
    //    request->send(200, "text/html", upload_html);
    //});

    wserver.on("/", HTTP_GET, [](AsyncWebServerRequest* request){
      handleUpload(request);
    });

    wserver.on("/doUpload", HTTP_POST, [](AsyncWebServerRequest* request) {}, DoUpload);

    wserver.on("/dnload", HTTP_GET, [](AsyncWebServerRequest* request){
      handleDnload(request);
    });

    wserver.on("/debuglog.txt", HTTP_GET, [](AsyncWebServerRequest* request){
      handleDnldlog(request);
    });

    wserver.on("/clear", HTTP_GET, [](AsyncWebServerRequest* request){
        Serial.println(F("Formatting spiffs..."));
        SPIFFS.format();
        request->redirect("/");
    });

    wserver.on("/reboot", HTTP_GET, [](AsyncWebServerRequest* request){
        request->redirect("/");
        DebugLogWrite("mini_web_server reboot");
        delay(500);
        SoC->reset();
    });    

    Web_start();
}

int ssid_num = -1;

void Web_setup(ufo_t* this_aircraft)
{
    if (!SPIFFS.begin(true))
    {
        Serial.println("An Error has occurred while mounting SPIFFS");
        return;
    }

    if (!SPIFFS.exists("/index.html"))
    {
        Serial.println("index.html does not exist");
        mini_server();
        return;
    }

    File file = SPIFFS.open("/index.html", "r");
    if (!file)
    {
        Serial.println("Error reading index.html");
        SPIFFS.remove("/index.html");
        mini_server();
        return;
    }

    yield();

    ws.onEvent(onWsEvent);
    wserver.addHandler(&ws);

    size_t filesize   = file.size();
    char*  index_html = (char *) malloc(filesize + 1);

    file.read((uint8_t *)index_html, filesize);
    index_html[filesize] = '\0';

    file.close();

    bool wrong_version = true;
    char *cp = strstr(index_html, "meta name=\"OGNbase-Version\"");
    if (cp != NULL) {
        cp += 37;
        char versionbuf[8];
        int ii = 0;
        while (*cp != '\"' && *cp != '\0' && ii < 7)
            versionbuf[ii++] = *cp++;
        versionbuf[ii] = '\0';
        if (strcmp(versionbuf,OGNBASE_HTML_VERSION) == 0)
            wrong_version = false;
    }

    if (wrong_version) {
        Serial.println("Wrong version of index.html");
        OLED_write("Wrong index.html version", 0, 27, true);
        delay(1000);
        SPIFFS.remove("/index.html");
        mini_server();
        return;
    }

    char*  offset;
    size_t size = 12000;
    char*  Settings_temp = (char *) malloc(size);

    if (Settings_temp == NULL)
        return;

    offset = Settings_temp;

    String station_addr = String(this_aircraft->addr, HEX);
    station_addr.toUpperCase();

    // to_string() formats to 6 decimals
    //String pos_lat = to_string(ogn_lat);
    //String pos_lon = to_string(ogn_lon);
    char buf_lat[16];
    snprintf(buf_lat,16,"%.6f",ogn_lat);
    char buf_lon[16];
    snprintf(buf_lon,16,"%.6f",ogn_lon);
    String pos_alt = "";
    pos_alt.concat(ogn_alt);
    String pos_geo = "";
    pos_geo.concat(ogn_geoid_separation);

    String version = SOFTRF_FIRMWARE_VERSION;

    // the first few messages here will never be shown, because
    // the status page is not loaded without a configuration file
    if (config_done == -5)
        ognopmode = "could not open SPIFFS file system";
    else if (config_done == -4)
        ognopmode = "config.json did not exist at boot";
    else if (config_done == -3)
        ognopmode = "could not open config.json at boot";
    else if (config_done == -2)
        ognopmode = "error parsing config.json at boot";
    else if (config_done == -1)
        ognopmode = "config.json version not valid at boot";
    else if (config_done <= 0)
        ognopmode = "no wifi info in config.json at boot";
    else if (ognrelay_base && ognrelay_time)
        ognopmode = "Base station, time relayed from remote";
    else if (ognrelay_base && ogn_gnsstime)
        ognopmode = "Base station, time from GNSS";
    else if (ognrelay_base)
        ognopmode = "Base station, time from NTP";
    else if (ognrelay_enable && ognrelay_time && ogn_gnsstime)
        ognopmode = "Remote station, relaying GNSS time";
    else if (ognrelay_enable && ogn_gnsstime)
        ognopmode = "Remote station, time from GNSS"; 
    else if (ognrelay_enable)
        ognopmode = "Remote station, no precise time";
    else if (ogn_gnsstime)
        ognopmode = "Single station, time from GNSS";
    else
        ognopmode = "Single station, time from NTP";

    yield();

    snprintf(offset, size, index_html,
             station_addr,
             version.c_str(),
             ognopmode,
             ogn_callsign,
             buf_lat,
             buf_lon,
             pos_alt,
             pos_geo,
             String(ogn_range),
             // (ogn_band == RF_BAND_AUTO ? "selected" : ""), RF_BAND_AUTO,
             (ogn_band == RF_BAND_EU ? "selected" : ""), RF_BAND_EU,
             (ogn_band == RF_BAND_US ? "selected" : ""), RF_BAND_US,
             (ogn_band == RF_BAND_AU ? "selected" : ""), RF_BAND_AU,
             (ogn_band == RF_BAND_NZ ? "selected" : ""), RF_BAND_NZ,
             (ogn_band == RF_BAND_RU ? "selected" : ""), RF_BAND_RU,
             (ogn_band == RF_BAND_CN ? "selected" : ""), RF_BAND_CN,
             (ogn_band == RF_BAND_UK ? "selected" : ""), RF_BAND_UK,
             (ogn_band == RF_BAND_IN ? "selected" : ""), RF_BAND_IN,
             (ogn_band == RF_BAND_IL ? "selected" : ""), RF_BAND_IL,
             (ogn_band == RF_BAND_KR ? "selected" : ""), RF_BAND_KR,

             (ogn_mobile ? "selected" : ""),
             (ogn_mobile ? "" : "selected"),

             (ogn_protocol_1 == RF_PROTOCOL_LEGACY ? "selected" : ""),
             RF_PROTOCOL_LEGACY, legacy_proto_desc.name,
             (ogn_protocol_1 == RF_PROTOCOL_OGNTP ? "selected" : ""),
             RF_PROTOCOL_OGNTP, ogntp_proto_desc.name,
             (ogn_protocol_1 == RF_PROTOCOL_P3I ? "selected" : ""),
             RF_PROTOCOL_P3I, p3i_proto_desc.name,
             (ogn_protocol_1 == RF_PROTOCOL_FANET ? "selected" : ""),
             RF_PROTOCOL_FANET, fanet_proto_desc.name,

             (ogn_protocol_2 == RF_PROTOCOL_LEGACY ? "selected" : ""),
             RF_PROTOCOL_LEGACY, legacy_proto_desc.name,
             (ogn_protocol_2 == RF_PROTOCOL_OGNTP ? "selected" : ""),
             RF_PROTOCOL_OGNTP, ogntp_proto_desc.name,
             (ogn_protocol_2 == RF_PROTOCOL_P3I ? "selected" : ""),
             RF_PROTOCOL_P3I, p3i_proto_desc.name,
             (ogn_protocol_2 == RF_PROTOCOL_FANET ? "selected" : ""),
             RF_PROTOCOL_FANET, fanet_proto_desc.name,

             (ogn_debug == true ? "selected" : ""),
             (ogn_debug == false ? "selected" : ""),
             String(ogn_debugport),

             (ogn_itrackbit == true ? "selected" : ""), "True",
             (ogn_itrackbit == false ? "selected" : ""), "False",

             (ogn_istealthbit == true ? "selected" : ""), "True",
             (ogn_istealthbit == false ? "selected" : ""), "False",

             //ogn_ssid[0].c_str(),
             WiFi.SSID().c_str(),

             /*Hide Wifi Password*/
             "hidepass",

             (ogn_sleepmode == 0 ? "selected" : ""), "Disabled",
             (ogn_sleepmode == 1 ? "selected" : ""), "Enabled",
             (ogn_sleepmode == 2 ? "selected" : ""), "Follow Remote",

             (zabbix_enable == 0 ? "selected" : ""), "Disabled",
             (zabbix_enable == 1 ? "selected" : ""), "Enabled",

             String(ogn_rxidle/60),
             String(ogn_wakeuptimer/60),
             String(ogn_morning),
             String(ogn_evening),

             (ognrelay_enable == 0 ? "selected" : ""), "Disabled",
             (ognrelay_enable == 1 ? "selected" : ""), "Enabled",
             (ognrelay_base == 0 ? "selected" : ""), "Disabled",
             (ognrelay_base == 1 ? "selected" : ""), "Enabled",
             (ognrelay_time == 0 ? "selected" : ""), "Disabled",
             (ognrelay_time == 1 ? "selected" : ""), "Enabled",
             (ogn_gnsstime == 0 ? "selected" : ""), "Disabled",
             (ogn_gnsstime == 1 ? "selected" : ""), "Enabled",
             "hidekey",

             (testmode_enable == 0 ? "selected" : ""), "Disabled",
             (testmode_enable == 1 ? "selected" : ""), "Enabled"

             );

    ssid_num = -1;
    for (int sn=0; sn<5; sn++) {
       if (WiFi.SSID() == ogn_ssid[sn])
           ssid_num = sn;   // using the SSID from this slot
    }

    yield();

    size_t len  = strlen(offset);
    String html = String(offset);

    wserver.on("/", HTTP_GET, [html](AsyncWebServerRequest* request){
        request->send(200, "text/html", html);
    });

    // Route to load style.css file
    wserver.on("/style.css", HTTP_GET, [](AsyncWebServerRequest* request){
        request->send(SPIFFS, "/style.css", "text/css");
    });

    wserver.on("/update", HTTP_GET, [](AsyncWebServerRequest* request){
        handleUpdate(request);
        request->redirect("/");
    });

    wserver.on("/doUpdate", HTTP_POST,
               [](AsyncWebServerRequest* request) {},
               [](AsyncWebServerRequest* request, const String& filename, size_t index, uint8_t* data,
                  size_t len, bool final) {
        handleDoUpdate(request, filename, index, data, len, final);
    });

    // wserver.on("/upload", HTTP_GET, [upload_html](AsyncWebServerRequest* request){
    //wserver.on("/upload", HTTP_GET, [](AsyncWebServerRequest* request){
    //    request->send(200, "text/html", upload_html);
    //});

    wserver.on("/upload", HTTP_GET, [](AsyncWebServerRequest* request){
        handleUpload(request);
    });

    wserver.on("/doUpload", HTTP_POST, [](AsyncWebServerRequest* request) {}, DoUpload);

    wserver.on("/dnload", HTTP_GET, [](AsyncWebServerRequest* request){
        handleDnload(request);
    });

    wserver.on("/debuglog.txt", HTTP_GET, [](AsyncWebServerRequest* request){
      handleDnldlog(request);
    });

    wserver.on("/clear", HTTP_GET, [](AsyncWebServerRequest* request){
        Serial.println(F("Formatting spiffs..."));
        SPIFFS.format();
        request->redirect("/");
    });    

    snprintf(stats_html, STATS_SIZE, "stats not available yet");

    wserver.on("/stats", HTTP_GET, [](AsyncWebServerRequest* request){
        update_stats();
        Serial.println(F("requesting stats page..."));
        request->send(200, "text/html", String(stats_html));
    });

    wserver.on("/refresh", HTTP_GET, [](AsyncWebServerRequest* request){
        /* refresh the stats at the top of the status page in 2 sec */
        ExportTimeWebRefresh = millis()/1000 - 21;
        request->redirect("/");
    });    

    wserver.on("/reboot", HTTP_GET, [](AsyncWebServerRequest* request){
        request->redirect("/");
        DebugLogWrite("web_server reboot");
        delay(500);
        SoC->reset();
    });    

    wserver.on("/remote_reboot", HTTP_GET, [](AsyncWebServerRequest* request){
        if (reboot_remote()) {
          Serial.println(F("sent remote reboot packet..."));
          OLED_write("remote reboot...", 0, 27, true);
          delay(1000);
          // should display something on the web page too.
          // for now just some hints:
          remote_sats = 0;            /* hints that remote is restarting */
          remote_uptime = 0;
          uptime = 0;
          ExportTimeWebRefresh = 0;   /* force a refresh of the stats at the top of the status page */
        }
        request->redirect("/");
    });    

//    wserver.onNotFound([](AsyncWebServerRequest *request){
//        request->send(404, "text/plain", "file not found.");
//    });

//    wserver.onNotFound(notFound);

    yield();

    // Send a GET request to <ESP_IP>/get?inputString=<inputMessage>
    wserver.on("/get", HTTP_GET, [](AsyncWebServerRequest* request) {
        if (request->hasParam("callsign"))
            ogn_callsign = request->getParam("callsign")->value().c_str();
            ogn_callsign.trim();
            ogn_callsign.replace("_","");
            if(ogn_callsign.length() > 9){
              ogn_callsign = ogn_callsign.substring(0,9);
            }
        if (request->hasParam("ogn_lat"))
            ogn_lat = request->getParam("ogn_lat")->value().toFloat();

        if (request->hasParam("ogn_lon"))
            ogn_lon = request->getParam("ogn_lon")->value().toFloat();

        if (request->hasParam("ogn_alt"))
            ogn_alt = request->getParam("ogn_alt")->value().toInt();

        //geoid_separation
        if (request->hasParam("ogn_geoid"))
            ogn_geoid_separation = request->getParam("ogn_geoid")->value().toInt();

        if (request->hasParam("ogn_mobile"))
            ogn_mobile = request->getParam("ogn_mobile")->value().toInt();

        yield();

        if (request->hasParam("ogn_freq"))
            ogn_band = request->getParam("ogn_freq")->value().toInt();

        if (request->hasParam("ogn_proto"))
            ogn_protocol_1 = request->getParam("ogn_proto")->value().toInt();

        ogn_protocol_1 = RF_PROTOCOL_LEGACY;    /* override, since only LEGACY supported for now */

        if (request->hasParam("ogn_proto2"))
            ogn_protocol_2 = request->getParam("ogn_proto2")->value().toInt();

        if (request->hasParam("ogn_d1090"))
            settings->d1090 = request->getParam("ogn_d1090")->value().toInt();

        if (request->hasParam("ogn_gdl90"))
            settings->gdl90 = request->getParam("ogn_gdl90")->value().toInt();

        if (request->hasParam("ogn_nmea"))
            settings->nmea_out = request->getParam("ogn_nmea")->value().toInt();

        if (request->hasParam("ogn_no_track_bit"))
            settings->no_track = request->getParam("ogn_no_track_bit")->value().toInt();

        if (request->hasParam("ogn_stealth_bit"))
            settings->stealth = request->getParam("ogn_stealth_bit")->value().toInt();

        if (request->hasParam("ogn_aprs_debug"))
            ogn_debug= request->getParam("ogn_aprs_debug")->value().toInt();

        if (request->hasParam("aprs_debug_port"))
            ogn_debugport = request->getParam("aprs_debug_port")->value().toInt();

        if (request->hasParam("ogn_range"))
            ogn_range = request->getParam("ogn_range")->value().toInt();

        //if (request->hasParam("ogn_agc"))
        //    settings->sxlna = request->getParam("ogn_agc")->value().toInt();

        yield();

        if (request->hasParam("ogn_ssid")) {
            if (ssid_num >= 0) {
               // new SSID introduced, overwrite same slot
               ogn_ssid[ssid_num] = request->getParam("ogn_ssid")->value().c_str();
            }
        }
        if (request->hasParam("ogn_wifi_password")){
            if (ssid_num >= 0) {
              if (request->getParam("ogn_wifi_password")->value() != "hidepass")
                ogn_wpass[ssid_num] = request->getParam("ogn_wifi_password")->value().c_str();
            }
        }

        if (request->hasParam("ogn_ignore_track"))
            ogn_itrackbit= request->getParam("ogn_ignore_track")->value().toInt();

        if (request->hasParam("ogn_ignore_stealth"))
            ogn_istealthbit= request->getParam("ogn_ignore_stealth")->value().toInt();

        if (request->hasParam("ogn_deep_sleep"))
            ogn_sleepmode = request->getParam("ogn_deep_sleep")->value().toInt();

        if (request->hasParam("zabbix_trap_en"))
            zabbix_enable = request->getParam("zabbix_trap_en")->value().toInt();

        yield();

        if (request->hasParam("ogn_sleep_time")) {
            ogn_rxidle = 60 * request->getParam("ogn_sleep_time")->value().toInt();
            if (ogn_rxidle != 0 && ogn_rxidle < 600)  ogn_rxidle = 600;
        }
        if (request->hasParam("ogn_wakeup_time")) {
            ogn_wakeuptimer = 60 * request->getParam("ogn_wakeup_time")->value().toInt();
            if (ogn_wakeuptimer != 0 && ogn_wakeuptimer < 600)  ogn_wakeuptimer = 600;
        }

        if (request->hasParam("ogn_morning")) {
            ogn_morning = request->getParam("ogn_morning")->value().toInt();
            if (ogn_morning <  5)  ogn_morning =  5;
            if (ogn_morning > 14)  ogn_morning = 14;
        }
        if (request->hasParam("ogn_evening")) {
            ogn_evening = request->getParam("ogn_evening")->value().toInt();
            if (ogn_evening < 15)  ogn_evening = 15;
            if (ogn_evening > 22)  ogn_evening = 22;
        }

        if (request->hasParam("relay_enable"))
            ognrelay_enable = request->getParam("relay_enable")->value().toInt();
        if (request->hasParam("base_enable"))
            ognrelay_base = request->getParam("base_enable")->value().toInt();
        if (ognrelay_base)  ognrelay_enable = 0;
        if (request->hasParam("relay_time"))
            ognrelay_time = request->getParam("relay_time")->value().toInt();
        if (request->hasParam("gnss_time"))
            ogn_gnsstime = request->getParam("gnss_time")->value().toInt();

        if (request->hasParam("ogn_relay_key")) {
            if (request->getParam("ogn_relay_key")->value() != "hidekey") {
              ognrelay_key = request->getParam("ogn_relay_key")->value().toInt();
            }
        }

        if (request->hasParam("test_mode_en"))
            testmode_enable = request->getParam("test_mode_en")->value().toInt();

        beers_show = false;

        yield();

        request->redirect("/");
#if 0
        // ogn_reset_all
        if (request->hasParam("ogn_reset_all"))
            if (request->getParam("ogn_reset_all")->value() == "on")
            {
                SPIFFS.format();
                RF_Shutdown();
                DebugLogWrite("ogn_reset_all reboot");
                delay(500);
                SoC->reset();
            }
#endif
        EEPROM_store();
        OGN_save_config();
        DebugLogWrite("web settings reboot");
        RF_Shutdown();
        delay(500);
        SoC->reset();
    });

    SoC->swSer_enableRx(true);
    free(Settings_temp);
    free(index_html);

    yield();

    // Start server
    Web_start();
    delay(500);
    ExportTimeWebRefresh = 0;   /* force a refresh of the stats at the top on next web_loop() */
}

void Web_loop(void)
{
    if (globalClient != NULL && globalClient->status() == WS_CONNECTED)
    {
        String values;

#if defined(TBEAM)
        values = (ognrelay_base && ognrelay_time) ? remote_sats : gnss.satellites.value();
#else
        values = remote_sats;   // was: power
#endif
        values += "_";
        if (ognrelay_base)
            values += numseen_1hr;     // was: RF_last_rssi
        else
            values += "--";
        values += "_";
        uint16_t reported_uptime =
            (ognrelay_base && ognrelay_time && time_synched) ? remote_uptime : uptime;
        if (reported_uptime < 60) {
            values += reported_uptime;
            values += "m";
        } else {
            values += reported_uptime/60;
            values += "h";
        }
        values += "_";
        values += traffic_packets_recvd;    // was satfix
        values += "_";
        values += packets_per_minute;   // was: timestamp
        values += "_";
        values += numtracked;  // was: largest_range
        globalClient->text(values);
    }

    // update_stats();
}
