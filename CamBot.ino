#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <ArduinoJson.h>

// #include "StreamWebServer.h"
#include "CaptureCamLib.h"
#include "UniversalTelegramBot.h"
#include "StreamWebServer.h"

enum Components {
  WEB_SERVER, TELEGRAM_BOT, MOTION_DETECTION
};

//Device Settings
const char vernum[] = "1.0";
const String devstr =  "CamBot";
Components activeComponent = TELEGRAM_BOT;

//Wifi Settings
const char ssid[] = " ";               // your network SSID (name)
const char password[] = " ";  // your network key
WiFiClientSecure client;

//Telegram Bot Settings
const String authorizedChatIds[] = {" ", " "};
const String BOTtoken = " ";
UniversalTelegramBot bot(BOTtoken, client);
const int Bot_mtbs = 1000;                  // mean time between scan messages
long Bot_lasttime;                          // last time messages' scan has been done
const String botCommands = F("["
  "{\"command\":\"photo\", \"description\":\"take a photo\"},"
  "{\"command\":\"clip\", \"description\":\"take short video clip\"},"
  "{\"command\":\"flash\", \"description\":\"toggle flash LED\"},"
  "{\"command\":\"enflash\", \"description\":\"enable flash on capture\"},"
  "{\"command\":\"disflash\", \"description\":\"disable flash on capture\"},"
  "{\"command\":\"entim\", \"description\":\"enable timed photo\"},"
  "{\"command\":\"distim\", \"description\":\"disable timed photo\"},"
  "{\"command\":\"10\", \"description\":\"1..10..1440 minutes timed photos\"},"
  "{\"command\":\"med\", \"description\":\"25 fps - play .5x speed\"},"
  "{\"command\":\"slow\", \"description\":\"8  fps - play 1x speed\"},"
  "{\"command\":\"vslow\", \"description\":\"2  fps - play 5x speed\"},"
  "{\"command\":\"status\", \"description\":\"get CamBot status\"},"
  "{\"command\":\"reboot\", \"description\":\"reboot the CamBot\"}"
  "]");

//Camera Settings
CaptureCam captureCam(FRAMESIZE_HD,     
 FRAMESIZE_HD,                            
 10,
 5);

//Stream Web Server Settings
StreamWebServer streamWebServer(true);

//State manager Settings
// MotionDetection motionDetection;
const int motionMtbs = 5000;  
long motionLastTime = millis();
bool motionState = false;
long motionStateRepeatCounter = 0;


//Action flags
bool reboot_request = false;
String activeChatId;

void setup() {
  Serial.begin(115200);

  WiFi.begin(ssid, password);
  WiFi.setSleep(false);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("WiFi connected");

  pinMode(FLASH_LED_PIN, OUTPUT);
  digitalWrite(FLASH_LED_PIN, captureCam.flashState);
  pinMode(12, INPUT_PULLUP);                // pull this down to stop recording  

  captureCam.setupCamera(); // Setup camera for capture
  streamWebServer.startCameraServer(); // Setup camera for streaming

  //bot.longPoll = 5; // Make the bot wait for a new message for up to 5 seconds
  client.setInsecure();

  bot.setMyCommands(botCommands);
  for (int i = 0; i < (sizeof(authorizedChatIds) / sizeof(authorizedChatIds[0])); i++) {        
    if(bot.sendMessage(authorizedChatIds[i], getStat("Bot is here :)"), "")){
      Serial.printf("Initial send to %s worked!\n", authorizedChatIds[i]);
    } else {
      Serial.printf("Initial send to %s failed!\n", authorizedChatIds[i]);
    }  
  }
}

void loop() {
  streamWebServer.motionState = motionState;
  streamWebServer.motionStateRepeatCounter = motionStateRepeatCounter;

  if (streamWebServer.isStreamingRightNow()) {
    Serial.println("Web Server stream initiated");
    if (activeComponent != WEB_SERVER) {
      activeComponent = WEB_SERVER;
      streamWebServer.flashEnabled = captureCam.flashState;      
      bot.pause = true;            
    }    

    delay(500);
    return;
  } 
  else if ((millis() > motionLastTime + motionMtbs) 
          && !captureCam.isProcessingVideoOrPhoto()
          && !bot.sendingImageOrVideo()) {      
    activeComponent = MOTION_DETECTION;
    streamWebServer.enable = false;
    int trueCounter = 0;
    int falseCounter = 0;    
    for (size_t i = 0; i < 5; i++) {      
      if (captureCam.captureAndIsMotionDetected()) trueCounter++;
      else falseCounter++;      
    }

    bool res = false;
    if (trueCounter >= 3) {
      res = true;      
    } 

    if (motionState == res) {
      motionStateRepeatCounter++;
    } else {
      motionStateRepeatCounter = 0;
    }

    motionState = res;
    Serial.printf("Motion: %d, repeats: %d\n", motionState, motionStateRepeatCounter);      

    motionLastTime = millis();
    streamWebServer.enable = true;
    return;
  }  
  else {
    if (activeComponent != TELEGRAM_BOT) {
      if (activeComponent == WEB_SERVER) {
        captureCam.flashState = streamWebServer.flashEnabled;
      }

      activeComponent = TELEGRAM_BOT;
      bot.pause = false;
    }    
  }

  client.setHandshakeTimeout(120000); // workaround for esp32-arduino 2.02 bug https://github.com/witnessmenow/Universal-Arduino-Telegram-Bot/issues/270#issuecomment-1003795884

  if (reboot_request) {    
    bot.sendMessage(activeChatId, getStat("Rebooted! Status: "), "");
    delay(10000);
    ESP.restart();
  }

  if (captureCam.picture_ready) {
    captureCam.picture_ready = false;
    send_the_picture("Here is the picture!");
  }

  if (captureCam.timed_picture_ready) {
    captureCam.timed_picture_ready = false;
    send_the_picture("Timed photo here! Next will be in " + String(captureCam.timePhoto_Minutes) + " minute(s).");  
  }

  if (captureCam.video_ready) {
    captureCam.video_ready = false;
    send_the_video("Here is the clip!");
  }

  captureCam.takeTimedPhotoIfNeed();

  if (millis() > Bot_lasttime + Bot_mtbs )  {

    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("***** WiFi reconnect *****");
      WiFi.reconnect();
      delay(5000);
    }

    int numNewMessages = bot.getUpdates(bot.last_message_received + 1);

    while (numNewMessages) {
      handleNewMessages(numNewMessages);
      numNewMessages = bot.getUpdates(bot.last_message_received + 1);
    }
    Bot_lasttime = millis();
  }  
}

void handleNewMessages(int numNewMessages) {
  for (int i = 0; i < numNewMessages; i++) {
    
    String chatId = String(bot.messages[i].chat_id);
    bool authorzied = false;
    for (int i = 0; i < (sizeof(authorizedChatIds) / sizeof(authorizedChatIds[0])); i++) {
      if (authorizedChatIds[i] == chatId) {
        authorzied = true;
        break;
      }
    }

    if (!authorzied) {
      bot.sendMessage(chatId, "You are unauthorized! Your chat id: " + chatId, "Markdown");
      return;
    }
    
    activeChatId = chatId;
    String text = bot.messages[i].text;
    Serial.printf("\nGot a message %s\n", text);

    bot.sendMessage(activeChatId, "Got: " + text, "Markdown");
    client.setHandshakeTimeout(120000);

    if (text == "/start") {          
      bot.sendMessage(activeChatId, getStat("Welcome! You can trigger any commands in Telegram menu. Also, here your`s first CamBot status :)"), "");
    } else if (text == "/status") {
      bot.sendMessage(activeChatId, getStat("Here is the status!"), "");
    } else if (text == "/reboot") {
      reboot_request = true;
    } else if (text == "/flash") {
      captureCam.flashState = !captureCam.flashState;
      digitalWrite(FLASH_LED_PIN, captureCam.flashState);
      if (captureCam.flashState) bot.sendMessage(activeChatId, "Flash turned on!", "");
      else bot.sendMessage(activeChatId, "Flash turned off!", "");
    } else if (text == "/disflash") {
      captureCam.flash_on_capture_enabled = false;       
    } else if (text == "/enflash") {
      captureCam.flash_on_capture_enabled = true;       
    } 
    
    //Interval Photos
    else if (text == "/entim") {
      captureCam.tim_enabled = true;
      captureCam.timePhoto_lasttime = millis();       
    } else if (text == "/distim") {
      captureCam.tim_enabled = false;       
    } else if (text.substring(1).toInt() >= 1 && text.substring(1).toInt() <= 1440) {
      captureCam.timePhoto_Minutes = text.substring(1).toInt();
      captureCam.timePhoto_lasttime = millis();       
    } 
    
    //Clip Settings
    else if (text == "/fast") {
      captureCam.setClipType(FAST);    
    } else if (text == "/med") {
      captureCam.setClipType(MED);
    } else if (text == "/slow") {
      captureCam.setClipType(SLOW);
    } else if (text == "/vslow") {
      captureCam.setClipType(VSLOW);
    } else if (text == "/clip") {
      bot.longPoll =  0;
      xTaskCreatePinnedToCore(recordTask, "the_camera_loop", 10000, NULL, 1, &captureCam.the_camera_loop_task, 1);      
    }

    else if (text == "/photo") {
      captureCam.takePhoto();
    }    
  }
}

void recordTask(void* pvParameter) { 
  captureCam.the_camera_loop();  
  vTaskDelete(captureCam.the_camera_loop_task); 
}

void send_the_picture(String caption) {
  bot.sendChatAction(activeChatId, "upload_photo");
  captureCam.prepareImageBuffAndPointer();
  Serial.println("\n>>>>> Sending as 512 byte blocks, with jzdelay of 0, bytes=  " + String(captureCam.fb_length));

  bot.sendMultipartFormDataToTelegramWithCaption("sendPhoto", "photo", "img.jpg",
                  "image/jpeg", caption, activeChatId, captureCam.fb_length,
                  CaptureCam::isMoreDataAvailable, CaptureCam::getNextByte, nullptr, nullptr);

  captureCam.releaseImageBuffAndPointer();
  bot.longPoll =  0;  
}

void send_the_video(String caption) {
  bot.sendChatAction(activeChatId, "upload_video");
  captureCam.prepareVideoBuffAndPointer();
  Serial.println("\n>>>>> Sending as 512 byte blocks, with a caption, and with jzdelay of 0, bytes=  " + String(captureCam.avi_len));
  
  bot.sendMultipartFormDataToTelegramWithCaption("sendDocument", "document", captureCam.strftime_buf,
                 "image/jpeg", caption, activeChatId, captureCam.avi_len,
                 CaptureCam::avi_more, CaptureCam::avi_next, nullptr, nullptr);

  captureCam.releaseImageBuffAndPointer();
  bot.longPoll = 5;
}

String getStat(String caption) {
  long rssi = WiFi.RSSI();
  String wifiSignalLevel = "No signal";
  if (rssi < -90) wifiSignalLevel = "Very Low";
  else if (rssi < -80) wifiSignalLevel = "Low";
  else if (rssi < -70) wifiSignalLevel = "Good";
  else if (rssi < -60) wifiSignalLevel = "Very Good";
  else if (rssi < -50) wifiSignalLevel = "Excellent";
  
  return caption + 
    "\n\n= = = Device = = =" +
    "\nDevice: " + devstr + 
    "\nVer: " + String(vernum) + 
    "\nRSSI (WiFi signal strength): " + String(WiFi.RSSI()) + " (" + wifiSignalLevel + ")"
    "\nLocal Server: " +  "http://" + WiFi.localIP().toString() + "/" +  
    "\nStream Enabled: " +  getEnOrDisEmoji(streamWebServer.enable) +
    "\nActive for: " +  getReadableActiveTime() + 
    "\nMotion detected|repeats: " +  getEnOrDisEmoji(motionState) + " | " + motionStateRepeatCounter +
    "\n\n= = = Memory = = ="
    "\nInternal Total heap: " + String(ESP.getHeapSize()) + " b" +
    "\nInternal Free Heap: " + String(ESP.getFreeHeap()) + " b" +
    "\nSPIRam Total heap: " + String(ESP.getPsramSize()) + " b" +
    "\nSPIRam Free Heap: " + String(ESP.getFreePsram()) + " b" +
    "\n\n= = = Flags = = ="
    "\nTimed photos: " + getEnOrDisEmoji(captureCam.tim_enabled) + " Interval: " + String(captureCam.timePhoto_Minutes) + " min(s)" +
    "\nFlash: " +  getEnOrDisEmoji(captureCam.flashState) + 
    "\nFlash on capture: " +  getEnOrDisEmoji(captureCam.flash_on_capture_enabled) +   
    "\nClip type: " + captureCam.getStringClipType();
}

String getEnOrDisEmoji(bool val) {
  return val ? "\xE2\x9C\x85" : "\xE2\x9D\x8C";
}

String getReadableActiveTime() {
  String readableTime;
  unsigned long currentMillis;
  unsigned long seconds;
  unsigned long minutes;
  unsigned long hours;
  unsigned long days;

  currentMillis = millis();
  seconds = currentMillis / 1000;
  minutes = seconds / 60;
  hours = minutes / 60;
  days = hours / 24;
  currentMillis %= 1000;
  seconds %= 60;
  minutes %= 60;
  hours %= 24;

  if (days > 0) {
    readableTime = String(days) + " day(s), ";
  }

  if (hours > 0) {
    readableTime += String(hours) + " hour(s), ";
  }

  if (minutes < 10) {
    readableTime += "0";
  }
  readableTime += String(minutes) + " min(s), ";

  if (seconds < 10) {
    readableTime += "0";
  }

  readableTime += String(seconds) + " sec(s)";

  return readableTime;
}