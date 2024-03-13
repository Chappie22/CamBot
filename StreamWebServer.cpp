#include "StreamWebServer.h"


StreamWebServer::StreamWebServer(bool enable) {
    WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);  
    StreamWebServer::enable = enable;   
}

bool StreamWebServer::isStreamingRightNow() {
    return streamingRightNow;
}

void StreamWebServer::startCameraServer() {
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.server_port = 80;

  httpd_uri_t state_uri = {
    .uri       = "/state",
    .method    = HTTP_GET,
    .handler   = state_handler,
    .user_ctx  = NULL
  };

  httpd_uri_t flash_change_uri = {
    .uri       = "/flash",
    .method    = HTTP_POST,
    .handler   = change_flash_state_handler,
    .user_ctx  = NULL
  };

  httpd_uri_t stream_uri = {
    .uri       = "/stream",
    .method    = HTTP_GET,
    .handler   = stream_handler,
    .user_ctx  = NULL
  };

  httpd_uri_t index_uri = {
        .uri       = "/",
        .method    = HTTP_GET,
        .handler   = index_handler,
        .user_ctx  = NULL
  };

    Serial.printf("Starting stream server on port: '%d'\n", config.server_port);
    if (httpd_start(&stream_httpd, &config) == ESP_OK) {
        httpd_register_uri_handler(stream_httpd, &index_uri);      
        httpd_register_uri_handler(stream_httpd, &state_uri);
        httpd_register_uri_handler(stream_httpd, &flash_change_uri);
    }    
       
    config.server_port += 1;
    config.ctrl_port += 1;
    Serial.printf("Starting snapshot server on port: '%d'\n", config.server_port);
    if (httpd_start(&camera_httpd, &config) == ESP_OK) {      
        httpd_register_uri_handler(camera_httpd, &stream_uri);
    } 
}

esp_err_t StreamWebServer::state_handler(httpd_req_t *req) {
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

    // Construct the JSON response object
    const char *flashStateResponse = flashEnabled ? "true" : "false";
    const char *motionStateResponse = motionState ? "true" : "false";
    const char *streamStateResponse = enable ? "true" : "false";
    char motionStateRepeatCounterResponse[20];
    char startTimeResponse[32];
    snprintf(startTimeResponse, sizeof(startTimeResponse), "%ld", millis());
    snprintf(motionStateRepeatCounterResponse, sizeof(motionStateRepeatCounterResponse), "%ld", motionStateRepeatCounter);

    const int jsonBufferSize = 150; 
    char responseJson[jsonBufferSize];
    snprintf(responseJson, jsonBufferSize, "{ \"flashEnabled\": %s, \"motionState\": %s, \"motionStateRepeatCounter\": %s, \"start\": %s, \"canStream\": %s }",
     flashStateResponse, motionStateResponse, motionStateRepeatCounterResponse, startTimeResponse, streamStateResponse);

    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, responseJson, strlen(responseJson));
    
    return ESP_OK;
}

esp_err_t StreamWebServer::change_flash_state_handler(httpd_req_t *req) {
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    flashEnabled = !flashEnabled;

    digitalWrite(FLASH_LED_PIN, flashEnabled);
    const char *flashStateResponse = flashEnabled ? "true" : "false";
    httpd_resp_set_type(req, "text/plain");
    httpd_resp_send(req, flashStateResponse, strlen(flashStateResponse));
    
    return ESP_OK;
}

esp_err_t StreamWebServer::index_handler(httpd_req_t *req) {
    // Set HTTP headers
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    esp_err_t res = httpd_resp_set_type(req, "text/html");
    if (res != ESP_OK) {
        return res;
    }

    // Send HTML response
    res = httpd_resp_send(req, (const char *)hexIndexHtml, hexIndexHtmlSize);
    return res;
}

size_t StreamWebServer::jpg_encode_stream(void * arg, size_t index, const void* data, size_t len){
    jpg_chunking_t *j = (jpg_chunking_t *)arg;
    if(!index){
        j->len = 0;
    }
    if(httpd_resp_send_chunk(j->req, (const char *)data, len) != ESP_OK){
        return 0;
    }
    j->len += len;
    return len;
}

esp_err_t StreamWebServer::stream_handler(httpd_req_t *req){
  Serial.println("stream_handler handled!");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  streamingRightNow = true;
  delay(500);
  camera_fb_t * fb = NULL;
  esp_err_t res = ESP_OK;
  size_t _jpg_buf_len = 0;
  uint8_t * _jpg_buf = NULL;
  char * part_buf[64];

  res = httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);
  if(res != ESP_OK){
    return res;
  }  

  while(enable){
    fb = esp_camera_fb_get();
    if (!fb) {
      Serial.println("Camera capture failed");
      res = ESP_FAIL;
    } else {
      if(fb->width > 400){
        if(fb->format != PIXFORMAT_JPEG){
          bool jpeg_converted = frame2jpg(fb, 80, &_jpg_buf, &_jpg_buf_len);
          esp_camera_fb_return(fb);
          fb = NULL;
          if(!jpeg_converted){
            Serial.println("JPEG compression failed");
            res = ESP_FAIL;
          }
        } else {
          _jpg_buf_len = fb->len;
          _jpg_buf = fb->buf;
        }
      }
    }
    
    if(res == ESP_OK){
      size_t hlen = snprintf((char *)part_buf, 64, _STREAM_PART, _jpg_buf_len);
      res = httpd_resp_send_chunk(req, (const char *)part_buf, hlen);
    }
    if(res == ESP_OK){
      res = httpd_resp_send_chunk(req, (const char *)_jpg_buf, _jpg_buf_len);
    }
    if(res == ESP_OK){
      res = httpd_resp_send_chunk(req, _STREAM_BOUNDARY, strlen(_STREAM_BOUNDARY));
    }
    if(fb){
      esp_camera_fb_return(fb);
      fb = NULL;
      _jpg_buf = NULL;
    } else if(_jpg_buf){
      free(_jpg_buf);
      _jpg_buf = NULL;
    }
    if(res != ESP_OK){
      break;
    }
    //Serial.printf("MJPG: %uB\n",(uint32_t)(_jpg_buf_len));
  }

  streamingRightNow = false;
  return res;
}