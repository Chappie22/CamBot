#include "CaptureCamLib.h"


CaptureCam::CaptureCam(framesize_t configframesize,      
      int framesize,
      int quality,
      int qualityconfig) {  
  this->configframesize = configframesize;    
  this->framesize = framesize;
  this->quality = quality;
  this->qualityconfig = qualityconfig;
  setClipType(MED);
}

bool CaptureCam::isProcessingVideoOrPhoto() {
  return isBusy;
}

bool CaptureCam::captureAndIsMotionDetected() {
    esp_camera_deinit();
    config.pixel_format = PIXFORMAT_GRAYSCALE;
    config.frame_size = FRAMESIZE_QVGA;
    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
      Serial.printf("Camera init failed with error 0x%x", err);
      return false;
    }

    bool res = false;    
    if (!captureStill()) {
        Serial.println("Failed capture on motion logic");
    }

    if (motionDetect()) {
        res = true;
    }

    updateFrame();
    
    esp_camera_deinit();
    config.pixel_format = PIXFORMAT_JPEG;
    config.frame_size = configframesize;
    err = esp_camera_init(&config);    
    if (err != ESP_OK) {
      Serial.printf("Camera init failed with error 0x%x", err);
      return false;
    }

    return res;
}

bool CaptureCam::captureStill() {
    camera_fb_t *frameBuffer = esp_camera_fb_get();
    if (!frameBuffer) {
        return false;
    }

    // Down-sample image in blocks
    for (uint32_t i = 0; i < WIDTH * HEIGHT; i++) {
        const uint16_t x = i % WIDTH;
        const uint16_t y = floor(i / WIDTH);
        const uint8_t blockX = floor(x / BLOCK_SIZE);
        const uint8_t blockY = floor(y / BLOCK_SIZE);
        const uint8_t pixel = frameBuffer->buf[i];
        // Average pixels in block (accumulate)
        currentFrame[blockY][blockX] += pixel;
    }

    // Average pixels in block (rescale)
    for (int y = 0; y < H; y++) {
        for (int x = 0; x < W; x++) {
            currentFrame[y][x] /= BLOCK_SIZE * BLOCK_SIZE;
        }
    }

    esp_camera_fb_return(frameBuffer); // Needed to free up camera memory

    return true;
}

bool CaptureCam::motionDetect() {
    uint16_t changes = 0;
    const uint16_t blocks = (WIDTH * HEIGHT) / (BLOCK_SIZE * BLOCK_SIZE);
    for (uint16_t y = 0; y < H; y++) {
        for (uint16_t x = 0; x < W; x++) {
            float current = currentFrame[y][x];
            float prev = prevFrame[y][x];
            float delta = abs(current - prev) / prev;

            if (delta >= BLOCK_DIFF_THRESHOLD) {
                motionView[x] = 1;
                changes++;
            }
        }
    }
    if (changes == 0) {
        return false;
    }

    for (uint16_t i = 0; i < VIEWPORT_PIXELS; i++) {
        motionView[i] = 0;
    }

    return (1.0 * changes / blocks) > IMAGE_DIFF_THRESHOLD;
}

void CaptureCam::updateFrame() {
    memcpy(prevFrame, currentFrame, sizeof(prevFrame));
}


String CaptureCam::getStringClipType() {
  switch (clipType)
  {
    case FAST: return "FAST";
    case MED: return "MED";
    case SLOW: return "SLOW";
    case VSLOW: return "VSLOW";  
    default: return "Uknown";
  }
}

void CaptureCam::setClipType(ClipTypes clipType) {
  this->clipType = clipType;
  if (clipType == FAST) {
    max_frames = 300;
    frame_interval = 0;
    speed_up_factor = 0.5;
  } else if (clipType == MED) {
    max_frames = 300;
    frame_interval = 125;
    speed_up_factor = 1;
  } else if (clipType == SLOW) {
    max_frames = 300;
    frame_interval = 500;
    speed_up_factor = 5;
  } else if (clipType == VSLOW) {
    max_frames = 300;
    frame_interval = 2000;
    speed_up_factor = 20;
  } 
}

void CaptureCam::takeTimedPhotoIfNeed() {
    if (millis() > (timePhoto_lasttime + timePhoto_Minutes * 60000) ) {
    if (tim_enabled) {
      if (!takePhoto(false)) {
        Serial.println("Camera Capture Failed");
      } else {
        timed_picture_ready = true;        
      }
      delay(10);
    }
    timePhoto_lasttime = millis();
  }
}

bool CaptureCam::takePhoto(bool changePictureFlag) {   
  turnOnFlashIfNeeded();
  isBusy = true;

  for (int j = 0; j < 4; j++) {
      camera_fb_t * newfb = esp_camera_fb_get();
      if (!newfb) {
        Serial.println("Camera Capture Failed");
      } else {
        esp_camera_fb_return(newfb);
        delay(10);
      }
  }  

    // Take Picture with Camera
    vid_fb = captureGoodJPEG();
    if (!vid_fb) {
      Serial.println("Camera capture failed");    
      return false;
    }

  turnOffFlashIfWasNeeded();

  if (changePictureFlag) {
    picture_ready = true;
  } 

  isBusy = false;
  return true;
}

bool CaptureCam::takePhoto() {
  return takePhoto(true);
}

bool CaptureCam::setupCamera() {
int avail_psram = ESP.getFreePsram();
  Serial.println("PSRAM size to store the video " + String(avail_psram));
  idx_buf_size = max_frames * 10 + 20;
  avi_buf_size = avail_psram - 900 * 1024; //900 for hd, 500 for vga
  Serial.println("Trying to allocate " + String(avi_buf_size));

  psram_avi_buf = (uint8_t*)ps_malloc(avi_buf_size);
  if (psram_avi_buf == 0) Serial.printf("psram_avi allocation failed\n");
  psram_idx_buf = (uint8_t*)ps_malloc(idx_buf_size); // save file in psram
  if (psram_idx_buf == 0) Serial.printf("psram_idx allocation failed\n");

  if (!initialSetupCamera()) {
    Serial.println("Camera Setup Failed!");
    while (true) {
      delay(100);
    }
  }

  for (int j = 0; j < 7; j++) {
    camera_fb_t * fb = esp_camera_fb_get();
    if (!fb) {
      Serial.println("Camera Capture Failed");
    } else {
      Serial.print("Pic, len="); Serial.print(fb->len);
      Serial.printf(", new fb %X\n", (long)fb->buf);
      esp_camera_fb_return(fb);
      delay(50);
    }
  }

  return true;
}


bool CaptureCam::initialSetupCamera() {  
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  

  if (psramFound()) {
    config.frame_size = configframesize;
    config.jpeg_quality = qualityconfig;
    config.fb_count = 4;
  } else {
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }

  static char * memtmp = (char *) malloc(32 * 1024);
  static char * memtmp2 = (char *) malloc(32 * 1024); //32767

  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return false;
  }
  free(memtmp2);
  memtmp2 = NULL;
  free(memtmp);
  memtmp = NULL;

  sensor_t * s = esp_camera_sensor_get();

  //  drop down frame size for higher initial frame rate
  s->set_framesize(s, (framesize_t)framesize);
  s->set_quality(s, quality);
  delay(200);
  return true;
}



void CaptureCam::prepareImageBuffAndPointer() {
  digitalWrite(33, LOW);  
  currentByte = 0;
  fb_length = vid_fb->len;
  fb_buffer = vid_fb->buf;
}

void CaptureCam::releaseImageBuffAndPointer() {
  esp_camera_fb_return(vid_fb);
  digitalWrite(33, HIGH);
}

void CaptureCam::prepareVideoBuffAndPointer() {  
  digitalWrite(33, LOW); 
  avi_ptr = 0;
  avi_buf = psram_avi_buf;
  avi_len = psram_avi_ptr - psram_avi_buf;
}

void CaptureCam::releaseVideoBuffAndPointer() {  
  digitalWrite(33, HIGH);
}

bool CaptureCam::isMoreDataAvailable() {
  return (fb_length - currentByte);
}

uint8_t CaptureCam::getNextByte() {
  currentByte++;
  return (fb_buffer[currentByte - 1]);
}

bool CaptureCam::avi_more() {
  return (avi_len - avi_ptr);
}

uint8_t CaptureCam::avi_next() {
  avi_ptr++;
  return (avi_buf[avi_ptr - 1]);
}

//
// Writes an uint32_t in Big Endian at current file position
//
void inline CaptureCam::print_quartet(unsigned long i, uint8_t * fd) {
  uint8_t y[4];
  y[0] = i % 0x100;
  y[1] = (i >> 8) % 0x100;
  y[2] = (i >> 16) % 0x100;
  y[3] = (i >> 24) % 0x100;
  memcpy( fd, y, 4);
}

//
// Writes 2 uint32_t in Big Endian at current file position
//
void inline CaptureCam::print_2quartet(unsigned long i, unsigned long j, uint8_t * fd) {
  uint8_t y[8];
  y[0] = i % 0x100;
  y[1] = (i >> 8) % 0x100;
  y[2] = (i >> 16) % 0x100;
  y[3] = (i >> 24) % 0x100;
  y[4] = j % 0x100;
  y[5] = (j >> 8) % 0x100;
  y[6] = (j >> 16) % 0x100;
  y[7] = (j >> 24) % 0x100;
  memcpy( fd, y, 8);
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//
//  captureGoodJPEG()  - take a picture and make sure it has a good jpeg
//
camera_fb_t * CaptureCam::captureGoodJPEG() {

  camera_fb_t * fb;

  long start;
  int failures = 0;

  do {
    int fblen = 0;
    int foundffd9 = 0;

    fb = esp_camera_fb_get();

    if (!fb) {
      Serial.println("Camera Capture Failed");
      failures++;
    } else {
      int get_fail = 0;
      fblen = fb->len;

      for (int j = 1; j <= 1025; j++) {
        if (fb->buf[fblen - j] != 0xD9) {
        } else {
          if (fb->buf[fblen - j - 1] == 0xFF ) {
            foundffd9 = 1;
            break;
          }
        }
      }

      if (!foundffd9) {
        Serial.printf("Bad jpeg, Frame %d, Len = %d \n", frame_cnt, fblen);
        esp_camera_fb_return(fb);
        failures++;
      } else {
        break;
      }
    }

  } while (failures < 10);   // normally leave the loop with a break()

  // if we get 10 bad frames in a row, then quality parameters are too high - set them lower
  if (failures == 10) {
    Serial.printf("10 failures");
    sensor_t * ss = esp_camera_sensor_get();
    int qual = ss->status.quality ;
    ss->set_quality(ss, qual + 3);
    quality = qual + 3;
    Serial.printf("\n\nDecreasing quality due to frame failures %d -> %d\n\n", qual, qual + 5);
    delay(1000);
  }
  return fb;
}

void CaptureCam::turnOnFlashIfNeeded() {
  if (flash_on_capture_enabled) {
    digitalWrite(FLASH_LED_PIN, HIGH);
  }
}

void CaptureCam::turnOffFlashIfWasNeeded() {
  if (flash_on_capture_enabled && !flashState) {
    digitalWrite(FLASH_LED_PIN, LOW);
  }
}

void CaptureCam::the_camera_loop () {
  turnOnFlashIfNeeded();
  isBusy = true;
  
  vid_fb = captureGoodJPEG();

  if (!vid_fb) {
    Serial.println("Camera capture failed");
    return;
  }
  picture_ready = true;

  if (avi_enabled) {
    frame_cnt = 0;

    ///////////////////////////// start a movie
    avi_start_time = millis();
    Serial.printf("\nStart the avi ... at %d\n", avi_start_time);
    Serial.printf("Framesize %d, quality %d, length %d seconds\n\n", framesize, quality,  max_frames * frame_interval / 1000);

    fb_next = captureGoodJPEG();                     // should take zero time
    last_frame_time = millis();
    start_avi();

    ///////////////////////////// all the frames of movie

    for (int j = 0; j < max_frames - 1 ; j++) { // max_frames
      current_frame_time = millis();

      if (current_frame_time - last_frame_time < frame_interval) {
        if (frame_cnt < 5 || frame_cnt > (max_frames - 5) )Serial.printf("frame %d, delay %d\n", frame_cnt, (int) frame_interval - (current_frame_time - last_frame_time));
        delay(frame_interval - (current_frame_time - last_frame_time));             // delay for timelapse
      }

      last_frame_time = millis();
      frame_cnt++;

      if (frame_cnt !=  1) esp_camera_fb_return(fb_curr);
      fb_curr = fb_next;           // we will write a frame, and get the camera preparing a new one

      another_save_avi(fb_curr );
      fb_next = captureGoodJPEG();               // should take near zero, unless the sd is faster than the camera, when we will have to wait for the camera

      digitalWrite(33, frame_cnt % 2);
      if (movi_size > avi_buf_size * .95) break;
    }

    ///////////////////////////// stop a movie
    Serial.println("End the Avi");

    esp_camera_fb_return(fb_curr);
    frame_cnt++;
    fb_curr = fb_next;
    fb_next = NULL;
    another_save_avi(fb_curr );
    digitalWrite(33, frame_cnt % 2);
    esp_camera_fb_return(fb_curr);
    fb_curr = NULL;
    end_avi();                                // end the movie
    digitalWrite(33, HIGH);          // light off
    avi_end_time = millis();
    float fps = 1.0 * frame_cnt / ((avi_end_time - avi_start_time) / 1000) ;
    Serial.printf("End the avi at %d.  It was %d frames, %d ms at %.2f fps...\n", millis(), frame_cnt, avi_end_time - avi_start_time, fps);
    frame_cnt = 0;             // start recording again on the next loop
    video_ready = true;
  }
  Serial.println("Deleting the camera task");
  delay(100);
  isBusy = false;
  turnOffFlashIfWasNeeded();
}

// start_avi - open the files and write in headers
void CaptureCam::start_avi() {

  Serial.println("Starting an avi ");

  time(&now);
  localtime_r(&now, &timeinfo);
  strftime(strftime_buf, sizeof(strftime_buf), "DoorCam %F %H.%M.%S.avi", &timeinfo);

  //memset(psram_avi_buf, 0, avi_buf_size);  // save some time
  //memset(psram_idx_buf, 0, idx_buf_size);

  psram_avi_ptr = 0;
  psram_idx_ptr = 0;

  memcpy(buf + 0x40, frameSizeData[framesize].frameWidth, 2);
  memcpy(buf + 0xA8, frameSizeData[framesize].frameWidth, 2);
  memcpy(buf + 0x44, frameSizeData[framesize].frameHeight, 2);
  memcpy(buf + 0xAC, frameSizeData[framesize].frameHeight, 2);

  psram_avi_ptr = psram_avi_buf;
  psram_idx_ptr = psram_idx_buf;

  memcpy( psram_avi_ptr, buf, AVIOFFSET);
  psram_avi_ptr += AVIOFFSET;

  startms = millis();

  jpeg_size = 0;
  movi_size = 0;
  uVideoLen = 0;
  idx_offset = 4;

}

//  another_save_avi saves another frame to the avi file, uodates index
//           -- pass in a fb pointer to the frame to add
//
void CaptureCam::another_save_avi(camera_fb_t * fb ) {

  int fblen;
  fblen = fb->len;

  int fb_block_length;
  uint8_t* fb_block_start;

  jpeg_size = fblen;

  remnant = (4 - (jpeg_size & 0x00000003)) & 0x00000003;

  long bw = millis();
  long frame_write_start = millis();

  memcpy(psram_avi_ptr, dc_buf, 4);

  int jpeg_size_rem = jpeg_size + remnant;

  print_quartet(jpeg_size_rem, psram_avi_ptr + 4);

  fb_block_start = fb->buf;

  if (fblen > fbs * 1024 - 8 ) {                     // fbs is the size of frame buffer static
    fb_block_length = fbs * 1024;
    fblen = fblen - (fbs * 1024 - 8);
    memcpy( psram_avi_ptr + 8, fb_block_start, fb_block_length - 8);
    fb_block_start = fb_block_start + fb_block_length - 8;
  } else {
    fb_block_length = fblen + 8  + remnant;
    memcpy( psram_avi_ptr + 8, fb_block_start, fb_block_length - 8);
    fblen = 0;
  }

  psram_avi_ptr += fb_block_length;

  while (fblen > 0) {
    if (fblen > fbs * 1024) {
      fb_block_length = fbs * 1024;
      fblen = fblen - fb_block_length;
    } else {
      fb_block_length = fblen  + remnant;
      fblen = 0;
    }

    memcpy( psram_avi_ptr, fb_block_start, fb_block_length);

    psram_avi_ptr += fb_block_length;

    fb_block_start = fb_block_start + fb_block_length;
  }

  movi_size += jpeg_size;
  uVideoLen += jpeg_size;

  print_2quartet(idx_offset, jpeg_size, psram_idx_ptr);
  psram_idx_ptr += 8;

  idx_offset = idx_offset + jpeg_size + remnant + 8;

  movi_size = movi_size + remnant;

}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//
//  end_avi writes the index, and closes the files
//
void CaptureCam::end_avi() {

  Serial.println("End of avi - closing the files");

  if (frame_cnt <  5 ) {
    Serial.println("Recording screwed up, less than 5 frames, forget index\n");
  } else {

    elapsedms = millis() - startms;

    float fRealFPS = (1000.0f * (float)frame_cnt) / ((float)elapsedms) * speed_up_factor;

    float fmicroseconds_per_frame = 1000000.0f / fRealFPS;
    uint8_t iAttainedFPS = round(fRealFPS) ;
    uint32_t us_per_frame = round(fmicroseconds_per_frame);

    //Modify the MJPEG header from the beginning of the file, overwriting various placeholders

    print_quartet(movi_size + 240 + 16 * frame_cnt + 8 * frame_cnt, psram_avi_buf + 4);
    print_quartet(us_per_frame, psram_avi_buf + 0x20);

    unsigned long max_bytes_per_sec = (1.0f * movi_size * iAttainedFPS) / frame_cnt;
    print_quartet(max_bytes_per_sec, psram_avi_buf + 0x24);
    print_quartet(frame_cnt, psram_avi_buf + 0x30);
    print_quartet(frame_cnt, psram_avi_buf + 0x8c);
    print_quartet((int)iAttainedFPS, psram_avi_buf + 0x84);
    print_quartet(movi_size + frame_cnt * 8 + 4, psram_avi_buf + 0xe8);

    Serial.println(F("\n*** Video recorded and saved ***\n"));

    Serial.printf("Recorded %5d frames in %5d seconds\n", frame_cnt, elapsedms / 1000);
    Serial.printf("File size is %u bytes\n", movi_size + 12 * frame_cnt + 4);
    Serial.printf("Adjusted FPS is %5.2f\n", fRealFPS);
    Serial.printf("Max data rate is %lu bytes/s\n", max_bytes_per_sec);
    Serial.printf("Frame duration is %d us\n", us_per_frame);
    Serial.printf("Average frame length is %d bytes\n", uVideoLen / frame_cnt);


    Serial.printf("Writng the index, %d frames\n", frame_cnt);

    memcpy (psram_avi_ptr, idx1_buf, 4);
    psram_avi_ptr += 4;

    print_quartet(frame_cnt * 16, psram_avi_ptr);
    psram_avi_ptr += 4;

    psram_idx_ptr = psram_idx_buf;

    for (int i = 0; i < frame_cnt; i++) {
      memcpy (psram_avi_ptr, dc_buf, 4);
      psram_avi_ptr += 4;
      memcpy (psram_avi_ptr, zero_buf, 4);
      psram_avi_ptr += 4;

      memcpy (psram_avi_ptr, psram_idx_ptr, 8);
      psram_avi_ptr += 8;
      psram_idx_ptr += 8;
    }
  }

  Serial.println("---");
  digitalWrite(33, HIGH);
}
