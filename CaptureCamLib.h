#include "esp_camera.h"
#include "esp_wifi.h"
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include "esp_system.h"
#include "CamPins.h"
#include <Arduino.h>

enum ClipTypes {
    FAST, MED, SLOW, VSLOW
};

struct frameSizeStruct {
        uint8_t frameWidth[2];
        uint8_t frameHeight[2];
    };

//  data structure from here https://github.com/s60sc/ESP32-CAM_MJPEG2SD/blob/master/avi.cpp, extended for ov5640
const static frameSizeStruct frameSizeData[] = {
    {{0x60, 0x00}, {0x60, 0x00}}, // FRAMESIZE_96X96,    // 96x96
    {{0xA0, 0x00}, {0x78, 0x00}}, // FRAMESIZE_QQVGA,    // 160x120
    {{0xB0, 0x00}, {0x90, 0x00}}, // FRAMESIZE_QCIF,     // 176x144
    {{0xF0, 0x00}, {0xB0, 0x00}}, // FRAMESIZE_HQVGA,    // 240x176
    {{0xF0, 0x00}, {0xF0, 0x00}}, // FRAMESIZE_240X240,  // 240x240
    {{0x40, 0x01}, {0xF0, 0x00}}, // FRAMESIZE_QVGA,     // 320x240
    {{0x90, 0x01}, {0x28, 0x01}}, // FRAMESIZE_CIF,      // 400x296
    {{0xE0, 0x01}, {0x40, 0x01}}, // FRAMESIZE_HVGA,     // 480x320
    {{0x80, 0x02}, {0xE0, 0x01}}, // FRAMESIZE_VGA,      // 640x480   8
    {{0x20, 0x03}, {0x58, 0x02}}, // FRAMESIZE_SVGA,     // 800x600   9
    {{0x00, 0x04}, {0x00, 0x03}}, // FRAMESIZE_XGA,      // 1024x768  10
    {{0x00, 0x05}, {0xD0, 0x02}}, // FRAMESIZE_HD,       // 1280x720  11
    {{0x00, 0x05}, {0x00, 0x04}}, // FRAMESIZE_SXGA,     // 1280x1024 12
    {{0x40, 0x06}, {0xB0, 0x04}}, // FRAMESIZE_UXGA,     // 1600x1200 13
    // 3MP Sensors
    {{0x80, 0x07}, {0x38, 0x04}}, // FRAMESIZE_FHD,      // 1920x1080 14
    {{0xD0, 0x02}, {0x00, 0x05}}, // FRAMESIZE_P_HD,     //  720x1280 15
    {{0x60, 0x03}, {0x00, 0x06}}, // FRAMESIZE_P_3MP,    //  864x1536 16
    {{0x00, 0x08}, {0x00, 0x06}}, // FRAMESIZE_QXGA,     // 2048x1536 17
    // 5MP Sensors
    {{0x00, 0x0A}, {0xA0, 0x05}}, // FRAMESIZE_QHD,      // 2560x1440 18
    {{0x00, 0x0A}, {0x40, 0x06}}, // FRAMESIZE_WQXGA,    // 2560x1600 19
    {{0x38, 0x04}, {0x80, 0x07}}, // FRAMESIZE_P_FHD,    // 1080x1920 20
    {{0x00, 0x0A}, {0x80, 0x07}}  // FRAMESIZE_QSXGA,    // 2560x1920 21
    };

class CaptureCam {
public:     
    CaptureCam(framesize_t configframesize,      
      int framesize,
      int quality,
      int qualityconfig);

    //Used for initializing AI THINKER
    bool setupCamera();
    
    //Used for interval (timed) photos
    void takeTimedPhotoIfNeed();

    //Used for making videos/avi
    TaskHandle_t the_camera_loop_task;
    void the_camera_loop();

    //Used for taking pictures/photos
    bool takePhoto();    
    camera_fb_t * captureGoodJPEG();

    //Need for the photo/picture transfer
    static bool isMoreDataAvailable();
    static uint8_t getNextByte();  
    inline static size_t fb_length;
    void prepareImageBuffAndPointer();
    void releaseImageBuffAndPointer();

    //Need for the video/avi transfer
    static bool avi_more();
    static uint8_t avi_next();
    char strftime_buf[64]; //Video name
    inline static size_t avi_len; //Buffer length
    void prepareVideoBuffAndPointer();
    void releaseVideoBuffAndPointer();
    
    //Flags for controlling photo/video/flash/time   states
    void setClipType(ClipTypes clipType);
    String getStringClipType();
    bool tim_enabled = false;
    int timePhoto_Minutes = 10;
    int timePhoto_lasttime;
    bool isProcessingVideoOrPhoto();
    
    //Need for motion capture
    bool captureAndIsMotionDetected();

    bool video_ready = false;
    bool timed_picture_ready = false;
    bool picture_ready = false;
    bool flash_on_capture_enabled = false;
    bool flashState = false;

private:
    //Need for motion capture
    #define WIDTH 320                 // Resolution Width
    #define HEIGHT 240                // Resolution height
    #define BLOCK_SIZE 96              // Size of each sensor block on the display (reduced granularity for speed)
    #define W (WIDTH / BLOCK_SIZE)
    #define H (HEIGHT / BLOCK_SIZE)
    #define BLOCK_DIFF_THRESHOLD 1.5
    #define IMAGE_DIFF_THRESHOLD 0.1
    #define VIEWPORT_PIXELS WIDTH / BLOCK_SIZE
    uint16_t prevFrame[H][W] = {0};
    uint16_t currentFrame[H][W] = {0};
    long motionView[VIEWPORT_PIXELS];
    bool captureStill();
    bool motionDetect();
    void updateFrame();

    camera_config_t config;
    ClipTypes clipType = MED;
    inline static bool isBusy = false;
    bool takePhoto(bool changePictureFlag);    
    int max_frames = 300;
    int frame_interval;  
    float speed_up_factor;  
    inline static int currentByte;
    camera_fb_t * vid_fb = NULL;
    framesize_t configframesize;
    int framesize; 
    int quality;
    int qualityconfig;

    uint8_t * psram_avi_ptr = 0;
    uint8_t * psram_avi_buf = NULL;

    bool initialSetupCamera();

    bool dataAvailable;
   
    inline static uint8_t* fb_buffer;

    
    camera_fb_t * fb = NULL;

    
    void IRAM_ATTR PIR_ISR(void* arg);


    bool active_interupt = false;
    bool hdcam = true;

    bool avi_enabled = true;
    int avi_buf_size = 0;
    int idx_buf_size = 0;

    uint8_t * psram_idx_buf = NULL;
    uint8_t * psram_idx_ptr = 0;


    void setupVideoSettings();

    // AVI
    inline static int avi_ptr;
    inline static uint8_t* avi_buf;


    void start_avi();
    void another_save_avi(camera_fb_t * fb );
    void end_avi();

    struct tm timeinfo;
    time_t now;

    camera_fb_t * fb_curr = NULL;
    camera_fb_t * fb_next = NULL;

    #define fbs 8 // how many kb of static ram for psram -> sram buffer for sd write - not really used because not dma for sd

    char avi_file_name[100];
    long avi_start_time = 0;
    long avi_end_time = 0;
    int start_record = 0;
    long current_frame_time;
    long last_frame_time;

//static
    int i = 0;
    uint16_t frame_cnt = 0;
    uint16_t remnant = 0;
    uint32_t length = 0;
    uint32_t startms;
    uint32_t elapsedms;
    uint32_t uVideoLen = 0;

    unsigned long movi_size = 0;
    unsigned long jpeg_size = 0;
    unsigned long idx_offset = 0;

    uint8_t zero_buf[4] = {0x00, 0x00, 0x00, 0x00};
    uint8_t dc_buf[4] = {0x30, 0x30, 0x64, 0x63};    // "00dc"
    uint8_t avi1_buf[4] = {0x41, 0x56, 0x49, 0x31};    // "AVI1"
    uint8_t idx1_buf[4] = {0x69, 0x64, 0x78, 0x31};    // "idx1"

    #define AVIOFFSET 240 // AVI main header length

    uint8_t buf[AVIOFFSET] = {
    0x52, 0x49, 0x46, 0x46, 0xD8, 0x01, 0x0E, 0x00, 0x41, 0x56, 0x49, 0x20, 0x4C, 0x49, 0x53, 0x54,
    0xD0, 0x00, 0x00, 0x00, 0x68, 0x64, 0x72, 0x6C, 0x61, 0x76, 0x69, 0x68, 0x38, 0x00, 0x00, 0x00,
    0xA0, 0x86, 0x01, 0x00, 0x80, 0x66, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00,
    0x64, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x80, 0x02, 0x00, 0x00, 0xe0, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x4C, 0x49, 0x53, 0x54, 0x84, 0x00, 0x00, 0x00,
    0x73, 0x74, 0x72, 0x6C, 0x73, 0x74, 0x72, 0x68, 0x30, 0x00, 0x00, 0x00, 0x76, 0x69, 0x64, 0x73,
    0x4D, 0x4A, 0x50, 0x47, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x01, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0A, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x73, 0x74, 0x72, 0x66,
    0x28, 0x00, 0x00, 0x00, 0x28, 0x00, 0x00, 0x00, 0x80, 0x02, 0x00, 0x00, 0xe0, 0x01, 0x00, 0x00,
    0x01, 0x00, 0x18, 0x00, 0x4D, 0x4A, 0x50, 0x47, 0x00, 0x84, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x49, 0x4E, 0x46, 0x4F,
    0x10, 0x00, 0x00, 0x00, 0x6A, 0x61, 0x6D, 0x65, 0x73, 0x7A, 0x61, 0x68, 0x61, 0x72, 0x79, 0x20,
    0x76, 0x39, 0x36, 0x20, 0x4C, 0x49, 0x53, 0x54, 0x00, 0x01, 0x0E, 0x00, 0x6D, 0x6F, 0x76, 0x69,
    };

    void print_quartet(unsigned long i, uint8_t * fd);
    void print_2quartet(unsigned long i, unsigned long j, uint8_t * fd);

    void turnOnFlashIfNeeded();
    void turnOffFlashIfWasNeeded();
};
