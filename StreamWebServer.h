#include "esp_camera.h"
#include <WiFi.h>
#include "esp_timer.h"
#include "img_converters.h"
#include "Arduino.h"
#include "fb_gfx.h"
#include "soc/soc.h" //disable brownout problems
#include "soc/rtc_cntl_reg.h"  //disable brownout problems
#include "esp_http_server.h"
#include "CamPins.h"
#include "WebServerIndex.h"

typedef struct {
        httpd_req_t *req;
        size_t len;
} jpg_chunking_t;

class StreamWebServer {
public:
    inline static bool enable;
    inline static bool flashEnabled;
    inline static bool motionState = false;
    inline static long motionStateRepeatCounter = 0;
    
    bool isStreamingRightNow();
    StreamWebServer(bool enable);
    void startCameraServer();

private:       
    inline static long startTimeMillis;
    inline static bool streamingRightNow = false;
    httpd_handle_t stream_httpd = NULL;
    httpd_handle_t camera_httpd = NULL;

    #define PART_BOUNDARY "123456789000000000000987654321"
    static constexpr const char* _STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
    static constexpr const char* _STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
    static constexpr const char* _STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

    static size_t jpg_encode_stream(void * arg, size_t index, const void* data, size_t len);
    
    static esp_err_t stream_handler(httpd_req_t *req);
    static esp_err_t index_handler(httpd_req_t *req);
    static esp_err_t state_handler(httpd_req_t *req);
    static esp_err_t change_flash_state_handler(httpd_req_t *req);
};