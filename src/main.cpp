#include <Arduino.h>
#include <lvgl.h>
#include "driver/gpio.h"

// ================= 1. 安全引脚定义 =================
#define PIN_WR  17
#define PIN_RS  18
#define PIN_CE  21
#define PIN_RD  38
#define PIN_RST 47

const int LCD_WIDTH = 640;
const int LCD_HEIGHT = 480;
const uint32_t DATA_MASK = 0x0001FFFE; 

#define WR_LOW()    GPIO.out_w1tc = (1 << PIN_WR)
#define WR_HIGH()   GPIO.out_w1ts = (1 << PIN_WR)
#define RS_LOW()    GPIO.out_w1tc = (1 << PIN_RS)
#define RS_HIGH()   GPIO.out_w1ts = (1 << PIN_RS)
#define CE_LOW()    GPIO.out_w1tc = (1 << PIN_CE)
#define CE_HIGH()   GPIO.out_w1ts = (1 << PIN_CE)

// ================= 2. 底层驱动 =================
static lv_disp_draw_buf_t draw_buf;
static lv_color_t *buf_1;
static lv_color_t *buf_2;

void LCD_GPIO_Init() {
    for(int i = 1; i <= 18; i++) pinMode(i, OUTPUT);
    pinMode(PIN_CE, OUTPUT); pinMode(PIN_RD, OUTPUT); pinMode(PIN_RST, OUTPUT);
    CE_HIGH(); WR_HIGH(); digitalWrite(PIN_RD, HIGH); 
    digitalWrite(PIN_RST, LOW); delay(50);
    digitalWrite(PIN_RST, HIGH); delay(120);
}

void IRAM_ATTR my_disp_flush(lv_disp_drv_t *disp_drv, const lv_area_t *area, lv_color_t *color_p) {
    CE_LOW(); 
    for (int y = area->y1; y <= area->y2; y++) {
        RS_LOW(); GPIO.out_w1tc = DATA_MASK; GPIO.out_w1ts = (0x0020 << 1); WR_LOW(); WR_HIGH();
        RS_HIGH(); GPIO.out_w1tc = DATA_MASK; GPIO.out_w1ts = (y << 1); WR_LOW(); WR_HIGH();
        RS_LOW(); GPIO.out_w1tc = DATA_MASK; GPIO.out_w1ts = (0x0021 << 1); WR_LOW(); WR_HIGH();
        RS_HIGH(); GPIO.out_w1tc = DATA_MASK; GPIO.out_w1ts = (area->x1 << 1); WR_LOW(); WR_HIGH();
        RS_LOW(); GPIO.out_w1tc = DATA_MASK; GPIO.out_w1ts = (0x0022 << 1); WR_LOW(); WR_HIGH();

        RS_HIGH(); 
        int width = area->x2 - area->x1 + 1;
        for (int i = 0; i < width; i++) {
            uint16_t color = color_p->full;
            GPIO.out_w1tc = DATA_MASK; GPIO.out_w1ts = (color << 1);
            WR_LOW(); WR_HIGH();
            color_p++; 
        }
    }
    CE_HIGH(); 
    lv_disp_flush_ready(disp_drv); 
}

// ================= 3. 实时滚动终端 UI =================
lv_obj_t * list_view; // 全局列表对象，方便在 loop 里插入数据
int packet_count = 0; // 模拟抓包数量

void build_radar_terminal() {
    // 极客风深灰背景
    lv_obj_set_style_bg_color(lv_scr_act(), lv_color_hex(0x121212), 0);

    // 顶部状态栏
    lv_obj_t * header = lv_obj_create(lv_scr_act());
    lv_obj_set_size(header, LCD_WIDTH, 40);
    lv_obj_set_style_bg_color(header, lv_color_hex(0x004d40), 0); // 深绿色
    lv_obj_set_style_border_width(header, 0, 0);
    lv_obj_set_style_radius(header, 0, 0);
    lv_obj_align(header, LV_ALIGN_TOP_MID, 0, 0);
    
    lv_obj_t * title = lv_label_create(header);
    lv_label_set_text(title, "ESP32-S3 BLE Drone Scanner - ACTV");
    lv_obj_set_style_text_color(title, lv_color_hex(0x00e676), 0); // 亮绿色
    lv_obj_center(title);

    // 中间的滚动数据列表区
    list_view = lv_list_create(lv_scr_act());
    lv_obj_set_size(list_view, LCD_WIDTH - 20, LCD_HEIGHT - 60);
    lv_obj_align(list_view, LV_ALIGN_BOTTOM_MID, 0, -10);
    
    // 定制列表的赛博朋克样式：透明背景，绿色边框
    lv_obj_set_style_bg_opa(list_view, 0, 0); 
    lv_obj_set_style_border_color(list_view, lv_color_hex(0x00e676), 0);
    lv_obj_set_style_border_width(list_view, 2, 0);
    lv_obj_set_style_text_color(list_view, lv_color_hex(0x69f0ae), 0); // 柔和的绿色文本
    
    // 初始化时加一条开机信息
    lv_list_add_text(list_view, "[SYS] Scanning initialized. Waiting for packets...");
}

// 模拟接收蓝牙数据并插入列表
void simulate_incoming_data() {
    packet_count++;
    
    char buffer[128];
    // 随机生成假数据：MAC 地址、信号强度、高度
    int rssi = -1 * random(40, 95);
    int alt = random(10, 150);
    int mac_tail = random(0x10, 0xFF);
    
    sprintf(buffer, "[%04d] MAC: AA:BB:CC:DD:EE:%02X | RSSI: %d dBm | ALT: %dm", packet_count, mac_tail, rssi, alt);
    
    // 动态添加一行文字到列表
    lv_list_add_text(list_view, buffer);
    
    // 为了防止内存爆炸，如果列表太长，我们删掉最老的（或者直接让它滚动）
    // LVGL 的 list_add 会自动处理滚动，这里我们为了演示效果，让它自动滚到底部
    lv_obj_scroll_to_y(list_view, 10000, LV_ANIM_ON); 
}


// ================= 4. 主程序 =================
uint32_t last_sim_time = 0;

void setup() {
    Serial.begin(115200);
    delay(2000);
    LCD_GPIO_Init();
    lv_init();

    uint32_t buf_size = LCD_WIDTH * LCD_HEIGHT / 10; 
    buf_1 = (lv_color_t *)heap_caps_malloc(buf_size * sizeof(lv_color_t), MALLOC_CAP_SPIRAM);
    buf_2 = (lv_color_t *)heap_caps_malloc(buf_size * sizeof(lv_color_t), MALLOC_CAP_SPIRAM);
    
    lv_disp_draw_buf_init(&draw_buf, buf_1, buf_2, buf_size);

    static lv_disp_drv_t disp_drv;
    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res = LCD_WIDTH;
    disp_drv.ver_res = LCD_HEIGHT;
    disp_drv.flush_cb = my_disp_flush; 
    disp_drv.draw_buf = &draw_buf;
    lv_disp_drv_register(&disp_drv);

    build_radar_terminal();
    last_sim_time = millis();
}

void loop() {
    lv_timer_handler(); 
    
    // 每隔 800 毫秒，模拟收到一条无人机数据
    if (millis() - last_sim_time > 800) {
        simulate_incoming_data();
        last_sim_time = millis();
    }
    
    delay(5); 
}