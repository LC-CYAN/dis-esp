#include <Arduino.h>
#include <lvgl.h>
#include "driver/gpio.h"
#include <SPI.h>

// ================= 1. 屏幕与触摸安全引脚定义 =================
#define PIN_WR  17
#define PIN_RS  18
#define PIN_CE  21
#define PIN_RD  38
#define PIN_RST 47

#define T_CLK   45 
#define T_MOSI  46
#define T_MISO  48
#define T_CS    42  

const int LCD_WIDTH = 640;
const int LCD_HEIGHT = 480;
const uint32_t DATA_MASK = 0x0001FFFE; 

#define WR_LOW()    GPIO.out_w1tc = (1 << PIN_WR)
#define WR_HIGH()   GPIO.out_w1ts = (1 << PIN_WR)
#define RS_LOW()    GPIO.out_w1tc = (1 << PIN_RS)
#define RS_HIGH()   GPIO.out_w1ts = (1 << PIN_RS)
#define CE_LOW()    GPIO.out_w1tc = (1 << PIN_CE)
#define CE_HIGH()   GPIO.out_w1ts = (1 << PIN_CE)

static lv_disp_draw_buf_t draw_buf;
static lv_color_t *buf_1;
static lv_color_t *buf_2;

// 全局 SPI 对象
SPIClass touchSPI(FSPI);

// ================= 2. 手搓极简 XPT2046 驱动 (完美支持自定义引脚) =================
// 读取 XPT2046 寄存器的底层函数
uint16_t xpt2046_read(uint8_t cmd) {
    uint16_t data = 0;
    digitalWrite(T_CS, LOW);
    touchSPI.transfer(cmd);
    data = touchSPI.transfer16(0x0000) >> 3; // 读 16 位，XPT2046 数据在头 12 位
    digitalWrite(T_CS, HIGH);
    return data;
}

// 供 LVGL 调用的读取回调
void my_touchpad_read(lv_indev_drv_t * indev_drv, lv_indev_data_t * data) {
    // XPT2046 命令控制字节：
    // 0xD0 (11010000) -> 读 X 坐标 (12位, 差分)
    // 0x90 (10010000) -> 读 Y 坐标 (12位, 差分)
    
    // 连续读几次求个平均，防止屏幕噪点干扰
    int x = 0, y = 0;
    int samples = 3;
    
    touchSPI.beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE0)); // 触摸 SPI 速度 2MHz
    for(int i=0; i<samples; i++) {
        x += xpt2046_read(0xD0);
        y += xpt2046_read(0x90);
    }
    touchSPI.endTransaction();

    x /= samples;
    y /= samples;

    // 判断是否被按下：如果读出的 ADC 值接近 0 或者满载 4095，说明没按或者读到了杂波
    // 正常按下的 ADC 值通常在 200 ~ 3900 之间
    if (x > 200 && x < 3900 && y > 200 && y < 3900) {
        
        // --- 坐标映射 (需要根据你的屏幕实际方向微调) ---
        // 假设 X 方向范围是 300~3800，映射到 0~640
        // 假设 Y 方向范围是 300~3800，映射到 0~480
        int mapped_x = map(x, 300, 3800, 0, LCD_WIDTH);
        int mapped_y = map(y, 300, 3800, 0, LCD_HEIGHT);

        // 边界保护
        if (mapped_x < 0) mapped_x = 0; else if (mapped_x > LCD_WIDTH - 1) mapped_x = LCD_WIDTH - 1;
        if (mapped_y < 0) mapped_y = 0; else if (mapped_y > LCD_HEIGHT - 1) mapped_y = LCD_HEIGHT - 1;

        data->point.x = mapped_x;
        data->point.y = mapped_y;
        data->state = LV_INDEV_STATE_PR; 
        
        // 调试用：如果点击没反应，把下面这行注释打开，看串口读到了什么原始数据
        // Serial.printf("Raw X:%d Y:%d | Mapped X:%d Y:%d\n", x, y, mapped_x, mapped_y);
    } else {
        data->state = LV_INDEV_STATE_REL; 
    }
}

// ================= 3. 屏幕极速刷新引擎 =================
void LCD_GPIO_Init() {
    for(int i = 1; i <= 18; i++) pinMode(i, OUTPUT);
    pinMode(PIN_CE, OUTPUT); pinMode(PIN_RD, OUTPUT); pinMode(PIN_RST, OUTPUT);
    CE_HIGH(); WR_HIGH(); digitalWrite(PIN_RD, HIGH); 
    digitalWrite(PIN_RST, LOW); delay(50);
    digitalWrite(PIN_RST, HIGH); delay(120);
}

void IRAM_ATTR my_disp_flush(lv_disp_drv_t *disp_drv, const lv_area_t *area, lv_color_t *color_p) {
    CE_LOW(); 
    int width = area->x2 - area->x1 + 1;
    for (int y = area->y1; y <= area->y2; y++) {
        RS_LOW(); GPIO.out_w1tc = DATA_MASK; GPIO.out_w1ts = (0x0020 << 1); WR_LOW(); WR_HIGH();
        RS_HIGH(); GPIO.out_w1tc = DATA_MASK; GPIO.out_w1ts = (y << 1); WR_LOW(); WR_HIGH();
        RS_LOW(); GPIO.out_w1tc = DATA_MASK; GPIO.out_w1ts = (0x0021 << 1); WR_LOW(); WR_HIGH();
        RS_HIGH(); GPIO.out_w1tc = DATA_MASK; GPIO.out_w1ts = (area->x1 << 1); WR_LOW(); WR_HIGH();
        RS_LOW(); GPIO.out_w1tc = DATA_MASK; GPIO.out_w1ts = (0x0022 << 1); WR_LOW(); WR_HIGH();

        RS_HIGH(); 
        int w = width;
        while (w >= 8) {
            GPIO.out_w1tc = DATA_MASK; GPIO.out_w1ts = (color_p[0].full << 1); WR_LOW(); WR_HIGH();
            GPIO.out_w1tc = DATA_MASK; GPIO.out_w1ts = (color_p[1].full << 1); WR_LOW(); WR_HIGH();
            GPIO.out_w1tc = DATA_MASK; GPIO.out_w1ts = (color_p[2].full << 1); WR_LOW(); WR_HIGH();
            GPIO.out_w1tc = DATA_MASK; GPIO.out_w1ts = (color_p[3].full << 1); WR_LOW(); WR_HIGH();
            GPIO.out_w1tc = DATA_MASK; GPIO.out_w1ts = (color_p[4].full << 1); WR_LOW(); WR_HIGH();
            GPIO.out_w1tc = DATA_MASK; GPIO.out_w1ts = (color_p[5].full << 1); WR_LOW(); WR_HIGH();
            GPIO.out_w1tc = DATA_MASK; GPIO.out_w1ts = (color_p[6].full << 1); WR_LOW(); WR_HIGH();
            GPIO.out_w1tc = DATA_MASK; GPIO.out_w1ts = (color_p[7].full << 1); WR_LOW(); WR_HIGH();
            color_p += 8; w -= 8;
        }
        while (w > 0) {
            GPIO.out_w1tc = DATA_MASK; GPIO.out_w1ts = (color_p->full << 1); WR_LOW(); WR_HIGH();
            color_p++; w--;
        }
    }
    CE_HIGH(); 
    lv_disp_flush_ready(disp_drv); 
}


// ================= 4. LVGL 测试 UI =================
static void btn_event_cb(lv_event_t * e) {
    lv_obj_t * btn = lv_event_get_target(e);
    lv_obj_t * label = lv_obj_get_child(btn, 0); 
    if (lv_event_get_code(e) == LV_EVENT_CLICKED) {
        lv_label_set_text(label, "Success!");
        lv_obj_set_style_bg_color(btn, lv_color_hex(0x00FF00), 0); 
        Serial.println("LVGL Hit!");
    }
}

void build_ui() {
    lv_obj_set_style_bg_color(lv_scr_act(), lv_color_hex(0x1a1a1a), 0);
    lv_obj_t * title = lv_label_create(lv_scr_act());
    lv_label_set_text(title, "Custom SPI Touch Test");
    lv_obj_set_style_text_color(title, lv_color_hex(0xFFFFFF), 0);
    lv_obj_align(title, LV_ALIGN_CENTER, 0, -80);

    lv_obj_t * btn = lv_btn_create(lv_scr_act());
    lv_obj_set_size(btn, 200, 80);
    lv_obj_align(btn, LV_ALIGN_CENTER, 0, 20);
    lv_obj_add_event_cb(btn, btn_event_cb, LV_EVENT_ALL, NULL);

    lv_obj_t * label = lv_label_create(btn);
    lv_label_set_text(label, "PRESS ME");
    lv_obj_center(label);
}

// ================= 5. 主程序 =================
void setup() {
    Serial.begin(115200);
    delay(2000);
    LCD_GPIO_Init();
    
    // 初始化触摸硬件
    pinMode(T_CS, OUTPUT);
    digitalWrite(T_CS, HIGH);
    touchSPI.begin(T_CLK, T_MISO, T_MOSI, T_CS);

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

    static lv_indev_drv_t indev_drv;
    lv_indev_drv_init(&indev_drv);
    indev_drv.type = LV_INDEV_TYPE_POINTER; 
    indev_drv.read_cb = my_touchpad_read;   
    lv_indev_drv_register(&indev_drv);

    build_ui();
}

void loop() {
    lv_timer_handler(); 
    delay(5); 
}