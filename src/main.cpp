#include <Arduino.h>
#include <lvgl.h>
#include "driver/gpio.h"
#include <SPI.h>
#include <Preferences.h>

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
SPIClass touchSPI(FSPI);

// ================= 2. 校准参数与 NVS 存储 =================
Preferences prefs;
#define FORCE_CALIBRATE 0 

int cal_raw_x_min = 0, cal_raw_x_max = 4095;
int cal_raw_y_min = 0, cal_raw_y_max = 4095;

// ================= 3. 屏幕底层驱动 =================
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

void LCD_FillRect(int x1, int y1, int w, int h, uint16_t color) {
    CE_LOW();
    for (int y = y1; y < y1 + h; y++) {
        RS_LOW(); GPIO.out_w1tc = DATA_MASK; GPIO.out_w1ts = (0x0020 << 1); WR_LOW(); WR_HIGH();
        RS_HIGH(); GPIO.out_w1tc = DATA_MASK; GPIO.out_w1ts = (y << 1); WR_LOW(); WR_HIGH();
        RS_LOW(); GPIO.out_w1tc = DATA_MASK; GPIO.out_w1ts = (0x0021 << 1); WR_LOW(); WR_HIGH();
        RS_HIGH(); GPIO.out_w1tc = DATA_MASK; GPIO.out_w1ts = (x1 << 1); WR_LOW(); WR_HIGH();
        RS_LOW(); GPIO.out_w1tc = DATA_MASK; GPIO.out_w1ts = (0x0022 << 1); WR_LOW(); WR_HIGH();
        RS_HIGH();
        GPIO.out_w1tc = DATA_MASK; GPIO.out_w1ts = (color << 1);
        for (int i = 0; i < w; i++) { WR_LOW(); WR_HIGH(); }
    }
    CE_HIGH();
}

void LCD_DrawCross(int x, int y, uint16_t color) {
    LCD_FillRect(x - 15, y - 1, 31, 3, color); 
    LCD_FillRect(x - 1, y - 15, 3, 31, color); 
}

// ================= 4. 触摸底层与滤波 =================
uint16_t xpt2046_read(uint8_t cmd) {
    digitalWrite(T_CS, LOW);
    touchSPI.transfer(cmd);
    uint16_t data = touchSPI.transfer16(0x0000) >> 3; 
    digitalWrite(T_CS, HIGH);
    return data;
}

bool Touch_GetRawAvg(int *x, int *y) {
    long rx = 0, ry = 0;
    int count = 0;
    touchSPI.beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE0));
    for(int i=0; i<15; i++) { 
        int tmpx = xpt2046_read(0xD0);
        int tmpy = xpt2046_read(0x90);
        if(tmpx > 200 && tmpx < 3900 && tmpy > 200 && tmpy < 3900) {
            rx += tmpx; ry += tmpy; count++;
        }
    }
    touchSPI.endTransaction();
    if(count > 8) { 
        *x = rx / count; *y = ry / count;
        return true;
    }
    return false;
}

void my_touchpad_read(lv_indev_drv_t * indev_drv, lv_indev_data_t * data) {
    int raw_x, raw_y;
    if (Touch_GetRawAvg(&raw_x, &raw_y)) {
        int mapped_x = map(raw_x, cal_raw_x_min, cal_raw_x_max, 64, LCD_WIDTH - 64);
        int mapped_y = map(raw_y, cal_raw_y_min, cal_raw_y_max, 48, LCD_HEIGHT - 48);

        if (mapped_x < 0) mapped_x = 0; else if (mapped_x > LCD_WIDTH - 1) mapped_x = LCD_WIDTH - 1;
        if (mapped_y < 0) mapped_y = 0; else if (mapped_y > LCD_HEIGHT - 1) mapped_y = LCD_HEIGHT - 1;

        data->point.x = mapped_x; data->point.y = mapped_y;
        data->state = LV_INDEV_STATE_PR; 
    } else {
        data->state = LV_INDEV_STATE_REL; 
    }
}

// ================= 5. 十字校准逻辑 =================
void Wait_For_Touch(int *x, int *y) {
    while(!Touch_GetRawAvg(x, y)) { delay(10); }
    int stable_x = 0, stable_y = 0;
    for(int i=0; i<10; i++) {
        int tx, ty;
        if(Touch_GetRawAvg(&tx, &ty)) { stable_x += tx; stable_y += ty; }
        delay(10);
    }
    *x = stable_x / 10; *y = stable_y / 10;
    int dummy_x, dummy_y;
    while(Touch_GetRawAvg(&dummy_x, &dummy_y)) { delay(10); }
    delay(200); 
}

void Perform_Calibration() {
    Serial.println("Starting Touch Calibration...");
    LCD_FillRect(0, 0, LCD_WIDTH, LCD_HEIGHT, 0x0000); 
    
    LCD_DrawCross(64, 48, 0xF800); 
    Wait_For_Touch(&cal_raw_x_min, &cal_raw_y_min);
    LCD_DrawCross(64, 48, 0x07E0); 
    delay(500);

    LCD_DrawCross(LCD_WIDTH - 64, LCD_HEIGHT - 48, 0xF800);
    Wait_For_Touch(&cal_raw_x_max, &cal_raw_y_max);
    LCD_DrawCross(LCD_WIDTH - 64, LCD_HEIGHT - 48, 0x07E0);
    
    prefs.putInt("cal_xmin", cal_raw_x_min);
    prefs.putInt("cal_xmax", cal_raw_x_max);
    prefs.putInt("cal_ymin", cal_raw_y_min);
    prefs.putInt("cal_ymax", cal_raw_y_max);
    
    Serial.println("Calibration Saved!");
    LCD_FillRect(0, 0, LCD_WIDTH, LCD_HEIGHT, 0x0000); 
}

// ================= 6. LVGL 测试 UI =================
static void btn_event_cb(lv_event_t * e) {
    lv_obj_t * btn = lv_event_get_target(e);
    lv_obj_t * label = lv_obj_get_child(btn, 0); 
    if (lv_event_get_code(e) == LV_EVENT_CLICKED) {
        lv_label_set_text(label, "Target Acquired!");
        lv_obj_set_style_bg_color(btn, lv_color_hex(0x00FF00), 0); 
    }
}

void build_ui() {
    lv_obj_set_style_bg_color(lv_scr_act(), lv_color_hex(0x1e1e1e), 0);

    lv_obj_t * title = lv_label_create(lv_scr_act());
    lv_label_set_text(title, "Precision Touch Test");
    lv_obj_set_style_text_color(title, lv_color_hex(0x00E676), 0);
    lv_obj_align(title, LV_ALIGN_CENTER, 0, -80);

    // 在屏幕四个角落放上几个小按钮，测试边缘准度
    lv_obj_t * btn_tl = lv_btn_create(lv_scr_act()); lv_obj_set_size(btn_tl, 60, 40); lv_obj_align(btn_tl, LV_ALIGN_TOP_LEFT, 10, 10);
    lv_obj_t * btn_tr = lv_btn_create(lv_scr_act()); lv_obj_set_size(btn_tr, 60, 40); lv_obj_align(btn_tr, LV_ALIGN_TOP_RIGHT, -10, 10);
    lv_obj_t * btn_bl = lv_btn_create(lv_scr_act()); lv_obj_set_size(btn_bl, 60, 40); lv_obj_align(btn_bl, LV_ALIGN_BOTTOM_LEFT, 10, -10);
    lv_obj_t * btn_br = lv_btn_create(lv_scr_act()); lv_obj_set_size(btn_br, 60, 40); lv_obj_align(btn_br, LV_ALIGN_BOTTOM_RIGHT, -10, -10);

    lv_obj_t * btn = lv_btn_create(lv_scr_act());
    lv_obj_set_size(btn, 200, 80);
    lv_obj_align(btn, LV_ALIGN_CENTER, 0, 20);
    lv_obj_add_event_cb(btn, btn_event_cb, LV_EVENT_ALL, NULL);

    lv_obj_t * label = lv_label_create(btn);
    lv_label_set_text(label, "Drag Finger Here");
    lv_obj_center(label);
}

// ================= 7. 主程序 =================
void setup() {
    Serial.begin(115200);
    delay(2000);
    LCD_GPIO_Init();
    
    pinMode(T_CS, OUTPUT); digitalWrite(T_CS, HIGH);
    touchSPI.begin(T_CLK, T_MISO, T_MOSI, T_CS);

    prefs.begin("touch", false);
    if (FORCE_CALIBRATE || !prefs.isKey("cal_xmin")) {
        Perform_Calibration();
    } else {
        cal_raw_x_min = prefs.getInt("cal_xmin", 0);
        cal_raw_x_max = prefs.getInt("cal_xmax", 4095);
        cal_raw_y_min = prefs.getInt("cal_ymin", 0);
        cal_raw_y_max = prefs.getInt("cal_ymax", 4095);
    }

    lv_init();
    uint32_t buf_size = LCD_WIDTH * LCD_HEIGHT / 10; 
    buf_1 = (lv_color_t *)heap_caps_malloc(buf_size * sizeof(lv_color_t), MALLOC_CAP_SPIRAM);
    buf_2 = (lv_color_t *)heap_caps_malloc(buf_size * sizeof(lv_color_t), MALLOC_CAP_SPIRAM);
    lv_disp_draw_buf_init(&draw_buf, buf_1, buf_2, buf_size);

    static lv_disp_drv_t disp_drv;
    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res = LCD_WIDTH; disp_drv.ver_res = LCD_HEIGHT;
    disp_drv.flush_cb = my_disp_flush; disp_drv.draw_buf = &draw_buf;
    lv_disp_drv_register(&disp_drv);

    static lv_indev_drv_t indev_drv;
    lv_indev_drv_init(&indev_drv);
    indev_drv.type = LV_INDEV_TYPE_POINTER; 
    indev_drv.read_cb = my_touchpad_read;   
    
    // --- 核心：将光标追踪器绑定到输入设备 ---
    lv_indev_t * my_indev = lv_indev_drv_register(&indev_drv);

    lv_obj_t * cursor_obj = lv_obj_create(lv_scr_act());
    lv_obj_set_size(cursor_obj, 12, 12);
    lv_obj_set_style_bg_color(cursor_obj, lv_color_hex(0xFF0000), 0); // 红色光标点
    lv_obj_set_style_border_width(cursor_obj, 2, 0);
    lv_obj_set_style_border_color(cursor_obj, lv_color_hex(0xFFFFFF), 0); // 白色描边
    lv_obj_set_style_radius(cursor_obj, LV_RADIUS_CIRCLE, 0); // 圆形
    // 移除点击属性，防止光标阻挡下方的按钮
    lv_obj_clear_flag(cursor_obj, LV_OBJ_FLAG_CLICKABLE); 

    lv_indev_set_cursor(my_indev, cursor_obj);

    build_ui();
}

void loop() {
    lv_timer_handler(); 
    delay(5); 
}