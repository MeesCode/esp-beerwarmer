
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"


struct oled_gfx
{   
    esp_lcd_panel_handle_t panel_handle;
    int width;
    int height;
};


void gfx_init(esp_lcd_panel_handle_t panel_handle, int width, int height);
void gfx_draw_bitmap(int x, int y, int w, int h, const char *bitmap);
void gfx_draw_text(int x, int y, const char *text);
void gfx_clear_area(int x, int y, int w, int h);
void gfx_fill_area(int x, int y, int w, int h);
void gfx_set_pixel(uint8_t x, uint8_t y);
void gfx_flush();
void gfx_clear_pixel(uint8_t x, uint8_t y);