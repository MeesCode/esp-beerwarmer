
#include "oled_gfx.h"
#include "string.h"

#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"

#include "font8x8_basic.h"

struct oled_gfx gfx;

char* display_buffer = NULL;

void gfx_init(esp_lcd_panel_handle_t panel_handle, int width, int height)
{
    gfx.panel_handle = panel_handle;
    gfx.width = width;
    gfx.height = height;
    display_buffer = (char*)calloc(1, width*height/8);
}

void gfx_draw_bitmap(int x, int y, int w, int h, const char *bitmap)
{
    for(int y1 = 0; y1 < h; y1++){
        for(int x1 = 0; x1 < w; x1++){
            if(bitmap[y1] & (1 << x1)){
                gfx_set_pixel(x + x1, y + y1);
            } else {
                gfx_clear_pixel(x + x1, y + y1);
            }
        }
    }
}

void gfx_clear_area(int x, int y, int w, int h)
{
    for(int y1 = y; y1 < y+h; y1++){
        for(int x1 = x; x1 < x+w; x1++){
            gfx_clear_pixel(x1, y1);
        }
    }
}

void gfx_fill_area(int x, int y, int w, int h)
{
    for(int y1 = y; y1 < y+h; y1++){
        for(int x1 = x; x1 < x+w; x1++){
            gfx_set_pixel(x1, y1);
        }
    }
}

void gfx_set_pixel(uint8_t x, uint8_t y) {
    if (x >= gfx.width || y >= gfx.height) return;  // Bounds check

    uint8_t page = y / 8;
    uint8_t bit = y % 8;

    display_buffer[page * gfx.width + x] |= (1 << bit);
}

void gfx_clear_pixel(uint8_t x, uint8_t y) {
    if (x >= gfx.width || y >= gfx.height) return;  // Bounds check

    uint8_t page = y / 8;
    uint8_t bit = y % 8;

    display_buffer[page * gfx.width + x] &= ~(1 << bit);
}

void gfx_draw_text(int x, int y, const char *text)
{
    int i = 0;
    while(text[i] != '\0'){
        gfx_draw_bitmap(x + i*8, y, 8, 8, (const char*)&font8x8_basic[(int)text[i]]);
        i++;
    }
}

void gfx_flush()
{
    esp_lcd_panel_draw_bitmap(gfx.panel_handle, 0, 0, gfx.width, gfx.height, display_buffer);
}