
#include "oled_gfx.h"

#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"

#include "font8x8_basic.h"

struct oled_gfx gfx;

void gfx_init(esp_lcd_panel_handle_t panel_handle, int width, int height)
{
    gfx.panel_handle = panel_handle;
    gfx.width = width;
    gfx.height = height;
}

void gfx_draw_bitmap(int x, int y, int w, int h, const char *bitmap)
{
    esp_lcd_panel_draw_bitmap(gfx.panel_handle, x, y, x+w, y+h, bitmap);
}

void gfx_draw_text(int x, int y, const char *text)
{
    int i = 0;
    while(text[i] != '\0'){
        gfx_draw_bitmap(x + i*8, y, 8, 8, (const char*)&font8x8_basic[(int)text[i]]);
        i++;
    }
}