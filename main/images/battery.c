#ifdef __has_include
    #if __has_include("lvgl.h")
        #ifndef LV_LVGL_H_INCLUDE_SIMPLE
            #define LV_LVGL_H_INCLUDE_SIMPLE
        #endif
    #endif
#endif

#if defined(LV_LVGL_H_INCLUDE_SIMPLE)
    #include "lvgl.h"
#else
    #include "lvgl/lvgl.h"
#endif


#ifndef LV_ATTRIBUTE_MEM_ALIGN
#define LV_ATTRIBUTE_MEM_ALIGN
#endif

#ifndef LV_ATTRIBUTE_IMG_BATTERY
#define LV_ATTRIBUTE_IMG_BATTERY
#endif

const LV_ATTRIBUTE_MEM_ALIGN LV_ATTRIBUTE_LARGE_CONST LV_ATTRIBUTE_IMG_BATTERY uint8_t battery_map[] = {
#if LV_COLOR_DEPTH == 1 || LV_COLOR_DEPTH == 8
  /*Pixel format: Alpha 8 bit, Red: 3 bit, Green: 3 bit, Blue: 2 bit*/
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xbf, 0x00, 0xff, 0x00, 0xff, 0x00, 0xff, 0x00, 0xff, 0x00, 0x70, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xbf, 0x00, 0xff, 0x00, 0xff, 0x00, 0xff, 0x00, 0xff, 0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x30, 0x00, 0xcf, 0x00, 0xdf, 0x00, 0xdf, 0x00, 0xef, 0x00, 0xff, 0x00, 0xff, 0x00, 0xff, 0x00, 0xff, 0x00, 0xef, 0x00, 0xdf, 0x00, 0xdf, 0x00, 0xbf, 0x00, 0x20, 
  0x00, 0xdf, 0x00, 0xff, 0x00, 0xff, 0x00, 0xff, 0x00, 0xff, 0x00, 0xff, 0x00, 0xff, 0x00, 0xff, 0x00, 0xff, 0x00, 0xff, 0x00, 0xff, 0x00, 0xff, 0x00, 0xff, 0x00, 0xb0, 
  0x00, 0xff, 0x00, 0xdf, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x00, 0xff, 0x00, 0xbf, 
  0x00, 0xff, 0x00, 0xdf, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x00, 0xff, 0x00, 0xbf, 
  0x00, 0xff, 0x00, 0xdf, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x00, 0xff, 0x00, 0xbf, 
  0x00, 0xff, 0x00, 0xdf, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x00, 0xff, 0x00, 0xbf, 
  0x00, 0xff, 0x00, 0xdf, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x00, 0xff, 0x00, 0xbf, 
  0x00, 0xff, 0x00, 0xdf, 0x00, 0x00, 0x00, 0x9f, 0x00, 0xbf, 0x00, 0xbf, 0x00, 0xbf, 0x00, 0xbf, 0x00, 0xbf, 0x00, 0xbf, 0x00, 0xbf, 0x00, 0x50, 0x00, 0xff, 0x00, 0xbf, 
  0x00, 0xff, 0x00, 0xdf, 0x00, 0x00, 0x00, 0xdf, 0x00, 0xff, 0x00, 0xff, 0x00, 0xff, 0x00, 0xff, 0x00, 0xff, 0x00, 0xff, 0x00, 0xff, 0x00, 0x60, 0x00, 0xff, 0x00, 0xbf, 
  0x00, 0xff, 0x00, 0xdf, 0x00, 0x00, 0x00, 0xdf, 0x00, 0xff, 0x00, 0xff, 0x00, 0xff, 0x00, 0xff, 0x00, 0xff, 0x00, 0xff, 0x00, 0xff, 0x00, 0x60, 0x00, 0xff, 0x00, 0xbf, 
  0x00, 0xff, 0x00, 0xdf, 0x00, 0x00, 0x00, 0x40, 0x00, 0x40, 0x00, 0x40, 0x00, 0x40, 0x00, 0x40, 0x00, 0x40, 0x00, 0x40, 0x00, 0x40, 0x00, 0x30, 0x00, 0xff, 0x00, 0xbf, 
  0x00, 0xff, 0x00, 0xdf, 0x00, 0x00, 0x00, 0x9f, 0x00, 0xbf, 0x00, 0xbf, 0x00, 0xbf, 0x00, 0xbf, 0x00, 0xbf, 0x00, 0xbf, 0x00, 0xbf, 0x00, 0x50, 0x00, 0xff, 0x00, 0xbf, 
  0x00, 0xff, 0x00, 0xdf, 0x00, 0x00, 0x00, 0xdf, 0x00, 0xff, 0x00, 0xff, 0x00, 0xff, 0x00, 0xff, 0x00, 0xff, 0x00, 0xff, 0x00, 0xff, 0x00, 0x60, 0x00, 0xff, 0x00, 0xbf, 
  0x00, 0xff, 0x00, 0xdf, 0x00, 0x00, 0x00, 0xbf, 0x00, 0xdf, 0x00, 0xdf, 0x00, 0xdf, 0x00, 0xdf, 0x00, 0xdf, 0x00, 0xdf, 0x00, 0xdf, 0x00, 0x60, 0x00, 0xff, 0x00, 0xbf, 
  0x00, 0xff, 0x00, 0xdf, 0x00, 0x00, 0x00, 0x20, 0x00, 0x20, 0x00, 0x20, 0x00, 0x20, 0x00, 0x20, 0x00, 0x20, 0x00, 0x20, 0x00, 0x20, 0x00, 0x20, 0x00, 0xff, 0x00, 0xbf, 
  0x00, 0xff, 0x00, 0xdf, 0x00, 0x00, 0x00, 0xdf, 0x00, 0xff, 0x00, 0xff, 0x00, 0xff, 0x00, 0xff, 0x00, 0xff, 0x00, 0xff, 0x00, 0xff, 0x00, 0x60, 0x00, 0xff, 0x00, 0xbf, 
  0x00, 0xff, 0x00, 0xdf, 0x00, 0x00, 0x00, 0xdf, 0x00, 0xff, 0x00, 0xff, 0x00, 0xff, 0x00, 0xff, 0x00, 0xff, 0x00, 0xff, 0x00, 0xff, 0x00, 0x60, 0x00, 0xff, 0x00, 0xbf, 
  0x00, 0xff, 0x00, 0xdf, 0x00, 0x00, 0x00, 0xbf, 0x00, 0xdf, 0x00, 0xdf, 0x00, 0xdf, 0x00, 0xdf, 0x00, 0xdf, 0x00, 0xdf, 0x00, 0xdf, 0x00, 0x60, 0x00, 0xff, 0x00, 0xbf, 
  0x00, 0xff, 0x00, 0xdf, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x00, 0xff, 0x00, 0xbf, 
  0x00, 0xef, 0x00, 0xff, 0x00, 0xdf, 0x00, 0xdf, 0x00, 0xdf, 0x00, 0xdf, 0x00, 0xdf, 0x00, 0xdf, 0x00, 0xdf, 0x00, 0xdf, 0x00, 0xdf, 0x00, 0xdf, 0x00, 0xff, 0x00, 0xbf, 
  0x00, 0x60, 0x00, 0xdf, 0x00, 0xff, 0x00, 0xff, 0x00, 0xff, 0x00, 0xff, 0x00, 0xff, 0x00, 0xff, 0x00, 0xff, 0x00, 0xff, 0x00, 0xff, 0x00, 0xff, 0x00, 0xdf, 0x00, 0x20, 
#endif
#if LV_COLOR_DEPTH == 16 && LV_COLOR_16_SWAP == 0
  /*Pixel format: Alpha 8 bit, Red: 5 bit, Green: 6 bit, Blue: 5 bit*/
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xbf, 0x00, 0x00, 0xff, 0x00, 0x00, 0xff, 0x00, 0x00, 0xff, 0x00, 0x00, 0xff, 0x00, 0x00, 0x70, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xbf, 0x00, 0x00, 0xff, 0x00, 0x00, 0xff, 0x00, 0x00, 0xff, 0x00, 0x00, 0xff, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x30, 0x00, 0x00, 0xcf, 0x00, 0x00, 0xdf, 0x00, 0x00, 0xdf, 0x00, 0x00, 0xef, 0x00, 0x00, 0xff, 0x00, 0x00, 0xff, 0x00, 0x00, 0xff, 0x00, 0x00, 0xff, 0x00, 0x00, 0xef, 0x00, 0x00, 0xdf, 0x00, 0x00, 0xdf, 0x00, 0x00, 0xbf, 0x00, 0x00, 0x20, 
  0x00, 0x00, 0xdf, 0x00, 0x00, 0xff, 0x00, 0x00, 0xff, 0x00, 0x00, 0xff, 0x00, 0x00, 0xff, 0x00, 0x00, 0xff, 0x00, 0x00, 0xff, 0x00, 0x00, 0xff, 0x00, 0x00, 0xff, 0x00, 0x00, 0xff, 0x00, 0x00, 0xff, 0x00, 0x00, 0xff, 0x00, 0x00, 0xff, 0x00, 0x00, 0xb0, 
  0x00, 0x00, 0xff, 0x00, 0x00, 0xdf, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x00, 0x00, 0xff, 0x00, 0x00, 0xbf, 
  0x00, 0x00, 0xff, 0x00, 0x00, 0xdf, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x00, 0x00, 0xff, 0x00, 0x00, 0xbf, 
  0x00, 0x00, 0xff, 0x00, 0x00, 0xdf, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x00, 0x00, 0xff, 0x00, 0x00, 0xbf, 
  0x00, 0x00, 0xff, 0x00, 0x00, 0xdf, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x00, 0x00, 0xff, 0x00, 0x00, 0xbf, 
  0x00, 0x00, 0xff, 0x00, 0x00, 0xdf, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x00, 0x00, 0xff, 0x00, 0x00, 0xbf, 
  0x00, 0x00, 0xff, 0x00, 0x00, 0xdf, 0x00, 0x00, 0x00, 0x00, 0x00, 0x9f, 0x00, 0x00, 0xbf, 0x00, 0x00, 0xbf, 0x00, 0x00, 0xbf, 0x00, 0x00, 0xbf, 0x00, 0x00, 0xbf, 0x00, 0x00, 0xbf, 0x00, 0x00, 0xbf, 0x00, 0x00, 0x50, 0x00, 0x00, 0xff, 0x00, 0x00, 0xbf, 
  0x00, 0x00, 0xff, 0x00, 0x00, 0xdf, 0x00, 0x00, 0x00, 0x00, 0x00, 0xdf, 0x00, 0x00, 0xff, 0x00, 0x00, 0xff, 0x00, 0x00, 0xff, 0x00, 0x00, 0xff, 0x00, 0x00, 0xff, 0x00, 0x00, 0xff, 0x00, 0x00, 0xff, 0x00, 0x00, 0x60, 0x00, 0x00, 0xff, 0x00, 0x00, 0xbf, 
  0x00, 0x00, 0xff, 0x00, 0x00, 0xdf, 0x00, 0x00, 0x00, 0x00, 0x00, 0xdf, 0x00, 0x00, 0xff, 0x00, 0x00, 0xff, 0x00, 0x00, 0xff, 0x00, 0x00, 0xff, 0x00, 0x00, 0xff, 0x00, 0x00, 0xff, 0x00, 0x00, 0xff, 0x00, 0x00, 0x60, 0x00, 0x00, 0xff, 0x00, 0x00, 0xbf, 
  0x00, 0x00, 0xff, 0x00, 0x00, 0xdf, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x00, 0x00, 0x40, 0x00, 0x00, 0x40, 0x00, 0x00, 0x40, 0x00, 0x00, 0x40, 0x00, 0x00, 0x40, 0x00, 0x00, 0x40, 0x00, 0x00, 0x40, 0x00, 0x00, 0x30, 0x00, 0x00, 0xff, 0x00, 0x00, 0xbf, 
  0x00, 0x00, 0xff, 0x00, 0x00, 0xdf, 0x00, 0x00, 0x00, 0x00, 0x00, 0x9f, 0x00, 0x00, 0xbf, 0x00, 0x00, 0xbf, 0x00, 0x00, 0xbf, 0x00, 0x00, 0xbf, 0x00, 0x00, 0xbf, 0x00, 0x00, 0xbf, 0x00, 0x00, 0xbf, 0x00, 0x00, 0x50, 0x00, 0x00, 0xff, 0x00, 0x00, 0xbf, 
  0x00, 0x00, 0xff, 0x00, 0x00, 0xdf, 0x00, 0x00, 0x00, 0x00, 0x00, 0xdf, 0x00, 0x00, 0xff, 0x00, 0x00, 0xff, 0x00, 0x00, 0xff, 0x00, 0x00, 0xff, 0x00, 0x00, 0xff, 0x00, 0x00, 0xff, 0x00, 0x00, 0xff, 0x00, 0x00, 0x60, 0x00, 0x00, 0xff, 0x00, 0x00, 0xbf, 
  0x00, 0x00, 0xff, 0x00, 0x00, 0xdf, 0x00, 0x00, 0x00, 0x00, 0x00, 0xbf, 0x00, 0x00, 0xdf, 0x00, 0x00, 0xdf, 0x00, 0x00, 0xdf, 0x00, 0x00, 0xdf, 0x00, 0x00, 0xdf, 0x00, 0x00, 0xdf, 0x00, 0x00, 0xdf, 0x00, 0x00, 0x60, 0x00, 0x00, 0xff, 0x00, 0x00, 0xbf, 
  0x00, 0x00, 0xff, 0x00, 0x00, 0xdf, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x00, 0x00, 0x20, 0x00, 0x00, 0x20, 0x00, 0x00, 0x20, 0x00, 0x00, 0x20, 0x00, 0x00, 0x20, 0x00, 0x00, 0x20, 0x00, 0x00, 0x20, 0x00, 0x00, 0x20, 0x00, 0x00, 0xff, 0x00, 0x00, 0xbf, 
  0x00, 0x00, 0xff, 0x00, 0x00, 0xdf, 0x00, 0x00, 0x00, 0x00, 0x00, 0xdf, 0x00, 0x00, 0xff, 0x00, 0x00, 0xff, 0x00, 0x00, 0xff, 0x00, 0x00, 0xff, 0x00, 0x00, 0xff, 0x00, 0x00, 0xff, 0x00, 0x00, 0xff, 0x00, 0x00, 0x60, 0x00, 0x00, 0xff, 0x00, 0x00, 0xbf, 
  0x00, 0x00, 0xff, 0x00, 0x00, 0xdf, 0x00, 0x00, 0x00, 0x00, 0x00, 0xdf, 0x00, 0x00, 0xff, 0x00, 0x00, 0xff, 0x00, 0x00, 0xff, 0x00, 0x00, 0xff, 0x00, 0x00, 0xff, 0x00, 0x00, 0xff, 0x00, 0x00, 0xff, 0x00, 0x00, 0x60, 0x00, 0x00, 0xff, 0x00, 0x00, 0xbf, 
  0x00, 0x00, 0xff, 0x00, 0x00, 0xdf, 0x00, 0x00, 0x00, 0x00, 0x00, 0xbf, 0x00, 0x00, 0xdf, 0x00, 0x00, 0xdf, 0x00, 0x00, 0xdf, 0x00, 0x00, 0xdf, 0x00, 0x00, 0xdf, 0x00, 0x00, 0xdf, 0x00, 0x00, 0xdf, 0x00, 0x00, 0x60, 0x00, 0x00, 0xff, 0x00, 0x00, 0xbf, 
  0x00, 0x00, 0xff, 0x00, 0x00, 0xdf, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x00, 0x00, 0xff, 0x00, 0x00, 0xbf, 
  0x00, 0x00, 0xef, 0x00, 0x00, 0xff, 0x00, 0x00, 0xdf, 0x00, 0x00, 0xdf, 0x00, 0x00, 0xdf, 0x00, 0x00, 0xdf, 0x00, 0x00, 0xdf, 0x00, 0x00, 0xdf, 0x00, 0x00, 0xdf, 0x00, 0x00, 0xdf, 0x00, 0x00, 0xdf, 0x00, 0x00, 0xdf, 0x00, 0x00, 0xff, 0x00, 0x00, 0xbf, 
  0x00, 0x00, 0x60, 0x00, 0x00, 0xdf, 0x00, 0x00, 0xff, 0x00, 0x00, 0xff, 0x00, 0x00, 0xff, 0x00, 0x00, 0xff, 0x00, 0x00, 0xff, 0x00, 0x00, 0xff, 0x00, 0x00, 0xff, 0x00, 0x00, 0xff, 0x00, 0x00, 0xff, 0x00, 0x00, 0xff, 0x00, 0x00, 0xdf, 0x00, 0x00, 0x20, 
#endif
#if LV_COLOR_DEPTH == 16 && LV_COLOR_16_SWAP != 0
  /*Pixel format: Alpha 8 bit, Red: 5 bit, Green: 6 bit, Blue: 5 bit  BUT the 2  color bytes are swapped*/
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xbf, 0x00, 0x00, 0xff, 0x00, 0x00, 0xff, 0x00, 0x00, 0xff, 0x00, 0x00, 0xff, 0x00, 0x00, 0x70, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xbf, 0x00, 0x00, 0xff, 0x00, 0x00, 0xff, 0x00, 0x00, 0xff, 0x00, 0x00, 0xff, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x30, 0x00, 0x00, 0xcf, 0x00, 0x00, 0xdf, 0x00, 0x00, 0xdf, 0x00, 0x00, 0xef, 0x00, 0x00, 0xff, 0x00, 0x00, 0xff, 0x00, 0x00, 0xff, 0x00, 0x00, 0xff, 0x00, 0x00, 0xef, 0x00, 0x00, 0xdf, 0x00, 0x00, 0xdf, 0x00, 0x00, 0xbf, 0x00, 0x00, 0x20, 
  0x00, 0x00, 0xdf, 0x00, 0x00, 0xff, 0x00, 0x00, 0xff, 0x00, 0x00, 0xff, 0x00, 0x00, 0xff, 0x00, 0x00, 0xff, 0x00, 0x00, 0xff, 0x00, 0x00, 0xff, 0x00, 0x00, 0xff, 0x00, 0x00, 0xff, 0x00, 0x00, 0xff, 0x00, 0x00, 0xff, 0x00, 0x00, 0xff, 0x00, 0x00, 0xb0, 
  0x00, 0x00, 0xff, 0x00, 0x00, 0xdf, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x00, 0x00, 0xff, 0x00, 0x00, 0xbf, 
  0x00, 0x00, 0xff, 0x00, 0x00, 0xdf, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x00, 0x00, 0xff, 0x00, 0x00, 0xbf, 
  0x00, 0x00, 0xff, 0x00, 0x00, 0xdf, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x00, 0x00, 0xff, 0x00, 0x00, 0xbf, 
  0x00, 0x00, 0xff, 0x00, 0x00, 0xdf, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x00, 0x00, 0xff, 0x00, 0x00, 0xbf, 
  0x00, 0x00, 0xff, 0x00, 0x00, 0xdf, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x00, 0x00, 0xff, 0x00, 0x00, 0xbf, 
  0x00, 0x00, 0xff, 0x00, 0x00, 0xdf, 0x00, 0x00, 0x00, 0x00, 0x00, 0x9f, 0x00, 0x00, 0xbf, 0x00, 0x00, 0xbf, 0x00, 0x00, 0xbf, 0x00, 0x00, 0xbf, 0x00, 0x00, 0xbf, 0x00, 0x00, 0xbf, 0x00, 0x00, 0xbf, 0x00, 0x00, 0x50, 0x00, 0x00, 0xff, 0x00, 0x00, 0xbf, 
  0x00, 0x00, 0xff, 0x00, 0x00, 0xdf, 0x00, 0x00, 0x00, 0x00, 0x00, 0xdf, 0x00, 0x00, 0xff, 0x00, 0x00, 0xff, 0x00, 0x00, 0xff, 0x00, 0x00, 0xff, 0x00, 0x00, 0xff, 0x00, 0x00, 0xff, 0x00, 0x00, 0xff, 0x00, 0x00, 0x60, 0x00, 0x00, 0xff, 0x00, 0x00, 0xbf, 
  0x00, 0x00, 0xff, 0x00, 0x00, 0xdf, 0x00, 0x00, 0x00, 0x00, 0x00, 0xdf, 0x00, 0x00, 0xff, 0x00, 0x00, 0xff, 0x00, 0x00, 0xff, 0x00, 0x00, 0xff, 0x00, 0x00, 0xff, 0x00, 0x00, 0xff, 0x00, 0x00, 0xff, 0x00, 0x00, 0x60, 0x00, 0x00, 0xff, 0x00, 0x00, 0xbf, 
  0x00, 0x00, 0xff, 0x00, 0x00, 0xdf, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x00, 0x00, 0x40, 0x00, 0x00, 0x40, 0x00, 0x00, 0x40, 0x00, 0x00, 0x40, 0x00, 0x00, 0x40, 0x00, 0x00, 0x40, 0x00, 0x00, 0x40, 0x00, 0x00, 0x30, 0x00, 0x00, 0xff, 0x00, 0x00, 0xbf, 
  0x00, 0x00, 0xff, 0x00, 0x00, 0xdf, 0x00, 0x00, 0x00, 0x00, 0x00, 0x9f, 0x00, 0x00, 0xbf, 0x00, 0x00, 0xbf, 0x00, 0x00, 0xbf, 0x00, 0x00, 0xbf, 0x00, 0x00, 0xbf, 0x00, 0x00, 0xbf, 0x00, 0x00, 0xbf, 0x00, 0x00, 0x50, 0x00, 0x00, 0xff, 0x00, 0x00, 0xbf, 
  0x00, 0x00, 0xff, 0x00, 0x00, 0xdf, 0x00, 0x00, 0x00, 0x00, 0x00, 0xdf, 0x00, 0x00, 0xff, 0x00, 0x00, 0xff, 0x00, 0x00, 0xff, 0x00, 0x00, 0xff, 0x00, 0x00, 0xff, 0x00, 0x00, 0xff, 0x00, 0x00, 0xff, 0x00, 0x00, 0x60, 0x00, 0x00, 0xff, 0x00, 0x00, 0xbf, 
  0x00, 0x00, 0xff, 0x00, 0x00, 0xdf, 0x00, 0x00, 0x00, 0x00, 0x00, 0xbf, 0x00, 0x00, 0xdf, 0x00, 0x00, 0xdf, 0x00, 0x00, 0xdf, 0x00, 0x00, 0xdf, 0x00, 0x00, 0xdf, 0x00, 0x00, 0xdf, 0x00, 0x00, 0xdf, 0x00, 0x00, 0x60, 0x00, 0x00, 0xff, 0x00, 0x00, 0xbf, 
  0x00, 0x00, 0xff, 0x00, 0x00, 0xdf, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x00, 0x00, 0x20, 0x00, 0x00, 0x20, 0x00, 0x00, 0x20, 0x00, 0x00, 0x20, 0x00, 0x00, 0x20, 0x00, 0x00, 0x20, 0x00, 0x00, 0x20, 0x00, 0x00, 0x20, 0x00, 0x00, 0xff, 0x00, 0x00, 0xbf, 
  0x00, 0x00, 0xff, 0x00, 0x00, 0xdf, 0x00, 0x00, 0x00, 0x00, 0x00, 0xdf, 0x00, 0x00, 0xff, 0x00, 0x00, 0xff, 0x00, 0x00, 0xff, 0x00, 0x00, 0xff, 0x00, 0x00, 0xff, 0x00, 0x00, 0xff, 0x00, 0x00, 0xff, 0x00, 0x00, 0x60, 0x00, 0x00, 0xff, 0x00, 0x00, 0xbf, 
  0x00, 0x00, 0xff, 0x00, 0x00, 0xdf, 0x00, 0x00, 0x00, 0x00, 0x00, 0xdf, 0x00, 0x00, 0xff, 0x00, 0x00, 0xff, 0x00, 0x00, 0xff, 0x00, 0x00, 0xff, 0x00, 0x00, 0xff, 0x00, 0x00, 0xff, 0x00, 0x00, 0xff, 0x00, 0x00, 0x60, 0x00, 0x00, 0xff, 0x00, 0x00, 0xbf, 
  0x00, 0x00, 0xff, 0x00, 0x00, 0xdf, 0x00, 0x00, 0x00, 0x00, 0x00, 0xbf, 0x00, 0x00, 0xdf, 0x00, 0x00, 0xdf, 0x00, 0x00, 0xdf, 0x00, 0x00, 0xdf, 0x00, 0x00, 0xdf, 0x00, 0x00, 0xdf, 0x00, 0x00, 0xdf, 0x00, 0x00, 0x60, 0x00, 0x00, 0xff, 0x00, 0x00, 0xbf, 
  0x00, 0x00, 0xff, 0x00, 0x00, 0xdf, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x00, 0x00, 0xff, 0x00, 0x00, 0xbf, 
  0x00, 0x00, 0xef, 0x00, 0x00, 0xff, 0x00, 0x00, 0xdf, 0x00, 0x00, 0xdf, 0x00, 0x00, 0xdf, 0x00, 0x00, 0xdf, 0x00, 0x00, 0xdf, 0x00, 0x00, 0xdf, 0x00, 0x00, 0xdf, 0x00, 0x00, 0xdf, 0x00, 0x00, 0xdf, 0x00, 0x00, 0xdf, 0x00, 0x00, 0xff, 0x00, 0x00, 0xbf, 
  0x00, 0x00, 0x60, 0x00, 0x00, 0xdf, 0x00, 0x00, 0xff, 0x00, 0x00, 0xff, 0x00, 0x00, 0xff, 0x00, 0x00, 0xff, 0x00, 0x00, 0xff, 0x00, 0x00, 0xff, 0x00, 0x00, 0xff, 0x00, 0x00, 0xff, 0x00, 0x00, 0xff, 0x00, 0x00, 0xff, 0x00, 0x00, 0xdf, 0x00, 0x00, 0x20, 
#endif
#if LV_COLOR_DEPTH == 32
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xbf, 0x00, 0x00, 0x00, 0xff, 0x00, 0x00, 0x00, 0xff, 0x00, 0x00, 0x00, 0xff, 0x00, 0x00, 0x00, 0xff, 0x00, 0x00, 0x00, 0x70, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xbf, 0x00, 0x00, 0x00, 0xff, 0x00, 0x00, 0x00, 0xff, 0x00, 0x00, 0x00, 0xff, 0x00, 0x00, 0x00, 0xff, 0x00, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x30, 0x00, 0x00, 0x00, 0xcf, 0x00, 0x00, 0x00, 0xdf, 0x00, 0x00, 0x00, 0xdf, 0x00, 0x00, 0x00, 0xef, 0x00, 0x00, 0x00, 0xff, 0x00, 0x00, 0x00, 0xff, 0x00, 0x00, 0x00, 0xff, 0x00, 0x00, 0x00, 0xff, 0x00, 0x00, 0x00, 0xef, 0x00, 0x00, 0x00, 0xdf, 0x00, 0x00, 0x00, 0xdf, 0x00, 0x00, 0x00, 0xbf, 0x00, 0x00, 0x00, 0x20, 
  0x00, 0x00, 0x00, 0xdf, 0x00, 0x00, 0x00, 0xff, 0x00, 0x00, 0x00, 0xff, 0x00, 0x00, 0x00, 0xff, 0x00, 0x00, 0x00, 0xff, 0x00, 0x00, 0x00, 0xff, 0x00, 0x00, 0x00, 0xff, 0x00, 0x00, 0x00, 0xff, 0x00, 0x00, 0x00, 0xff, 0x00, 0x00, 0x00, 0xff, 0x00, 0x00, 0x00, 0xff, 0x00, 0x00, 0x00, 0xff, 0x00, 0x00, 0x00, 0xff, 0x00, 0x00, 0x00, 0xb0, 
  0x00, 0x00, 0x00, 0xff, 0x00, 0x00, 0x00, 0xdf, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x00, 0x00, 0x00, 0xff, 0x00, 0x00, 0x00, 0xbf, 
  0x00, 0x00, 0x00, 0xff, 0x00, 0x00, 0x00, 0xdf, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x00, 0x00, 0x00, 0xff, 0x00, 0x00, 0x00, 0xbf, 
  0x00, 0x00, 0x00, 0xff, 0x00, 0x00, 0x00, 0xdf, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x00, 0x00, 0x00, 0xff, 0x00, 0x00, 0x00, 0xbf, 
  0x00, 0x00, 0x00, 0xff, 0x00, 0x00, 0x00, 0xdf, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x00, 0x00, 0x00, 0xff, 0x00, 0x00, 0x00, 0xbf, 
  0x00, 0x00, 0x00, 0xff, 0x00, 0x00, 0x00, 0xdf, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x00, 0x00, 0x00, 0xff, 0x00, 0x00, 0x00, 0xbf, 
  0x00, 0x00, 0x00, 0xff, 0x00, 0x00, 0x00, 0xdf, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x9f, 0x00, 0x00, 0x00, 0xbf, 0x00, 0x00, 0x00, 0xbf, 0x00, 0x00, 0x00, 0xbf, 0x00, 0x00, 0x00, 0xbf, 0x00, 0x00, 0x00, 0xbf, 0x00, 0x00, 0x00, 0xbf, 0x00, 0x00, 0x00, 0xbf, 0x00, 0x00, 0x00, 0x50, 0x00, 0x00, 0x00, 0xff, 0x00, 0x00, 0x00, 0xbf, 
  0x00, 0x00, 0x00, 0xff, 0x00, 0x00, 0x00, 0xdf, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xdf, 0x00, 0x00, 0x00, 0xff, 0x00, 0x00, 0x00, 0xff, 0x00, 0x00, 0x00, 0xff, 0x00, 0x00, 0x00, 0xff, 0x00, 0x00, 0x00, 0xff, 0x00, 0x00, 0x00, 0xff, 0x00, 0x00, 0x00, 0xff, 0x00, 0x00, 0x00, 0x60, 0x00, 0x00, 0x00, 0xff, 0x00, 0x00, 0x00, 0xbf, 
  0x00, 0x00, 0x00, 0xff, 0x00, 0x00, 0x00, 0xdf, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xdf, 0x00, 0x00, 0x00, 0xff, 0x00, 0x00, 0x00, 0xff, 0x00, 0x00, 0x00, 0xff, 0x00, 0x00, 0x00, 0xff, 0x00, 0x00, 0x00, 0xff, 0x00, 0x00, 0x00, 0xff, 0x00, 0x00, 0x00, 0xff, 0x00, 0x00, 0x00, 0x60, 0x00, 0x00, 0x00, 0xff, 0x00, 0x00, 0x00, 0xbf, 
  0x00, 0x00, 0x00, 0xff, 0x00, 0x00, 0x00, 0xdf, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x00, 0x00, 0x00, 0x40, 0x00, 0x00, 0x00, 0x40, 0x00, 0x00, 0x00, 0x40, 0x00, 0x00, 0x00, 0x40, 0x00, 0x00, 0x00, 0x40, 0x00, 0x00, 0x00, 0x40, 0x00, 0x00, 0x00, 0x40, 0x00, 0x00, 0x00, 0x30, 0x00, 0x00, 0x00, 0xff, 0x00, 0x00, 0x00, 0xbf, 
  0x00, 0x00, 0x00, 0xff, 0x00, 0x00, 0x00, 0xdf, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x9f, 0x00, 0x00, 0x00, 0xbf, 0x00, 0x00, 0x00, 0xbf, 0x00, 0x00, 0x00, 0xbf, 0x00, 0x00, 0x00, 0xbf, 0x00, 0x00, 0x00, 0xbf, 0x00, 0x00, 0x00, 0xbf, 0x00, 0x00, 0x00, 0xbf, 0x00, 0x00, 0x00, 0x50, 0x00, 0x00, 0x00, 0xff, 0x00, 0x00, 0x00, 0xbf, 
  0x00, 0x00, 0x00, 0xff, 0x00, 0x00, 0x00, 0xdf, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xdf, 0x00, 0x00, 0x00, 0xff, 0x00, 0x00, 0x00, 0xff, 0x00, 0x00, 0x00, 0xff, 0x00, 0x00, 0x00, 0xff, 0x00, 0x00, 0x00, 0xff, 0x00, 0x00, 0x00, 0xff, 0x00, 0x00, 0x00, 0xff, 0x00, 0x00, 0x00, 0x60, 0x00, 0x00, 0x00, 0xff, 0x00, 0x00, 0x00, 0xbf, 
  0x00, 0x00, 0x00, 0xff, 0x00, 0x00, 0x00, 0xdf, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xbf, 0x00, 0x00, 0x00, 0xdf, 0x00, 0x00, 0x00, 0xdf, 0x00, 0x00, 0x00, 0xdf, 0x00, 0x00, 0x00, 0xdf, 0x00, 0x00, 0x00, 0xdf, 0x00, 0x00, 0x00, 0xdf, 0x00, 0x00, 0x00, 0xdf, 0x00, 0x00, 0x00, 0x60, 0x00, 0x00, 0x00, 0xff, 0x00, 0x00, 0x00, 0xbf, 
  0x00, 0x00, 0x00, 0xff, 0x00, 0x00, 0x00, 0xdf, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x00, 0x00, 0x00, 0x20, 0x00, 0x00, 0x00, 0x20, 0x00, 0x00, 0x00, 0x20, 0x00, 0x00, 0x00, 0x20, 0x00, 0x00, 0x00, 0x20, 0x00, 0x00, 0x00, 0x20, 0x00, 0x00, 0x00, 0x20, 0x00, 0x00, 0x00, 0x20, 0x00, 0x00, 0x00, 0xff, 0x00, 0x00, 0x00, 0xbf, 
  0x00, 0x00, 0x00, 0xff, 0x00, 0x00, 0x00, 0xdf, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xdf, 0x00, 0x00, 0x00, 0xff, 0x00, 0x00, 0x00, 0xff, 0x00, 0x00, 0x00, 0xff, 0x00, 0x00, 0x00, 0xff, 0x00, 0x00, 0x00, 0xff, 0x00, 0x00, 0x00, 0xff, 0x00, 0x00, 0x00, 0xff, 0x00, 0x00, 0x00, 0x60, 0x00, 0x00, 0x00, 0xff, 0x00, 0x00, 0x00, 0xbf, 
  0x00, 0x00, 0x00, 0xff, 0x00, 0x00, 0x00, 0xdf, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xdf, 0x00, 0x00, 0x00, 0xff, 0x00, 0x00, 0x00, 0xff, 0x00, 0x00, 0x00, 0xff, 0x00, 0x00, 0x00, 0xff, 0x00, 0x00, 0x00, 0xff, 0x00, 0x00, 0x00, 0xff, 0x00, 0x00, 0x00, 0xff, 0x00, 0x00, 0x00, 0x60, 0x00, 0x00, 0x00, 0xff, 0x00, 0x00, 0x00, 0xbf, 
  0x00, 0x00, 0x00, 0xff, 0x00, 0x00, 0x00, 0xdf, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xbf, 0x00, 0x00, 0x00, 0xdf, 0x00, 0x00, 0x00, 0xdf, 0x00, 0x00, 0x00, 0xdf, 0x00, 0x00, 0x00, 0xdf, 0x00, 0x00, 0x00, 0xdf, 0x00, 0x00, 0x00, 0xdf, 0x00, 0x00, 0x00, 0xdf, 0x00, 0x00, 0x00, 0x60, 0x00, 0x00, 0x00, 0xff, 0x00, 0x00, 0x00, 0xbf, 
  0x00, 0x00, 0x00, 0xff, 0x00, 0x00, 0x00, 0xdf, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x00, 0x00, 0x00, 0xff, 0x00, 0x00, 0x00, 0xbf, 
  0x00, 0x00, 0x00, 0xef, 0x00, 0x00, 0x00, 0xff, 0x00, 0x00, 0x00, 0xdf, 0x00, 0x00, 0x00, 0xdf, 0x00, 0x00, 0x00, 0xdf, 0x00, 0x00, 0x00, 0xdf, 0x00, 0x00, 0x00, 0xdf, 0x00, 0x00, 0x00, 0xdf, 0x00, 0x00, 0x00, 0xdf, 0x00, 0x00, 0x00, 0xdf, 0x00, 0x00, 0x00, 0xdf, 0x00, 0x00, 0x00, 0xdf, 0x00, 0x00, 0x00, 0xff, 0x00, 0x00, 0x00, 0xbf, 
  0x00, 0x00, 0x00, 0x60, 0x00, 0x00, 0x00, 0xdf, 0x00, 0x00, 0x00, 0xff, 0x00, 0x00, 0x00, 0xff, 0x00, 0x00, 0x00, 0xff, 0x00, 0x00, 0x00, 0xff, 0x00, 0x00, 0x00, 0xff, 0x00, 0x00, 0x00, 0xff, 0x00, 0x00, 0x00, 0xff, 0x00, 0x00, 0x00, 0xff, 0x00, 0x00, 0x00, 0xff, 0x00, 0x00, 0x00, 0xff, 0x00, 0x00, 0x00, 0xdf, 0x00, 0x00, 0x00, 0x20, 
#endif
};

const lv_img_dsc_t battery = {
  .header.cf = LV_IMG_CF_TRUE_COLOR_ALPHA,
  .header.always_zero = 0,
  .header.reserved = 0,
  .header.w = 14,
  .header.h = 23,
  .data_size = 322 * LV_IMG_PX_SIZE_ALPHA_BYTE,
  .data = battery_map,
};
