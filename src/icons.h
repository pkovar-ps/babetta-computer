/*
 * Bike Computer for Babetta
 * Palubní počítač pro Babettu
 * 
 * icons.h
 * Characters and graphics for the UI
 * 
 * Petr Kovář 2021
*/

#include <avr/pgmspace.h>

/* LCD Graphics / icons */
static const unsigned char PROGMEM icon_ambient_temp[] = {
  0x70, 0x50, 0x50, 0x50, 0xd8, 0xf8, 0xf8, 0x70
};

static const unsigned char PROGMEM icon_engine_temp[] = {
  0xf8, 0x00, 0xf8, 0xf8, 0x20, 0x20, 0x70, 0x50
};

static const unsigned char PROGMEM icon_bluetooth[] = {
  0x00, 0x00, 0x7f, 0xc0, 0xcc, 0x60, 0xd6, 0xe0, 0xc6, 0xe0, 0xd6, 0xe0, 0xce, 0xe0, 0x7f, 0xc0
};

static const unsigned char PROGMEM icon_temp_C[] = {
  0x00, 0x00, 0x63, 0x80, 0x94, 0x40, 0x94, 0x00, 0x64, 0x00, 0x04, 0x00, 0x04, 0x40, 0x03, 0x80
};

static const unsigned char PROGMEM icon_RPM[] = {
  0x00, 0x00, 0xee, 0x88, 0xaa, 0xd8, 0xee, 0xa8, 0xc8, 0x88, 0xa8, 0x88
};

static const unsigned char PROGMEM icon_kmh[] = {
  0x00, 0x00, 0x80, 0x20, 0x80, 0x20, 0xaf, 0xb8, 0xca, 0xa8, 0xaa, 0xa8
};

static const unsigned char PROGMEM icon_toggle_on[] = {
  0x3f, 0xfc, 0x40, 0x3e, 0x80, 0x7f, 0x80, 0x7f, 0x80, 0x7f, 0x80, 0x7f, 0x40, 0x3e, 0x3f, 0xfc
};

static const unsigned char PROGMEM icon_toggle_off[] = {
  0x3f, 0xfc, 0x7c, 0x02, 0xfe, 0x01, 0xfe, 0x01, 0xfe, 0x01, 0xfe, 0x01, 0x7c, 0x02, 0x3f, 0xfc
};

static const unsigned char PROGMEM startup_logo[] = {
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0xf8, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x0f, 0xfc, 0x04, 0x00, 0x00, 0x20, 0x00, 0x01, 0x84, 0x00, 0x00, 
  0x1c, 0x7e, 0x0c, 0x00, 0x00, 0x30, 0x00, 0x01, 0x8e, 0x00, 0x00, 0x30, 0x3f, 0x0c, 0x00, 0x00, 
  0x30, 0x00, 0x01, 0x8e, 0x00, 0x00, 0x70, 0x1b, 0x0c, 0x00, 0x00, 0x30, 0x00, 0x01, 0x8e, 0x00, 
  0x00, 0x60, 0x39, 0x8c, 0xf0, 0x3c, 0x37, 0x81, 0xe1, 0xff, 0x8f, 0x80, 0x60, 0x31, 0x8d, 0xf8, 
  0xff, 0x3f, 0xc3, 0xf9, 0xff, 0x9f, 0xc0, 0x60, 0x71, 0x8d, 0x1c, 0xc3, 0x30, 0xe7, 0x19, 0x8e, 
  0x38, 0xe0, 0x60, 0x31, 0x8e, 0x0d, 0xc3, 0xb0, 0x6e, 0x1d, 0x8e, 0x30, 0x60, 0x60, 0x39, 0x8e, 
  0x0f, 0x81, 0xb0, 0x7f, 0xfd, 0x8e, 0x70, 0x60, 0x60, 0x1b, 0x06, 0x0f, 0x81, 0xb0, 0x7f, 0xfd, 
  0x8e, 0x70, 0x60, 0x30, 0x3b, 0x06, 0x0d, 0xc1, 0xb0, 0x6e, 0x01, 0xc6, 0x30, 0x60, 0x38, 0x3e, 
  0x07, 0x1c, 0xe5, 0xb8, 0xe7, 0x81, 0xe7, 0xbd, 0xe0, 0x1f, 0xfc, 0x03, 0xf8, 0x7d, 0x9f, 0xc3, 
  0x80, 0xf3, 0x9f, 0xe0, 0x07, 0xf8, 0x00, 0xe0, 0x3d, 0x87, 0x01, 0x80, 0x31, 0x87, 0x60, 0x00, 
  0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};