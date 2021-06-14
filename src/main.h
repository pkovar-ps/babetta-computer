/*
 * Bike Computer for Babetta
 * Palubní počítač pro Babettu
 * 
 * main.h
 * Header file with function definitions
 * 
 * Petr Kovář 2021
*/

#ifndef MAIN_H
#define MAIN_H

#include <Arduino.h>

void on_btn_press();
void on_rotation();
void on_engine_rotation();
void reset_trip();
void toggle_bluetooth();
void toggle_backlight();
void toggle_lcd_invert();


#endif