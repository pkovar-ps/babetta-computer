/*
 * Bike Computer for Babetta
 * Palubní počítač pro Babettu
 * 
 * Code used for temperature measurements derived from: https://www.circuitbasics.com/arduino-thermistor-temperature-sensor-tutorial/
 * 
 * Petr Kovář 2021
*/

/* Constants */
#define MEASURE_INTERVAL 1000
#define WH_CIRCUMFERENCE 1245
#define MENU_ITEMS 3
#define MEASUREMENT_HIST_CNT 3 // for averageing speed 
#define UPDATE_INTERVAL 500
#define SLEEP_TIMEOUT 180000 // in ms
#define TANK_SIZE 3300 // in ml

#define NUM_OF_MODES 7
enum modes {
  screen_overview,
  screen_overview2,
  screen_speedo,
  screen_rpm,
  screen_temps,
  screen_stats,
  screen_settings
};

/* Pin Definitions */
#define BACKLIGHT_PIN PA2 // LCD backlight
#define AMBIENT_THERMISTOR_PIN PA1
#define ENGINE_THERMISTOR_PIN PA0
#define REED_SWITCH_PIN PB1 // speed sensor
#define MODE_SWITCH_PIN PB8 // push button for switching screens/selecting
#define RPM_SENSOR_PIN PB0 // optocoupler
// Bluetooth module
#define BT_EN_PIN PB5 
#define BT_TX_PIN PB6 
#define BT_RX_PIN PB7