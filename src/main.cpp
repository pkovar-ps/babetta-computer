/*
 * Bike Computer for Babetta
 * Palubní počítač pro Babettu
 * 
 * main.cpp
 * Main program
 * 
 * Code used for temperature measurements derived from: https://www.circuitbasics.com/arduino-thermistor-temperature-sensor-tutorial/
 * 
 * Petr Kovář 2021
*/

#include "main.h"
#include "config.h"
#include "icons.h"

#include <SPI.h>
#include <Adafruit_PCD8544.h>
#include <Adafruit_GFX.h>

#include <Fonts/FreeSansBold24pt7b.h>
#include <Fonts/FreeSansBold12pt7b.h>

#include <SoftwareSerial.h>

#include <STM32RTC.h>
#include "STM32LowPower.h"

/* -------------------------------------------------- */

/* Get the rtc object */
STM32RTC& rtc = STM32RTC::getInstance();

/* -------------------------------------------------- */

uint8_t mode = screen_overview; // default is overview
bool BT = false;
bool backlight = false;
bool inverted_lcd = false;
uint8_t selected_menu_item = 0;

unsigned long last_change = 0; // used for sleeping after no registered activity
bool sleeping = false;

// PCD8544 definitions (Nokia 5110 LCD)
Adafruit_PCD8544 lcd = Adafruit_PCD8544(PA6, PA4, PA3); // CS, CE, RST
SoftwareSerial BTserial(BT_TX_PIN, BT_RX_PIN);

/* Thermistors */
double R1a = 10000;
double Tca;
double c1a = 1.009249522e-03, c2a = 2.378405444e-04, c3a = 2.019202697e-07; // Steinhart-Hart Coefficients

double R1e = 30000; // external 100k but it seems that internal resistor stays activated? (temp fix)
double Tce;
double c1e = -0.9667140742e-03, c2e = 4.350236730e-04, c3e = -4.428372908e-07; // Steinhart-Hart Coefficients

/* save time of events */
long last_btn_interrupt = 0;
long last_wheel_interrupt = 0;
long last_engine_interrupt = 0;
long last_speed_measure = 0;
long last_engine_measure = 0;
double speed_measurements[MEASUREMENT_HIST_CNT] = {0};
double rpm_measurements[MEASUREMENT_HIST_CNT] = {0};

unsigned int num_of_wheel_rotations = 0;
unsigned int num_of_engine_rotations = 0;

double measured_speed_kmh = 0.0;
double max_speed = 0;
double trip = 0.0;
double tank_trip = 0.0;
uint16_t measured_rpm = 0;
uint16_t max_rpm = 0;
double max_engine_temp = 0;

void lcd_init() {
  lcd.begin();
  lcd.command( PCD8544_FUNCTIONSET ); // power up the display driver
  lcd.setContrast(56);
  lcd.setBias(4);
  lcd.setTextColor(BLACK);
  lcd.clearDisplay();
}

void lcd_splash() {
  lcd.drawBitmap(0, 0, startup_logo, 84, 48, BLACK);
  lcd.display();
  delay(1000);
}

void SystemClock_Config(void)
{
   RCC_ClkInitTypeDef RCC_ClkInitStruct;
   RCC_OscInitTypeDef RCC_OscInitStruct;
   
   /* Enable Power Control clock */
   __HAL_RCC_PWR_CLK_ENABLE();
   
   /* The voltage scaling allows optimizing the power consumption when the device is 
      clocked below the maximum system frequency, to update the voltage scaling value 
      regarding system frequency refer to product datasheet.  */
   __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
 
   /* Enable HSE Oscillator and activate PLL with HSE as source */
   RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
   RCC_OscInitStruct.HSEState = RCC_HSE_ON;
   RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
   RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
   RCC_OscInitStruct.PLL.PLLM = 25;
   RCC_OscInitStruct.PLL.PLLN = 192;
   RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV8;
   RCC_OscInitStruct.PLL.PLLQ = 4;
   if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
   {
     /* Initialization Error */
     Error_Handler();
   }
   
   /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 
      clocks dividers */
   RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
   RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
   RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
   RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;  
   RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;  
   if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
   {
     /* Initialization Error */
     Error_Handler();
   }
}

void wake_up() {
  sleeping = false;
  if (backlight) digitalWrite(BACKLIGHT_PIN, LOW);
  lcd_init();
  lcd_splash();
}

/*
 * ISR for button press
 */
void on_btn_press() {

  if (sleeping) {
    wake_up();
    return;
  }

  if (millis() - last_btn_interrupt >= 300) {
  
    last_btn_interrupt = millis();
  
    while (digitalRead(MODE_SWITCH_PIN)) {
      long pressed_time = millis();
  
      if (pressed_time - last_btn_interrupt >= 1000) {

        if (mode == screen_settings) {
          switch (selected_menu_item) {
            case 0: toggle_bluetooth(); break;
            case 1: toggle_backlight(); break;
            case 2: toggle_lcd_invert(); break;
            default: break;
          }
        } else if (mode == screen_overview) {
          reset_trip();
        }

        return;
      } 

    }
  
    //mode++;
    if (mode == screen_settings) {
      if (selected_menu_item < MENU_ITEMS - 1) {
        selected_menu_item++;
      } else {
        selected_menu_item = 0;
        mode++;
      }
    } else {
      mode++;
    }

    if (mode >= NUM_OF_MODES)
      mode = 0;

  }
    
}

/*
 * ISR triggered by one wheel rotation 
 */
void on_rotation() {

  if (sleeping) {
    wake_up();
    return;
  }
  
  if (millis() - last_wheel_interrupt >= 10) {
    num_of_wheel_rotations++;
  }
   
  last_wheel_interrupt = millis();

}

/*
 * ISR triggered by crankshaft rotation
 */
void on_engine_rotation() {

  if (sleeping) {
    wake_up();
    return;
  }

  if (millis() - last_engine_interrupt >= 1) {
    num_of_engine_rotations++;
  }
   
  last_engine_interrupt = millis();
  
}

void reset_trip() {
  lcd.clearDisplay();
  lcd.setCursor(0,32);
  trip = 0.0;
  lcd.print("Trip reseted!");
  lcd.display();
  delay(500);
}

void toggle_bluetooth() {
  BT = !BT;
  if (BT) {
    digitalWrite(BT_EN_PIN, HIGH);
    BTserial.begin(9600);
  } else {
    digitalWrite(BT_EN_PIN, LOW);
    BTserial.end();
  }
}

void toggle_backlight() {
  backlight = !backlight;
  if (backlight) {
    digitalWrite(BACKLIGHT_PIN, LOW);
  } else {
    digitalWrite(BACKLIGHT_PIN, HIGH);
  }
}

void toggle_lcd_invert() {
  inverted_lcd = !inverted_lcd;
  if (inverted_lcd) {
    lcd.command( PCD8544_DISPLAYCONTROL | PCD8544_DISPLAYINVERTED);
  } else {
    lcd.command( PCD8544_DISPLAYCONTROL | PCD8544_DISPLAYNORMAL);
  }
}

void measure_speed() {

  // Calculate speed
  if (millis() - last_speed_measure >= MEASURE_INTERVAL) {

    double average = 0.0;
    int non_zero = 0;

    if (speed_measurements[0] != 0) {

      for (int i = 0; i < MEASUREMENT_HIST_CNT; i++) {
        if (speed_measurements[i] > 0) {
          average += speed_measurements[i];
          non_zero++;
        }
      }

      if (non_zero > 0) {
        average = average/non_zero;
      } else {
        average = 0.0;
      }
      
    } else {
      average = 0.0;
    }
    
    
    double measured_distance = (average * WH_CIRCUMFERENCE) / (MEASURE_INTERVAL/1000); // for MEASURE_INTERVAL
    trip += measured_distance/1000000;
    measured_speed_kmh = (measured_distance/1000) * 3.6;
    if (measured_speed_kmh > max_speed) max_speed = measured_speed_kmh;

    // advance queue
    for (int i = MEASUREMENT_HIST_CNT - 1; i > 0; i--) {
      speed_measurements[i] = speed_measurements[i - 1];
    }
    
    speed_measurements[0] = num_of_wheel_rotations;
    
    last_speed_measure = millis();
    num_of_wheel_rotations = 0;
    
  }
  
}

void measure_rpm() {

  if (millis() - last_engine_measure >= UPDATE_INTERVAL) {

    double average = 0;
    int non_zero = 0;


    for (int i = 0; i < MEASUREMENT_HIST_CNT; i++) {
      if (rpm_measurements[i] > 0) {
        average += rpm_measurements[i];
        non_zero++;
      }
    }

    if (non_zero > 0) {
      average = average/non_zero;
    } else {
      average = 0;
    }

    measured_rpm = average * (1000 / UPDATE_INTERVAL) * 60;
    if (measured_rpm > max_rpm) max_rpm = measured_rpm;

    // advance queue
    for (int i = MEASUREMENT_HIST_CNT - 1; i > 0; i--) {
      rpm_measurements[i] = rpm_measurements[i - 1];
    }
    
    rpm_measurements[0] = num_of_engine_rotations;
    
    last_engine_measure = millis();
    num_of_engine_rotations = 0;
  }

}

void measure_temperatures() {
  
  // Ambient
  int Voa = analogRead(AMBIENT_THERMISTOR_PIN);
  double R2a = R1a * (1023.0 / (float)Voa - 1.0);
  double logR2a = log(R2a);
  double Ta = (1.0 / (c1a + c2a*logR2a + c3a*logR2a*logR2a*logR2a));
  Tca = Ta - 273.15;

  // Engine
  int Voe = analogRead(ENGINE_THERMISTOR_PIN);
  double R2e = R1e * (1023.0 / (float)Voe - 1.0);
  double logR2e = log(R2e);
  double Te = (1.0 / (c1e + c2e*logR2e + c3e*logR2e*logR2e*logR2e));
  Tce = Te - 273.15;
  if (Tce > max_engine_temp) max_engine_temp = Tce;
  
}

void setup() {

  //SystemClock_Config()

  /* STM32F4 power savings */
  pinMode(PA8, INPUT_ANALOG);
  pinMode(PA9, INPUT_ANALOG);
  pinMode(PA10, INPUT_ANALOG);
  pinMode(PA11, INPUT_ANALOG);
  pinMode(PA12, INPUT_ANALOG);
  pinMode(PA15, INPUT_ANALOG);
  pinMode(PB4, INPUT_ANALOG);
  pinMode(PB3, INPUT_ANALOG);
  pinMode(PB10, INPUT_ANALOG);
  pinMode(PB12, INPUT_ANALOG);
  pinMode(PB13, INPUT_ANALOG);
  pinMode(PB14, INPUT_ANALOG);
  pinMode(PB15, INPUT_ANALOG);

  __HAL_RCC_GPIOC_CLK_DISABLE();
  __HAL_RCC_GPIOD_CLK_DISABLE();
  __HAL_RCC_GPIOE_CLK_DISABLE();
  __HAL_RCC_GPIOH_CLK_DISABLE();
  __HAL_RCC_USART1_CLK_DISABLE();
  __HAL_RCC_USART2_CLK_DISABLE();
  __HAL_RCC_USART6_CLK_DISABLE();
  __HAL_RCC_I2C1_CLK_DISABLE();
  __HAL_RCC_I2C2_CLK_DISABLE();
  __HAL_RCC_I2C3_CLK_DISABLE();
  __HAL_RCC_SDIO_CLK_DISABLE();
  __HAL_RCC_SPI2_CLK_DISABLE();
  __HAL_RCC_SPI3_CLK_DISABLE();
  __HAL_RCC_SPI4_CLK_DISABLE();
  __HAL_RCC_USB_OTG_FS_CLK_DISABLE();
  __HAL_RCC_SDIO_CLK_DISABLE();
  __HAL_RCC_SDMMC1_CLK_DISABLE();
  __HAL_RCC_DMA1_CLK_DISABLE();
  __HAL_RCC_DMA2_CLK_DISABLE();
  __HAL_RCC_CRC_CLK_DISABLE();
  __HAL_RCC_WWDG_CLK_DISABLE();

  __HAL_RCC_GPIOC_CLK_SLEEP_DISABLE();
  __HAL_RCC_GPIOD_CLK_SLEEP_DISABLE();
  __HAL_RCC_GPIOE_CLK_SLEEP_DISABLE();
  __HAL_RCC_GPIOH_CLK_SLEEP_DISABLE();
  __HAL_RCC_USART1_CLK_SLEEP_DISABLE();
  __HAL_RCC_USART2_CLK_SLEEP_DISABLE();
  __HAL_RCC_USART6_CLK_SLEEP_DISABLE();
  __HAL_RCC_I2C1_CLK_SLEEP_DISABLE();
  __HAL_RCC_I2C2_CLK_SLEEP_DISABLE();
  __HAL_RCC_I2C3_CLK_SLEEP_DISABLE();
  __HAL_RCC_SDIO_CLK_SLEEP_DISABLE();
  __HAL_RCC_SPI2_CLK_SLEEP_DISABLE();
  __HAL_RCC_SPI3_CLK_SLEEP_DISABLE();
  __HAL_RCC_SPI4_CLK_SLEEP_DISABLE();
  __HAL_RCC_USB_OTG_FS_CLK_SLEEP_DISABLE();
  __HAL_RCC_SDIO_CLK_SLEEP_DISABLE();
  __HAL_RCC_SDMMC1_CLK_SLEEP_DISABLE();
  __HAL_RCC_DMA1_CLK_SLEEP_DISABLE();
  __HAL_RCC_DMA2_CLK_SLEEP_DISABLE();
  __HAL_RCC_CRC_CLK_SLEEP_DISABLE();
  __HAL_RCC_WWDG_CLK_SLEEP_DISABLE();
  /* ----------------------------- */
  

  pinMode(AMBIENT_THERMISTOR_PIN, INPUT_ANALOG);
  pinMode(ENGINE_THERMISTOR_PIN, INPUT_ANALOG);
  pinMode(MODE_SWITCH_PIN, INPUT_PULLDOWN);
  pinMode(RPM_SENSOR_PIN, INPUT);
  pinMode(REED_SWITCH_PIN, INPUT);
  pinMode(BT_EN_PIN, OUTPUT);
  pinMode(BACKLIGHT_PIN, OUTPUT);

  digitalWrite(AMBIENT_THERMISTOR_PIN, HIGH);
  digitalWrite(ENGINE_THERMISTOR_PIN, HIGH);

  // start with backlight and bluetooth off
  digitalWrite(BT_EN_PIN, BT);
  digitalWrite(BACKLIGHT_PIN, HIGH);

  attachInterrupt(digitalPinToInterrupt(MODE_SWITCH_PIN), on_btn_press, RISING);
  attachInterrupt(digitalPinToInterrupt(RPM_SENSOR_PIN), on_engine_rotation, FALLING);
  attachInterrupt(digitalPinToInterrupt(REED_SWITCH_PIN), on_rotation, FALLING);
  
  rtc.begin();
  LowPower.begin();
  LowPower.attachInterruptWakeup(digitalPinToInterrupt(MODE_SWITCH_PIN), on_btn_press, RISING);
  LowPower.attachInterruptWakeup(digitalPinToInterrupt(RPM_SENSOR_PIN), on_engine_rotation, FALLING);
  LowPower.attachInterruptWakeup(digitalPinToInterrupt(REED_SWITCH_PIN), on_rotation, FALLING);

  lcd_init();
  lcd_splash();

}

void loop() {

  // Do the measuremets
  measure_temperatures();
  measure_speed();
  measure_rpm();

  if (measured_rpm > 0 || measured_speed_kmh > 0 || millis() - last_btn_interrupt < SLEEP_TIMEOUT)
    last_change = millis();

  /* Display info */
  lcd.clearDisplay();
  switch (mode) {
    case screen_overview: 
    
      if (measured_speed_kmh > 0 || measured_rpm > 0) {
        // Speed
        lcd.setFont(&FreeSansBold12pt7b);
        if (measured_speed_kmh < 10) {
          lcd.setCursor(10,20);
        } else {
          lcd.setCursor(1,20);
        }
        lcd.print(measured_speed_kmh, 0);
        lcd.drawBitmap(11, 22, icon_kmh, 13, 6, BLACK);
        lcd.setFont(); 
        lcd.setTextSize(1);

        // RPM
        // right alignment
        if (measured_rpm < 10) { // one numerical
          lcd.setCursor(78, 0);
        } else if (measured_rpm < 100) { // two ...
          lcd.setCursor(72, 0);
        } else if (measured_rpm < 1000) {
          lcd.setCursor(66, 0);
        } else if (measured_rpm < 10000) {
          lcd.setCursor(60, 0);
        } else if (measured_rpm < 100000) {
          lcd.setCursor(54, 0);
        }
        lcd.print(measured_rpm);
        lcd.drawBitmap(70, 8, icon_RPM, 13, 6, BLACK);


        // mini clock
        lcd.setCursor(54,31);
        if (rtc.getHours() < 10) 
          lcd.print("0");
        lcd.print(rtc.getHours());
        lcd.print(":");
        if (rtc.getMinutes() < 10)
          lcd.print("0");
        lcd.print(rtc.getMinutes());

      } else {

        lcd.setCursor(12,13);
        lcd.setFont(&FreeSansBold12pt7b);
        
        if (rtc.getHours() < 10) 
          lcd.print("0");
        lcd.print(rtc.getHours());
        lcd.print(":");
        if (rtc.getMinutes() < 10)
          lcd.print("0");
        lcd.print(rtc.getMinutes());

        lcd.setFont();
        lcd.setCursor(18, 22);
        lcd.print(rtc.getDay());
        lcd.print("/");
        lcd.print(rtc.getMonth());
        lcd.print("/20");
        lcd.print(rtc.getYear());

      }

      if (BT) {
        lcd.drawBitmap(72, 20, icon_bluetooth, 11, 8, BLACK);
      }
      
      // trip
      lcd.setTextSize(1);
      lcd.setCursor(0,31);
      lcd.print(trip, 3);
      lcd.print("km");  
    
      // Temperatures
      lcd.drawBitmap(0, 40, icon_ambient_temp, 5, 8, BLACK);
      lcd.setCursor(7,41);
      lcd.print(Tca, 1);
      lcd.drawBitmap(32, 40, icon_temp_C, 10, 8, BLACK);
    
      lcd.drawBitmap(47, 40, icon_engine_temp, 5, 8, BLACK);
      lcd.setCursor(54,41);
      lcd.print(Tce, 0);
      lcd.drawBitmap(73, 40, icon_temp_C, 10, 8, BLACK);
      
      break;

    case screen_overview2:

      lcd.setTextSize(1);
      lcd.setFont(&FreeSansBold12pt7b);

      lcd.setCursor(0, 6);
      
      // right alignment
      if (measured_rpm < 10) { // one numerical
        lcd.setCursor(71, 16);
      } else if (measured_rpm < 100) { // two ...
        lcd.setCursor(57, 16);
      } else if (measured_rpm < 1000) {
        lcd.setCursor(43, 16);
      } else if (measured_rpm < 10000) {
        lcd.setCursor(29, 16);
      } else if (measured_rpm < 100000) {
        lcd.setCursor(15, 16);
      }

      lcd.print(measured_rpm);

      lcd.drawBitmap(67, 20, icon_RPM, 13, 6, BLACK);

      
      lcd.drawBitmap(0, 36, icon_engine_temp, 5, 8, BLACK);
      lcd.setCursor(10, 46);
      lcd.print(Tce, 1);

      lcd.drawBitmap(67, 36, icon_temp_C, 16, 8, BLACK);

      lcd.setFont();
    
      
      break;

    case screen_speedo:

      lcd.setFont(&FreeSansBold24pt7b);

      lcd.setTextSize(1);
      if (measured_speed_kmh < 10) {
        lcd.setCursor(29,35);
      } else {
        lcd.setCursor(14,35);
      }
      lcd.print(measured_speed_kmh, 0);
      lcd.drawBitmap(35, 39, icon_kmh, 13, 6, BLACK);

      lcd.setFont();
    
      break;

    case screen_rpm:
    
      lcd.setTextSize(1);
      lcd.setFont(&FreeSansBold12pt7b);
      lcd.setCursor(0, 36);
      
      // right alignment
      if (measured_rpm < 10) { // one numerical
        lcd.setCursor(71, 36);
      } else if (measured_rpm < 100) { // two ...
        lcd.setCursor(57, 36);
      } else if (measured_rpm < 1000) {
        lcd.setCursor(43, 36);
      } else if (measured_rpm < 10000) {
        lcd.setCursor(29, 36);
      } else if (measured_rpm < 100000) {
        lcd.setCursor(15, 36);
      }

      lcd.print(measured_rpm);

      lcd.drawBitmap(35, 39, icon_RPM, 13, 6, BLACK);
      lcd.setFont();
    
      break;

    case screen_temps:

      lcd.setTextSize(1);
      lcd.setFont(&FreeSansBold12pt7b);

      lcd.drawBitmap(0, 6, icon_ambient_temp, 5, 8, BLACK);
      lcd.setCursor(10, 18);
      lcd.print(Tca, 1);
      
      lcd.drawBitmap(0, 36, icon_engine_temp, 5, 8, BLACK);
      lcd.setCursor(10, 46);
      lcd.print(Tce, 1);

      lcd.drawBitmap(67, 20, icon_temp_C, 16, 8, BLACK);

      lcd.setFont();

      break;
      
    case screen_stats:

      lcd.setTextSize(1);
      lcd.setCursor(12,0);
      lcd.print("Statistiky");
      lcd.drawLine(0, 8, 83, 8, BLACK);

      lcd.setCursor(0, 12);
      lcd.print("Max SPD-");
      lcd.print(max_speed, 1);

      lcd.setCursor(0, 22);
      lcd.print("Max RPM-");
      lcd.print(max_rpm);

      lcd.setCursor(0, 32);
      lcd.print("Max");
      lcd.drawBitmap(18, 32, icon_engine_temp, 5, 8, BLACK);
      lcd.setCursor(25, 32);
      lcd.print("-");
      lcd.print(max_engine_temp, 0);

      break;

    case screen_settings:
      lcd.setTextSize(1);
      lcd.setCursor(16,0);
      lcd.print("Nastaveni");
      lcd.drawLine(0, 8, 83, 8, BLACK);
      
      lcd.setCursor(3, 12);
      lcd.print("Bluetooth");
      BT ? lcd.drawBitmap(65, 12, icon_toggle_on, 16, 8, BLACK) : lcd.drawBitmap(65, 12, icon_toggle_off, 16, 8, BLACK);
      if (selected_menu_item == 0)
        lcd.drawRoundRect(0, 10, 84, 12, 3, BLACK);

      lcd.setCursor(3, 23);
      lcd.print("Podsviceni");
      backlight ? lcd.drawBitmap(65, 23, icon_toggle_on, 16, 8, BLACK) : lcd.drawBitmap(65, 23, icon_toggle_off, 16, 8, BLACK);
      if (selected_menu_item == 1)
        lcd.drawRoundRect(0, 21, 84, 12, 3, BLACK);

      
      lcd.setCursor(3, 34);
      lcd.print("Invertovat");
      inverted_lcd ? lcd.drawBitmap(65, 34, icon_toggle_on, 16, 8, BLACK) : lcd.drawBitmap(65, 34, icon_toggle_off, 16, 8, BLACK);
      if (selected_menu_item == 2)
        lcd.drawRoundRect(0, 32, 84, 12, 3, BLACK);
    
      break;

    default: break;
  }
  
  lcd.display();

  /* Send data over BT - JSON serialized */
  if (BT) {

    while (BTserial.available() > 0) {

      char readchar;
      if ((readchar = BTserial.read()) != '$') break;
      switch (readchar = BTserial.read()) {

        char tmp;

        case 'T': 

          if ((readchar = BTserial.read()) != '|') break;
          char hours, minutes;
            
          tmp = (BTserial.read() - 48);
          hours = (tmp * 10) + (BTserial.read() - 48);

          if ((readchar = BTserial.read()) != ':') break;

          tmp = (BTserial.read() - 48);
          minutes = (tmp * 10) + (BTserial.read() - 48);

          rtc.setTime(hours, minutes, 0);

          break;

        case 'D': 

          if ((readchar = BTserial.read()) != '|') break;
          char day, month, year;
            
          tmp = (BTserial.read() - 48);
          day = (tmp * 10) + (BTserial.read() - 48);

          if ((readchar = BTserial.read()) != '/') break;

          tmp = (BTserial.read() - 48);
          month = (tmp * 10) + (BTserial.read() - 48);

          if ((readchar = BTserial.read()) != '/') break;

          tmp = (BTserial.read() - 48);
          year = (tmp * 10) + (BTserial.read() - 48);

          rtc.setDate(day, month, year);

          break;
        
        default: break;
      }

    }

    
    BTserial.println("{");
    
    BTserial.print("\t");
    BTserial.print("\"speed\": ");
    BTserial.print(measured_speed_kmh);
    BTserial.println(",");

    BTserial.print("\t");
    BTserial.print("\"rpm\": ");
    BTserial.print(measured_rpm);
    BTserial.println(",");

    BTserial.print("\t");
    BTserial.print("\"maxSpeed\": ");
    BTserial.print(max_speed);
    BTserial.println(",");

    BTserial.print("\t");
    BTserial.print("\"maxRpm\": ");
    BTserial.print(max_rpm);
    BTserial.println(",");

    BTserial.print("\t");
    BTserial.print("\"trip\": ");
    BTserial.print(trip);
    BTserial.println(",");

    BTserial.print("\t");
    BTserial.print("\"tempAmbient\": ");
    BTserial.print(Tca);
    BTserial.println(",");

    BTserial.print("\t");
    BTserial.print("\"tempEngine\": ");
    BTserial.print(Tce);
    BTserial.println("");

    BTserial.println("}");
    
  }

  /* Sleep after inactivity */
  if (millis() - last_change >= SLEEP_TIMEOUT) {
    lcd.clearDisplay();
    lcd.display();
    lcd.command( PCD8544_FUNCTIONSET | PCD8544_POWERDOWN);
    if (backlight) digitalWrite(BACKLIGHT_PIN, HIGH);
    sleeping = true;
    LowPower.deepSleep();
  } 
  
  delay(UPDATE_INTERVAL);
  
}