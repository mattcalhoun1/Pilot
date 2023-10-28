import lcd.RPi_I2C_driver as driver
import time

class LcdScreen:
    def __init__(self):
        self.__lcd = driver.lcd()
        self.__max_len = 16

    def set_backlight_on(self):
        self.__lcd.backlight(1)

    def set_backlight_off(self):
        self.__lcd.backlight(0)

    def show_message (self, message, line_number):
        # clear any existing message first
        self.__lcd.lcd_display_string(f"{' '*16}",line_number)

        if len(message) > self.__max_len:
            message = message[:self.__max_len]

        self.__lcd.lcd_display_string(f"{message:^16}",line_number)
    
    def clear (self):
        self.__lcd.lcd_clear()
    