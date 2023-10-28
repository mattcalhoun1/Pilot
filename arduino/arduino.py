import serial
import time
import random
import logging

class Arduino:
    def __init__(self, usb_port = '/dev/ttyACM1', baud = 9600, timeout = 1):
        self.__serial = serial.Serial(port=usb_port, baudrate=baud, timeout=timeout)

        # device could be /dev/ttyUSB0 or /dev/ttyACM0

    def handle_message(self, message):
        # subclass needs to do something here
        pass

    def is_ready (self):
        # is there a ready message waiting?
        while self.has_message():
            msg = self.get_message()
            if msg == '!READY!':
                return True
            #else:
            #    print(f"MSG:{msg}")
            self.handle_message(msg)
            return True

        return True
    
    def cleanup (self):
        self.__serial.close()

    def refresh_connection (self):
        self.clear_input_buffer()
        self.__serial.close()
        self.__serial.open()

    def clear_input_buffer (self):
        self.__serial.reset_input_buffer()

    def has_message (self):
        hasmsg = self.__serial.in_waiting > 0
        return hasmsg
    
    def get_message (self):
        # read bytes until we hit a carriage return or newline
        complete_message = False
        message_buffer = ""
        while complete_message == False:
            next_byte = self.__serial.read()
            try:
                next_str = next_byte.decode('utf-8')
                if next_str == '\n' or next_str == '\r' or next_str == '\0' or next_str == '\x00':
                    complete_message = True
                else:
                    message_buffer += next_str
            except Exception as e:
                print(f"Exception reading serial port: {e}")

        return message_buffer.rstrip()
    
    def send_message (self, message):
        self.__serial.write(f"{message}\n".encode('UTF-8'))
        logging.getLogger(__name__).info(f"Sent: {message}")

    def wait_for_ready (self, max_wait_secs = 10):
        # wait for ready
        logging.getLogger(__name__).debug("Waiting for arduino to become ready")
        start = time.time()
        ready = False
        while time.time() - start < max_wait_secs and not ready:
            ready = self.is_ready()
            time.sleep(.1)

        if ready:
            logging.getLogger(__name__).debug("Arduino is ready")        
        else:
            logging.getLogger(__name__).warning("Arduino is NOT ready")        

        return ready
    
    def wait_for_result (self, max_wait_secs = 10):
        logging.getLogger(__name__).info("Waiting for arduino result")
        start = time.time()
        received_result = False
        result = False
        while not received_result and time.time() - start < max_wait_secs:
            # wait for hte response
            while not self.has_message():
                time.sleep(.1)
                
            a_msg = self.get_message()
            if 'Result:' in a_msg:
                logging.getLogger(__name__).info(f"Received: {a_msg}")
                received_result = True
                result = a_msg.endswith(":0") # 0 is success

        return result            

if __name__ == '__main__':
    logging.basicConfig(format='%(levelname)s:%(message)s', level=logging.INFO)

    arduino = Arduino()

    # wait for ready
    if(arduino.wait_for_ready()):
        # send the go command
        moving = False

        while True:
            lastChange = time.time()
            if moving:
                moving = False
                arduino.send_message('STOP:0')
            else:
                # randomly either roate or go forward
                if random.Random().randint(a=0,b=1) == 0:
                    # rotate 90
                    arduino.send_message('ROTATE:90.0')
                    moving = True            
                else:
                    arduino.send_message('GO:0|2000')
                    moving = True

            received_result = False
            while not received_result:
                # wait for hte response
                while not arduino.has_message():
                    time.sleep(.1)
                    
                a_msg = arduino.get_message()
                if 'Result:' in a_msg:
                    logging.getLogger(__name__).info(f"Received: {a_msg}")
                    received_result = True
    else:
        logging.getLogger(__name__).error("Arduino not ready!")
