import logging
from arduino.tank import Tank

if __name__ == '__main__':
    logging.basicConfig(format='%(levelname)s:%(message)s', level=logging.INFO)

    tank = Tank()
    if tank.wait_for_ready():
        result = tank.go(speed=1, duration_millis=500)
        logging.getLogger(__name__).info(f"Go Forward Result: {result}")

        result = tank.stop()
        logging.getLogger(__name__).info(f"Stop Result: {result}")

        tank.cleanup()