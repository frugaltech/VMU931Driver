import logging
import usb.core
import usb.util
import struct
from time import sleep


class VMU931CDCDriver:
    basic_configuration = {
        'magnetometer_enabled': None,
        'gyroscope_enabled': None,
        'accelerometer_enabled': None,
        'gyroscope_resolution': 500,
        'accelerometer_resolution': 2,
        'low_output_rate': None,
        'heading_streaming': False,
        'euler_streaming': False,
        'magnetometer_streaming': False,
        'quaternion_streaming': False,
        'gyroscope_streaming': True,
        'accelerometer_streaming': False
    }

    message_types = {'gyro': 'g', 'euler': 'e', 'magnet': 'c', 'accel': 'a'}

    def __init__(self):
        self.device = None
        self.writer = None
        self.reader = None

    def connect(self):
        self.device = usb.core.find(idVendor=0x16d0, idProduct=0x0cba)
        if self.device is None:
            raise ValueError('Our device is not connected')
        self.device.reset()
        sleep(0.5)
        configuration = self.device.get_active_configuration()
        print(configuration)
        self.device.set_configuration()
        interface = configuration[(1, 0)]
        self.writer = interface[1]
        self.reader = interface[0]

    def get_vmu_packet(self, m_type='euler'):
        if self.device is None:
            logging.error('No VMU device found')
            return None

        data = bytearray()
        iter_count = 0
        while True:
            # Bail out if the device just doesn't seem to cooperate
            iter_count += 1
            if iter_count > 5000:
                logging.warning('Waited %s iterations without getting a message', iter_count)
                return

            # Get whatever bytes are available in the device buffer
            data.extend(self.reader.read(64))

            message = None
            message_type = None
            message_end = None
            message_size = 0
            i = 0
            for i, c in enumerate(data):
                # Check for message start and that there are enough bytes for a complete message
                if c is 0x01 and len(data[i:]) > 20:
                    try:
                        message_size = data[i + 1] - 4
                        message_type = chr(data[i + 2])
                        message = data[i + 3:i + 3 + message_size]
                        message_end = data[i + 3 + message_size]
                    except:
                        # This block of data was bad somehow - try a new one
                        data = bytearray()
                        break

                    if message_end is 0x04 and message_type is VMU931CDCDriver.message_types[m_type]:
                        # Now parse the message - it has to be one of the
                        ts, x, y, z = struct.unpack(">Ifff", message[:16])
                        logging.debug('Captured a message of type %s', m_type)
                        return (ts, x, y, z)
                    else:
                        logging.debug('Scanning past non-matching message')
                        if message_end is None:
                            data = bytearray()
                        else:
                            data = data[i + 3 + message_size:]
                        sleep(0.02)


if __name__ == '__main__':
    driver = VMU931CDCDriver()
    driver.connect()
    while True:
        status = driver.get_vmu_packet()
        print(status)
        sleep(5)
