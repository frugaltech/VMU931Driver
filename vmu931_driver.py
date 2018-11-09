import serial
import logging
import struct
from time import sleep


class VMU931Driver:
    gyro_resolutions = {250: 0, 500: 1, 1000: 2, 2000: 3}
    accelerometer_resolutions = {2: 4, 4: 5, 8: 6, 16: 7}

    basic_configuration = {
        'magnetometer_enabled': None,
        'gyroscope_enabled': None,
        'accelerometer_enabled': None,
        'gyroscope_resolution': 2000,
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
        self.device = 'COM3'
        self.read_timeout = 2.0
        self.write_timeout = 2.0
        driverstatus = self.get_configuration()
        self.set_configuration(driverstatus, self.basic_configuration)

    """Get the configuration message from the VMU device. Needed to figure out what to change."""

    def get_configuration(self):
        config_done = False
        while not config_done:
            with serial.Serial(port=self.device, timeout=self.read_timeout, write_timeout=self.write_timeout) as port:
                self._ask_for_status(port)
                # Read until a message start is encountered
                while port.read()[0] is not 0x01:
                    continue
                # Read the message length
                message_size = port.read()[0] - 4
                # Get the message type as a char
                message_type = chr(port.read()[0])
                # Get the actual message
                message = port.read(message_size)
                # Get the message terminator
                message_end = port.read()[0]

                # Check the message_end to see if we actually intercepted a message or if we just got some part of a stream
                if message_end is not 0x04:
                    logging.warning('Encountered a broken message or some other stream artifact.')
                    continue

                # We got a status message - yay!
                if message_type is 's':
                    status = self._parse_status_message(message)
                    config_done = True
                    logging.debug('Retrieved status object from device at %s: %s', self.device, status)
                    return status

    """Set the VMU configuration. For now, it is just static, allowing gyro streaming"""

    def set_configuration(self, current_configuration=None, new_configuration=None):
        if current_configuration is None or new_configuration is None:
            raise ValueError('Current and new configuration must be supplied as argument')

        with serial.Serial(port=self.device, timeout=self.read_timeout, write_timeout=self.write_timeout) as port:
            self.set_gyro_resolution(port, new_configuration['gyroscope_resolution'])
            self.set_accelerometer_resolution(port, new_configuration['accelerometer_resolution'])
            if current_configuration['accelerometer_streaming'] ^ new_configuration['accelerometer_streaming']:
                self.send_message(port, 'vara')
            if current_configuration['gyroscope_streaming'] ^ new_configuration['gyroscope_streaming']:
                self.send_message(port, 'varg')
            if current_configuration['magnetometer_streaming'] ^ new_configuration['magnetometer_streaming']:
                self.send_message(port, 'varc')
            if current_configuration['quaternion_streaming'] ^ new_configuration['quaternion_streaming']:
                self.send_message(port, 'varq')
            if current_configuration['euler_streaming'] ^ new_configuration['euler_streaming']:
                self.send_message(port, 'vare')
            if current_configuration['heading_streaming'] ^ new_configuration['heading_streaming']:
                self.send_message(port, 'varh')

    """Get a message of type m_type (gyro, ...) and return ts, x, y and z values"""

    def get_message(self, m_type='gyro'):
        if m_type not in VMU931Driver.message_types.keys():
            raise ValueError('Message type must be one of: {}'.format(VMU931Driver.message_types.keys()))

        message_captured = False
        while not message_captured:
            with serial.Serial(port=self.device, timeout=self.read_timeout, write_timeout=self.write_timeout) as port:
                while port.read()[0] is not 0x01:
                    continue
                # Read the message length
                message_size = port.read()[0] - 4
                # Get the message type as a char
                message_type = chr(port.read()[0])
                # Get the actual message
                message = port.read(message_size)
                # Get the message terminator
                message_end = port.read()[0]

                # Check the message_end to see if we actually intercepted a message or if we just got some part of a stream
                if message_end is not 0x04:
                    continue

                if message_type is not VMU931Driver.message_types[m_type]:
                    continue

                # Now parse the message - it has to be one of the
                ts, x, y, z = struct.unpack(">Ifff", message[:16])
                message_captured = True
                logging.debug('Captured a message of type %s', m_type)
                return (ts, x, y, z)

    """Simple question - ask for status"""

    def _ask_for_status(self, port):
        self.send_message(port, 'vars')

    """Set gyro resolution to 250, 500, 1000 or 2000 samples per second"""

    def set_gyro_resolution(self, port, resolution):
        if not resolution in VMU931Driver.gyro_resolutions.keys():
            raise ValueError('Resolution must be one of: {}'.format(VMU931Driver.gyro_resolutions.keys()))
        self.send_message(port, 'var{}'.format(VMU931Driver.gyro_resolutions.get(resolution)))

    """Set accelerometer resolution to one of 2, 4, 8 or 16"""

    def set_accelerometer_resolution(self, port, resolution):
        if not resolution in VMU931Driver.accelerometer_resolutions.keys():
            raise ValueError('Resolution must be one of: {}'.format(VMU931Driver.accelerometer_resolutions.keys()))
        self.send_message(port, 'var{}'.format(VMU931Driver.accelerometer_resolutions.get(resolution)))

    """Encode and send a message on the supplied port. Message should just be a string."""

    def send_message(self, port=None, message=None):
        if port is None or message is None or len(message) == 0:
            raise ValueError('port must be defined and open, message must not be None')
        try:
            port.write(message.encode('ASCII'))
        except Exception as e:
            logging.error('Unable to post message %s to device at: %s', message, self.device)
            raise e

    """Parse a status message according to the VMU931 manual available here: https://variense.com/Docs/VMU931/VMU931_UserGuide.pdf"""

    @staticmethod
    def _parse_status_message(message):
        status, res, low_output, data = struct.unpack(">BBBI", message[:7])

        mag_status = status & 0b00000100 != 0
        gyro_status = status & 0b00000010 != 0
        acc_status = status & 0b00000001 != 0

        gyro_res = None

        if res & 0b10000000 != 0:
            gyro_res = 2000
        elif res & 0b01000000 != 0:
            gyro_res = 1000
        elif res & 0b00100000 != 0:
            gyro_res = 500
        elif res & 0b00010000 != 0:
            gyro_res = 250

        acc_res = None

        if res & 0b00001000 != 0:
            acc_res = 16
        elif res & 0b000000100 != 0:
            acc_res = 8
        elif res & 0b00000010 != 0:
            acc_res = 4
        elif res & 0b00000001 != 0:
            acc_res = 2

        low_output_rate = low_output & 0b00000001 != 0

        heading_streaming = data & 0b01000000 != 0
        euler_streaming = data & 0b00010000 != 0
        mag_streaming = data & 0b00001000 != 0
        quat_streaming = data & 0b00000100 != 0
        gyro_streaming = data & 0b00000010 != 0
        acc_streaming = data & 0b00000001 != 0

        config = VMU931Driver.basic_configuration.copy()
        config['magnetometer_enabled'] = mag_status
        config['gyroscope_enabled'] = gyro_status
        config['accelerometer_enabled'] = acc_status
        config['gyroscope_resolution'] = gyro_res
        config['accelerometer_resolution'] = acc_res
        config['low_output_rate'] = low_output_rate
        config['heading_streaming'] = heading_streaming
        config['euler_streaming'] = euler_streaming
        config['magnetometer_streaming'] = mag_streaming
        config['quaternion_streaming'] = quat_streaming
        config['gyroscope_streaming'] = gyro_streaming
        config['accelerometer_streaming'] = acc_streaming

        return config


if __name__ == '__main__':
    driver = VMU931Driver()
    config = driver.get_configuration()
    print(config)
    new_config = VMU931Driver.basic_configuration.copy()
    driver.set_configuration(current_configuration=config, new_configuration=new_config)
    while True:
        ts, x, y, z = driver.get_message('gyro')
        print('Got: x:{}, y:{}, z:{}'.format(x, y, z))
        sleep(5)
