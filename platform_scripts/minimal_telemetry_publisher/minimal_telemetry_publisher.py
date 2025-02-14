from logging import getLogger, basicConfig, INFO
from std_msgs.msg import Bool
from telemetry import TelemetryHeaderBuilder
import io
import os
import pickle
import rospy
import socket
import subprocess
import sys
import time
import yaml


basicConfig(level=INFO)
logger = getLogger(os.path.basename(__file__))


class MinimalTelemetryPublisher(object):
    CONFIG_YAML_PATH = '/home/nvidia/IB2/src/communication/trans_communication/config/config.yml'
    LEX_MIN_USER_DATA_BYTES = 68
    LEX_MAX_USER_DATA_BYTES = 1472
    SENDING_SLEEP_TIME_SEC = 3
    SENDING_SOURCE_PORT = 22234
    SENDING_TARGET_IP = 'localhost'
    SENDING_TARGET_PORT = 34567
    TELEMETRY_ID_NORMAL_FLIGHT_SOFTWARE_STATUS = 9001
    TELEMETRY_ID_PLATFORM_FLIGHT_SOFTWARE_STATUS = 9002
    intball_app_config = None

    def __init__(self):
        # Read setting YAML file
        try:
            with open(MinimalTelemetryPublisher.CONFIG_YAML_PATH, 'r') as yaml_file:
                logger.info('config yaml path {}'.format(MinimalTelemetryPublisher.CONFIG_YAML_PATH))
                self.intball_app_config = yaml.load(
                    yaml_file, Loader=yaml.FullLoader)
                logger.info('loaded yaml {}'.format(
                    self.intball_app_config))
        except Exception as e:
            logger.error(e)
            logger.error('trans communication node will stop.'
                         'because of the failure to read the config file: "{}".'
                         .format(MinimalTelemetryPublisher.CONFIG_YAML_PATH))
            logger.info('TransCommunication.receive out')
            raise e
        self.telemetry_header_builder = TelemetryHeaderBuilder(self.intball_app_config)

        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.bind(('', MinimalTelemetryPublisher.SENDING_SOURCE_PORT))

    def run(self):

        while True:
            try:
                data_dict = {}

                self.telemetry_header_builder.write_header(0xFFFF, 1, 1, data_dict, 0xFF,
                                                           ros_timestamp=rospy.Time.from_sec(time.time()))

                # State of the normal flight software (ROS)
                normal_flight_software_status = Bool()
                # Check that the process of roslaunch is up and running
                cmd_return = subprocess.run(('ps aux | '
                                             'grep -v grep | '
                                             'grep \'roslaunch task_manager ib2_bringup.launch\' | '
                                             'wc -l'),
                                            shell=True, check=False,
                                            stdout=subprocess.PIPE, universal_newlines=True).stdout.strip()
                normal_flight_software_status.data = (cmd_return
                                                      and cmd_return.isdecimal()
                                                      and int(cmd_return) != 0)
                temp_buf = io.BytesIO()
                normal_flight_software_status.serialize(temp_buf)
                data_dict[MinimalTelemetryPublisher.TELEMETRY_ID_NORMAL_FLIGHT_SOFTWARE_STATUS] = temp_buf.getvalue()

                # State of the platform flight software (ROS)
                platform_flight_software_status = Bool()
                # Check that the process of roslaunch is up and running
                cmd_return = subprocess.run(('ps aux | '
                                             'grep -v grep | '
                                             'grep \'roslaunch platform_manager ib2_bringup.launch\' | '
                                             'wc -l'),
                                            shell=True, check=False,
                                            stdout=subprocess.PIPE, universal_newlines=True).stdout.strip()
                platform_flight_software_status.data = (cmd_return
                                                        and cmd_return.isdecimal()
                                                        and int(cmd_return) != 0)
                temp_buf = io.BytesIO()
                platform_flight_software_status.serialize(temp_buf)
                data_dict[MinimalTelemetryPublisher.TELEMETRY_ID_PLATFORM_FLIGHT_SOFTWARE_STATUS] = temp_buf.getvalue()

                # Format the data for transmission
                pickled_telemetry = pickle.dumps(data_dict, protocol=3)
                zero_padded_pickled_telemetry = pickled_telemetry + (self.LEX_MIN_USER_DATA_BYTES -
                                                                     sys.getsizeof(pickled_telemetry)) * b'\0'
                if len(zero_padded_pickled_telemetry) > MinimalTelemetryPublisher.LEX_MAX_USER_DATA_BYTES:
                    logger.error('[send_telemetry] telemetry size {} exceeds max size (={} bytes)'.format(
                        len(zero_padded_pickled_telemetry), MinimalTelemetryPublisher.LEX_MAX_USER_DATA_BYTES))
                    raise Exception("telemetry size error")

                # Send telemetry
                self.socket.sendto(zero_padded_pickled_telemetry,
                                   (MinimalTelemetryPublisher.SENDING_TARGET_IP,
                                    MinimalTelemetryPublisher.SENDING_TARGET_PORT))
                logger.info('[send_telemetry] finish to send. port {}, len {}, byte_size {}'.format(
                            self.socket, len(zero_padded_pickled_telemetry),
                            sys.getsizeof(zero_padded_pickled_telemetry)))

                time.sleep(MinimalTelemetryPublisher.SENDING_SLEEP_TIME_SEC)
            except Exception as e:
                logger.error('[send_telemetry]fail to send. error {}'.format(e))
                time.sleep(MinimalTelemetryPublisher.SENDING_SLEEP_TIME_SEC)


if __name__ == '__main__':
    publisher = MinimalTelemetryPublisher()
    publisher.run()
