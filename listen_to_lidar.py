import serial
import threading
from calc_lidar_data import calc_lidar_data


def listen_to_lidar(port: str = '/dev/tty.usbserial-0001') -> tuple[dict, callable]:
    serial_port = serial.Serial(port=port, baudrate=230400, timeout=5.0, bytesize=8, parity='N', stopbits=1)
    # distances are angles (in degrees) mapped to distance values (in centimeters)
    # last_packet_data is a LidarData instance containing the parsed data of the last packet
    data = {
        'distances': {},
        'last_packet_data': None
    }
    data_channel = {'interrupt': False}

    def update_data():
        # in bytes (speed and start_angle + 13 measurements + end_angle and timestamp + crc_check)
        packet_length = 4 + (3 * 12) + 4 + 1

        while not data_channel['interrupt']:
            last_byte_was_header = False
            buffer = ""

            # serial read loop
            # reads until it sees the header and var_len bytes, then parses the packet and resets the buffer
            while True:
                byte = serial_port.read()
                # convert byte to integer (big endian)
                byte_as_int = int.from_bytes(byte, 'big')

                # check for header byte
                if byte_as_int == 0x54:
                    buffer += byte.hex()
                    last_byte_was_header = True
                    # read next byte
                    continue

                # check for var_len byte (fixed value, comes after header byte)
                # if yes -> parse data, and update "data" variable
                if last_byte_was_header and byte_as_int == 0x2c:
                    buffer += byte.hex()

                    # if the packet length of the received packet doesn't have the expected length, something went wrong
                    # -> drop packet and restart read loop
                    if not len(buffer[0:-4]) == packet_length * 2:
                        buffer = ""
                        break

                    lidar_data = calc_lidar_data(buffer[0:-4])

                    # cleanup outdated values
                    for angle in lidar_data.angle_i:
                        start_angle, end_angle = lidar_data.start_angle, lidar_data.end_angle
                        # remove angle only if it is in the range of the current data packet
                        if angle in data['distances'] and (start_angle < angle < end_angle or (
                                end_angle > start_angle and (angle > start_angle or angle < end_angle))):
                            del data['distances'][angle]

                    # write new distance data to distances
                    for i, angle in enumerate(lidar_data.angle_i):
                        data['distances'][angle] = lidar_data.distance_i[i]

                    # override last_packet_data
                    data['last_packet_data'] = lidar_data

                    # reset buffer
                    buffer = ""
                    break
                else:  # is not header or var_len -> write to buffer
                    buffer += byte.hex()

                last_byte_was_header = False

    # call update_data in a separate thread to make listen_to_lidar return and update_data still update the "data" var
    read_thread = threading.Thread(target=update_data)
    read_thread.start()

    def stop():
        # send interrupt signal
        data_channel['interrupt'] = True
        # wait for thread to terminate
        read_thread.join()
        serial_port.close()

    return data, stop
