class LidarData:
    def __init__(self, start_angle, end_angle, crc_check, speed, time_stamp, confidence_i, angle_i, distance_i):
        self.start_angle = start_angle
        self.end_angle = end_angle
        self.crc_check = crc_check
        self.speed = speed
        self.time_stamp = time_stamp

        self.confidence_i = confidence_i
        self.angle_i = angle_i
        self.distance_i = distance_i


def calc_lidar_data(packet):
    # packet is a string representing a received packet in hexadecimal (1 byte / 8 bits := 2 characters)

    # retrieves nth byte of packet (as hexadecimal string; one character)
    # n can be negative to get nth last byte
    def get_byte(n: int) -> str:
        if n == -1:
            return packet[n * 2:]
        return packet[n * 2:(n + 1) * 2]

    # get infos from data packet (bytes are flipped for each value)
    # speed in degrees per second
    speed = int(get_byte(1) + get_byte(0), 16)
    # start and end angle in thousands of a degree
    start_angle = float(int(get_byte(3) + get_byte(2), 16)) / 100  # degrees (after division)
    end_angle = float(int(get_byte(-4) + get_byte(-5), 16)) / 100  # degrees (after division)
    # timestamp in milliseconds (returns to 0 after 30000)
    time_stamp = int(get_byte(-2) + get_byte(-3), 16)
    crc_check = int(get_byte(-1), 16)

    confidence_i = list()
    angle_i = list()
    distance_i = list()

    # calculate the distance between (adjacent) measurements in degrees
    # the total travel of the motor (in degrees) divided by the amount of measurements (fixed at 12)
    if end_angle > start_angle:
        angle_step = float(end_angle - start_angle) / 12
    else:
        angle_step = float((end_angle + 360) - start_angle) / 12

    def circle(deg):
        return deg - 360 if deg >= 360 else deg

    # size of one measurement in bytes
    measurement_packet_size = 3
    # get data from each measurement (bytes are flipped for each value)
    # measurement data starts at the 5th byte (index: 4)
    for i in range(0, 12):
        # measurement position in measurement data section of packet
        measurement_position = measurement_packet_size * i
        # distance in millimeters
        distance_bytes = get_byte(5 + measurement_position) + get_byte(4 + measurement_position)
        distance_i.append(int(distance_bytes, 16) / 10)  # centimeters (after division)
        # intensity of signal (unitless)
        confidence_i.append(int(get_byte(6 + measurement_position), 16))
        # angle (position) of the measurement in degrees
        angle_i.append(circle(start_angle + (angle_step * i)))

    lidar_data = LidarData(start_angle, end_angle, crc_check, speed, time_stamp, confidence_i, angle_i, distance_i)
    return lidar_data
