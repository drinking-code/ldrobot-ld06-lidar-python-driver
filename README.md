# Basic Driver for the LD06 LiDAR Sensor

This repository provides a basic (currently read-only) driver for LDROBOT's LD06 LiDAR sensor.  
This code was largely adopted from [henjin0's repository](https://github.com/henjin0/LIDAR_LD06_python_loder).

> ⚠️ THE CODE IS CURRENTLY NOT TESTED TO WORK WHATSOEVER ⚠️

# Usage

`listen_to_lidar.py` provides a function to retrieve distance data from a serial port:

## `listen_to_lidar(port: string)`

The argument `port` defines the serial port to listen to when reading from the LiDAR. It defaults
to `"/dev/tty.usbserial-0001"`.  
`listen_to_lidar()` returns a tuple. The first value of the tuple is a dictionary "`data`" with the following key/value
pairs:

**First return value: `data`**

| Key                | Description                                                                                                 |
|--------------------|-------------------------------------------------------------------------------------------------------------|
| `distances`        | A dictionary with angles (integers; in degrees) as keys and distances (integers; in centimeters) as values. |
| `last_packet_data` | An instance of the `LidarData` class that contains the parsed data of the last received packet.             |

**Properties of `LidarData`**

| Property Name  | Description                                                                                                | Unit               | Type          |
|----------------|------------------------------------------------------------------------------------------------------------|--------------------|---------------|
| `start_angle`  | _Beginning_ angle of the range of the measurement data. This is also the angle of the _first_ measurement. | degrees            | `float`       |
| `end_angle`    | _Ending_ angle of the range of the measurement data. This is also the angle of the _last_ measurement.     | degrees            | `float`       |
| `speed`        | Speed of the motor.                                                                                        | degrees per second | `int`         |
| `time_stamp`   | Non-universal timestamp. Restarts after 30000.                                                             | milliseconds       | `int`         |
| `confidence_i` | List of the respective _intensity_ of the returning light in a measurement.                                | -                  | `list[int]`   |
| `angle_i`      | List of the respective _angle_ of a measurement.                                                           | degrees            | `list[int]`   |
| `distance_i`   | List of the respective _distance_ measured.                                                                | centimeters        | `list[float]` |

This data continuously gets updated after once calling `listen_to_lidar()`. To stop updating the data dictionary, use
the `stop()` function, which is the second return value.

### Example
```python
import time
from listen_to_lidar import listen_to_lidar

lidar_data, stop = listen_to_lidar()
print(lidar_data['distances']) # prints the dictionary with all the accumulated distance data
time.sleep(1)
print(lidar_data['distances']) # prints the updated distance data

stop()
```
