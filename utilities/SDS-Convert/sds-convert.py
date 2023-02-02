# Copyright (c) 2023 Arm Limited. All rights reserved.
#
# SPDX-License-Identifier: Apache-2.0
#
# Licensed under the Apache License, Version 2.0 (the License); you may
# not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an AS IS BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# Python SDS Data Converter

import argparse
import csv
import sys
from struct import calcsize, unpack

import yaml


class RecordManager:
    def __init__(self):
        self.HEADER_SIZE    = 8
        self.TIMESTAMP_SIZE = 4
        self.data_buff      = bytearray()
        self.timestamp      = []
        self.data_size      = []

    # Flush data buffer
    def __flush(self):
        self.data_buff      = bytearray()
        self.timestamp      = []
        self.data_size      = []

    # Private function for retrieving data from record
    def __getRecord(self, file):
        record = bytearray(file.read(self.HEADER_SIZE))
        if len(record) == self.HEADER_SIZE:
            self.timestamp.append(unpack("I", record[:self.TIMESTAMP_SIZE])[0])
            self.data_size.append(unpack("I", record[self.TIMESTAMP_SIZE:])[0])
            self.data_buff.extend(bytearray(file.read(self.data_size[-1])))

    # Extract all data from .sds recording and return a dictionary
    # Dictionary consists of: timestamp, data_size, raw_data
    def getData(self, file):
        record_num = 0

        while True:
            self.__getRecord(file)
            if len(self.timestamp) == record_num:
                break
            else:
                record_num += 1

        data = {"timestamp" : self.timestamp, \
                "data_size" : self.data_size, \
                "raw_data" : self.data_buff}

        self.__flush()

        return data


# Convert C style data type to Python style
def getDataType(data_type):
    if   data_type == "int16_t":
        d_type = "h"
    elif data_type == "uint16_t":
        d_type = "H"
    elif data_type == "int32_t":
        d_type = "i"
    elif data_type == "uint32_t":
        d_type = "I"
    elif data_type == "float":
        d_type = "f"
    elif data_type == "double":
        d_type = "d"
    else:
        print(f"Unknown data type: {data_type}\n")
        d_type = "I"

    return d_type

# Select correct Qeexo sensor data name
def qeexoColumnName(sensor):
    sensor_name = sensor.strip('./\\').split('.')[0]
    if   sensor_name == 'Accelerometer':
        qeexo_name = 'accel'
    elif sensor_name == 'Gyroscope':
        qeexo_name = 'gyro'
    elif sensor_name == 'Magnometer':
        qeexo_name = 'magno'
    elif sensor_name == 'Temperature':
        qeexo_name = 'temperature'
    elif sensor_name == 'Humidity':
        qeexo_name = 'humidity'
    elif sensor_name == 'Pressure':
        qeexo_name = 'pressure'
    elif sensor_name == 'Microphone':
        qeexo_name = 'microphone'
    elif sensor_name == 'Analog microphone':
        qeexo_name = 'microphone_analog'
    elif sensor_name == 'Light':
        qeexo_name = 'light'
    elif sensor_name == 'Ambient light':
        qeexo_name = 'ambient'
    elif sensor_name == 'RCDA':
        qeexo_name = 'rcda'
    elif sensor_name == 'ETOH':
        qeexo_name = 'etoh'
    elif sensor_name == 'TVOC':
        qeexo_name = 'tvoc'
    elif sensor_name == 'IAQ':
        qeexo_name = 'iaq'
    elif sensor_name == 'ECO2':
        qeexo_name = 'eco2'
    elif sensor_name == 'RMOX':
        qeexo_name = 'rmox'
    elif sensor_name == 'Low power accelerometer':
        qeexo_name = 'accel_lowpower'
    elif sensor_name == 'High sensitivity accelerometer':
        qeexo_name = 'accel_highsensitive'
    else:
        qeexo_name = sensor_name.lower()

    return qeexo_name

# Open CSV file and write header
def createCSV(filename, columns):
    global csv_file, writer

    try:
        csv_file = open(filename, "w", newline='')
        writer = csv.writer(csv_file)
    except Exception as e:
        sys.exit(f"Error: {e}")

    csv_header = ['timestamp']
    for sensor in columns:
        sensor_name = qeexoColumnName(sensor)
        csv_header.append(sensor_name)
    csv_header.append('label')

    writer.writerow(csv_header)

# Write data to CSV file
def writeCSV(interval, data, meta_data, data_label):
    # Set sensor position counters to 0 and extract base timestamps for each sensor
    cnt = {}
    cnt_old = {}
    timestamp_base = []
    for sensor in data:
        cnt_old[sensor] = 0
        cnt[sensor] = 0
        timestamp_base.append(data[sensor]["timestamp"][0])

    # Select timestamp with lowest value and round to first next interval
    csv_timestamp = ((min(timestamp_base)//interval) * interval) + interval

    while True:
        # Create a list of lists, based on the number of sensors
        csv_row = [[] for i in range(0, len(data.keys()))]

        for sensor in data:
            timestamp = data[sensor]["timestamp"]
            data_size = data[sensor]["data_size"]

            # Check if end of file is reached. If it is, skip current sensor
            # Otherwise store current position counter in cnt_old
            if cnt[sensor] < len(timestamp):
                cnt_old[sensor] = cnt[sensor]
            else:
                continue

            # Increment position counter of this sensor if elapsed time in record
            # is less than next CSV timestamp
            while timestamp[cnt[sensor]] < csv_timestamp:
                cnt[sensor] += 1
                # Break while loop if end of file is reached
                if cnt[sensor] == len(timestamp):
                    break

            # Calculate number of data bytes for current timestamp
            n_bytes = sum(data_size[cnt_old[sensor]:cnt[sensor]])

            # Flush used bytes from buffer
            raw_data = data[sensor]["raw_data"][:n_bytes]
            data[sensor]["raw_data"] = data[sensor]["raw_data"][n_bytes:]

            sensor_data = {}
            desc_n = 0
            desc_n_max = len(meta_data[sensor])
            for channel in meta_data[sensor]:
                tmp_data = []
                # Extract channel data type information from YAML file
                d_type = getDataType(channel["type"])
                # Calculate number of bytes needed for decoding the data in .sds file
                d_byte = calcsize(d_type)
                # Disunite raw data into a list of data points according to the number
                # of bytes needed for each data point
                tmp_data = [raw_data[i:(i + d_byte)] for i in range(0, len(raw_data), d_byte)]
                # Keep only every n-th data point
                tmp_data = tmp_data[desc_n::desc_n_max]
                # Decode retrieved data points
                tmp_data = list(unpack(f"{int(len(tmp_data))}{d_type}", b''.join(tmp_data)))
                # Scale and offset data points
                if "scale" in channel:
                    scale = channel["scale"]
                else:
                    scale = 1
                if "offset" in channel:
                    offset = channel["offset"]
                else:
                    offset = 0
                scaled_data = [((x * scale) + offset) for x in tmp_data]
                # Store decoded data in a dictionary
                sensor_data[desc_n] = scaled_data
                # Increment channel description number
                desc_n += 1

            # Group every n-th element of each channel into a list
            csv_data = []
            if len(sensor_data) > 1:
                for i in range(0, len(sensor_data[0])):
                    tmp_data = []
                    for channel in sensor_data:
                        tmp_data.append(sensor_data[channel][i])
                    csv_data.append(tmp_data)
            elif len(sensor_data) == 1:
                csv_data = sensor_data[0]

            # Insert sensor data for this CSV timestamp
            csv_row[sensor] = csv_data

        # Write current row into CSV file and increment CSV timestamp by one interval.
        # If there is no data present in this row, exit while loop without writing to the file.
        if csv_row != [[] for i in range(0, len(data.keys()))]:
            writer.writerow([csv_timestamp] + csv_row + [data_label])
            csv_timestamp += interval
        else:
            break

    csv_file.close()

# Main function
def main():
    parser = argparse.ArgumentParser(description="Convert SDS data to selected format")

    required = parser.add_argument_group("required")
    required.add_argument("-y", "--yaml", help="YAML sensor description file", nargs="+", required=True)
    required.add_argument("-s", "--sds", help="SDS data recording file", nargs="+", required=True)
    required.add_argument("-o", "--out", help="Output file", required=True)
    required.add_argument("-f", "--format", help="Output data format", choices=["qeexo_v2_csv"], required=True)

    optional = parser.add_argument_group("optional")
    optional.add_argument("--label", help="Qeexo class label for sensor data", default='')
    optional.add_argument("--interval", help="Qeexo timestamp interval in ms", type=int, default=50)

    args = parser.parse_args()

    # Check if interval is zero
    if args.interval == 0:
        sys.exit(f"Invalid interval option: {args.interval} ms")

    # Load data from .yml file
    meta_data = {}
    i = 0
    for filename in args.yaml:
        try:
            file = open(filename, "r")
            meta_data[i] = yaml.load(file, Loader=yaml.FullLoader)["sds"]["content"]
            file.close()
        except Exception as e:
            sys.exit(f"Error: {e}")
        i += 1

    # Load data from .sds file
    Record = RecordManager()
    data = {}
    i = 0
    for filename in args.sds:
        try:
            file = open(filename, "rb")
            data[i] = Record.getData(file)
            file.close()
        except Exception as e:
            sys.exit(f"Error: {e}")
        i += 1

    # CSV
    createCSV(args.out, args.sds)
    writeCSV(args.interval, data, meta_data, args.label)


if __name__ == "__main__":
    main()
