from contextlib import closing
from more_itertools import time_limited
from ouster import client, pcap
from datetime import datetime
import sys

# connect to sensor and record lidar/imu packets
SENSOR_IP = '169.254.77.123'
LIDAR_PORT = 7502
IMU_PORT = 7503

with closing(client.Sensor(SENSOR_IP, LIDAR_PORT, IMU_PORT, buf_size=640, timeout=10)) as source:

    # make a descriptive filename for metadata/pcap files
    time_part = datetime.now().strftime("%Y%m%d_%H%M%S")
    meta = source.metadata

    fname_base = f"{meta.prod_line}_{meta.sn}_{meta.mode}_{time_part}"
    print(f"Saving sensor metadata to: {fname_base}.json")

    folder_name = sys.argv[1]
    print(folder_name)

    source.write_metadata(f"{folder_name}/{fname_base}.json")
    print(f"Writing to: {folder_name}/{fname_base}.pcap (Ctrl-C to stop early)")

    n_packets = pcap.record(source, f"{folder_name}/{fname_base}.pcap")

    print(f"Captured {n_packets} packets")
