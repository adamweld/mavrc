'''
test mavlink messages
'''
from __future__ import print_function

from pymavlink import mavutil 

import time

from argparse import ArgumentParser
parser = ArgumentParser(description=__doc__)

parser.add_argument("--baudrate", type=int,
                          help="master port baud rate", default=115200)
parser.add_argument("--device", required=True, help="serial device")
parser.add_argument("--source-system", dest='SOURCE_SYSTEM', type=int,
                          default=255, help='MAVLink source system for this GCS')
args = parser.parse_args()

def wait_heartbeat(m):
    '''wait for a heartbeat so we know the target system IDs'''
    print("Waiting for APM heartbeat")
    msg = m.recv_match(type='HEARTBEAT', blocking=True)
    print("Heartbeat from APM (system %u component %u)" % (m.target_system, m.target_system))

# create a mavlink serial instance
conn = mavutil.mavlink_connection(args.device, baud=args.baudrate, source_system=args.SOURCE_SYSTEM)

mavutil.set_dialect('common')

# wait for the heartbeat msg to find the system ID
wait_heartbeat(conn)

# print a status list
print('Dump received MAVLINK packets')

while True:
    try:
        print(conn.recv_match().to_dict())
    except:
        pass
    time.sleep(0.1)


