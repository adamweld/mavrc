'''
test mavlink messages
'''

from __future__ import print_function

import time

from pymavlink import mavutil 

from argparse import ArgumentParser
parser = ArgumentParser(description=__doc__)

parser.add_argument("--baudrate", type=int,
                          help="master port baud rate", default=115200)
parser.add_argument("-d", "--device", required=True, help="serial device")
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

# wait for the heartbeat msg to find the system ID
wait_heartbeat(conn)


def force_pwm(channel_id, pwm=1500):
    rc_channel_values = [65535 for _ in range(18)]
    rc_channel_values[channel_id - 1] = pwm
    conn.mav.rc_channels_override_send(
        conn.target_system,                # target_system
        conn.target_component,             # target_component
        *rc_channel_values)                  # RC channel list, in microseconds.


while True:
    force_pwm(1,100)
    time.sleep(2)
    force_pwm(1,1900)
    time.sleep(2)
'''
while True:
    msg =  conn.recv_match(type = 'RC_CHANNELS_SCALED', blocking = True)
    #msg = msg.to_dict()
    print(msg.load)
    

# print a status list
print('System Status Report')
#status = conn.messages['SYS_STATUS']
#print(status)
#print(f'cpu load: {conn.messages['SYS_STATUS'].status.load}')


print(conn.messages)

# Check how many channels are available on RC
try:
    chan = conn.messages['RC_CHANNELS'].chancount
    print(chan)
except:
    print('failure to receive channel count')

# Attempt to receive chan1 value
try:
    rc = conn.messages['RC_CHANNELS_SCALED'].chan1_scaled
    print(rc)
except:
    print('No channel 1 value received')

# Print IMU data
try:
    imu = conn.messages['ATTITUDE']
    print(f'Roll: {imu.roll}    Pitch: {imu.pitch}  Yaw: {imu.yaw}')
except:
    print('No ATTITUDE message received')

'''
