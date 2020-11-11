'''
simple proof of concept program to listen for switch commands and "start/stop" collection
'''
import time

from pymavlink import mavutil

from argparse import ArgumentParser
parser = ArgumentParser(description=__doc__)

parser.add_argument("-b", "--baudrate", type=int,
                          help="master port baud rate", default=115200)
'''
parser.add_argument("--channel", type=string,
                          help="RC channel to read, e.g. chan1_raw", default='chan8_raw')
                          '''
parser.add_argument("-d", "--device", required=True, help="serial device, e.g. /dev/ttyS3")
parser.add_argument("--source-system", dest='SOURCE_SYSTEM', type=int,
                          default=255, help='MAVLink source system for this GCS')
args = parser.parse_args()

f = open("mavrc_log.txt", "w")


# function to print and write to log at the same time
def pl(message):
    print(message)
    f.write(message)

def wait_heartbeat(m):
    '''wait for a heartbeat so we know the target system IDs'''
    print("Waiting for PX4 heartbeat")
    msg = m.recv_match(type='HEARTBEAT', blocking=True)
    print("Heartbeat from PX4 (system %u component %u)" % (m.target_system, m.target_system))


# create a mavlink serial instance
try:
    conn = mavutil.mavlink_connection(args.device, baud=args.baudrate, source_system=args.SOURCE_SYSTEM)
except:
    print('Connection Failed')
    exit()

# wait for the heartbeat msg to find the system ID
wait_heartbeat(conn)

while True:
    try:
        msg =  conn.recv_match(type = 'RC_CHANNELS', blocking = True)
        print('Chan1:    {}'.format(msg.chan1_raw))
        time.sleep(.1)
    except:
        f.close()
        print('exception, closing')
        break


'''
while True:
    #print(msg.args.channel)
    print(msg.chan1_raw)
'''
