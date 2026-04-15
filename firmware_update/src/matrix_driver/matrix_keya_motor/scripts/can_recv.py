#!/usr/bin/env python3
#
# The "real" Motor node, transmitting the MotorStatus message
# periodically.
#

from struct import *
import can
# import int
# def create_message(speed, load):
#     return can.Message(arbitration_id=0x010,
#                        extended_id=False,
#                        data=struct.pack('<HB', speed, load))


def main():
    can.rc['interface'] = 'socketcan'
    can.rc['channel'] = 'can0'
    can_bus = can.interface.Bus()

    # task = can_bus.send_periodic(create_message(0, 0), 1.0)
    m1read = False
    m2read = False
    m1 = 0
    m2 = 0
    while True:
        
        message = can_bus.recv()
        #  print(message)
        if message.arbitration_id == 0x581:
            
            if((message.data[0] == 0x4b) and (message.data[1] == 0x03) and (message.data[2] == 0x21) and (message.data[3] == 0x01)):
                m1 = bytearray([message.data[4], message.data[5]])
                m1 = unpack('<h', m1)
                m1 = m1[0] / 10.0000
                m1read = True
                
            if((message.data[0] == 0x4b) and (message.data[1] == 0x03) and (message.data[2] == 0x21) and (message.data[3] == 0x02)):
                m2 = bytearray([message.data[4], message.data[5]])
                m2 = unpack('<h', m2)
                m2 = m2[0] / 10.0000
                m2read = True
        else:
            pass        
        
        if m1read and m2read:
            m1read = False
            m2read = False
            print(m1, m2)
            m1 = 0
            m2 = 0
        else:
            pass

    print('Done!')


if __name__ == '__main__':
    main()