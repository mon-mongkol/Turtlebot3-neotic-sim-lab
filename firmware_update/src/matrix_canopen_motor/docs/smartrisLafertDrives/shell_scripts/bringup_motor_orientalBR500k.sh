#!/bin/sh
# sudo ip link set can0 up type can bitrate 1000000
sudo ip link set can0 down
sleep 5.0
sudo ip link set can0 up type can bitrate 500000
#sudo ip link set can2 up type can bitrate 1000000
sleep 2.0

cansend can0 000#8100

sleep 5.0

# Set Mode of Operation
#cansend can0 601#2F60600003000000
#cansend can0 602#2F60600003000000
#sleep 0.5

# Go to the State “Switched-On”
# Change state in machine state of CANOpen Profile DS402 in “Switched On”. MASTER must send twice time the SDO “control word” object, index 0x6040, with value 6 and with value 7
cansend can0 601#2B40600006000000
cansend can0 602#2B40600006000000
sleep 0.5

cansend can0 601#2B40600007000000
cansend can0 602#2B40600007000000
sleep 0.5

# Set Acceleration 1000 rpm/s
#cansend can0 601#23836000E8030000
#cansend can0 602#23836000E8030000
#sleep 0.5

# Deceleration 1000 rpm/s
#cansend can0 601#23846000E8030000
#cansend can0 602#23846000E8030000
###sleep 0.5

# Go to the State “Operation Enabled”.
cansend can0 601#2B4060000F000000
cansend can0 602#2B4060000F000000
sleep 0.5

cansend can0 000#0100

# Set Point Velocity 

echo Done
