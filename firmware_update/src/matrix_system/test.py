#!/usr/bin/env python3

import nmcli
# addrs = psutil.net_if_addrs()
# a = addrs.keys()

# print(type(a))

# for key in a:
#     print(key)
#     if key == "can"


devs = nmcli.device()
for a in dev:
    if a.device == "can0":
        print(a.device)

import os
os.system('echo \"a\" | sudo -S ip link set can0 up type can bitrate 250000')
    