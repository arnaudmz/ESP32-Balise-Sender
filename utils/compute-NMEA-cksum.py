#!/usr/bin/env python

import sys
cksum = 0
for c in sys.argv[1]:
    if c != '$':
      cksum ^= ord(c)
    sys.stdout.write(c)
sys.stdout.write(f'*{cksum:02X}\n')

# L96
# ./compute-NMEA-cksum.py '$PQ1PPS,W,4,140' => Enable PPS, 140ms
# ./compute-NMEA-cksum.py '$PQBAUD,W,115200' => switch permanently do 115200 bauds
# CASIC (AT6558) => https://m5stack.oss-cn-shenzhen.aliyuncs.com/resource/docs/datasheet/unit/Multimode_satellite_navigation_receiver_cn.pdf
# ./compute-NMEA-cksum.py '$PCAS03,1,0,0,0,1,0,0,0' => Enable GGA and RMC only
# ./compute-NMEA-cksum.py '$PCAS04,7'               => Enable GPS + BDS + GLONASS
# ./compute-NMEA-cksum.py '$PCAS01,5'               => Switch to baudrate 115200
# ./compute-NMEA-cksum.py '$PCAS00'                 => Save settings to flash
