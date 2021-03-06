import ach
from ctypes import Structure,c_uint16,c_double,c_ubyte,c_uint32,c_int16
# import ach
# import sys


CRITR_SERVER_IP = '104.131.172.175'
CRITR_SERVER_PORT = '1234'
CRITR_JOINT_COUNT = 20
CRITR_DC_COUNT = 2
CRITR_CHAN_REF_NAME = 'critr-ref'
CRITR_LOOP_PERIOD  = 0.005

#struct with joint references
class CRITR_REF(Structure):
    _pack_ = 1
    _fields_ = [("ref",    c_int16*CRITR_JOINT_COUNT),
                ("dcref",    c_int16*CRITR_DC_COUNT)]
