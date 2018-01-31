#!/usr/bin/python

'''
 Copyright © 2016-2018 The Things Products
 Use of this source code is governed by the MIT license that can be found in
 the LICENSE file.
'''

import sys
import binascii
import struct
import json

if len(sys.argv) != 2:
    print "call with hex string to parse"
    exit(-1)


hex_string = sys.argv[1].strip()


lookupWarnings = [
    "Invalid frequency plan",
    "HTTP communication failure activation",
    "HTTP communication failure configuration",
    "HTTP communication failure frequency plan",
    "HTTP communication failure firmware",
    "MQTT communication failure",
    "Firmware storage failure",
    "LoRa module configuration load failure",
    "Network change",
    "LoRa idle for one day",
    "LoRa idle for one week",
    "LoRa TX too late",
    ]

def bits(a):
    i = 0
    while a > 0:
        if a & 1:
            yield i
        a = a >> 1
        i = i + 1


loopupApplicationErrors = [
    "Illegal state change",
    "TCP init failure",
    "Serial flash failure",
    ]
  
def parsePowercycle(payload):
    reboot_reason, = struct.unpack("<B", payload[0:1])
    return {"power_cycle": {"reboot_reason": hex(reboot_reason)}}

def parseGeneralException(payload):
    code, epc = struct.unpack("<LL", payload)
    return {"general_exception": {"code": code, "epc": hex(epc)}}

def parseInvalidPointer(payload):
    epc, nested_epc = struct.unpack("<LL", payload)
    return {"invalid_pointer": {"epc": hex(epc), "nested_epc": hex(nested_epc)}}

def parseAssert(payload):
    line, = struct.unpack("<H", payload[6:8])
    return {"assert": {"file": payload[0:6], "line": line}}
    
def parseApplicationError(payload):
    id, = struct.unpack("<L", payload[0:4])
    return {"application_error": {"id": id, "description": loopupApplicationErrors[id]}}

def parseWarning(payload):
    mask, = struct.unpack(">L", payload[0:4])
    return {"warning": {"mask": hex(mask), "descriptions": [lookupWarnings[b] for b in bits(mask)]}}
    

parseFunctions = {
    1: parsePowercycle,
    2: parseGeneralException,
    3: parseInvalidPointer,
    4: parseAssert,
    5: parseApplicationError,
    6: parseWarning
    }

try:
    binary_string = binascii.unhexlify(hex_string)
    sys.stdout.write(json.dumps(parseFunctions[int(hex_string[0:2],16)](binary_string[1:])))
except:
    sys.stdout.write("unable to parse: {}".format(hex_string))

