#!/usr/bin/env python3
"""
MPFS2 image generator for Microchip Harmony.

Reads all files from ./static/, builds an MPFS2 binary, and writes
../firmware/src/mpfs_img2.c in the same format as the original.

Also generates FileRcrd.bin and DynRcrd.bin index files required by the
Harmony HTTP server for dynamic variable (~token~) substitution.

MPFS2 binary layout:
  [0..3]  "MPFS"
  [4..5]  version 0x02 0x01
  [6..7]  num_files (uint16 LE)
  [8 .. 8+N*2-1]       hash table  (N * uint16 LE, one per file)
  [8+N*2 .. 8+N*2+N*22-1]  FAT records (N * 22 bytes each)
  FAT record (22 bytes):
    string   uint32 LE  absolute offset of NUL-terminated filename
    data     uint32 LE  absolute offset of file data
    len      uint32 LE  byte length of file data
    timestamp uint32 LE  Unix timestamp (from file mtime)
    microtime uint32 LE  always 0
    flags    uint16 LE  0x0002 for files with dynamic vars, else 0x0000
  string section: NUL-terminated filenames, concatenated
  data   section: raw file contents, concatenated

FileRcrd.bin format (inside MPFS2 image):
  4 bytes: number of file records (uint32 LE)
  Per record (10 bytes):
    2 bytes: nameHash (uint16 LE)
    4 bytes: offset into DynRcrd.bin for this file's section (uint32 LE)
    4 bytes: number of dynamic variables in this file (uint32 LE)

DynRcrd.bin format (inside MPFS2 image):
  Per file section:
    6 bytes: header padding (skipped by http.c on first access)
    Per dynamic variable (8 bytes):
      4 bytes: byte offset of opening ~ in file data (uint32 LE)
      4 bytes: callback ID (uint32 LE)
"""

import os
import re
import struct
import sys
from datetime import datetime

STATIC_DIR = os.path.join(os.path.dirname(__file__), 'static')
OUTPUT_C   = os.path.join(os.path.dirname(__file__), '..', 'firmware', 'src', 'mpfs_img2.c')

MPFS2_FLAG_HASINDEX = 0x0002  # file has dynamic variable index (mpfs_local.h)

# Maps token name (inside ~...~) to TCPIP_HTTP_Print callback ID.
# Must match the switch statement in firmware/src/http_print.c.
TOKEN_TO_ID = {
    'inc:header.inc': 0,
    'nextSSID':       1,
    'prevSSID':       2,
    'prevWLAN':       3,
    'nextWLAN':       4,
    'scanresult':     5,
    'scan':           6,
    'gwsettings':     7,
    'gwstatus':       8,
    'gwudp':          9,
    'gwfooter':       10,
}

def name_hash(name: str) -> int:
    """Microchip MPFS2 filename hash (matches mpfs.c MPFS_Open and http.c nameHash)."""
    h = 0
    for c in name:
        h += ord(c)
        h = (h << 1) & 0xFFFF
    return h

def collect_files(static_dir: str):
    """Walk static_dir and return sorted list of (mpfs_name, abs_path)."""
    files = []
    for root, dirs, names in os.walk(static_dir):
        dirs.sort()
        for fname in sorted(names):
            abs_path = os.path.join(root, fname)
            rel      = os.path.relpath(abs_path, static_dir)
            # MPFS2 uses forward slashes; strip leading ./
            mpfs_name = rel.replace(os.sep, '/')
            files.append((mpfs_name, abs_path))
    return files

def find_tokens(data: bytes):
    """Return list of (byte_offset, callback_id) for ~token~ patterns."""
    result = []
    i = 0
    while i < len(data):
        if data[i:i+1] == b'~':
            j = data.find(b'~', i + 1)
            if j != -1:
                try:
                    token_name = data[i+1:j].decode('ascii')
                except UnicodeDecodeError:
                    i += 1
                    continue
                if token_name in TOKEN_TO_ID:
                    result.append((i, TOKEN_TO_ID[token_name]))
                else:
                    print(f"  WARNING: unknown token ~{token_name}~ — no callback ID assigned")
                i = j + 1
            else:
                i += 1
        else:
            i += 1
    return result

def build_index_files(files, file_data_list):
    """Build FileRcrd.bin and DynRcrd.bin byte content from static file data."""
    dyn_files = []  # list of (mpfs_name, [(byte_offset, callback_id)])
    for (mpfs_name, _), data in zip(files, file_data_list):
        tokens = find_tokens(data)
        if tokens:
            dyn_files.append((mpfs_name, tokens))

    # Build DynRcrd.bin: per-file section = 6-byte header + 8 bytes per variable
    dynrcrd = b''
    dynrcrd_offsets = {}
    for mpfs_name, tokens in dyn_files:
        dynrcrd_offsets[mpfs_name] = len(dynrcrd)
        dynrcrd += b'\x00' * 6  # header skipped by http.c (+6 on first seek)
        for byte_offset, callback_id in tokens:
            dynrcrd += struct.pack('<II', byte_offset, callback_id)

    # Build FileRcrd.bin: count + per-file records
    filercrd = struct.pack('<I', len(dyn_files))
    for mpfs_name, tokens in dyn_files:
        h = name_hash(mpfs_name)
        filercrd += struct.pack('<HII', h, dynrcrd_offsets[mpfs_name], len(tokens))

    print(f"\nDynamic variable index:")
    for mpfs_name, tokens in dyn_files:
        print(f"  {mpfs_name}: {len(tokens)} variable(s)")
        for offset, cbid in tokens:
            token_name = next(k for k,v in TOKEN_TO_ID.items() if v == cbid)
            print(f"    offset {offset}: ~{token_name}~ (ID {cbid})")

    return filercrd, dynrcrd

def build_image(files, file_data_list):
    """Build the MPFS2 binary blob."""
    n = len(files)

    timestamps = []
    for _, abs_path in files:
        if abs_path is None:
            timestamps.append(0)
        else:
            timestamps.append(int(os.path.getmtime(abs_path)))

    # Header: 8 bytes
    # Hash table: n * 2 bytes
    # FAT records: n * 22 bytes
    header_size    = 8
    hashtable_size = n * 2
    fat_size       = n * 22
    fat_offset     = header_size + hashtable_size

    string_base = fat_offset + fat_size

    string_data    = b''
    string_offsets = []
    for mpfs_name, _ in files:
        string_offsets.append(string_base + len(string_data))
        string_data += mpfs_name.encode('ascii') + b'\x00'

    data_base = string_base + len(string_data)
    if data_base & 1:
        string_data += b'\x00'
        data_base += 1

    data_offsets = []
    data_section = b''
    for d in file_data_list:
        data_offsets.append(data_base + len(data_section))
        data_section += d
        if len(data_section) & 1:
            data_section += b'\x00'

    hashtable = b''
    for mpfs_name, _ in files:
        hashtable += struct.pack('<H', name_hash(mpfs_name))

    fat = b''
    for i in range(n):
        flags = MPFS2_FLAG_HASINDEX if b'~' in file_data_list[i] else 0x0000
        fat += struct.pack('<IIIIIH',
                          string_offsets[i],
                          data_offsets[i],
                          len(file_data_list[i]),
                          timestamps[i],
                          0,
                          flags)

    header = b'MPFS\x02\x01' + struct.pack('<H', n)
    image  = header + hashtable + fat + string_data + data_section
    return image

def image_to_c(image: bytes, out_path: str):
    """Write the MPFS2 image as a C source file matching the original format."""
    now = datetime.utcnow().strftime('%a %b %d %Y %H:%M:%S')
    lines = []
    lines.append('/***************************************************************')
    lines.append(' * File Name:   mpfs_img2.c')
    lines.append(' * Description: Defines an MPFS2 image to be stored in program memory.')
    lines.append(' * Processor:   PIC32')
    lines.append(' * Compiler:    Microchip C32,XC32')
    lines.append(' *')
    lines.append(' * NOT FOR HAND MODIFICATION')
    lines.append(' * This file is automatically generated by the MPFS2 Utility')
    lines.append(' * ALL MODIFICATIONS WILL BE OVERWRITTEN BY THE MPFS2 GENERATOR')
    lines.append(f' * Generated {now} ')
    lines.append(' *')
    lines.append(' * Software License Agreement')
    lines.append(' *')
    lines.append(' * Copyright (C) 2012 Microchip Technology Inc.  All rights')
    lines.append(' * reserved.')
    lines.append(' *')
    lines.append(' * Microchip licenses to you the right to use, modify, copy, and distribute')
    lines.append(' * software only embedded on a Microchip microcontroller or digital signal ')
    lines.append(' * controller that is integrated into your product or third party product')
    lines.append(' * (pursuant to the sublicense terms in the accompanying license agreement)')
    lines.append('')
    lines.append(' * You should refer to the license agreement accompanying this ')
    lines.append(' * Software for additional information regarding your rights and ')
    lines.append(' * obligations.')
    lines.append(' *')
    lines.append(' * THE SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT')
    lines.append(' * WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT')
    lines.append(' * LIMITATION, ANY WARRANTY OF MERCHANTABILITY, FITNESS FOR A ')
    lines.append(' * PARTICULAR PURPOSE, TITLE AND NON-INFRINGEMENT. IN NO EVENT SHALL')
    lines.append(' * MICROCHIP BE LIABLE FOR ANY INCIDENTAL, SPECIAL, INDIRECT OR')
    lines.append(' * CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF')
    lines.append(' * PROCUREMENT OF SUBSTITUTE GOODS, TECHNOLOGY OR SERVICES, ANY CLAIMS')
    lines.append(' * BY THIRD PARTIES (INCLUDING BUT NOT LIMITED TO ANY DEFENSE ')
    lines.append(' * THEREOF), ANY CLAIMS FOR INDEMNITY OR CONTRIBUTION, OR OTHER ')
    lines.append(' * SIMILAR COSTS, WHETHER ASSERTED ON THE BASIS OF CONTRACT, TORT')
    lines.append(' * (INCLUDING NEGLIGENCE), BREACH OF WARRANTY, OR OTHERWISE.')
    lines.append(' *')
    lines.append(' ***************************************************************/')
    lines.append('#include <stdint.h>')
    lines.append('#include "system_config.h"')
    lines.append('')
    lines.append('/**************************************')
    lines.append(' * MPFS2 Image Data')
    lines.append(' **************************************/ ')
    lines.append('')

    total = len(image)
    data_lines = []
    for i in range(0, total, 16):
        chunk = image[i:i+16]
        is_last_row = (i + 16 >= total)
        hex_vals = []
        for j, b in enumerate(chunk):
            global_idx = i + j
            if global_idx < total - 1:
                hex_vals.append(f'0x{b:02x},')
            else:
                hex_vals.append(f'0x{b:02x}')
        hex_part = ''.join(hex_vals)
        hex_part = hex_part.ljust(16 * 5 - 1)
        ascii_part = ''.join(chr(b) if 0x20 <= b < 0x7f else '.' for b in chunk)
        ascii_part = ascii_part.replace('*/', './')
        ascii_part = ascii_part.ljust(16)
        row = f'\t{hex_part} /* {ascii_part} */'
        if not is_last_row:
            row += ' \\'
        data_lines.append(row)

    lines.append('const uint8_t __attribute__((space(prog),address(DRV_NVM_MEDIA_START_ADDRESS))) '
                 '__attribute__((keep)) NVM_MEDIA_DATA[DRV_NVM_MEDIA_SIZE*1024] = { \\')
    lines.extend(data_lines)
    lines.append('')
    lines.append('};')
    lines.append('')
    lines.append('/**************************************')
    lines.append(' * MPFS2 C linkable symbols')
    lines.append(' **************************************/')
    lines.append('// The entire data array is defined as a single variable to ')
    lines.append('// ensure that the linker does not reorder the data chunks in Flash when compiler ')
    lines.append('// optimizations are turned on.')
    lines.append('struct MEDIA_STORAGE_PARTITION_INFO')
    lines.append('{')
    lines.append('\tconst char*     partName;           // name that identifies the partition')
    lines.append('\t\t\t\t\t\t\t\t\t\t// valid names: "mpfs2", "tcpip_storage";')
    lines.append('\tunsigned long   partStartOffset;    // the absolute starting offset on that media ')
    lines.append('\tunsigned long   partSize;           // size of the partition, in bytes')
    lines.append('} MPFS_IMAGE_PARTITION_INFO = ')
    lines.append('{')
    lines.append('\t"mpfs2",')
    lines.append('\t(unsigned long)NVM_MEDIA_DATA,')
    lines.append('\tsizeof(NVM_MEDIA_DATA),')
    lines.append('};')
    lines.append('')
    lines.append('/**************************************************************')
    lines.append(' * End of MPFS')
    lines.append(' **************************************************************/')

    with open(out_path, 'w') as f:
        f.write('\n'.join(lines) + '\n')

def main():
    files = collect_files(STATIC_DIR)
    print(f"Found {len(files)} files in {STATIC_DIR}:")
    for name, _ in files:
        print(f"  {name}")

    # Read all static file data
    file_data_list = []
    for _, abs_path in files:
        with open(abs_path, 'rb') as f:
            file_data_list.append(f.read())

    # Build dynamic variable index files
    filercrd_data, dynrcrd_data = build_index_files(files, file_data_list)

    # Add FileRcrd.bin and DynRcrd.bin to file list (abs_path=None = generated)
    files.append(('FileRcrd.bin', None))
    file_data_list.append(filercrd_data)
    files.append(('DynRcrd.bin', None))
    file_data_list.append(dynrcrd_data)

    print(f"\nFileRcrd.bin: {len(filercrd_data)} bytes")
    print(f"DynRcrd.bin:  {len(dynrcrd_data)} bytes")

    image = build_image(files, file_data_list)
    print(f"\nGenerated MPFS2 image: {len(image)} bytes ({len(files)} files total)")

    image_to_c(image, OUTPUT_C)
    print(f"Written to {OUTPUT_C}")

if __name__ == '__main__':
    main()
