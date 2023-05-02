#!/usr/bin/python3

import sys
import re
from pathlib import Path
import fileinput

""" This script is intended to fix the issue with flash strings on ESP866 described """
""" in issue #8 (https://github.com/frankjoshua/rosserial_arduino_lib/issues/8) """
""" It can also be used on the raw output of the rosserial_arduino library creation """
""" routines to add flash strings as appropriate. """
""" Written by Pierce Nichols (rocketgeek@gmail.com) 9/28/29 """

gettype_pattern = re.compile(r'^\s*const char \* getType\(\)\{\s*return\s*(PSTR|)\s*\(\s*"([\w/_]*)"\s*\);\s*\};\s*$')
getmd5_pattern = re.compile(r'^\s*const char \* getMD5\(\)\{\s*return\s*(PSTR|)\s*\(\s*"([0-9a-f]*)"\s*\);\s*\};\s*$')
getprogmem_pattern = re.compile(r'^\s*static const char ([A-Z]*)\[\] PROGMEM = "([\w/_]*)"\s*;\s*$')
code_start = '        const char * '
code_end = '");};'
code_gettype = 'getType() { return '
code_md5 = 'getMD5() { return '
code_norm = ' ("'
code_pstr = ' PSTR("'
pm_start = '    static const char '
pm_progmem = '[] PROGMEM = "'
pm_norm = '[] = "'
pm_end = '";'

def process_header (path_to_header):
    for line in fileinput.input(path_to_header, inplace=True):
        line = line.rstrip('\r\n')
        gt_match = gettype_pattern.search(line)
        md_match = getmd5_pattern.search(line)
        pm_match = getprogmem_pattern.search(line)
        if (gt_match):
            print("    #ifdef ESP8266")
            print(code_start + code_gettype + code_norm + gt_match.group(2) + code_end)
            print("    #else")
            print(code_start + code_gettype + code_pstr + gt_match.group(2) + code_end)
            print("    #endif")
        elif (md_match):
            print("    #ifdef ESP8266")
            print(code_start + code_md5 + code_norm + md_match.group(2) + code_end)
            print("    #else")
            print(code_start + code_md5 + code_pstr + md_match.group(2) + code_end)
            print("    #endif")
        elif (pm_match):
            print("#ifdef ESP8266")
            print(pm_start + pm_match.group(1) + pm_norm + pm_match.group(2) + pm_end)
            print("#else")
            print(pm_start + pm_match.group(1) + pm_progmem + pm_match.group(2) + pm_end)
            print("#endif")
        else:
            print(line)

rootpath = sys.argv[1]                  # First argument is the root of the directory tree to fix
p = Path(rootpath)                      # Turn it into a path
header_list = list(p.glob('**/*.h'))    # Grab a list of all the header files in this directory tree
for header in header_list:
    process_header(header)
