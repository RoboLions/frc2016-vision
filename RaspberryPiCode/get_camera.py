from __future__ import print_function
import re
import os
import sys

def main(id):
    dir = "/sys/devices/platform/soc/3f980000.usb/usb1/1-1/1-1.{0}/1-1.{0}:1.0/video4linux/".format(id)
    match = re.search
    try:
        for file in os.listdir(dir):
            try:
                match = re.match(r'video(\d+)', file)
                if match:
                    return int(match.group(1))
            except ValueError:
                pass
    except OSError:
        pass
    return None

if __name__ == "__main__":
    try:
        id = main(int(sys.argv[1]))
        if id == None:
            sys.exit(1)
        print(id)
    except (IndexError, ValueError):
        sys.exit(2)
