#!/usr/bin/python
"""

@copyright:
     Copyright (C) Mellanox Technologies Ltd. 2001-2020.  ALL RIGHTS RESERVED.

    This software product is a proprietary product of Mellanox Technologies
    Ltd. (the "Company") and all right, title, and interest in and to the
    software product, including all associated intellectual property rights,
    are and shall remain exclusively with the Company.

    This software product is governed by the End User License Agreement
    provided with the software product.

@date:
    07 Oct 2020

@author:
    Mykola Kostenok
"""
#############################
# Global imports
#############################
import sys
import os
import inotify.adapters

if __name__ == '__main__':
    if len(sys.argv) != 2:
        print "Invalid argument nums. Pass event filepath as argument."
        sys.exit(0)
    FILEPATH = sys.argv[1]
    INOTIFY = inotify.adapters.Inotify()
    if not INOTIFY:
        print "inotify_adapter error"
        sys.exit(0)
    if not os.path.exists(FILEPATH):
        print "no file {}".format(FILEPATH)
        sys.exit(0)
    INOTIFY.add_watch(FILEPATH, mask=inotify.constants.IN_CLOSE_WRITE)
    for event in INOTIFY.event_gen(yield_nones=False):
        (_, type_names, path, filename) = event
        handler_file = open(FILEPATH)
        if not handler_file:
            print "no file {}".format(FILEPATH)
            sys.exit(0)
        val = handler_file.read(1)
        handler_file.close()
        print "{} {}".format(os.path.basename(FILEPATH), val)
        # Do some event based action. For lc{n}_verifiled:
        # Validation line card type, max power consumption, CPLD version, VPD, INI blob.
        # Validate VPD /var/run/hw-management/lc1/eeprom/vpd.
        # Validate INI /var/run/hw-management/lc1/eeprom/ini.
        # Check /var/run/hw-management/lc1/system/max_power is enough power.
        # Continue init flow - power on line card.

    sys.exit(0)
