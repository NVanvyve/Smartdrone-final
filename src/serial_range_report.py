#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function

import ConfigParser
import os
import termios

ENCODING = "UTF-8"
BAUDRATE = termios.B115200
NUM_ANCHORS = 3


class RangeReport:
    """
    Ranging report messages sent over the USB port.
    Format source : TREK1000 User Manual, page 42, Range Report Format.

        mid                 The “mr” message consists of tag to anchor raw ranges, “mc” tag to anchor range bias
                            corrected ranges – used for tag location and “ma” anchor to anchor range bias corrected
                            ranges – used for anchor auto-positioning.

        mask                Mask indicates which units are detected
        unit_and_ranges     List of tuples (unit, range). Range is in millimeters.
        units
        nranges
        nseq                An increasing sequence number
        rangetime           time of last range reported
        tid                 Identifying the tag
        aid                 Identifying the anchor
    """

    def __init__(self, report):
        splitted = report.split(" ")
        assert len(splitted) == 10
        self.mid = splitted[0]

        self.mask = int(splitted[1], 16)
        self.unit_and_ranges = []
        self.units = []
        for k in range(0, 4):
            if (0x1 << k) & self.mask:
                unit_range = int(splitted[2 + k], 16)
                self.unit_and_ranges.append((k, unit_range))
                self.units.append(k)
        self.nranges = int(splitted[6], 16)
        self.rseq = int(splitted[7], 16)
        self.rangetime = int(splitted[8], 16)
        self.tid = int(splitted[9][1])
        self.aid = int(splitted[9][3])

    def __str__(self):
        return "MID:" + self.mid + ", MASK:" + str(self.mask) + ", RANGES:" + str(
            self.unit_and_ranges) + ", NRANGES:" + str(self.nranges) + ", RSEQ:" + str(
            self.rseq) + ", RANGETIME:" + str(
            self.rangetime) + ", Tag ID:" + str(self.tid) + ", Anchor ID:" + str(self.aid)


def read_config_file(file_path):
    config = ConfigParser.ConfigParser()
    config.read(file_path)
    port = str(config.get("tag", "Port"))
    anchorArray = []
    for i in range(0, NUM_ANCHORS):
        values = [float(i) for i in config.get("anchors", str(i)).split(",")]
        anchorArray.append(values)
    assert (len(anchorArray) == NUM_ANCHORS)
    return port, anchorArray

