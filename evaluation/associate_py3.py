#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2013, Juergen Sturm, TUM
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of TUM nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#

### Edit by Yuan Lin for Python 3 use
### March 25, 2025

"""
This script reads two timestamped data files (e.g. RGB and depth images from a Kinect sensor),
and associates the entries based on closest matching timestamps within a tolerance threshold.
"""

import argparse

def read_file_list(filename, remove_bounds=False):
    """
    Reads a trajectory from a text file.
    
    File format:
    Each line: "timestamp data1 data2 ..."
    
    Returns:
    dict mapping timestamp (float) to a list of strings (data)
    """
    with open(filename) as f:
        data = f.read()

    lines = data.replace(",", " ").replace("\t", " ").split("\n")
    if remove_bounds:
        lines = lines[100:-100]
    entries = [
        [v.strip() for v in line.split(" ") if v.strip() != ""]
        for line in lines
        if len(line) > 0 and line[0] != "#"
    ]
    entries = [(float(e[0]), e[1:]) for e in entries if len(e) > 1]
    return dict(entries)

def associate(first_list, second_list, offset, max_difference):
    """
    Finds the best matches between two timestamped dictionaries.
    
    Args:
        first_list: dict of {timestamp: data}
        second_list: dict of {timestamp: data}
        offset: time offset to apply to second_list timestamps
        max_difference: maximum allowed timestamp difference

    Returns:
        List of tuples: (timestamp1, timestamp2)
    """
    first_keys = list(first_list.keys())
    second_keys = list(second_list.keys())

    potential_matches = [
        (abs(a - (b + offset)), a, b)
        for a in first_keys
        for b in second_keys
        if abs(a - (b + offset)) < max_difference
    ]
    potential_matches.sort()

    matches = []
    used_first = set()
    used_second = set()

    for diff, a, b in potential_matches:
        if a not in used_first and b not in used_second:
            used_first.add(a)
            used_second.add(b)
            matches.append((a, b))

    matches.sort()
    return matches

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Associate two timestamped data files.')
    parser.add_argument('first_file', help='first text file (format: timestamp data)')
    parser.add_argument('second_file', help='second text file (format: timestamp data)')
    parser.add_argument('--first_only', action='store_true',
                        help='only output associated lines from first file')
    parser.add_argument('--offset', type=float, default=0.0,
                        help='time offset added to second file timestamps (default: 0.0)')
    parser.add_argument('--max_difference', type=float, default=0.02,
                        help='max allowed time difference between matched entries (default: 0.02)')
    parser.add_argument('--remove_bounds', action='store_true',
                        help='remove the first and last 100 lines (optional for KITTI, etc.)')

    args = parser.parse_args()

    first_list = read_file_list(args.first_file, args.remove_bounds)
    second_list = read_file_list(args.second_file, args.remove_bounds)

    matches = associate(first_list, second_list, args.offset, args.max_difference)

    if args.first_only:
        for a, b in matches:
            print(f"{a:.6f} {' '.join(first_list[a])}")
    else:
        for a, b in matches:
            print(f"{a:.6f} {' '.join(first_list[a])} {b - args.offset:.6f} {' '.join(second_list[b])}")
