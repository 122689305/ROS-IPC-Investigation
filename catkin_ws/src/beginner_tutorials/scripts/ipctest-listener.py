#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
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
#  * Neither the name of Willow Garage, Inc. nor the names of its
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
# Revision $Id$

## Simple talker demo that listens to std_msgs/Strings published
## to the 'chatter' topic

import rospy
import sys
from std_msgs.msg import String
import time

run_time, data_size, comm_rate, queue_size = [int(x) for x in sys.argv[1:]]
recv_cnt = []
last_recv_time = 0
start_time = 0
total_data_size = 0

def callback(data):
    global recv_cnt, last_recv_time, start_time, total_data_size
    recv_time = time.time()
    elapsed = recv_time - start_time
    print("------callback-----")
    if elapsed <= run_time:
        interval = recv_time - last_recv_time
        print(interval, elapsed, "recvd")
        last_recv_time = recv_time
        datasize = len(data.data)
        total_data_size += datasize
        recv_cnt.append((interval, datasize))
    else:
        print(elapsed, "expired")
    print("=======callback=======")

def cback(data):
    global recv_cnt, last_recv_time, start_time, total_data_size
    timestamp, clumsy_data = data.data.split()
    timestamp = float(timestamp)
    recv_time = time.time()
    delay = recv_time - timestamp
    interval = recv_time - last_recv_time
    last_recv_time = recv_time
    recv_cnt.append((interval, delay, len(clumsy_data)))

    elapsed = recv_time - start_time
    if elapsed <= run_time or start_time == 0:
        total_data_size += len(data.data)

def listener():
    global last_recv_time, start_time
    start_time_shift = 1

    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber('tcptest', String, cback)
    time.sleep(start_time_shift)
    start_time = last_recv_time = time.time()

    time.sleep(run_time*2)
    #print(sum(x[1] for x in recv_cnt))
    print(total_data_size)
    for interval, delay, datasize in recv_cnt:
        print("%.32f %.32f %d"%(interval, delay, datasize))

if __name__ == '__main__':
    listener()
