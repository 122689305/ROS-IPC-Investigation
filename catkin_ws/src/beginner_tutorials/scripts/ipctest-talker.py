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

## Simple talker demo that published std_msgs/Strings messages
## to the 'chatter' topic

import rospy
import sys
from std_msgs.msg import String
import time
import numpy as np

run_time, data_size, comm_rate, queue_size = [int(x) for x in sys.argv[1:]]

def talker():
    start_time_shift = 1
    pub = rospy.Publisher('tcptest', String, queue_size=queue_size)
    rospy.init_node('talker', anonymous=True)
    time.sleep(start_time_shift)
    rate = rospy.Rate(comm_rate) # 10hz
    time_start = time.time()
    sent_cnt = 0
    while time.time() - time_start < run_time and (not rospy.is_shutdown()):
        ds = np.random.normal(loc=data_size, scale=data_size/10)
        clumsy_data = '*'*int(ds)
        heart_beat = "%.32f %s"%(time.time(), clumsy_data)
        pub.publish(heart_beat)
        sent_cnt += len(heart_beat)
        rate.sleep()
    print sent_cnt
    time.sleep(run_time)

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
