#!/usr/bin/python
"""
Copyright (c) 2015,
Enrique Fernandez Perdomo
Clearpath Robotics, Inc.
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of Systems, Robotics and Vision Group, University of
      the Balearican Islands nor the names of its contributors may be used to
      endorse or promote products derived from this software without specific
      prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
"""

import rospy
import roslib
import rosbag

import sys
import os.path
import argparse


class MessageInserter:
    """
    Message inserter, which allows to insert a message into a bag file. The
    message is inserted at the given topic and should have a known type. The
    message is inserted at the bag time 0.0 by default, but can be inserted at
    a later time.
    """

    def __init__(self, bag, topic, topic_type, message, time=0.0):
        self._bag = bag
        self._topic = topic
        self._topic_type = topic_type
        self._message = message
        self._time = time

        self._insert_message()

    def _insert_message(self):
        """
        Insert the message on the bag file.
        """
        # Open bag file:
        try:
            bag = rosbag.Bag(self._bag, 'w')
        except IOError as e:
            rospy.logerr('Failed to open bag file %s: %s!' % (self._bag, e.strerror))
            rospy.signal_shutdown('Failed to open bag file %s: %s!' % (self._bag, e.strerror))
            return
        except rosbag.ROSBagException as e:
            rospy.logerr('Failed to read bag file %s: %s!' % (self._bag, e.message))
            rospy.signal_shutdown('Failed to read bag file %s: %s!' % (self._bag, e.message))
            return

        # Convert serialized message into the given type:
        # @todo check how rostopic pub does that!!!
        # line 1356 genpy.message.fill_message_args(...)
        # rostopic/__init__.py

        # Insert message:
        bag.write(self._topic, self._message, self._time)

        # Close bag file:
        bag.close()


if __name__ == "__main__":
    rospy.init_node('insert_msg', anonymous=True)

    parser = argparse.ArgumentParser(
        description='Inserts a message into a given topic and type in the bag '
                    'file after some time (0 by default) from the start.')

    parser.add_argument('bag', help='input/output bag file')
    parser.add_argument('topic', help='input topic')
    parser.add_argument('type', help='input topic type')
    parser.add_argument('message', help='input message')
    parser.add_argument('--start', help='bag time where to insert the message at', default=0.0)

    args = parser.parse_args()

    try:
        MessageInserter(args.bag, args.topic, args.type, args.message, args.start)
    except Exception, e:
        import traceback
        traceback.print_exc()
