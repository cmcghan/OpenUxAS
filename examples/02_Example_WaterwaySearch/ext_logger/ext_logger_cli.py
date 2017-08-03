#! /usr/bin/env python
# Copyright 2017 Government of the United States of America, as represented by the Secretary of the Air Force.
# No copyright is claimed in the United States under Title 17, U. S. Code. All Other Rights Reserved.
# Copyright 2017 University of Cincinnati. All rights reserved. See LICENSE.md file at:
# https://github.com/afrl-rq/OpenUxAS
# Additional copyright may be held by others, as reflected in the commit history.
"""
Large portions of this code were copied from the python example in:
OpenUxAS/examples/02_Example_WaterwaySearch/ext_logger/ext_logger.py

This code is meant to act like the UxAS-equivalent of 'rostopic echo' for ROS.
"""
import zmq, json, string, sys
sys.path.insert(0, '../../../src/LMCP/py')
from lmcp import LMCPFactory

if __name__ == '__main__':

    #
    # get commandline arguments and select parameters for run
    #
    if (len(sys.argv) < 2): # need name of script and at least one argument
        print("ERROR: no commandline argument given.")
        print("Acceptable arguments are (any topic being published in the UxAS ecosystem)" % okargs.keys())
        sys.exit(0)
    
    # get the parameters associated with the arguments
    full_lmcp_type_namestr = sys.argv[1]

    # prepare LMCP factory
    factory = LMCPFactory.LMCPFactory();

    # Socket to talk to server
    context = zmq.Context()
    socket_sub = context.socket(zmq.SUB)
    socket_sub.connect("tcp://127.0.0.1:5560")

    # subscribe to all possible messages
    # duplicate of UxAS config file, but necessary
    for c in string.ascii_letters:
        socket_sub.setsockopt(zmq.SUBSCRIBE, c)

    socket_send = context.socket(zmq.PUSH)
    socket_send.connect("tcp://127.0.0.1:5561")

    # main loop: receive a message, then process it
    while True:
        print("--------------------------------------------")
        
        data = socket_sub.recv()
        # messages are single part with a header followed by
        # serialized LMCP
        # header format: [address]$[format]|[type]|[group]|[entity]|[service]$
        address, attributes, msg = data.split('$', 2)
        msg_format, msg_type, msg_group, entityid, serviceid = attributes.split('|',4)
        obj = factory.getObject(msg)
        
        # sending as entityid{0} and serviceid{0}, so check for loopback
        # CreateServiceMessage currently has xml parsing problems, so remove
        if (int(entityid) == 0 and int(serviceid) == 0) or obj.FULL_LMCP_TYPE_NAME == "uxas.messages.uxnative.CreateNewService":
            continue

        if obj.FULL_LMCP_TYPE_NAME == full_lmcp_type_namestr: # then we found it / the match!
            print("Received: " + obj.FULL_LMCP_TYPE_NAME)

            print("type(obj): %r" % type(obj))

            d = obj.toDict()
            print("obj.toDict(): '%r'\n" % d)
            #j = json.dumps(d)
            #print("JSON 'dumps' from dict: '%r'\n\n" % j)

# --eof--
