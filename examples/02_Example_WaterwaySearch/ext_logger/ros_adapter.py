#! /usr/bin/env python
# Copyright 2017 Government of the United States of America, as represented by the Secretary of the Air Force.
# No copyright is claimed in the United States under Title 17, U. S. Code. All Other Rights Reserved.
# Copyright 2017 University of Cincinnati. All rights reserved. See LICENSE.md file at:
# https://github.com/afrl-rq/OpenUxAS
# Additional copyright may be held by others, as reflected in the commit history.
"""
Coded by Catharine McGhan, University of Cincinnati.

Some portions of this code were copied from the python example in:
OpenUxAS/examples/02_Example_WaterwaySearch/ext_logger/ext_logger.py

This code is meant to interoperate with both ROS (via rosbridge_interface)
and UxAS (OpenUxAS via LmcpGen, over the ZeroMQ protocol that uses).

Note that ROS uses static messaging while LmcpGen/UxAS uses dynamic messages.
This make it fairly easy to send messages from ROS to UxAS, but not in
the reverse.

The UxAS-equivalent of rosbridge-to-roscore is ros_adapter-to-UxAS.
ros_adapter connects to rosbridge_server to get/send UxAS messages over
to the ROS-side of things, and vice-versa (repeater of ROS topics to UxAS).
"""

#
# for talking to ZeroMQ server / OpenUxAS (and OpenAMASE):
#

import zmq, json, string, sys
sys.path.insert(0, '../../../src/LMCP/py')
from lmcp import LMCPFactory

#
# for talking to roscore via rosbridge_suite:
#

# Note that the following should work so long as you modify the sys.path
# to include the directory which holds the top-level directory of each
# package that you need, and you'll need to include __init__.py in each
# directory inside that "package" so that you can import what you need.
#
# This should be done for every python-executable file. The executable /
# file that invokes the python interpreter needs to be able to find all
# packages that the entire run needs. Changing sys.path from this invoked
# file (relative to the invoked file) works because everything in a
# python (interpreter run) instance sees the same sys.path.
#
import sys # for sys.exit() and sys.path.append()
file_dir = sys.path[0] # *** initialized on program startup, directory of the script used to invoke the python interpreter ***
sys.path.append(file_dir + '/../../../..') # modify sys.path to include directory containing rss_git_lite "package" (same level as OpenUxAS and LmcpGen)
#print("sys.path = %r\n" % sys.path)

from rss_git_lite.common import rosConnectWrapper as rC
from rss_git_lite.common import ws4pyRosMsgSrvFunctions_gen as wsG #ws4pyROS
from rss_git_lite.common import getConnectionIPaddress as gC
from rss_git_lite.common import ws4pyFunctions as wsF
from rss_git_lite.common import ws4pyRosParamFunctions as wsPF

# note: this script/file only requires LmcpGen / OpenUxAS MDMs libraries and rosbridge connection tools from rss_git_lite,
#       this does NOT require ROS messaging to be working / ROS packages to be installed on the system that is running this script! :)
#       *** ...except for ros-kinetic-geodesy to start with (will move to using plain vanilla python-pyproj soon ***

# ros_adapter setup:

# -- need init file that gives info on connections between UxAS channels and ROS topics
#     -- hardcode initially...
# -- need un/re/packers of ROS-to-UxAS (and UxAS-to-ROS) messaging info, including transformations
#     -- hardcode initially, (partially?-)autogenerate from MDMs/ROS stuff later?

# --> start with AirVehicleState from ROS->UxAS, MissionCommand(, LineSearchTask, VehicleActionCommand) from UxAS->ROS for now

# for this example, we are connecting to a Turtlebot model in Gazebo
# (we could use a Pioneer 3-DX model in Gazebo or MobileSim instead, though)
# -- this will be Dubins' path-type motion, at "constant altitude"

# procedure:
# -- set up initial 'listen-to-these' on the UxAS side as ROS-publishers with the correct topics, datatypes, and un/re/packers
# -- set up initial 'copy-these-over' for (all?/only?) the ROS topics that UxAS cares about
#    -- for auto-forward of all ROS messages, would need to have converter of .msg to MDMs format for all *_msgs libraries that we care about
#    -- for ROS topic autoforwarding, need a convention for topic-name-plus-message-name for send (since several topics can use same datatype in ROS; ROS is not "doubled-up" in this respect like LmcpGen / OpenUxAS is)
# -- loop over send/receive:
#    -- read new messages from UxAS
#    -- check to see if any match the sort of message that needs to be passed along to ROS (in dictionary? list?)
#       and make a list of those that need to be converted over UxAS->ROS
#        -- loop over converting each one-by-one and sending them
#    -- loop over the set of subscribed ROS topics (in list?)
#    -- check to see if any new messages were received from ROS
#       and make a list of those that need to be converted over ROS->UxAS
#        -- loop over converting each one-by-one and sending them
#   (-- optional?: check for new ROS topics to add to list that needs to be sent/repeated over from ROS to UxAS?)


def bound(num,minmax):
    """[min_bound,max_bound] = minmax"""
    if num < maxmin[0]:
        num = maxmin[0]
    if num > maxmin[1]:
        num = maxmin[1]
    return num
    
def pack_pose(datalist):
    """
    datalist = [ [x,y,...] , waytype ]

    waytype = 1 is [x,y] format
    waytype = 2 is [x,y,z,qx,qy,qz,qw] format
    
    if no waytype is given(??) then default waytpe=1
    """
    # different bit here, need to pack the datalist a bit harder:
    #waypoints_var = datalist[0]
    if (len(datalist)<2): # this isn't quite right, will get confused if only list-of-lists given
        waypoints_var = datalist
        waytype=1
    else:
        #waypoints_var = datalist[0]
        #waytype = datalist[1]
        [waypoints_var, waytype] = datalist[0:2] # = datalist

    if waytype == 1:
        [x,y] = waypoints_var
        z = qx = qy = qz = qw = 0
    elif waytype == 2:
        [x,yz,qx,qy,qz,qw] = waypoints_var
    else: # unknown waytype, bad
        return None

    json_pose = {'position': {'x': x, 'y': y, 'z': z}, 'orientation': {'x': qx, 'y': qy, 'z': qz, 'w': qw}}
    return json_pose

def convertLatLongAlttoXYAlt(Lat,Long,Alt):
    """
    input: Lat in deg.meters
           Long in deg.meters
           Alt in deg.meters
    output: [X,Y,Alt(Z)] in meters(?)
    
    geographic_msgs/GeoPoint (lat-long-alt in deg.meters) <-> WGS84 x-y-z format
    
    sudo apt install ros-kinetic-geodesy # also installs python-pyproj... need to find a non-ROS-kinetic library for this (python-pyproj direct?)
    """
    import geodesy # technically doesn't include ROS nodes, though is a ROS package apparently???
    import geodesy.utm
    # geographic_msgs/GeoPoint (lat-long-alt in deg.meters) <-> WGS84 x-y-z format
    utm_pt = geodesy.utm.fromLatLong(Lat,Long,Alt)
    X = utm_pt.northing
    Y = -utm_pt.easting
    Alt = utm_pt.altitude

    return [X,Y,Alt]

def convertXYAlttoLatLongAlt(offsetLatLongAlt,X,Y,Alt):
    """
    input: offsetLatLongAlt=[offsetLat,offsetLong,offsetAlt] in deg.meters
           X in meters(?)
           Y in meters(?),
           Alt in meters(?)
    output: [Lat,Long,Alt] in deg.meters
    
    geographic_msgs/GeoPoint (lat-long-alt in deg.meters) <-> WGS84 x-y-z format
    
    sudo apt install ros-kinetic-geodesy # also installs python-pyproj... need to find a non-ROS-kinetic library for this (python-pyproj direct?)
    """
    [offsetLat, offsetLong] = offsetLatLong
    
    import geodesy # technically doesn't include ROS nodes, though is a ROS package apparently???
    import geodesy.utm
    # get offsets translated back to x-y-z for "global-local" coords
    utm_pt = geodesy.utm.fromLatLong(offsetLat,offsetLong,offsetAlt)
    xoffset = utm_pt.northing
    yoffset = -utm_pt.easting
    zoffset = utm_pt.altitude
    utm_pt_with_offset = geodesy.utm.UTMPoint(-Y + yoffset, X + xoffset, Alt + zoffset, utm_pt.zone, utm_pt.band) # as: (easting, northing, altitude, zone, band)
    wgs84_pt = utm_pt_with_offset.toMsg() # gives back a geographic_msgs/GeoPoint (putting it back in lat-long-alt format again) # *** Matthew reports possible issues here ***
    Lat = wgs84_pt.latitude
    Long = wgs84_pt.longitude
    #Alt = wgs84_pt.altitude
    # cheating on Altitude and AltitudeType here, is going to be "looping around"/reusing from received MissionCommand data...
    
    return [Lat,Long,Alt]

#class pack_SessionStatus(object):
def pack_SessionStatus(dictdatalist):
    """
    Gazebo I/O: http://gazebosim.org/tutorials?tut=ros_comm
    ---
    rostopic echo /clock # for sim time
    rosgraph_msgs/Clock, from /gazebo (sim)
    clock: 'secs', 'nsecs'
    ------------------------
    afrl.cmasi.SessionStatus
    str({'datatype': 'CMASI/SessionStatus', 'datastring': str({'Parameters': [], 'State': 1, 'RealTimeMultiple': 1.0, 'ScenarioTime': ????, 'StartTime': 0})})
    """
#    def __init__(self):
#        self.timescalled = 0
#        self.dict_clock = None
        
#    def pack(dictdatalist): # is list containing a single "dictmsg" (as jsondict['msg'])
    """
    currently, assumes gazebo has started first and is already running
    (this is not trying to track Gazebo state / status)
    -- to add this functionality, try:
       1) checking new time info against old saved info before overwriting self.dict_clock
          (in order to infer the Gazebo sim. state)
       2) set 'datastring.State' to something closer to "reality"
    """
    
    numNones = 0
    #holddictdatalist = [self.dict_clock, self.dict_odom, self.dict_imu, self.dict_curwaypt]
    for i in range(len(dictdatalist)):
        #if not (dictdatalist[i] is None): # get have new data, should save and use that
        #    holddictdatalist[i] = dictdatalist[i]
        #if holddictdatalist[i] is None:
        if dictdatalist[i] is None:
            numNones = numNones + 1
    
    if numNones > 0: # this should only happen on the first few calls if not given the dicts first
        print("WARNING: pack_SessionStatus() doesn't have enough data to build struct, skipping send...")
        return None
    else: # we have enough data to give back a packaged JSON dict for sending!
        [dict_clock] = holddictdatalist
        
        #timescalled = timescalled + 1

        time_in_msecs = dict_clock['clock']['secs']*1000 + int(dict_clock['clock']['nsecs']/1000000)

        dicadd = {'Parameters': [],
                  'State': 1, # simulation is... Stopped=0, Running=1, Paused=2, Reset=3
                  'RealTimeMultiple': 1.0,
                  'ScenarioTime': time_in_msecs, # units: millisecond
                  'StartTime': 0 # The simulation or scenario start time. This is absolute time in milliseconds since epoch (1 Jan 1970 00:00 GMT). If this field is zero, then no start time is specfied and each sim component is to use the first receipt of this Struct with a SimStatus of "Running" as the start time.
                 }
        
        #
        # putting it all together...
        #
        
        dictpack = {'datatype': 'CMASI/SessionStatus', \
                    'datastring': str(dictadd) \
                   }
        
        return dictpack

#class pack_AirVehicleState(object):
def pack_AirVehicleState(dictdatalist):
    """
    Gazebo I/O: http://gazebosim.org/tutorials?tut=ros_comm
    ---
    rostopic echo /clock # for sim time
    rosgraph_msgs/Clock, from /gazebo (sim)
    clock: 'secs', 'nsecs'
    
    Turtlebot in gazebo sends:
    ---
    rostopic echo /odom
    nav_msgs/Odometry, from /gazebo
    child_frame_id: base_footprint
    includes header, child_frame_id, pose, and twist

    rostopic echo /mobile_base/sensors/imu_data
    sensor_msgs/Imu, from /gazebo (turtlebot model)
    header: frame_id is 'base_link'
    includes orientation (quaternion), angular_velocity (rad/s), linear_acceleration (m/s^2), all with *_covariance variables as well

    need to pass around new .msg curwaypt (send back from waypoint planner, or whatever's talking to the waypoint planner...):
    ---
    int64 WaypointID
    # -- this Lat-Long-Alt comes from line 168-170 of examples/02_Example_WaterwaySearch/Scenario_WaterwaySearch.xml
    #float64 Altitude
    #float64 AltitudeType
    # ...maybe make this a RosParam call instead of part of the curwaypt, since is static for each run?
    #float64 LatStart # (saved) beginning starting position, use as offset for (x,y)!
    #float64 LongStart # (saved) beginning starting position, use as offset for (x,y)!

    ------------------------
    afrl.cmasi.AirVehicleState
    str({
    'datatype': 'CMASI/AirVehicleState',
    'datastring': str(
    {'ID': ????, # int64 # A unique ID for this entity. IDs should be greater than zero (500???)
    'u': ????, # units: meter/sec # Velocity in the body x-direction (postive out nose)
    'v': ????, # units: meter/sec # Velocity in the body y-direction (positive out right wing)
    'w': ????, # units: meter/sec # Velocity in the body z-direction (positve downward)
    'udot': ????, # units: meter/sec/sec # Acceleration in the body x-direction (postive out nose)
    'vdot': ????, # units: meter/sec/sec # Acceleration in the body y-direction (positive out right wing)
    'wdot': ????, # units: meter/sec/sec # Acceleration in the body z-direction (positve downward)
    'Heading': ????, # units: degree # Angle between true North and the projection of the body x-axis in the North-East plane.
    'Pitch': ????, # units: degree # Pitch of vehicle around body y-axis (positive upwards)
    'Roll': ????, # units: degree # Roll angle of the vehicle around body x-axis (positive right wing down)
    'p': ????, # units: degree/sec # roll-rate of vehicle (angular velocity around body x-axis). Positive right-wing down.
    'q': ????, # units: degree/sec # pitch rate of the vehicle (angular velocity around body y-axis). Positive nose-up.
    'r': ????, # units: degree/sec # yaw rate of the vehicle (angular velocity around body z-axis). Positive nose right.
    'Course': ???? # units: degrees # Course/Groundtrack angle of the entity referenced to true North
    'Groundspeed': ????, # units: m/s
    # ----------------------------------------
    # The perceived entity location
    # Extended by: cmasi_msgs/Waypoint
    # Child cmasi_msgs/Waypoint extended by: cmasi_msgs/PathWaypoint
    # Child cmasi_msgs/PathWaypoint extended by: (n/a, no children)
    #'Location': ['datatype': 'cmasi_msgs/Location3D', 'datastring': str({????})],
    'Location': {'datatype': 'CMASI/Location3D', 'datastring': str({'Latitude': ????, 'Altitude': ????, 'AltitudeType': 1, 'Longitude': ????})},
    'EnergyAvailable': 100.00, # units: %
    'ActualEnergyRate': 0.00027799999225, # units: %/sec # consumption rate of available energy, %-max-capacity per sec # from Waterways example run
    # ----------------------------------------
    # A list of states for any onboard payloads
    # Extended by: cmasi_msgs/GimballedPayloadState, cmasi_msgs/GimbalState, cmasi_msgs/VideoStreamState, impact_msgs/RadioState, impact_msgs/PowerPlantState
    # Child cmasi_msgs/GimballedPayloadState extended by: cmasi_msgs/CameraState
    #'PayloadStateList': [{'datatype': 'cmasi_msgs/PayloadState', 'datastring': str({...}) }],
    #'PayloadStateList': [{'datatype': 'CMASI/GimbalState', 'datastring': str({...})}, {...}, ...],
    #'PayloadStateList': [{'datatype': 'CMASI/CameraState', 'datastring': str({...})}, {...}, ...],
    'PayloadStateList': [], # for now...
    'CurrentWaypoint': ???? # int64 # The ID of the current waypoint. Only valid if the vehicle is in waypoint following mode. (starts at 0)
    'CurrentCommand': 0, # int64 (23 if Mode=1?)
    'Mode': 0, # Waypoint=0, Loiter=1, FlightDirector=2, TargetTrack=3, FollowLeader=4, LostComm=5
    # ----------------------------------------
    # Tasks that this entity is currently executing. An empty list indicates no associated tasks. The task number should coincide with the task number in the task request. For instance, if a waypoint is associated with a search task, then the task number associated with that search should be included in this list.
    'AssociatedTasks': [], # integer values
    'Time': ????, # integer, timestamp of data, units: millisecond since 1 Jan 1970 # should match "afrl.cmasi.SessionStatus' 'ScenarioTime' message that is sent just prior to this
    'Info': [], # list that maps keys to values for the inclusion of extra, custom information about this entity
    'Airspeed': ????, # true airspeed, meter/sec
    'VerticalSpeed': 0.0, # rate of change of altitude, positive-up
    'WindSpeed': 0.0,
    'WindDirection': 0.0}
    )
    })
    """

#    def __init__(self): # init everything
#        self.timescalled = 0
#        self.dict_clock = None
#        self.dict_odom = None
#        self.dict_imu = None
#        self.dict_curwaypt = None
        
#    def pack(self, dictdatalist): # will use most recent data (if given None, will use last data we have
    numNones = 0
    #holddictdatalist = [self.dict_clock, self.dict_odom, self.dict_imu, self.dict_curwaypt]
    for i in range(len(dictdatalist)):
        #if not (dictdatalist[i] is None): # get have new data, should save and use that
        #    holddictdatalist[i] = dictdatalist[i]
        #if holddictdatalist[i] is None:
        if dictdatalist[i] is None:
            numNones = numNones + 1
    
    if numNones > 0: # this should only happen on the first few calls if not given the dicts first
        print("WARNING: pack_AirVehicleState() doesn't have enough data to build struct, skipping send...")
        return None
    else: # we have enough data to give back a packaged JSON dict for sending!
        #[self.dict_clock, self.dict_odom, self.dict_imu, self.dict_curwaypt] = holddictdatalist
        [dict_clock, dict_odom, dict_imu, dict_curwaypt] = dictdatalist
        
        #timescalled = timescalled + 1

        time_in_msecs = self.dict_clock['clock']['secs']*1000 + int(self.dict_clock['clock']['nsecs']/1000000)

        #
        # bodyframe of turtlebot is x out front/"nose", y out left (so use -y for "positive out right wing"), z up (so use -z for "positive downward")
        #
        
        # linear velocity
        u = dict_odom['twist']['twist']['linear']['x']
        v = -dict_odom['twist']['twist']['linear']['y']
        w = -dict_odom['twist']['twist']['linear']['z']
        ground_speed = math.sqrt(u*u + v*v) # speed in x-y "ground"plane :)
        air_speed = math.sqrt(u*u + v*v + w*w) # speed (magnitude) in 3D space

        # angular velocity
        p = dict_odom['twist']['twist']['angular']['x']
        q = -dict_odom['twist']['twist']['angular']['y']
        r = -dict_odom['twist']['twist']['angular']['z']
        
        udot = dict_imu['linear_acceleration']['x']
        vdot = -dict_imu['linear_acceleration']['y']
        wdot = -dict_imu['linear_acceleration']['z']
        #p = math.degrees(dict_imu['angular_velocity']['x'])
        #q = math.degrees(-dict_imu['angular_velocity']['y'])
        #r = math.degrees(-dict_imu['angular_velocity']['z'])
        
        qx = dict_imu['orientation']['x'] # gives quaternion
        qy = dict_imu['orientation']['y'] # gives quaternion
        qz = dict_imu['orientation']['z'] # gives quaternion
        qw = dict_imu['orientation']['w'] # gives quaternion
        # the ROS way...
        #import tf
        #(r,p,y)=tf.transformations.euler_from_quaternion([x,y,z,w],axes='szyx')
        # the UxAS way...
        # UxAS uses Psi-Theta_Phi Euler angles (z-y-x / heading-attitude-bank / yaw-pitch-roll), tangential-plane attitude
        # (for more info on Tait-Bryan angles representation, see: https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles)
        import math
        roll_bank_x = math.degrees(math.atan2( 2*(qw*qx + qy*qz) , 1-2*(qx*qx + qy*qy) ))
        pitch_attitude_y = math.degrees(math.asin( bound(2*(qw*qy - qz*qx),[-1,1]) )) # cut this off at both ends, just in case of computation error
        heading_yaw_z = math.degrees(math.atan2( 2*(qw*qz + qx*qy) , 1-2*(qy*qy + qz*qz) ))

        #
        # getting Location3D point created...
        #

        # *** this Lat-Long-Alt comes from line 168-170 of examples/02_Example_WaterwaySearch/Scenario_WaterwaySearch.xml ***
        # vehicleID is 400 (not 500)
        #
        # current waypoint ID, Altitude, AltitudeType, etc.
        #curwayptID = dict_curwaypt['WaypointID']
        curwayptID = dict_curwaypt['data'] # std_msgs/Int64
        #curwayptAltitude = self.dict_curwaypt['Altitude']
        #curwayptAltitudeType = self.dict_curwaypt['AltitudeType']
        ## ...maybe make this a RosParam call instead of part of the curwaypt, since is static for each run?
        #curwayptLatStart = self.dict_curwaypt['LatStart'] # (saved) beginning starting position, use as offset for (x,y)!
        #curwayptLongStart = self.dict_curwaypt['LongStart'] # (saved) beginning starting position, use as offset for (x,y)!
        curwayptVehicleID = 400
        curwayptAltitude = 700
        curwayptAltitudeType = 1 # MSL
        # ...maybe make this a RosParam call instead of part of the curwaypt, since is static for each run?
        curwayptLatStart = 45.3171 # (saved) beginning starting position, use as offset for (x,y)!
        curwayptLongStart = -120.9923 # (saved) beginning starting position, use as offset for (x,y)!

        x = dict_odom['pose']['pose']['position']['x'] # meters
        y = dict_odom['pose']['pose']['position']['y'] # meters
        z = dict_odom['pose']['pose']['position']['z'] # meters
        #qx = dict_odom['pose']['pose']['orientation']['x']
        #qy = dict_odom['pose']['pose']['orientation']['y']
        #qz = dict_odom['pose']['pose']['orientation']['z']
        #qw = dict_odom['pose']['pose']['orientation']['w']
        
        [latitude,longitude,altitude] = convertXYAlttoLatLongAlt([curwayptLatStart,curwayptLongStart,curwayptAltitude],x,y,z)
        # cheating on Altitude and AltitudeType here, is going to be "looping around"/reusing from received MissionCommand data...
        dictloc3d = {'Latitude': latitude, # units: degree
                     'Altitude': curwayptAltitude, # units: meter
                     'AltitudeType': curwayptAltitudeType, # default: 1(=MSL) # 0(=AGL) Height above ground/survace level, 1(=MSL) Height above WGS84 ellipsoid, mean sea level
                     'Longitude': longitude # units: degree
                    }
        
        #
        # putting it all together
        #
        
        dictadd = {'ID': vehicleID,
                   'u': u, # units: meter/sec # Velocity in the body x-direction (postive out nose)
                   'v': v, # units: meter/sec # Velocity in the body y-direction (positive out right wing)
                   'w': w, # units: meter/sec # Velocity in the body z-direction (positve downward)
                   'udot': udot, # units: meter/sec/sec # Acceleration in the body x-direction (postive out nose)
                   'vdot': vdot, # units: meter/sec/sec # Acceleration in the body y-direction (positive out right wing)
                   'wdot': wdot, # units: meter/sec/sec # Acceleration in the body z-direction (positve downward)
                   'Heading': heading_yaw_z, # units: degree # Angle between true North and the projection of the body x-axis in the North-East plane.
                   'Pitch': pitch_attitude_y, # units: degree # Pitch of vehicle around body y-axis (positive upwards)
                   'Roll': roll_bank_x, # units: degree # Roll angle of the vehicle around body x-axis (positive right wing down)
                   'p': p, # units: degree/sec # roll-rate of vehicle (angular velocity around body x-axis). Positive right-wing down.
                   'q': q, # units: degree/sec # pitch rate of the vehicle (angular velocity around body y-axis). Positive nose-up.
                   'r': r, # units: degree/sec # yaw rate of the vehicle (angular velocity around body z-axis). Positive nose right.
                   'Course': heading_yaw_z, # units: degrees # Course/Groundtrack angle of the entity referenced to true North
                   'Groundspeed': ground_speed, # units: m/s
                   'Location': {'datatype': 'CMASI/Location3D', 'datastring': str(dictloc3d)},
                   'EnergyAvailable': 100.00,
                   'ActualEnergyRate': 0.00027799999225,
                   'PayloadStateList': [], # for now...
                   'CurrentWaypoint': curwayptID, # int64 # The ID of the current waypoint. Only valid if the vehicle is in waypoint following mode. (starts at 0)
                   'CurrentCommand': 0, # int64 (23 if Mode=1?)
                   'Mode': 0, # Waypoint=0, Loiter=1, FlightDirector=2, TargetTrack=3, FollowLeader=4, LostComm=5
                   # Tasks that this entity is currently executing. An empty list indicates no associated tasks. The task number should coincide with the task number in the task request. For instance, if a waypoint is associated with a search task, then the task number associated with that search should be included in this list.
                   'AssociatedTasks': [], # integer values
                   'Time': time_in_msecs, # integer, timestamp of data, units: millisecond since 1 Jan 1970 # should match "afrl.cmasi.SessionStatus' 'ScenarioTime' message that is sent just prior to this
                   'Info': [],
                   'Airspeed': air_speed, # true airspeed, meter/sec
                   'VerticalSpeed': 0.0,
                   'WindSpeed': 0.0,
                   'WindDirection': 0.0
                  }
        
        dictpack = {'datatype': 'CMASI/AirVehicleState',
                    'datastring': str(dictadd)
                   }
        
        return dictpack

#class unpack_MissionCommand(object):
def repack_MissionCommand(dictdatalist):
    """
    {
     \'Status\': 1,
     \'VehicleActionList\': [],
     \'VehicleID\': 400,
     \'FirstWaypoint\': 100004002,
     \'WaypointList\': [
                        {\'datatype\': \'CMASI/Waypoint\', \'datastring\': "{\'VehicleActionList\': [], \'TurnType\': 0, \'ClimbRate\': 0.0, \'Altitude\': 700.0, \'Number\': 100004001, \'Longitude\': -120.990835757, \'SpeedType\': 0, \'AltitudeType\': 1, \'ContingencyWaypointA\': 0, \'Latitude\': 45.3166492531, \'AssociatedTasks\': [], \'Speed\': 22.0, \'ContingencyWaypointB\': 0, \'NextWaypoint\': 100004002}"},
                        {...},
                        ...
                        {...}
                       ],
     \'CommandID\': 119
    }
    """
#    def __init__():
#         # init everything
#        self.timescalled = 0
#        self.dict_clock = None
#        self.dict_odom = None
#        self.dict_imu = None
#        self.dict_curwaypt = None
        
#    def unpack(self, dictdatalist): # will use most recent data (if given None, will use last data we have
    numNones = 0
    #holddictdatalist = [self.dict_MC]
    for i in range(len(dictdatalist)):
    #    if not (dictdatalist[i] is None): # get have new data, should save and use that
    #        holddictdatalist[i] = dictdatalist[i]
        #if holddictdatalist[i] is None:
        if dictdatalist[i] is None:
            numNones = numNones + 1
    
    if numNones > 0: # this should only happen on the first few calls if not given the dicts first
        print("WARNING: repack_MissionCommand() doesn't have enough data to build struct, skipping send...")
        return None
    else: # we have enough data to give back a packaged JSON dict for sending!
        [dict_MC] = dictdatalist
        
        #timescalled = timescalled + 1

        if dict_MC['datatype'] == 'CMASI/MissionCommand' and dict_MC['datastring']['VehicleID'] == 400: # then we've got the right sort of message to unpack, and for the right vehicle even! :)
            topdata = ast.literal_eval(dict_MC['datastring']) # get first chunk of dictionary out to mess with
            
            keys = ['Status','VehicleActionList','VehicleID']
            expected = [1, [], 400]
            for key,exp in zip(keys,expected):
                if topdata[key] != exp: # unexpected
                    print("WARNING: " + key + " is %r, should be %r." % (topdata[key],exp))
                    
            theFirstWaypoint = topdata['FirstWaypoint']
            theCommandID = topdata['CommandID']
            grabWaypoints = []
            for wayptdict in topdata['WaypointList']:
                if wayptdict['datatype'] != 'CMASI/Waypoint':
                    print("ERROR: expecting 'CMASI/Waypoint' from MissionCommand, got: '%s'" % wayptdict['datatype'])
                else: # we got the data in a format we know how to deal with!
                    wayptdata = ast.literal_eval(wayptdict['datastring']) # get waypt dictionary out to mess with
                    grabWaypoints.append(wayptdata)
            
            sortWaypoints = []
            curwayptnum = theFirstWaypoint
            prevwaypt = None
            for i in range(len(grabWaypoints)): # going to potentially loop over every waypoint in the list, unless the connection between them is broken somehow? (bad)
                found = 0
                for j in range(len(grabWaypoints)): # search over entire list for "correct" waypoint
                    if grabWaypoints[j]['Number'] == curwayptnum: # found the current waypoint!
                        sortWaypoints.append(grabWaypoints[j])
                        prevwaypt = curwaypt
                        curwaypt = grabWaypoints[j]['NextWaypoint']
                        found = 1
                        break
                if (found == 0):
                    print("DEBUG: at end of list? breaking out of outer loop.")
                    break

            # send altitude, longitude, latitude in the ordered list (and allow overwrites when new command IDs are given from UxAS? or should we send waypt num.s and append?
            
            # geometry_msgs/PoseArray:
            poseslist = []
            for i in range(len(sortWaypoints)):
                [x,y,alt] = convertLatLongAlttoXYAlt(sortWaypoints[i]['Latitude'],sortWaypoints[i]['Longitude'],sortWaypoints[i]['Altitude'])
                poseslist.append(pack_pose([[x,y],1])) # gives position - orientation packaging, type 1 is [x,y] only
            dictpack = {'header': rC.json_header(t=0,frame_id=str(sortWaypoints[i]['Number'])), 'poses': poseslist} # frame_id is being used to store the 'Number' of the waypt

            # nav_msgs/Path:
            #poseslist = []
            #for i in range(len(sortWaypoints)):
            #    [x,y,alt] = convertLatLongAlttoXYAlt(sortWaypoints[i]['Latitude'],sortWaypoints[i]['Longitude'],sortWaypoints[i]['Altitude'])
            #    poseslist.append({'header': wsG.json_header(t=0,frame_id=str(sortWaypoints[i]['Number']), 'pose': pack_pose([[x,y],1])}) # gives position - orientation packaging, type 1 is [x,y] only # frame_id is being used to store the 'Number' of the waypt
            #dictpack = {'header': wsG.json_header(t=0), 'poses': poseslist}
            
            return dictpack

        else:
            return None


def unpack_dictmsg_out_of_msg(dictmsg):
    return dictmsg['msg']

def pack_already_jsonmsg(jsonmsg):
    return jsonmsg
    
#def overwrite_incoming_data_only_if_new(dictdatalist,olddatalist):
#    if len(dictdatalist) != len(olddatalist):
#        print("ERROR: newlist not same length as oldlist. Exiting...")
#        sys.exit(0)
#    else:
#        numNones = 0
#        holddictdatalist = list(olddatalist)
#        for i in range(len(dictdatalist)):
#            if not (dictdatalist[i] is None): # get have new data, should save and use that
#                holddictdatalist[i] = dictdatalist[i]
#            if holddictdatalist[i] is None:
#                numNones = numNones + 1
#    return [numNones, holddictdatalist]


if __name__ == '__main__':
    
    #
    # set up initial connection to UxAS "party-line" so can read-in messages
    #
    
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

    #                       'LmcpGen topic/datatype name': [['ROS topic','ROS datatype',ROS un/re/packer function(s)/classes], [.,.,.], ...] , ...
    dict_lmcptypes_toros = {'afrl.cmasi.MissionCommand': [ ['/from_uxas/MissionCommand', 'uxas_required_msgs/DatatypeDatastring', pack_already_jsonmsg] ],
                                                           #['/from_uxas/MissionCommand/waypointlist', 'geometry_msgs/PoseArray', repack_MissionCommand] ],
                            'afrl.cmasi.LineSearchTask': [ ['/from_uxas/LineSearchTask', 'uxas_required_msgs/DatatypeDatastring', pack_already_jsonmsg] ],
                            'afrl.cmasi.VehicleActionCommand': [ ['/from_uxas/VehicleActionCommand', 'uxas_required_msgs/DatatypeDatastring', pack_already_jsonmsg] ]
                           }
                            
    order_lmcptypes_toros = ['afrl.cmasi.MissionCommand','afrl.cmasi.LineSearchTask','afrl.cmasi.VehicleActionCommand']


    #
    # for connection to turtlebot:
    #

    #                         'LmcpGen topic/datatype name': [ [['ROS topic','ROS datatype'],[.,.], ...], ROS un/re/packer function/class]
    dict_lmcptypes_fromros = {'afrl.cmasi.AirVehicleState': [ [['/clock','rosgraph_msgs/Clock'],
                                                               ['/odom','nav_msgs/Odometry'],
                                                               ['/mobile_base/sensors/imu_data','sensor_msgs/Imu'],
                                                               #['/curwaypt','uxas_required_msgs/onewaypt']],
                                                               ['/curwaypt','std_msgs/Int64']],
                                                              pack_AirVehicleState ],
                              'afrl.cmasi.SessionStatus': [ [['/clock','rosgraph_msgs/Clock']],
                                                            pack_SessionStatus ]
                             }

    order_lmcptypes_fromros = ['afrl.cmasi.SessionStatus', 'afrl.cmasi.AirVehicleState']
    #holddata_lmcptypes_fromros = []
    #for lmcptype in order_lmcptypes_fromros:
    #    holddata_lmcptypes_fromros.append([None]*len(dict_lmcptypes_fromros[lmcptype][0])) # make a list of holder chunks for the number of channels for each lmcptypes_from_ros
    rosdictInhold = {}
    
    #
    # set up ROS publishers for translation from Lmcp-to-ROS
    #

    #connection = "ws://localhost:9090/" # example string for connecting to rosbridge_server
    connection = gC.getConnectionIPaddress(3) # this should give back: "ws://localhost:9090/"

    # set up ROS publishers:
    ws_pubdict = {} # will hold all publisher class objects, keyed to their topicname
    ws_publist = [] # will hold all 'topicname's
    for key in order_lmcptypes_toros:
        for i in range(len(dict_lmcptypes_toros[key])):
            [topicname,datatypename,un_pack_function] = dict_lmcptypes_toros[key][i]
            if topicname in ws_pubdict.keys():
                print("DEBUG: error, topicname '%s' for key '%s' already in dict_lmcptypes_toros!" % (topicname.key))
                print("DEBUG: can't double-up on publishing topicnames, exiting...")
                sys.exit(1)
            else:
                # if we were explicitly giving the same topicname to this as the OpenUxAS channel...
                #holdsplit = topicname.split('.')
                #datatypename = (holdsplit[-2]).lower() + '_msg/' + holdsplit[-1]
                # rC.RosMsg(connectlib, connection, pubsub, topicname, datatypename, un_pack_function=None)
                wshold = rC.RosMsg('ws4py', connection, 'pub', topicname, datatypename, pack_already_jsonmsg)
                ws_pubdict.update( {topicname: wshold} ) # add this to list # dictionaries and classes are mutable, so we should be okay?
                ws_publist.append(topicname)
    
    #
    # set up ROS subscribers for translation from ROS-to-Lmcp
    #    
    
    # set up ROS subscribers:
    ws_subdict = {} # will hold all subscriber class objects, keyed to their topicname
    ws_sublist = [] # will hold all 'topicname's
    for key in order_lmcptypes_fromros:
        for i in range(len(dict_lmcptypes_fromros[key][0])):
            [topicname,datatypename] = dict_lmcptypes_fromros[key][0][i]
            if topicname in ws_subdict.keys():
                print("DEBUG: warning, topicname '%s' for key '%s' already in dict_lmcptypes_fromros!" % (topicname,key))
                print("DEBUG: shouldn't double-up on subscribing topicnames, skipping 'resubscription'...")
                #sys.exit(1)
            else:
                # if we were explicitly giving the same topicname to this as the OpenUxAS channel...
                #holdsplit = topicname.split('.')
                #datatypename = (holdsplit[-2]).lower() + '_msg/' + holdsplit[-1]
                # rC.RosMsg(connectlib, connection, pubsub, topicname, datatypename, un_pack_function=None)
                wshold = rC.RosMsg('ws4py', connection, 'sub', topicname, datatypename, unpack_dictmsg_out_of_msg)
                ws_subdict.update( {topicname: wshold} ) # add this to list # dictionaries and classes are mutable, so we should be okay?
                ws_sublist.append(topicname)
                rosdictInhold.update({ topicname: None }) # also initialize the stuff with 'None' to begin with
    
    """
    # to "read" something here, try at the commandline before running this program: `rostopic pub /robot0/runtype std_msgs/Int8 "data: 12" -l`
    ws = RosMsg('ws4py', connection, 'sub', '/robot0/runtype', 'std_msgs/Int8', ws4pyROS.unpack_runtype)
    # retrieves data if it exists, None if no receipt since last call
    #runtype = ws.copy_and_clear_received() # receive "directly", no checks on channel pub/sub status
    runtype = ws.receive() # "safer"-receive
    #if runtype is None:
    #    print("No data received on topic '/robot/runtype'")
    #else:
    #    print("Data received from topic '/robot0/runtype'!")
    #    print("runtype = %r " % runtype)
    
    
    #
    # if you are running MobileSim and RosAria, you could use the below command to control the robot's motion:
    # (to "hear" this, try at the commandline before running this program: `rostopic echo /RosAria/cmd_vel`)
    #
    ws2 = RosMsg('ws4py', connection, 'pub', '/RosAria/cmd_vel', 'geometry_msgs/Twist', ws4pyROS.pack_cmdvel)
    #
    # if you are running p3dx in gazebo, you could use this ws2 instead set up control the robot's motion:
    # (to "hear" this, try at the commandline before running this program: `rostopic echo /cmd_vel`)
    #
    #ws2 = RosMsg('ws4py', connection, 'pub', '/cmd_vel', 'geometry_msgs/Twist', ws4pyROS.pack_cmdvel)
    # then set up and send the command:
    cmdvelcmd = {'linear': {'x': fullpose.linear.x, 'y': fullpose.linear.y, 'z': fullpose.linear.z}, \
                 'angular': {'x': fullpose.angular.x, 'y': fullpose.angular.y, 'z': fullpose.angular.z}}
    ws2.rosconn.send_message(cmdvelcmd) # to send directly in JSON format through ws4py connection interface
    print("cmdvelcmd sent!\n%r" % cmdvelcmd)
    """
    
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
        
        #lmcp_found = 0
        for full_lmcp_type_namestr in order_lmcptypes_toros: # only "afrl.cmasi.MissionCommand" for now...
            if obj.FULL_LMCP_TYPE_NAME == full_lmcp_type_namestr: # then we found it / the match!
                #lmcp_found = 1
                # do stuff
                
                print("Received: " + obj.FULL_LMCP_TYPE_NAME)

                d = obj.toDict()
                
                print("dict loaded from json: '%r'\n" % d)
                print("type(obj): %r" % type(obj))
                
                key = obj.FULL_LMCP_TYPE_NAME
                for i in range(len(dict_lmcptypes_toros[key])):
                    [topicname,datatypename,un_pack_function] = dict_lmcptypes_toros[key][i]
                    wshold = ws_pubdict[topicname] # get the correct ws publisher
                    wshold.send(d) # send the data using the correct ws publisher
                
                break # only one match per obj(ect), duh, so stop here :)
                
        #
        # attempt to read ROS data across all open channels, store updated info, pack for each to-UxAS object, and send along to UxAS
        #
        
        # rosdictInhold is set as {} once very far up, will hold most recent data received from the ROS-side channels always
        for lmcptype in order_lmcptypes_fromros: # for each lmcptype we're supposed to send, get updated data from all ROS topics required for send
            # requires that ROS topics be set up first...
            for i in range(len(dict_lmcptypes_fromros[lmcptype][0])):
                [rostopicname,rostypename] = dict_lmcptypes_fromros[lmcptype][0][i]
                dicthold = ws_subdict[rostopicname].receive()
                if not (dicthold is None): # we got new data :)
                    rosdictInhold.update({ rostopicname: dicthold }) # most recent data received by topicname

        # take data from ROS and send
        tosendtoUxAS = {}
        for lmcptype in order_lmcptypes_fromros: # for each lmcptype we're supposed to send, put together necessary data in a list and pack into format needed
            # get the data packed into sections that are needed for each UxAS object
            dictdatalist = []
            # requires that ROS topics have been queried for data first...
            for i in range(len(dict_lmcptypes_fromros[lmcptype][0])):
                [rostopicname,rostypename] = dict_lmcptypes_fromros[lmcptype][0][i]
                if rostopicname in rosdictInhold: # should have been built in from the start with at least a 'None' above
                    dictdatalist.append(rosdictInhold[rostopicname])
                else: # but if not, then set to None
                    print("WARNING: rostopicname '%s' not in rosdictInhold. Setting data in dictdatalist for packaging to None." % rostopicname)
                    dictdatalist.append(None)
            jsondicthold = dict_lmcptypes_fromros[lmcptype][1](dictdatalist) # pack function call, expected-input in order in which listed in dict_lmcptypes_fromros
#            tosendtoUxAS.update( {lmcptype: jsondicthold} )
#        
#        for lmcptype in order_lmcptypes_fromros: # for each lmcptype we're supposed to send, take updated data and get new jsondict to send
#            jsondicthold = tosendtoUxAS[lmcptype]
            
            # create UxAS obj from dictionary
            obj = factory.unpackFromDict(jsondicthold)
            # send object if object was created successfully (not give a None)
            if obj is None:
                print("WARNING: jsondicthold = '%s' did not create useful object for lmcptype '%s'. Skipping." % (str(jsondicthold),lmcptype))
            else:
                # syntax to send back to UxAS
                header = str(obj.FULL_LMCP_TYPE_NAME) + "$lmcp|" + str(obj.FULL_LMCP_TYPE_NAME) + "||0|0$"
                smsg = LMCPFactory.packMessage(obj, True)
                socket_send.send(header + smsg)
                print("  Sent:   " + obj.FULL_LMCP_TYPE_NAME)
        






### *** old stuff below this line ***
"""
if __name__ == '__main__':

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

        print("Received: " + obj.FULL_LMCP_TYPE_NAME)

        if obj.FULL_LMCP_TYPE_NAME == "afrl.cmasi.cmasi.MissionCommand":
            print("Received: " + obj.FULL_LMCP_TYPE_NAME)

            d = obj.toDict()
            
            print("dict loaded from json: '%r'\n" % d)
            print("type(obj): %r" % type(obj))
            

        # convert to XML
        xmlStr = obj.toXMLStr("")
        
        # convert from XML
        obj = factory.unpackFromXMLString(xmlStr)[0]
        
        # convert to JSON
        d = obj.toDict()
        j = json.dumps(d)
        print("obj.toDict(): '%r'\n" % d)
        print("JSON 'dumps' from dict: '%r'\n\n" % j)

	    # convert from JSON
        d2 = json.loads(j)
        obj2 = factory.unpackFromDict(d2)
        print("dict loaded from json: '%r'\n" % d2)
        print("type(obj): %r" % type(obj2))
        print("obj unpacked from dict: '%r'\n\n" % obj2)

        print("d2 same as original d? %r" % (d == d2,))
        if not (obj == obj2):
            #print("obj = '%s'" % obj.toString())
            #print("obj2 = '%s'" % obj2.toString())
            dd1 = obj.toDict()
            dd2 = obj2.toDict()
            # exporting back to dicts, to see whether the DATA really is different (not just listed as diff b/c diff subobjects)
            print("dicts-from-class-objects the same? %r" % (dd1 == dd2,))
            if not (dd1 == dd2):
                print("obj2 not same as original obj and dicts show different data, too (yikes!!)")
                print("dict obj = '%s'" % str(dd1))
                print("dict obj2 = '%s'" % str(dd2))        

        # syntax to create an object from scratch
        obj = factory.createObjectByName("CMASI", "KeyValuePair")
        obj.set_Key("Hello")
        obj.set_Value("World")
        
        # syntax to send back to UxAS
        header = str(obj.FULL_LMCP_TYPE_NAME) + "$lmcp|" + str(obj.FULL_LMCP_TYPE_NAME) + "||0|0$"
        smsg = LMCPFactory.packMessage(obj, True)
        socket_send.send(header + smsg)
        print("  Sent:   " + obj.FULL_LMCP_TYPE_NAME)
"""

# all turtlebot libraries are installed system-wide and standard (see https://github.com/AS4SR/vagrant-rss !)
# (nothing modified needed under the catkin_ws/src directory to start with)

# sudo apt-get install ros-kinetic-yujin-ocs # gives us yocs_cmd_vel_mux and other bits -- should already have this, though
# seems to work okay-ish even with yocs_cmd_vel_mux + mobile_base_nodelet_manager crashing
# /opt/ros/kinetic/share/turtlebot_gazebo/launch/turtlebot_world.launch
# /opt/ros/kinetic/share/turtlebot_gazebo/launch/includes/kobuki.launch.xml

# from: http://learn.turtlebot.com/2015/02/03/7/
# roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=/opt/ros/kinetic/share/turtlebot_gazebo/worlds/empty.world

# receives /mobile_base/commands/velocity
# geometry_msgs/Twist, to /gazebo (turtlebot model)
# need to repeat message or will stop moving after awhile
# angular is in radians/s (use "from math import radians" to get deg/s in the right format)

# from: https://github.com/markwsilliman/turtlebot/blob/master/draw_a_square.py
# alt. input is supposed to be: 'cmd_vel_mux/input/navi'
# rate = rospy.Rate(10) # 10 Hz
# rate.sleep() # will sleep for 1/10 sec per .sleep() call

# rospy.on_shutdown(self.shutdown_fn) # give it the function to run if receive a Ctrl-C
