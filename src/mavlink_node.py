#!/usr/bin/env python3
#!-*- coding: utf-8 -*-

from time import sleep,time
from pymavlink import mavutil
from pymavlink.dialects.v10 import ardupilotmega
import rospy
import os
from gs_board import BoardManager
from gs_flight import FlightController, CallbackEvent
from gs_sensors import SensorManager
from gs_navigation import NavigationManager
from threading import Thread
import math
TARGET_SYSTEM = None
MISSIN_COUNT = None
TARGET_COMP = 1
MISSION_LIST = []
CUSTOM_MODE = 0

def start_mission(mission_list):
    def callback(event):
        global mission_list
        global mission_flight
        global start_point
        event = event.data
        i = 0
        if event == CallbackEvent.ENGINES_STARTED:
            if mission_list[i][0] == mavutil.mavlink.MAV_CMD_NAV_TAKEOFF:
                mission_flight.takeoff()
        elif event == CallbackEvent.TAKEOFF_COMPLETE:
            if ( mission_list[i][1] != 0.0 ) or ( mission_list[i][1] != float("nan") ):
                mission_flight.updateYaw(mission_list[i][1])
            mision_flight.goToPoint(mission_list[i][2],mission_list[i][3], mission_list[i][4])
            i+=1
        elif event == CallbackEvent.POINT_REACHED:
            if i == len(mission_list):
                if ( start_point[1] != 0.0 ) or ( start_point[1] != float("nan") ):
                    mission_flight.updateYaw(start_point[1])
                mission_flight.landing()
            elif mission_list[i][0] == mavutil.mavlink.MAV_CMD_NAV_WAYPOINT:
                if ( mission_list[i][1] != 0.0 ) or ( mission_list[i][1] != float("nan") ):
                    mission_flight.updateYaw(mission_list[i][1])
                mision_flight.goToPoint(mission_list[i][2],mission_list[i][3], mission_list[i][4])
                i += 1
            elif misiion_list[i][0] == mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH:
                mission_flight.goToPoint(start_point[2], start_point[3], start_point[4])
                i += 1
            elif mission_list[i][0] == mavutil.mavlink.MAV_CMD_NAV_LAND:
                mission_flight.landing()
        elif event == CallbackEvent.COPTER_LANDED:
            mission = False

    global MODE
    mission = True
    if len(mission_list) != 1:
        mission_list[0], mission_list[1] = mission_list[1], mission_list[0]
        start_point = mission_list[1]
    else:
        start_point = mission_list[0]
    print(type(mission_list[0][1]))
    mission_flight = FlightController(callback)
    while mission:
        pass

def send_status(board, sensors, navigation):
    global MODE
    global CUSTOM_MODE
    while not rospy.is_shutdown():
        master.mav.heartbeat_send(
            type = mavutil.mavlink.MAV_TYPE_QUADROTOR,
            autopilot = mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA,
            base_mode = MODE,
            custom_mode = CUSTOM_MODE,
            system_status = mavutil.mavlink.MAV_STATE_STANDBY
        )
        roll,pitch,yaw = sensors.orientation()
        roll = roll*math.pi/180
        pitch = pitch*math.pi/180
        if navigation.system() == navigation.gps.name:
            yaw = (yaw + 180) % 360
        yaw = yaw*math.pi/180
        master.mav.attitude_send(
            time_boot_ms = int(board.time()),
            roll = roll,
            pitch = pitch,
            yaw = yaw,
            rollspeed = 0.01,
            pitchspeed = 0.01,
            yawspeed = 0.01
        )
        rel_altitude = sensors.altitude()
        latitude,longitude,altitude = navigation.gps.position()
        master.mav.global_position_int_send(
            time_boot_ms = int(board.time()),
            lat = int(latitude * 1e7),
            lon = int(longitude * 1e7),
            alt = int(altitude * 1e3),
            relative_alt = int(rel_altitude),
            vx = 0,
            vy = 0,
            vz = 0,
            hdg = 0
        )
        voltage, _ = sensors.power()
        master.mav.sys_status_send(
            onboard_control_sensors_present = 32,
            onboard_control_sensors_enabled = 32,
            onboard_control_sensors_health = 1,
            load = 0,
            voltage_battery = int(voltage * 1000.0),
            current_battery = 5400,
            battery_remaining = -1,
            drop_rate_comm = 0,
            errors_comm = 0,
            errors_count1 = 0,
            errors_count2 = 0,
            errors_count3 = 0,
            errors_count4 = 0
        )

def mode_callback(event):
    global MODE
    event = event.data
    if event == CallbackEvent.ENGINES_STARTED:
        MODE = mavutil.mavlink.MAV_MODE_GUIDED_ARMED
    elif event == CallbackEvent.COPTER_LANDED:
        MODE = mavutil.mavlink.MAV_MODE_GUIDED_DISARMED
    elif event == CallbackEvent.ALL:
        MODE = mavutil.mavlink.MAV_MODE_GUIDED_DISARMED

rospy.init_node("mavlink_node")
try:
    board = BoardManager()
    flight = FlightController(mode_callback)
    sensors = SensorManager()
    navigation = NavigationManager()
    while not board.runStatus():
        pass
    
    hostname = os.popen('ip addr show wlan0').read().split("inet ")[1].split("/")[0]
    master = mavutil.mavlink_connection('udpin:'+hostname+':14550',source_component = TARGET_COMP)
    print("MAVLink Server active on {}:14550".format(hostname))
    master.wait_heartbeat()

    protocol = False
    MODE = mavutil.mavlink.MAV_MODE_GUIDED_DISARMED

    while not rospy.is_shutdown():
        msg = master.recv_match()
        if msg != None:
            print(msg.to_dict())

            if not protocol:
                Thread(target = send_status,args = (board ,sensors, navigation)).start()
                protocol = True

            if type(msg) == ardupilotmega.MAVLink_mission_count_message:
                MISSIN_COUNT = msg.count
                TARGET_SYSTEM = msg.target_system
                MISSION_LIST = []
                master.mav.mission_request_int_send(
                        seq = 0,
                        target_component = TARGET_COMP,
                        target_system = TARGET_SYSTEM
                )
            elif type(msg) == mavutil.mavlink.MAVLink_mission_item_int_message:
                if msg.seq == MISSIN_COUNT-1:
                    master.mav.mission_ack_send(
                            target_system = TARGET_SYSTEM,
                            target_component = TARGET_COMP,
                            type = mavutil.mavlink.MAV_MISSION_ACCEPTED
                    )
                    TARGET_SYSTEM = None
                    MISSIN_COUNT = None
                else:
                    master.mav.mission_request_int_send(
                            seq = msg.seq+1,
                            target_component = TARGET_COMP,
                            target_system = TARGET_SYSTEM
                    )
                MISSION_LIST.append([msg.command,msg.param4,msg.x,msg.y,msg.z])
            elif type(msg) == ardupilotmega.MAVLink_mission_set_current_message:
                master.mav.command_ack_send(
                        command = mavutil.mavlink.MAV_CMD_MISSION_START,
                        result = mavutil.mavlink.MAV_RESULT_ACCEPTED
                )
                Thread(target = start_mission, args = ([MISSION_LIST[msg.seq]],)).start()
            elif type(msg) == ardupilotmega.MAVLink_param_request_list_message:
                master.mav.param_value_send(
                        param_id = b"pioneermax_gs_em",
                        param_value = 0.0,
                        param_type = mavutil.mavlink.MAV_PARAM_TYPE_UINT8,
                        param_count = 0,
                        param_index = 0
                )
            elif type(msg) == ardupilotmega.MAVLink_mission_request_list_message:
                    TARGET_SYSTEM = msg.target_system
                    master.mav.mission_count_send(
                        target_system = TARGET_SYSTEM,
                        target_component = TARGET_COMP,
                        count = 0
                )
            elif type(msg) == ardupilotmega.MAVLink_request_data_stream_message:
                master.mav.data_stream_send(
                        stream_id = msg.req_stream_id,
                        message_rate = 0,
                        on_off = 0
                )
            elif type(msg) == ardupilotmega.MAVLink_set_mode_message:
                CUSTOM_MODE = msg.custom_mode
                MODE = msg.base_mode
            elif type(msg) == ardupilotmega.MAVLink_command_long_message:
                if msg.command == 519:
                    master.mav.command_ack_send(
                            command = 519,
                            result = mavutil.mavlink.MAV_RESULT_ACCEPTED
                    )
                    protocol = True
                elif msg.command == mavutil.mavlink.MAV_CMD_MISSION_START:
                    if len(MISSION_LIST) != 0:
                        master.mav.command_ack_send(
                                command = mavutil.mavlink.MAV_CMD_MISSION_START,
                                result = mavutil.mavlink.MAV_RESULT_ACCEPTED
                        )
                        Thread(target = start_mission, args = (MISSION_LIST, )).start()
                    else:
                        master.mav.command_ack_send(
                                command = mavutil.mavlink.MAV_CMD_MISSION_START,
                                result = mavutil.mavlink.MAV_RESULT_DENIED 
                        )
                elif(msg.command==mavutil.mavlink.MAV_CMD_NAV_TAKEOFF):
                    Thread(target = flight.takeoff).start()
                    master.mav.command_ack_send(
                            command = mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                            result = mavutil.mavlink.MAV_RESULT_ACCEPTED
                    )
                elif msg.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
                    if msg.param1 == 0.0:
                        master.mav.command_ack_send(
                                command = mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                                result = mavutil.mavlink.MAV_RESULT_ACCEPTED
                        )
                        MODE = mavutil.mavlink.MAV_MODE_GUIDED_DISARMED
                        Thread(target = flight.landing).start()
                    elif msg.param1 == 1.0:
                        master.mav.command_ack_send(
                                command = mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                                result = mavutil.mavlink.MAV_RESULT_ACCEPTED
                        )
                        Thread(target=flight.preflight).start()
    master.close()
except Exception as e:
    print(str(e))
