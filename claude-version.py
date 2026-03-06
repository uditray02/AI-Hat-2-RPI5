#!/usr/bin/env python3
"""
strike-v4.py

Hailo Detection + DroneKit Control
+ Full ArduPilot Diagnostics Logging
+ Auto Takeoff to 15m
+ Car Tracking with Yaw Correction
"""

import os
import time
import math
from datetime import datetime
import numpy as np
import gi

gi.require_version("Gst", "1.0")
from gi.repository import Gst

import hailo
from dronekit import connect, VehicleMode
from pymavlink import mavutil

from hailo_apps.python.pipeline_apps.detection_simple.detection_simple_pipeline import (
    GStreamerDetectionSimpleApp,
)
from hailo_apps.python.core.common.hailo_logger import get_logger
from hailo_apps.python.core.gstreamer.gstreamer_app import app_callback_class


# ====================== CONFIG ====================== #

TARGET_CLASS     = "person"
TARGET_ALTITUDE  = 15.0       # metres
VX_CONST         = 8.0        # m/s forward speed
KP_Y             = 0.075      # lateral gain
KP_Z             = 0.05       # vertical gain
KP_YAW           = 0.05       # yaw rate gain  (deg/s per pixel)
MAX_VEL_Y        = 3.5        # m/s
MAX_VEL_Z        = 8.0        # m/s
MAX_YAW_RATE     = 30.0       # deg/s
CMD_DT           = 0.10       # command interval (s)
MAX_LOST_TIME    = 0.5        # hover after this many seconds without target
FRAME_W          = 640        # must match Hailo pipeline output
FRAME_H          = 480


# ====================== LOGGING ====================== #

os.makedirs("logs", exist_ok=True)
LOG_FILE = f"logs/ardupilot_runtime_{datetime.now().strftime('%Y%m%d_%H%M%S')}.txt"
_log_fp = open(LOG_FILE, "w", buffering=1)


def log(msg: str) -> None:
    timestamp = datetime.now().strftime("%H:%M:%S")
    line = f"[{timestamp}] {msg}"
    print(line)
    _log_fp.write(line + "\n")


# ====================== USER DATA ====================== #

class UserAppCallback(app_callback_class):
    def __init__(self) -> None:
        super().__init__()
        self.vehicle = None
        self.last_seen_time = time.time()
        self.last_cmd_time  = time.time()


# ====================== TAKEOFF ====================== #

def arm_and_takeoff(vehicle, target_altitude: float) -> None:
    log("===== TAKEOFF SEQUENCE =====")

    while not vehicle.is_armable:
        log("Waiting for vehicle to become armable...")
        time.sleep(1)

    while vehicle.gps_0.fix_type < 3:
        log(f"Waiting for GPS 3D fix... (current fix_type={vehicle.gps_0.fix_type})")
        time.sleep(1)

    log("GPS 3D lock acquired ✔")

    vehicle.mode = VehicleMode("GUIDED")
    time.sleep(2)

    vehicle.armed = True
    timeout = 10
    while not vehicle.armed and timeout > 0:
        log("Waiting for arming...")
        time.sleep(1)
        timeout -= 1

    if not vehicle.armed:
        log("ERROR: Failed to arm vehicle. Aborting.")
        return

    log("Motors armed ✔")
    vehicle.simple_takeoff(target_altitude)
    log(f"Taking off to {target_altitude} m...")

    while True:
        alt = vehicle.location.global_relative_frame.alt
        log(f"Altitude: {alt:.2f} m")
        if alt >= target_altitude * 0.95:
            log("Reached target altitude ✔")
            break
        time.sleep(1)


# ====================== DIAGNOSTICS ====================== #

def dump_vehicle_state(vehicle) -> None:
    log("===== VEHICLE STATUS =====")
    log(f"Firmware       : {vehicle.version}")
    log(f"Mode           : {vehicle.mode.name}")
    log(f"Armed          : {vehicle.armed}")
    log(f"Is Armable     : {vehicle.is_armable}")
    log(f"System Status  : {vehicle.system_status.state}")
    log(f"GPS Fix        : {vehicle.gps_0.fix_type}")
    log(f"Satellites     : {vehicle.gps_0.satellites_visible}")
    log(f"EKF OK         : {vehicle.ekf_ok}")
    log(f"Battery        : {vehicle.battery.voltage}V  {vehicle.battery.level}%")


# ====================== MAVLINK HELPERS ====================== #

def send_velocity_body(vehicle, vx: float, vy: float, vz: float) -> bool:
    """Send body-frame NED velocity. Returns False on error."""
    try:
        msg = vehicle.message_factory.set_position_target_local_ned_encode(
            0, 0, 0,
            mavutil.mavlink.MAV_FRAME_BODY_NED,
            0b0000111111000111,  # enable vx, vy, vz only
            0, 0, 0,
            float(vx), float(vy), float(vz),
            0, 0, 0,
            0, 0,
        )
        vehicle.send_mavlink(msg)
        vehicle.flush()
        return True
    except Exception as e:
        log(f"MAVLink send error (velocity): {e}")
        return False


def send_yaw_rate(vehicle, yaw_rate_deg: float) -> bool:
    """Command yaw rate in deg/s (positive = clockwise)."""
    try:
        msg = vehicle.message_factory.command_long_encode(
            0, 0,
            mavutil.mavlink.MAV_CMD_CONDITION_YAW,
            0,
            abs(yaw_rate_deg),  # param1: yaw speed deg/s
            0,                  # param2: unused
            1 if yaw_rate_deg >= 0 else -1,  # param3: direction
            1,                  # param4: 1=relative
            0, 0, 0,
        )
        vehicle.send_mavlink(msg)
        vehicle.flush()
        return True
    except Exception as e:
        log(f"MAVLink send error (yaw): {e}")
        return False


# ====================== CALLBACK ====================== #

def app_callback(element, buffer, user_data: UserAppCallback):

    vehicle = user_data.vehicle
    if buffer is None or vehicle is None:
        return

    if vehicle.mode.name != "GUIDED" or not vehicle.armed:
        return

    cx = FRAME_W / 2.0
    cy = FRAME_H / 2.0
    now = time.time()

    roi        = hailo.get_roi_from_buffer(buffer)
    detections = roi.get_objects_typed(hailo.HAILO_DETECTION)

    target_boxes   = []
    confidences    = []

    for det in list(detections):
        if det.get_label() != TARGET_CLASS:
            roi.remove_object(det)
            continue
        target_boxes.append(det.get_bbox())
        confidences.append(det.get_confidence())

    log(f"{TARGET_CLASS.upper()} COUNT: {len(target_boxes)}")

    if target_boxes:
        user_data.last_seen_time = now

        idx  = int(np.argmax(confidences))
        bbox = target_boxes[idx]

        x1 = bbox.xmin() * FRAME_W
        y1 = bbox.ymin() * FRAME_H
        x2 = bbox.xmax() * FRAME_W
        y2 = bbox.ymax() * FRAME_H

        bbox_cx = (x1 + x2) / 2.0
        bbox_cy = (y1 + y2) / 2.0

        err_x = bbox_cx - cx   # +ve = target right of center
        err_y = bbox_cy - cy   # +ve = target below center

        # In MAV_FRAME_BODY_NED: vz +ve = DOWN
        # Camera: err_y +ve means target is lower in frame
        # If camera faces forward-down at 15m, target below = drone too high = descend = vz +ve ✔
        vx = VX_CONST
        vy = float(np.clip(KP_Y  * err_x,  -MAX_VEL_Y, MAX_VEL_Y))
        vz = float(np.clip(KP_Z  * err_y,  -MAX_VEL_Z, MAX_VEL_Z))

        # Yaw toward target to keep nose pointed at it
        yaw_rate = float(np.clip(KP_YAW * err_x, -MAX_YAW_RATE, MAX_YAW_RATE))

        if now - user_data.last_cmd_time >= CMD_DT:
            vel_ok = send_velocity_body(vehicle, vx, vy, vz)
            yaw_ok = send_yaw_rate(vehicle, yaw_rate)
            user_data.last_cmd_time = now

            log(
                f"TRACK | conf={confidences[idx]:.2f} "
                f"| err_x={err_x:.1f}px err_y={err_y:.1f}px "
                f"| vx={vx:.2f} vy={vy:.2f} vz={vz:.2f} yaw_rate={yaw_rate:.1f} "
                f"| vel_ok={vel_ok} yaw_ok={yaw_ok}"
            )

    else:
        if now - user_data.last_seen_time > MAX_LOST_TIME:
            if now - user_data.last_cmd_time >= CMD_DT:
                send_velocity_body(vehicle, 0.0, 0.0, 0.0)
                user_data.last_cmd_time = now
                log("TARGET LOST — Hovering")


# ====================== MAIN ====================== #

def main():
    log("===== STARTING SYSTEM =====")
    connection_string = "udp:0.0.0.0:14550"
    log(f"Connecting to {connection_string}")

    vehicle = connect(connection_string, wait_ready=True)
    log("Vehicle Connected ✔")

    dump_vehicle_state(vehicle)
    arm_and_takeoff(vehicle, TARGET_ALTITUDE)

    user_data = UserAppCallback()
    user_data.vehicle = vehicle

    app = GStreamerDetectionSimpleApp(app_callback, user_data)
    log("Starting Hailo pipeline...")

    try:
        app.run()
    except KeyboardInterrupt:
        log("Interrupted by user.")
    finally:
        log("Closing vehicle connection...")
        vehicle.close()
        _log_fp.flush()
        _log_fp.close()
        log("Shutdown complete.")


if __name__ == "__main__":
    main()
