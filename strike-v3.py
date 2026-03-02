#!/usr/bin/env python3
"""
v3-strike.py

Hailo Detection + DroneKit Control
+ Full ArduPilot Diagnostics Logging
+ Auto Takeoff to 10m
"""

import os
import time
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


# ====================== LOGGING ====================== #

os.makedirs("logs", exist_ok=True)
LOG_FILE = f"logs/ardupilot_runtime_{datetime.now().strftime('%Y%m%d_%H%M%S')}.txt"
log_fp = open(LOG_FILE, "w", buffering=1)


def log(msg: str) -> None:
    timestamp = datetime.now().strftime("%H:%M:%S")
    line = f"[{timestamp}] {msg}"
    print(line)
    log_fp.write(line + "\n")
    log_fp.flush()


# ====================== USER DATA ====================== #

class UserAppCallback(app_callback_class):
    def __init__(self) -> None:
        super().__init__()
        self.vehicle = None
        self.last_seen_time = time.time()
        self.last_cmd_time = time.time()


# ====================== TAKEOFF ====================== #

def arm_and_takeoff(vehicle, target_altitude: float) -> None:
    log("===== TAKEOFF SEQUENCE =====")

    while not vehicle.is_armable:
        log("Waiting for vehicle to become armable...")
        time.sleep(1)
        
    #while vehicle.gps_0.fix_type < 3:
     #   log("Waiting for GPS 3D fix...")
      #  time.sleep(1)

    log("GPS lock acquired ✔")

    vehicle.mode = VehicleMode("GUIDED")
    time.sleep(2)

    vehicle.armed = True
    time.sleep(2)
    log(f"Armed state: {vehicle.armed}")

    log("Motors armed ✔")

    vehicle.simple_takeoff(target_altitude)
    log(f"Taking off to {target_altitude} meters...")

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
    log(f"Firmware: {vehicle.version}")
    log(f"Mode: {vehicle.mode.name}")
    log(f"Armed: {vehicle.armed}")
    log(f"Is Armable: {vehicle.is_armable}")
    log(f"System Status: {vehicle.system_status.state}")
    log(f"GPS Fix: {vehicle.gps_0.fix_type}")
    log(f"Satellites: {vehicle.gps_0.satellites_visible}")
    log(f"EKF OK: {vehicle.ekf_ok}")
    log(f"Battery: {vehicle.battery.voltage}V {vehicle.battery.level}%")


# ====================== VELOCITY CONTROL ====================== #

def send_velocity_body(vehicle, vx: float, vy: float, vz: float) -> None:
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,
        0,
        0,
        mavutil.mavlink.MAV_FRAME_BODY_NED,
        0b0000111111000111,
        0,
        0,
        0,
        float(vx),
        float(vy),
        float(vz),
        0,
        0,
        0,
        0,
        0,
    )
    vehicle.send_mavlink(msg)
    try:
        vehicle.flush()
    except Exception:
        pass


# ====================== CALLBACK ====================== #

def app_callback(element, buffer, user_data: UserAppCallback):

    vehicle = user_data.vehicle
    if buffer is None or vehicle is None:
        return

    if vehicle.mode.name != "GUIDED" or not vehicle.armed:
        return

    frame_width = 640
    frame_height = 480
    cx = frame_width / 2
    cy = frame_height / 2

    #for 15m altitude
    vx_const = 8.0
    Kp_y = 0.075
    Kp_z = 0.05
    max_vel_y = 3.5
    max_vel_z = 8.0
    dt = 0.10
    MAX_LOST_TIME = 0.5

    now = time.time()

    roi = hailo.get_roi_from_buffer(buffer)
    detections = roi.get_objects_typed(hailo.HAILO_DETECTION)

    person_boxes = []
    confidences = []

    for det in list(detections):
        if det.get_label() != "person":
            roi.remove_object(det)
            continue
        person_boxes.append(det.get_bbox())
        confidences.append(det.get_confidence())
        
    person_count = len(person_boxes)
    log(f"PERSON COUNT: {person_count}")
    
    if person_boxes:
        user_data.last_seen_time = now

        idx = int(np.argmax(confidences))
        bbox = person_boxes[idx]

        x1 = bbox.xmin() * frame_width
        y1 = bbox.ymin() * frame_height
        x2 = bbox.xmax() * frame_width
        y2 = bbox.ymax() * frame_height

        bbox_cx = (x1 + x2) / 2.0
        bbox_cy = (y1 + y2) / 2.0

        err_x = bbox_cx - cx
        err_y = bbox_cy - cy

        vx = vx_const
        vy = float(np.clip(Kp_y * err_x, -max_vel_y, max_vel_y))
        vz = float(np.clip(Kp_z * err_y, -max_vel_z, max_vel_z))

        if now - user_data.last_cmd_time >= dt:
            send_velocity_body(vehicle, vx, vy, vz)
            user_data.last_cmd_time = now

            log(
                f"TRACK | conf={confidences[idx]:.2f} "
                f"| err_x={err_x:.1f}px err_y={err_y:.1f}px "
                f"| vx={vx:.2f} vy={vy:.2f} vz={vz:.2f}"
            )

    else:
        if now - user_data.last_seen_time > MAX_LOST_TIME:
            if now - user_data.last_cmd_time >= dt:
                send_velocity_body(vehicle, 0.0, 0.0, 0.0)
                user_data.last_cmd_time = now
                log("TARGET LOST — Hovering")


# ====================== MAIN ====================== #

def main():

    log("===== STARTING SYSTEM =====")
    #connection_string = "127.0.0.1:14550"
    connection_string = "udp:0.0.0.0:14550"
    log(f"Connecting to {connection_string}")

    vehicle = connect(connection_string, wait_ready=True)
    log("Vehicle Connected ✔")

    dump_vehicle_state(vehicle)

    arm_and_takeoff(vehicle, 15.0)

    user_data = UserAppCallback()
    user_data.vehicle = vehicle

    app = GStreamerDetectionSimpleApp(app_callback, user_data)
    log("Starting Hailo pipeline...")
    app.run()

    vehicle.close()
    log("Shutdown complete.")


if __name__ == "__main__":
    main()
