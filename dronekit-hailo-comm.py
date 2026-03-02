# region imports
# Standard library imports

# Third-party imports
import gi
gi.require_version("Gst", "1.0")

from gi.repository import Gst
import hailo

# DroneKit imports
from dronekit import connect, VehicleMode

# Local application-specific imports
from hailo_apps.python.pipeline_apps.detection_simple.detection_simple_pipeline import (
    GStreamerDetectionSimpleApp,
)
from hailo_apps.python.core.common.hailo_logger import get_logger
from hailo_apps.python.core.gstreamer.gstreamer_app import app_callback_class

hailo_logger = get_logger(__name__)

# endregion imports


# User-defined class to be used in the callback function
class user_app_callback_class(app_callback_class):
    def __init__(self):
        super().__init__()
        self.vehicle = None   # DroneKit vehicle object


# User-defined callback function
def app_callback(element, buffer, user_data):
    frame_idx = user_data.get_count()
    hailo_logger.debug("Processing frame %s", frame_idx)

    if buffer is None:
        hailo_logger.warning("Received None buffer at frame=%s", frame_idx)
        return

    string_to_print = f"Frame count: {frame_idx}\n"

    # ----- Vehicle telemetry (READ ONLY) -----
    vehicle = user_data.vehicle
    if vehicle:
        string_to_print += (
            f"Mode: {vehicle.mode.name} | "
            f"Armed: {vehicle.armed} | "
            f"Alt: {vehicle.location.global_relative_frame.alt:.2f} m | "
            f"Heading: {vehicle.heading}\n"
        )

    # ----- Hailo detections -----
    for detection in hailo.get_roi_from_buffer(buffer).get_objects_typed(
        hailo.HAILO_DETECTION
    ):
        string_to_print += (
            f"Detection: {detection.get_label()} "
            f"Confidence: {detection.get_confidence():.2f}\n"
        )

    print(string_to_print)
    return


def main():
    hailo_logger.info("Starting Detection Simple App.")

    user_data = user_app_callback_class()

    # -------- DroneKit vehicle connection --------
    connection_string = "127.0.0.1:14550"
    hailo_logger.info("Connecting to vehicle on %s", connection_string)

    user_data.vehicle = connect(
        connection_string,
        wait_ready=True,
        baud=115200
    )

    hailo_logger.info("Vehicle connected")
    hailo_logger.info("Mode: %s", user_data.vehicle.mode.name)
    hailo_logger.info("Armed: %s", user_data.vehicle.armed)

    # -------- Start Hailo pipeline --------
    app = GStreamerDetectionSimpleApp(app_callback, user_data)
    app.run()


if __name__ == "__main__":
    main()
