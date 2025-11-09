# Combined GPS + Hailo Object Detection with GPS Placeholder
# Created by Peter Russchenberg on 10/15/2025

from pathlib import Path
import os
import cv2
import hailo
import atexit
from datetime import datetime
import threading
import time
import csv
import queue
from hailo_apps.hailo_app_python.core.common.buffer_utils import get_caps_from_pad, get_numpy_from_buffer
from hailo_apps.hailo_app_python.core.gstreamer.gstreamer_app import app_callback_class
from hailo_apps.hailo_app_python.apps.detection.detection_pipeline import GStreamerDetectionApp
from gi.repository import Gst
from pymavlink import mavutil

# Class to store the latest GPS data in a thread-safe manner
class LatestGPS:
    def __init__(self):
        self.lock = threading.Lock()
        self.lat = None
        self.lon = None
        self.alt = None
        self.relative_alt = None
        self.vx = self.vy = self.vz = None
        self.hdg = None

    # Update GPS data safely
    def update(self, msg):
        with self.lock:
            self.lat = msg.lat / 1e7
            self.lon = msg.lon / 1e7
            self.alt = msg.alt / 1000
            self.relative_alt = msg.relative_alt / 1000
            self.vx = msg.vx / 100
            self.vy = msg.vy / 100
            self.vz = msg.vz / 100
            self.hdg = msg.hdg / 100

    # Get current GPS values safely
    def get(self):
        with self.lock:
            return (self.lat, self.lon, self.alt, self.relative_alt, self.vx, self.vy, self.vz, self.hdg)


# Thread that listens for GPS updates via MAVLink
def gps_listener(gps_obj, connection_string="udp:0.0.0.0:14550"):
    master = mavutil.mavlink_connection(connection_string)
    print(f"Connecting to MAVLink at {connection_string}...")
    master.wait_heartbeat()
    print("GPS listener connected")
    
    # Request GPS position data at 1 Hz
    master.mav.request_data_stream_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_POSITION,
        1, 
        1
    )
    
    # Continuously receive GPS messages
    while True:
        msg = master.recv_match(type=['GLOBAL_POSITION_INT'], blocking=True, timeout=2)
        if msg is None:
            continue
        gps_obj.update(msg)


# Callback class for Hailo object detection pipeline
class UserAppCallbackWithGPS(app_callback_class):
    TARGET_CLASS = "person"  # Object class to detect
    SNAPSHOT_INTERVAL_SEC = 1.0  # Minimum time between snapshots
    TARGET_VIDEO_FPS = 20.0  # Video frame rate

    def __init__(self, gps_obj: LatestGPS):
        super().__init__()
        self.frame_count = 0
        self.total_objects_detected = 0
        self.csv_rows = []
        self.queue = queue.Queue(maxsize=100)
        self.gps_obj = gps_obj

        # Setup storage folders for snapshots and video
        usb_root = Path("/media/weedscout")
        drives = [d for d in usb_root.iterdir() if d.is_dir() and d.name != "System Volume Information"]
        if drives:
            drive_root = drives[0]
            self.base_dir = drive_root / "WeedScout_Results"
            self.base_dir.mkdir(parents=True, exist_ok=True)
            print(f"USB drive detected: {drive_root}")
        else:
            print("No writable USB drive detected, saving to Desktop")
            self.base_dir = Path.home() / "Desktop" / "WeedScout_Results"
            self.base_dir.mkdir(parents=True, exist_ok=True)

        # Create date-specific folder
        now = datetime.now().astimezone()
        date_str = now.strftime("%Y-%m-%d")
        time_str = now.strftime("%H-%M")
        self.date_folder = self.base_dir / date_str
        self.date_folder.mkdir(parents=True, exist_ok=True)

        # Temporary folder for this run
        self.output_dir = self.date_folder / f"{time_str}-temp"
        self.output_dir.mkdir(parents=True, exist_ok=True)

        # CSV file for logging snapshot info
        base_csv_name = f"WeedScout_{date_str}"
        csv_file = self.output_dir / f"{base_csv_name}.csv"
        counter = 1
        while csv_file.exists():
            csv_file = self.output_dir / f"{base_csv_name}_{counter}.csv"
            counter += 1
        self.csv_file = csv_file

        # Write CSV header
        with open(self.csv_file, mode='w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(["Snapshot #", "Timestamp", "Objects Detected", "Confidence (%)",
                             "Snapshot Filename", "Lat", "Lon", "Alt"])

        self.video_writer = None




# Main callback called by the detection pipeline for each frame
def app_callback(pad, info, user_data: UserAppCallbackWithGPS):
    buffer = info.get_buffer()
    if buffer is None:
        return Gst.PadProbeReturn.OK

    user_data.frame_count += 1
    try:
        fmt, w, h = get_caps_from_pad(pad)
        frame = get_numpy_from_buffer(buffer, fmt, w, h)
    except Exception as e:
        print(f"Failed to read frame: {e}")
        return Gst.PadProbeReturn.OK

    frame_overlay = frame.copy()
    roi = hailo.get_roi_from_buffer(buffer)
    detections = roi.get_objects_typed(hailo.HAILO_DETECTION)
    detected_count = 0
    max_conf = 0.0

    # Draw bounding boxes and labels for detected objects
    if detections:
        for det in detections:
            if det.get_label().lower() != user_data.TARGET_CLASS.lower():
                continue
            detected_count += 1
            conf = det.get_confidence() * 100.0
            max_conf = max(max_conf, conf)
            bbox = det.get_bbox()
            x_min = int(bbox.xmin() * frame.shape[1])
            y_min = int(bbox.ymin() * frame.shape[0])
            x_max = int(bbox.xmax() * frame.shape[1])
            y_max = int(bbox.ymax() * frame.shape[0])
            cv2.rectangle(frame_overlay, (x_min, y_min), (x_max, y_max), (0, 255, 0), 2)
            cv2.putText(frame_overlay, f"{det.get_label()}:{conf:.1f}%", (x_min, max(0, y_min - 5)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

    user_data.total_objects_detected += detected_count

    # Overlay detection info and current time
    current_time_str = datetime.now().strftime("%H:%M:%S")
    overlay_text = f"Objects detected: {detected_count} | Time: {current_time_str}"
    cv2.putText(frame_overlay, overlay_text, (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

    # Overlay GPS coordinates or placeholder
    lat, lon, alt, _, _, _, _, _ = user_data.gps_obj.get()
    if lat is not None:
        gps_text = f"Lat: {lat:.7f}, Lon: {lon:.7f}, Alt: {alt:.1f}m"
    else:
        gps_text = "GPS: N/A"
    cv2.putText(frame_overlay, gps_text, (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)


    # Push frame and metadata to queue for video/snapshot saving
    try:
        user_data.queue.put_nowait({
            "frame": frame_overlay,
            "detected_count": detected_count,
            "max_conf": max_conf,
            "frame_number": user_data.frame_count,
            "timestamp": time.time(),
            "lat": lat,
            "lon": lon,
            "alt": alt
        })
    except queue.Full:
        pass

    return Gst.PadProbeReturn.OK


# Thread for writing video frames and snapshots
def writer_thread(user_data: UserAppCallbackWithGPS):
    last_snapshot_time = 0.0
    video_start_time = None
    last_written_frame_number = -1
    snapshot_counter = 1

    while True:
        try:
            item = user_data.queue.get(timeout=0.1)
        except queue.Empty:
            continue

        frame_overlay = item["frame"]
        detected_count = item["detected_count"]
        max_conf = item["max_conf"]
        frame_timestamp = item["timestamp"]
        lat, lon, alt = item["lat"], item["lon"], item["alt"]

        # Initialize video writer
        # if user_data.video_writer is None:
        #     h, w = frame_overlay.shape[:2]
        #     fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        #     user_data.video_writer = cv2.VideoWriter(
        #         str(user_data.output_dir / "video.mp4"),
        #         fourcc,
        #         user_data.TARGET_VIDEO_FPS,
        #         (w, h)
        #     )
        #     video_start_time = frame_timestamp
        if user_data.video_writer is None:
            h, w = frame_overlay.shape[:2]
            fourcc = cv2.VideoWriter_fourcc(*'MJPG')  # Change to MJPEG
            user_data.video_writer = cv2.VideoWriter(
                str(user_data.output_dir / "video.avi"),  # Change file extension to .avi
                fourcc,
                user_data.TARGET_VIDEO_FPS,
                (w, h)
            )
            video_start_time = frame_timestamp


        # Write video frames
        # target_frame_number = int((frame_timestamp - video_start_time) * user_data.TARGET_VIDEO_FPS)
        # while last_written_frame_number < target_frame_number:
        #     user_data.video_writer.write(cv2.cvtColor(frame_overlay, cv2.COLOR_RGB2BGR))
        #     last_written_frame_number += 1
        user_data.video_writer.write(cv2.cvtColor(frame_overlay, cv2.COLOR_RGB2BGR))



        # Save snapshot if interval passed and object detected
        if detected_count > 0 and (frame_timestamp - last_snapshot_time >= user_data.SNAPSHOT_INTERVAL_SEC):
            timestamp_str = datetime.now().strftime("%H-%M-%S")
            snapshot_name = f"{snapshot_counter:03d}_{timestamp_str}.jpg"
            snapshot_file = user_data.output_dir / snapshot_name
            snapshot_counter += 1

            cv2.imwrite(str(snapshot_file), cv2.cvtColor(frame_overlay, cv2.COLOR_RGB2BGR),
                        [int(cv2.IMWRITE_JPEG_QUALITY), 80])
            last_snapshot_time = frame_timestamp

            # Save CSV row with GPS
            row = [
                snapshot_counter - 1,
                datetime.now().strftime("%H:%M:%S"),
                detected_count,
                round(max_conf, 1),
                snapshot_file.name,
                f"{lat:.7f}" if lat else "N/A",
                f"{lon:.7f}" if lon else "N/A",
                f"{alt:.1f}" if alt else "N/A"
            ]
            user_data.csv_rows.append(row)
            try:
                with open(user_data.csv_file, mode='a', newline='') as f:
                    writer = csv.writer(f)
                    writer.writerow(row)
            except Exception as e:
                print(f"Failed to write CSV row: {e}")


# Cleanup resources and rename folder
def cleanup(user_data):
    if user_data.video_writer:
        user_data.video_writer.release()
        print("VideoWriter released")

    cv2.destroyAllWindows()

    now = datetime.now()
    run_time_str = now.strftime("%-I-%M%p").lower()
    final_folder = f"{run_time_str}_{user_data.total_objects_detected}objects"
    final_path = user_data.output_dir.parent / final_folder
    try:
        if final_path != user_data.output_dir:
            user_data.output_dir.rename(final_path)
        print(f"Run complete. Saved to: {final_path}")
    except Exception as e:
        print(f"Error renaming folder: {e}")


# Main execution
if __name__ == "__main__":
    project_root = Path(__file__).resolve().parent.parent
    os.environ["HAILO_ENV_FILE"] = str(project_root / ".env")

    # Start GPS listener thread
    gps_obj = LatestGPS()
    threading.Thread(target=gps_listener, args=(gps_obj,), daemon=True).start()

    # Initialize detection callback and threads
    user_data = UserAppCallbackWithGPS(gps_obj)
    atexit.register(lambda: cleanup(user_data))
    threading.Thread(target=writer_thread, args=(user_data,), daemon=True).start()

    # Run the Hailo detection pipeline
    app = GStreamerDetectionApp(app_callback, user_data)
    app.run()
