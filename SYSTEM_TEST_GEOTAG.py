# Combined GPS + Hailo Object Detection with LEDs & Auto Stop on Landing
# Created by Peter Russchenberg on 11/10/2025
# Updated 11/19/2025 for detection with precise geotagging, optimized snapshot logic, and optimized program start/stop logic

from pathlib import Path
import os
import cv2
import hailo
import atexit
from datetime import datetime
import pytz
import threading
import time
import csv
import math
import queue
import RPi.GPIO as GPIO
from hailo_apps.hailo_app_python.core.common.buffer_utils import get_caps_from_pad, get_numpy_from_buffer
from hailo_apps.hailo_app_python.core.gstreamer.gstreamer_app import app_callback_class
from hailo_apps.hailo_app_python.apps.detection.detection_pipeline import GStreamerDetectionApp
from gi.repository import Gst
from pymavlink import mavutil
import numpy as np

LED_STARTUP = 23
LED_RUNNING = 24
GPIO.setmode(GPIO.BCM)
GPIO.setup(LED_STARTUP, GPIO.OUT)
GPIO.setup(LED_RUNNING, GPIO.OUT)

for _ in range(5):
    GPIO.output(LED_STARTUP, GPIO.HIGH)
    GPIO.output(LED_RUNNING, GPIO.HIGH)
    time.sleep(0.2)
    GPIO.output(LED_STARTUP, GPIO.LOW)
    GPIO.output(LED_RUNNING, GPIO.LOW)
    time.sleep(0.2)

class LatestGPS:
    def __init__(self):
        self.lock = threading.Lock()
        self.lat = None
        self.lon = None
        self.alt = None
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0

    def update_position(self, msg):
        with self.lock:
            self.lat = msg.lat / 1e7
            self.lon = msg.lon / 1e7
            self.alt = msg.relative_alt / 1000.0

    def update_attitude(self, msg):
        with self.lock:
            self.roll = math.radians(msg.roll)
            self.pitch = math.radians(msg.pitch)
            self.yaw = math.radians(msg.yaw)

    def get(self):
        with self.lock:
            return self.lat, self.lon, self.alt, self.roll, self.pitch, self.yaw
#def gps_listener(gps_obj, connection_string="/dev/ttyAMA0", baud=57600):
def gps_listener(gps_obj, connection_string="/dev/ttyAMA0"):
    master = mavutil.mavlink_connection(connection_string)
    print(f"Connecting to MAVLink at {connection_string}...")
    master.wait_heartbeat()
    print("GPS listener connected")

    master.mav.request_data_stream_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_POSITION,
        1, 1
    )

    while True:
        msg = master.recv_match(type=['GLOBAL_POSITION_INT'], blocking=True, timeout=2)
        if msg:
            gps_obj.update_position(msg)
        att_msg = master.recv_match(type=['ATTITUDE'], blocking=False)
        if att_msg:
            gps_obj.update_attitude(att_msg)

def haversine(lat1, lon1, lat2, lon2):
    R = 6371000
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlambda = math.radians(lon2 - lon1)
    a = math.sin(dphi/2)**2 + math.cos(phi1)*math.cos(phi2)*math.sin(dlambda/2)**2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
    return R * c

def pixel_to_ground(lat, lon, alt, roll, pitch, yaw, x_pixel, y_pixel, W, H, FOV_diag=63):
    x_n = (x_pixel - W/2)/(W/2)
    y_n = (y_pixel - H/2)/(H/2)
    fov_rad = math.radians(FOV_diag)
    diag_pixels = math.sqrt(W**2 + H**2)
    f = 0.5 * diag_pixels / math.tan(fov_rad/2)
    ray_cam = np.array([x_n*f, y_n*f, f])
    ray_cam = ray_cam / np.linalg.norm(ray_cam)
    Rx = np.array([[1,0,0],
                   [0,math.cos(roll),-math.sin(roll)],
                   [0,math.sin(roll), math.cos(roll)]])
    Ry = np.array([[math.cos(pitch),0,math.sin(pitch)],
                   [0,1,0],
                   [-math.sin(pitch),0,math.cos(pitch)]])
    Rz = np.array([[math.cos(yaw),-math.sin(yaw),0],
                   [math.sin(yaw), math.cos(yaw),0],
                   [0,0,1]])
    R = Rz @ Ry @ Rx
    ray_ned = R @ ray_cam
    if ray_ned[2] == 0:
        ray_ned[2] = 1e-6
    t = alt / ray_ned[2]
    north = ray_ned[0]*t
    east = ray_ned[1]*t
    lat_obj = lat + north/111320
    lon_obj = lon + east/(111320*math.cos(math.radians(lat)))
    return lat_obj, lon_obj

class UserAppCallbackWithGPS(app_callback_class):
    TARGET_CLASS = "Cone"
    TARGET_VIDEO_FPS = 20.0

    def __init__(self, gps_obj, mav_master):
        super().__init__()
        self.gps_obj = gps_obj
        self.mav = mav_master
        self.frame_count = 0
        self.queue = queue.Queue(maxsize=50)
        self.tracked_objects = {}
        self.next_object_id = 1
        self.confirmation_frames = 3
        self.redetect_distance = 0.3
        self.flight_started = False
        self.flight_ended = False
        self.total_objects_detected = 0

        usb_root = Path("/media")
        drives = []
        for sub in usb_root.iterdir():
            if sub.is_dir():
                for mount in sub.iterdir():
                    if mount.is_dir() and mount.name != "System Volume Information":
                        drives.append(mount)
        if drives:
            drive_root = drives[0]
            self.base_dir = drive_root / "WEEDSCOUT"
            self.base_dir.mkdir(parents=True, exist_ok=True)
        else:
            self.base_dir = Path.home() / "Desktop" / "WEEDSCOUT"
            self.base_dir.mkdir(parents=True, exist_ok=True)

        now = datetime.now(pytz.timezone("US/Central"))
        date_str = now.strftime("%Y-%m-%d")
        time_str = now.strftime("%H-%M")
        self.date_folder = self.base_dir / date_str
        self.date_folder.mkdir(parents=True, exist_ok=True)
        self.output_dir = self.date_folder / f"{time_str} temp"
        self.output_dir.mkdir(parents=True, exist_ok=True)

        base_csv_name = f"Weedscout_{date_str}"
        csv_file = self.output_dir / f"{base_csv_name}.csv"
        counter = 1
        while csv_file.exists():
            csv_file = self.output_dir / f"{base_csv_name}_{counter}.csv"
            counter += 1
        self.csv_file = csv_file
        with open(self.csv_file, mode='w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(["ID#", "Snapshot #", "Timestamp (CST)", "Lat", "Lon"])

        self.video_writer = None

    def check_takeoff_and_landing(self):
        try:
            heartbeat = self.mav.recv_match(type='HEARTBEAT', blocking=False)
            armed = False
            if heartbeat:
                armed = (heartbeat.base_mode & 128) != 0
            msg = self.mav.recv_match(type='GLOBAL_POSITION_INT', blocking=False)
            alt = None
            if msg:
                alt = msg.relative_alt / 1000.0
            if armed and alt and alt > 3.0 and not self.flight_started:
                self.flight_started = True
            if self.flight_started and alt and alt < 1.5 and not self.flight_ended:
                self.flight_ended = True
                return True
        except:
            pass
        return False

preview_frame = None
preview_running = True
led_running_active = True

def running_led_thread():
    global led_running_active
    while led_running_active:
        GPIO.output(LED_RUNNING, GPIO.HIGH)
        time.sleep(0.5)
        GPIO.output(LED_RUNNING, GPIO.LOW)
        time.sleep(0.5)

def preview_thread():
    global preview_frame, preview_running
    while preview_running:
        if preview_frame is not None:
            cv2.imshow("Preview", cv2.cvtColor(preview_frame, cv2.COLOR_RGB2BGR))
        if cv2.waitKey(1) & 0xFF == 27:
            preview_running = False
            Gst.main_quit()
            break
        time.sleep(0.01)

def app_callback(pad, info, user_data: UserAppCallbackWithGPS):
    buffer = info.get_buffer()
    if buffer is None:
        return Gst.PadProbeReturn.OK

    if user_data.check_takeoff_and_landing():
        global preview_running
        preview_running = False
        Gst.main_quit()
        return Gst.PadProbeReturn.OK

    user_data.frame_count += 1
    try:
        fmt, W, H = get_caps_from_pad(pad)
        frame = get_numpy_from_buffer(buffer, fmt, W, H)
    except:
        return Gst.PadProbeReturn.OK

    lat, lon, alt, roll, pitch, yaw = user_data.gps_obj.get()
    frame_overlay = frame.copy()
    roi = hailo.get_roi_from_buffer(buffer)
    detections = roi.get_objects_typed(hailo.HAILO_DETECTION)

    if detections and alt:
        for det in detections:
            if det.get_label().lower() != user_data.TARGET_CLASS.lower():
                continue
            bbox = det.get_bbox()
            x_min = int(bbox.xmin()*W)
            y_min = int(bbox.ymin()*H)
            x_max = int(bbox.xmax()*W)
            y_max = int(bbox.ymax()*H)
            x_center = (x_min + x_max)/2
            y_center = (y_min + y_max)/2
            obj_lat, obj_lon = pixel_to_ground(lat, lon, alt, roll, pitch, yaw, x_center, y_center, W, H)

            matched_id = None
            for oid, tracked in user_data.tracked_objects.items():
                dist = haversine(obj_lat, obj_lon, tracked['lat'], tracked['lon'])
                if dist < user_data.redetect_distance:
                    matched_id = oid
                    break

            if matched_id:
                tracked = user_data.tracked_objects[matched_id]
                tracked['bbox'] = (x_min, y_min, x_max, y_max)
                tracked['frames_seen'] += 1
                tracked['last_seen'] = time.time()
            else:
                oid = user_data.next_object_id
                user_data.next_object_id += 1
                user_data.tracked_objects[oid] = {
                    'bbox': (x_min, y_min, x_max, y_max),
                    'frames_seen': 1,
                    'confirmed': False,
                    'lat': obj_lat,
                    'lon': obj_lon,
                    'last_seen': time.time()
                }

    snapshot_needed = False
    for oid, tracked in list(user_data.tracked_objects.items()):
        x_min, y_min, x_max, y_max = tracked['bbox']
        if not tracked['confirmed'] and tracked['frames_seen'] >= user_data.confirmation_frames:
            tracked['confirmed'] = True
            snapshot_needed = True
            now_cst = datetime.now(pytz.timezone("US/Central"))
            row = [oid, user_data.frame_count, now_cst.strftime("%H:%M:%S"), tracked['lat'], tracked['lon']]
            try:
                with open(user_data.csv_file, mode='a', newline='') as f:
                    writer = csv.writer(f)
                    writer.writerow(row)
            except:
                pass
        if tracked['confirmed']:
            color = (0,255,0)
            for other_oid, other_tracked in user_data.tracked_objects.items():
                if other_oid != oid:
                    dist = haversine(tracked['lat'], tracked['lon'], other_tracked['lat'], other_tracked['lon'])
                    if dist < user_data.redetect_distance:
                        color = (0,0,255)
                        break
            cv2.rectangle(frame_overlay, (x_min, y_min), (x_max, y_max), color, 2)
            text = f"ID# {oid} | Lat: {tracked['lat']:.6f} Lon: {tracked['lon']:.6f}"
            cv2.putText(frame_overlay, text, (x_min, max(0, y_min-5)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

    if snapshot_needed:
        now_cst = datetime.now(pytz.timezone("US/Central"))
        snapshot_name = f"frame_{user_data.frame_count}_{now_cst.strftime('%H-%M-%S')}.jpg"
        snapshot_file = user_data.output_dir / snapshot_name
        cv2.imwrite(str(snapshot_file), cv2.cvtColor(frame_overlay, cv2.COLOR_RGB2BGR),
                    [int(cv2.IMWRITE_JPEG_QUALITY), 80])

    global preview_frame
    preview_frame = frame_overlay.copy()

    try:
        user_data.queue.put_nowait({
            "frame": frame_overlay,
            "frame_number": user_data.frame_count,
            "timestamp": time.time()
        })
    except queue.Full:
        pass

def writer_thread(user_data):
    last_written_frame_number = -1
    video_start_time = None
    while preview_running or not user_data.queue.empty():
        try:
            item = user_data.queue.get(timeout=0.1)
        except queue.Empty:
            continue
        frame_overlay = item["frame"]
        frame_timestamp = item["timestamp"]
        if user_data.video_writer is None:
            h, w = frame_overlay.shape[:2]
            fourcc = cv2.VideoWriter_fourcc(*'mp4v')
            user_data.video_writer = cv2.VideoWriter(
                str(user_data.output_dir / "video.mp4"),
                fourcc,
                user_data.TARGET_VIDEO_FPS,
                (w, h)
            )
            video_start_time = frame_timestamp
        target_frame_number = int((frame_timestamp - video_start_time)*user_data.TARGET_VIDEO_FPS)
        while last_written_frame_number < target_frame_number:
            user_data.video_writer.write(cv2.cvtColor(frame_overlay, cv2.COLOR_RGB2BGR))
            last_written_frame_number += 1

def cleanup(user_data):
    global led_running_active
    led_running_active = False
    if user_data.video_writer:
        user_data.video_writer.release()
    cv2.destroyAllWindows()
    end_time = time.time() + 20
    while time.time() < end_time:
        GPIO.output(LED_STARTUP, GPIO.HIGH)
        GPIO.output(LED_RUNNING, GPIO.HIGH)
        time.sleep(0.5)
        GPIO.output(LED_STARTUP, GPIO.LOW)
        GPIO.output(LED_RUNNING, GPIO.LOW)
        time.sleep(0.5)
    GPIO.cleanup()

if __name__ == "__main__":
    project_root = Path(__file__).resolve().parent.parent
    os.environ["HAILO_ENV_FILE"] = str(project_root / ".env")

    gps_obj = LatestGPS()
    threading.Thread(target=gps_listener, args=(gps_obj,), daemon=True).start()

    user_data = UserAppCallbackWithGPS(gps_obj, None)
    atexit.register(lambda: cleanup(user_data))
    threading.Thread(target=preview_thread, daemon=True).start()
    threading.Thread(target=writer_thread, args=(user_data,), daemon=True).start()
    threading.Thread(target=running_led_thread, daemon=True).start()

    app = GStreamerDetectionApp(app_callback, user_data)
    app.run()
