# region imports
import time
from threading import Thread, Lock
from queue import Queue, Full
from dataclasses import dataclass
import json

import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst
import cv2
from pymavlink import mavutil

import hailo
from hailo_apps.hailo_app_python.core.common.buffer_utils import get_caps_from_pad, get_numpy_from_buffer
from hailo_apps.hailo_app_python.core.gstreamer.gstreamer_app import app_callback_class
from hailo_apps.hailo_app_python.apps.detection.detection_pipeline import GStreamerDetectionApp
# endregion imports

# -----------------------------------------------------------------------------------------------
# Data Classes
# -----------------------------------------------------------------------------------------------
@dataclass
class DetectionEvent:
    frame_number: int
    timestamp: float
    detections: list
    gps_data: dict
    frame: any = None


# -----------------------------------------------------------------------------------------------
# MAVLink GPS Handler
# -----------------------------------------------------------------------------------------------
class MAVLinkGPSHandler:
    def __init__(self, connection_string='udp:0.0.0.0:14550'):
        """
        Initialize MAVLink connection
        Args:
            connection_string: Connection string (e.g., 'udp:0.0.0.0:14550', '/dev/ttyACM0', 'tcp:127.0.0.1:5760')
        """
        self.connection_string = connection_string
        self.master = None
        self.gps_data = {
            'timestamp': 0.0,
            'lat': 0.0,
            'lon': 0.0,
            'alt': 0.0,
            'relative_alt': 0.0,
            'vx': 0.0,
            'vy': 0.0,
            'vz': 0.0,
            'heading': 0.0
        }
        self.lock = Lock()
        self.running = False
        self.thread = None

        try:
            print(f"Connecting to ArduPilot on {self.connection_string}...")
            self.master = mavutil.mavlink_connection(self.connection_string)
            
            print("Waiting for heartbeat...")
            self.master.wait_heartbeat()
            print("Connected! Heartbeat received.")

            # Can request specific GPS data rates here if needed
        except Exception as e:
            print(f"Failed to connect to ArduPilot: {e}")


    def wait_for_arm(self):
        """Wait until the drone is armed by reading HEARTBEAT messages."""
        print("Waiting for drone to be armed...")
        armed = False
        while not armed:
            heartbeat = self.master.recv_match(type='HEARTBEAT', blocking=True, timeout=5)
            if heartbeat is None:
                continue

            try:
                armed = (heartbeat.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) != 0
            except Exception:
                # If msg doesn't have base_mode for some reason, skip it
                continue

        print("Drone is armed!")

    def start(self):
        """Start GPS data collection in background thread"""
        self.running = True
        self.thread = Thread(target=self._update_gps_loop, daemon=True)
        self.thread.start()
        return True
    
    def stop(self):
        """Stop GPS data collection"""
        self.running = False
        if self.thread:
            self.thread.join(timeout=2)
    
    def _update_gps_loop(self):
        """Background thread to continuously update GPS data"""
        while self.running:
            try:
                msg = self.master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
                if msg:
                    with self.lock:
                        self.gps_data['timestamp'] = time.time()
                        self.gps_data['lat'] = msg.lat / 1e7  # degrees
                        self.gps_data['lon'] = msg.lon / 1e7  # degrees
                        self.gps_data['alt'] = msg.alt / 1000.0  # mm to meters
                        self.gps_data['relative_alt'] = msg.relative_alt / 1000.0  # mm to meters
                        self.gps_data['vx'] = msg.vx / 100.0  # cm/s to m/s
                        self.gps_data['vy'] = msg.vy / 100.0  # cm/s to m/s
                        self.gps_data['vz'] = msg.vz / 100.0  # cm/s to m/s
                        self.gps_data['heading'] = msg.hdg / 100.0  # centidegrees to degrees

                # print(f"GPS Data: {self.gps_data}")
            except Exception as e:
                print(f"Error reading GPS data: {e}")
                time.sleep(0.1)
    
    def get_gps_data(self):
        """Get current GPS data (thread-safe)"""
        with self.lock:
            return self.gps_data.copy()


# -----------------------------------------------------------------------------------------------
# Detection Processor
# -----------------------------------------------------------------------------------------------
class DetectionProcessor:
    def __init__(self, gps_handler, max_queue_size=100):
        self.gps_handler = gps_handler
        self.queue = Queue(maxsize=max_queue_size)
        self.running = False
        self.thread = None
        self.detection_log = []
        self.stats = {'processed': 0, 'dropped': 0}
        
    def start(self):
        self.running = True
        self.thread = Thread(target=self._process_loop, daemon=True)
        self.thread.start()
    
    def stop(self):
        self.running = False
        if self.thread:
            self.thread.join(timeout=5)
    
    def enqueue(self, frame_number, detections, frame=None):
        """Called from GStreamer callback - must be non-blocking"""
        try:
            event = DetectionEvent(
                frame_number=frame_number,
                timestamp=time.time(),
                detections=detections,
                gps_data=self.gps_handler.get_gps_data(),
                frame=frame.copy() if frame is not None else None
            )
            self.queue.put(event, block=False)
        except Full:
            self.stats['dropped'] += 1
            print(f"Queue full. Dropped frame {frame_number}")
    
    def _process_loop(self):
        while self.running:
            try:
                event = self.queue.get(timeout=1)
                self._process_event(event)
                self.stats['processed'] += 1
                self.queue.task_done()
            except:
                pass
    
    def _process_event(self, event):
        """Heavy processing happens here - not blocking the pipeline"""
        log_entry = {
            'frame': event.frame_number,
            'timestamp': event.timestamp,
            'detections': event.detections,
            'gps': event.gps_data
        }
        self.detection_log.append(log_entry)
        
        # Print summary
        print(f"Frame {event.frame_number}: {len(event.detections)} detections | "
              f"GPS: {event.gps_data['lat']:.6f}, {event.gps_data['lon']:.6f} | "
              f"Alt: {event.gps_data['relative_alt']:.1f}m")
    
    def save_log(self, filename='detection_log.json'):
        with open(filename, 'w') as f:
            json.dump({'stats': self.stats, 'detections': self.detection_log}, f, indent=2)
        print(f"Saved {len(self.detection_log)} detections to {filename}")



# -----------------------------------------------------------------------------------------------
# Class to be used in the callback function
# -----------------------------------------------------------------------------------------------
class user_app_callback_class(app_callback_class):
    def __init__(self, detection_processor):
        super().__init__()
        self.detection_processor = detection_processor

# -----------------------------------------------------------------------------------------------
# Callback function
# -----------------------------------------------------------------------------------------------
def app_callback(pad, info, user_data):
    buffer = info.get_buffer()
    if buffer is None:
        return Gst.PadProbeReturn.OK
    
    user_data.increment()
    frame_number = user_data.get_count()


    format, width, height = get_caps_from_pad(pad)
    
    frame = None
    if user_data.use_frame and format is not None and width is not None and height is not None:
        frame = get_numpy_from_buffer(buffer, format, width, height)
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
    
    # Get the detections from the buffer
    roi = hailo.get_roi_from_buffer(buffer)
    hailo_detections = roi.get_objects_typed(hailo.HAILO_DETECTION)
    
    # Parse the detections
    detections = []
    for detection in hailo_detections:
        label = detection.get_label()
        bbox = detection.get_bbox()
        confidence = detection.get_confidence()

        track_id = 0
        track = detection.get_objects_typed(hailo.HAILO_UNIQUE_ID)
        if track:
            track_id = track[0].get_id()
        
        detections.append({
            'id': track_id,
            'label': label,
            'confidence': confidence,
            'bbox': (bbox.xmin(), bbox.ymin(), bbox.width(), bbox.height())
        })
    
    # Push to queue (non-blocking)
    if detections:
        user_data.processor.enqueue(frame_number, detections, frame)
    


    #     if label == "person":
    #         # Get track ID
    #         track_id = 0
    #         track = detection.get_objects_typed(hailo.HAILO_UNIQUE_ID)
    #         if len(track) == 1:
    #             track_id = track[0].get_id()
            
    #         detection_info = {
    #             'id': track_id,
    #             'label': label,
    #             'confidence': confidence,
    #             'bbox': (bbox.xmin(), bbox.ymin(), bbox.width(), bbox.height())
    #         }
            
    #         # Log detection with GPS data
    #         log_entry = user_data.log_detection_with_gps(detection_info)
            
    #         string_to_print += (
    #             f"Detection: ID: {track_id} Label: {label} Confidence: {confidence:.2f}\n"
    #             f"  GPS: Lat={gps_data['lat']:.7f}, Lon={gps_data['lon']:.7f}, "
    #             f"Alt={gps_data['alt']:.1f}m, Rel Alt={gps_data['relative_alt']:.1f}m\n"
    #             f"  Heading={gps_data['heading']:.1f}°, Speed={gps_data['ground_speed']:.1f}m/s, "
    #             f"Sats={gps_data['satellites_visible']}, Fix={fix_type_str}\n"
    #         )
    #         detection_count += 1
    
    # if user_data.use_frame:
    #     # Display detection count
    #     cv2.putText(frame, f"Detections: {detection_count}", (10, 30), 
    #                 cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        
    #     # Display GPS info
    #     gps_text = f"GPS: {gps_data['lat']:.6f}, {gps_data['lon']:.6f}"
    #     cv2.putText(frame, gps_text, (10, 70), 
    #                 cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        
    #     alt_text = f"Alt: {gps_data['relative_alt']:.1f}m | Hdg: {gps_data['heading']:.0f}°"
    #     cv2.putText(frame, alt_text, (10, 100), 
    #                 cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        
    #     fix_text = f"Fix: {fix_type_str} | Sats: {gps_data['satellites_visible']}"
    #     cv2.putText(frame, fix_text, (10, 130), 
    #                 cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        
    #     # Convert the frame to BGR
    #     frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
    #     user_data.set_frame(frame)
    
    # print(string_to_print)

    return Gst.PadProbeReturn.OK



if __name__ == "__main__":
    # Initialize GPS handler
    # Connection options:
    # - UDP: 'udpin:0.0.0.0:14550' (most common for companion computers)
    # - Serial: '/dev/ttyACM0' or '/dev/ttyUSB0'
    # - TCP: 'tcp:127.0.0.1:5760'
    gps_handler = MAVLinkGPSHandler('udp:0.0.0.0:14550')
    gps_handler.wait_for_arm()

    if not gps_handler.start():
        print("Failed to start GPS handler. Exiting.")
        exit(1)

    processor = DetectionProcessor(gps_handler)
    processor.start()


    try:
        user_data = user_app_callback_class(processor)
        app = GStreamerDetectionApp(app_callback, user_data)
        app.run()
    finally:
        processor.stop()
        gps_handler.stop()
        processor.save_log('detection_log.json')
        print(f"Stats: {processor.stats}")




"""
Things to consider
- Do we want to trigger the code on drone arm? Does arm disengage automatically sometimes?
    - Maybe just once its mission starts?
- Do we want GLOBAL_POSITION_INT or GPS_RAW_INT
    - Do we need to use GPS_RTK?

"""
