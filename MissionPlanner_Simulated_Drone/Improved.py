# region imports
# Standard library imports
import time
from threading import Thread, Lock
# Third-party imports
import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst
import cv2
from pymavlink import mavutil
# Local application-specific imports
import hailo
from hailo_apps.hailo_app_python.core.common.buffer_utils import get_caps_from_pad, get_numpy_from_buffer
from hailo_apps.hailo_app_python.core.gstreamer.gstreamer_app import app_callback_class
from hailo_apps.hailo_app_python.apps.detection.detection_pipeline import GStreamerDetectionApp
# endregion imports

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
            'lat': 0.0,
            'lon': 0.0,
            'alt': 0.0,
            'relative_alt': 0.0,
            'vx': 0.0,
            'vy': 0.0,
            'vz': 0.0,
            'heading': 0.0,
            # 'satellites_visible': 0,
            # 'fix_type': 0,
            # 'timestamp': 0
        }
        self.lock = Lock()
        self.running = False
        self.thread = None

        print(f"Connecting to ArduPilot on {self.connection_string}...")
        self.master = mavutil.mavlink_connection(self.connection_string)
        
        print("Waiting for heartbeat...")
        self.master.wait_heartbeat()
        print("Connected! Heartbeat received.")


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

    def connect(self):
        """Establish connection to ArduPilot"""
        try:
            print(f"Connecting to ArduPilot on {self.connection_string}...")
            self.master = mavutil.mavlink_connection(self.connection_string)
            self.master.wait_heartbeat()
            print("Connected! Heartbeat received.")
            
            # Request data streams
            self.master.mav.request_data_stream_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_DATA_STREAM_ALL,
                4,  # 4 Hz update rate
                1   # Start streaming
            )
            return True
        except Exception as e:
            print(f"Failed to connect to ArduPilot: {e}")
            return False
    
    def start(self):
        """Start GPS data collection in background thread"""
        if self.connect():
            self.running = True
            self.thread = Thread(target=self._update_gps_loop, daemon=True)
            self.thread.start()
            return True
        return False
    
    def stop(self):
        """Stop GPS data collection"""
        self.running = False
        if self.thread:
            self.thread.join(timeout=2)
    
    def _update_gps_loop(self):
        """Background thread to continuously update GPS data"""
        while self.running:
            try:
                # Get GPS_RAW_INT message
                msg = self.master.recv_match(type='GPS_RAW_INT', blocking=True, timeout=1)
                if msg:
                    with self.lock:
                        self.gps_data['lat'] = msg.lat / 1e7  # Convert to degrees
                        self.gps_data['lon'] = msg.lon / 1e7
                        self.gps_data['alt'] = msg.alt / 1000.0  # Convert to meters
                        self.gps_data['ground_speed'] = msg.vel / 100.0  # cm/s to m/s
                        self.gps_data['satellites_visible'] = msg.satellites_visible
                        self.gps_data['fix_type'] = msg.fix_type
                        self.gps_data['timestamp'] = time.time()
                
                # Get GLOBAL_POSITION_INT for relative altitude and heading
                msg = self.master.recv_match(type='GLOBAL_POSITION_INT', blocking=False)
                if msg:
                    with self.lock:
                        self.gps_data['relative_alt'] = msg.relative_alt / 1000.0  # mm to meters
                        self.gps_data['heading'] = msg.hdg / 100.0  # centidegrees to degrees
                        
            except Exception as e:
                print(f"Error reading GPS data: {e}")
                time.sleep(0.1)
    
    def get_gps_data(self):
        """Get current GPS data (thread-safe)"""
        with self.lock:
            return self.gps_data.copy()
    
    def get_fix_type_string(self, fix_type):
        """Convert fix type to readable string"""
        fix_types = {
            0: "No GPS",
            1: "No Fix",
            2: "2D Fix",
            3: "3D Fix",
            4: "DGPS",
            5: "RTK Float",
            6: "RTK Fixed"
        }
        return fix_types.get(fix_type, "Unknown")

# -----------------------------------------------------------------------------------------------
# User-defined class to be used in the callback function
# -----------------------------------------------------------------------------------------------
class user_app_callback_class(app_callback_class):
    def __init__(self, gps_handler):
        super().__init__()
        self.gps_handler = gps_handler
        self.detection_log = []  # Store detection events with GPS data
        
    def log_detection_with_gps(self, detection_info):
        """Log detection with current GPS coordinates"""
        gps_data = self.gps_handler.get_gps_data()
        
        log_entry = {
            'frame': self.get_count(),
            'timestamp': time.time(),
            'detection': detection_info,
            'gps': gps_data
        }
        
        self.detection_log.append(log_entry)
        return log_entry
    
    def save_log_to_file(self, filename='detection_log.txt'):
        """Save detection log to file"""
        with open(filename, 'w') as f:
            for entry in self.detection_log:
                f.write(f"Frame: {entry['frame']}\n")
                f.write(f"Time: {time.ctime(entry['timestamp'])}\n")
                f.write(f"Detection: {entry['detection']}\n")
                f.write(f"GPS: Lat={entry['gps']['lat']:.7f}, Lon={entry['gps']['lon']:.7f}, ")
                f.write(f"Alt={entry['gps']['alt']:.1f}m, Heading={entry['gps']['heading']:.1f}°\n")
                f.write("-" * 80 + "\n")

# -----------------------------------------------------------------------------------------------
# User-defined callback function
# -----------------------------------------------------------------------------------------------
def app_callback(pad, info, user_data):
    buffer = info.get_buffer()
    if buffer is None:
        return Gst.PadProbeReturn.OK
    
    user_data.increment()
    string_to_print = f"Frame count: {user_data.get_count()}\n"
    
    # Get GPS data
    gps_data = user_data.gps_handler.get_gps_data()
    fix_type_str = user_data.gps_handler.get_fix_type_string(gps_data['fix_type'])
    
    # Get the caps from the pad
    format, width, height = get_caps_from_pad(pad)
    
    # Get video frame if needed
    frame = None
    if user_data.use_frame and format is not None and width is not None and height is not None:
        frame = get_numpy_from_buffer(buffer, format, width, height)
    
    # Get the detections from the buffer
    roi = hailo.get_roi_from_buffer(buffer)
    detections = roi.get_objects_typed(hailo.HAILO_DETECTION)
    
    # Parse the detections
    detection_count = 0
    for detection in detections:
        label = detection.get_label()
        bbox = detection.get_bbox()
        confidence = detection.get_confidence()
        
        if label == "person":
            # Get track ID
            track_id = 0
            track = detection.get_objects_typed(hailo.HAILO_UNIQUE_ID)
            if len(track) == 1:
                track_id = track[0].get_id()
            
            detection_info = {
                'id': track_id,
                'label': label,
                'confidence': confidence,
                'bbox': (bbox.xmin(), bbox.ymin(), bbox.width(), bbox.height())
            }
            
            # Log detection with GPS data
            log_entry = user_data.log_detection_with_gps(detection_info)
            
            string_to_print += (
                f"Detection: ID: {track_id} Label: {label} Confidence: {confidence:.2f}\n"
                f"  GPS: Lat={gps_data['lat']:.7f}, Lon={gps_data['lon']:.7f}, "
                f"Alt={gps_data['alt']:.1f}m, Rel Alt={gps_data['relative_alt']:.1f}m\n"
                f"  Heading={gps_data['heading']:.1f}°, Speed={gps_data['ground_speed']:.1f}m/s, "
                f"Sats={gps_data['satellites_visible']}, Fix={fix_type_str}\n"
            )
            detection_count += 1
    
    if user_data.use_frame:
        # Display detection count
        cv2.putText(frame, f"Detections: {detection_count}", (10, 30), 
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        
        # Display GPS info
        gps_text = f"GPS: {gps_data['lat']:.6f}, {gps_data['lon']:.6f}"
        cv2.putText(frame, gps_text, (10, 70), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        
        alt_text = f"Alt: {gps_data['relative_alt']:.1f}m | Hdg: {gps_data['heading']:.0f}°"
        cv2.putText(frame, alt_text, (10, 100), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        
        fix_text = f"Fix: {fix_type_str} | Sats: {gps_data['satellites_visible']}"
        cv2.putText(frame, fix_text, (10, 130), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        
        # Convert the frame to BGR
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        user_data.set_frame(frame)
    
    print(string_to_print)
    return Gst.PadProbeReturn.OK

if __name__ == "__main__":
    # Initialize GPS handler
    # Connection options:
    # - UDP: 'udpin:0.0.0.0:14550' (most common for companion computers)
    # - Serial: '/dev/ttyACM0' or '/dev/ttyUSB0'
    # - TCP: 'tcp:127.0.0.1:5760'
    gps_handler = MAVLinkGPSHandler('udp:0.0.0.0:14550')
    gps_handler.start()
    
    # # Start GPS data collection
    # if not gps_handler.start():
    #     print("Failed to start GPS handler. Exiting.")
    #     exit(1)
    
    # try:
    #     # Create an instance of the user app callback class
    #     user_data = user_app_callback_class(gps_handler)
    #     app = GStreamerDetectionApp(app_callback, user_data)
    #     app.run()
    # finally:
    #     # Save detection log and stop GPS handler
    #     user_data.save_log_to_file('detection_log.txt')
    #     print(f"Saved {len(user_data.detection_log)} detection events to detection_log.txt")
    #     gps_handler.stop()