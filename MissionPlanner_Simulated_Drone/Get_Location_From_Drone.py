from pymavlink import mavutil
import time
from datetime import datetime

"""
This is the version of the program that you can use if you are running 
ArduPilot's Mission Planner locally (on same computer)
"""

class GPSListener:
    def __init__(self, connection_string='tcp:127.0.0.1:5762', sample_rate_hz=1.0):
        print(f"Connecting to SITL at {connection_string}")
        self.master = mavutil.mavlink_connection(connection_string)
        self.sample_interval = 1.0 / sample_rate_hz  # Convert Hz to seconds
        self.last_print_time = 0
        
        print("Waiting for heartbeat...")
        self.master.wait_heartbeat()
        
        # Request GPS data - ensure minimum rate of 1 Hz
        request_rate = max(1, int(sample_rate_hz))
        self.request_gps_rate(request_rate)
        
        print(f"Connected! Listening for GPS data at {request_rate} Hz...")
    
    def request_gps_rate(self, rate_hz):
        """Request GPS data at specific rate from SITL"""
        self.master.mav.request_data_stream_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_POSITION,
            rate_hz,  # Rate in Hz
            1         # Start streaming
        )
    
    def listen(self):
        """Listen for GPS messages with clean rate limiting"""
        
        try:
            while True:
                msg = self.master.recv_match(
                    type=['GLOBAL_POSITION_INT', 'GPS_RAW_INT', 'GPS_STATUS', 'GPS2_RAW'],
                    blocking=True, 
                    timeout=2  # Increased timeout for slower rates
                )
                
                if msg is None:
                    print("No GPS data received...")
                    continue
                
                # Local filtering of messages (we get messages at 1 Hz minimum from SITL, but can process at a different rate if we want)
                current_time = time.time()
                if current_time - self.last_print_time < self.sample_interval:
                    continue  # Skip this message
                
                self.last_print_time = current_time
                
                timestamp = datetime.now().strftime("%H:%M:%S")
                msg_type = msg.get_type()
                
                if msg_type == 'GLOBAL_POSITION_INT':
                    print(f"[{timestamp}] GPS Position:")
                    print(f"  Lat: {msg.lat/1e7:.7f}°")
                    print(f"  Lon: {msg.lon/1e7:.7f}°") 
                    print(f"  Alt: {msg.alt/1000:.1f}m")
                    print(f"  Speed: {msg.vx/100:.1f}, {msg.vy/100:.1f}, {msg.vz/100:.1f} m/s")
                    print(f"  Heading: {msg.hdg/100:.1f}°")
                    print()
                
                elif msg_type == 'GPS_RAW_INT':
                    print(f"[{timestamp}] GPS Raw:")
                    print(f"  Fix: {msg.fix_type} ({msg.satellites_visible} sats)")
                    print(f"  Lat: {msg.lat/1e7:.7f}°")
                    print(f"  Lon: {msg.lon/1e7:.7f}°")
                    print(f"  Alt: {msg.alt/1000:.1f}m")
                    print(f"  Speed: {msg.vel/100:.1f} m/s")
                    print(f"  Course: {msg.cog/100:.1f}°")
                    print()
                
                elif msg_type == 'GPS_STATUS':
                    print(f"[{timestamp}] GPS Status: {msg.satellites_visible} satellites")
                    print()
                
                # NO sleep here - rate limiting is handled above
        
        except KeyboardInterrupt:
            print("\nStopping GPS listener...")
        finally:
            self.master.close()

if __name__ == '__main__':
    listener = GPSListener('tcp:127.0.0.1:5762', sample_rate_hz=0.5)
    listener.listen()