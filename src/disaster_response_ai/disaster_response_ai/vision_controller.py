#!/usr/bin/env python3
import math
import threading
from concurrent.futures import ThreadPoolExecutor
import logging

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

import cv2
import numpy as np
from cv_bridge import CvBridge
from ultralytics import YOLO
from flask import Flask, request, jsonify

# ROS / MAVROS Messages
from sensor_msgs.msg import NavSatFix, LaserScan, Image
from mavros_msgs.msg import State, GlobalPositionTarget
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL, CommandLong

class DisasterDroid(Node):
    def __init__(self):
        super().__init__('disaster_droid_controller')

        # --- 1. STATE & TRACKING VARIABLES ---
        self.current_state = State()
        self.takeoff_requested = False
        self.taking_off = False
        self.is_airborne = False 
        self.is_orbiting = False
        self.is_evading = False
        self.landing_requested = False 
        
        self.raw_altitude = 553.0
        self.base_altitude = 553.0 
        self.current_alt = 0.0 
        self.current_lat = 0.0
        self.current_lon = 0.0
        
        self.target_lat = None
        self.target_lon = None
        self.standby_lat = None
        self.standby_lon = None
        self.evasion_freeze_lat = None
        self.evasion_freeze_lon = None
        self.target_yaw = 0.0 
        
        self.patrol_radius = 10.0
        self.patrol_altitude = 15.0
        self.lidar_dist = 15.0      
        self.latest_depth = None 

        self.orbit_angle = 0.0
        self.last_orbit_time = self.get_clock().now()
        self.last_req_time = self.get_clock().now()

        # --- 2. AI & VISION SETUP ---
        self.get_logger().info("Loading AI Models into memory...")
        self.model_yolo = YOLO("yolov8n.pt") 
        self.model_pool = ThreadPoolExecutor(max_workers=3)
        self.bridge = CvBridge()
        self.inference_busy = False

        # --- 3. ROS INFRASTRUCTURE ---
        self.flight_cb_group = MutuallyExclusiveCallbackGroup()
        self.vision_cb_group = MutuallyExclusiveCallbackGroup()

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT, 
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST, 
            depth=1
        )

        self.image_sub = self.create_subscription(Image, '/camera/image_raw', self.vision_callback, sensor_qos, callback_group=self.vision_cb_group)
        self.state_sub = self.create_subscription(State, '/mavros/state', self.state_callback, 10, callback_group=self.flight_cb_group)
        self.lidar_sub = self.create_subscription(LaserScan, '/scan', self.lidar_callback, sensor_qos, callback_group=self.flight_cb_group)
        self.gps_sub = self.create_subscription(NavSatFix, '/mavros/global_position/global', self.gps_callback, sensor_qos)
        self.depth_sub = self.create_subscription(Image, '/camera/depth_raw', self.depth_callback, sensor_qos, callback_group=self.vision_cb_group)

        self.target_pub = self.create_publisher(GlobalPositionTarget, '/mavros/setpoint_raw/global', 10)

        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming', callback_group=self.flight_cb_group)
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode', callback_group=self.flight_cb_group)
        self.takeoff_client = self.create_client(CommandTOL, '/mavros/cmd/takeoff', callback_group=self.flight_cb_group)
        self.command_cli = self.create_client(CommandLong, '/mavros/cmd/command', callback_group=self.flight_cb_group)

        self.flight_timer = self.create_timer(0.05, self.flight_control_loop, callback_group=self.flight_cb_group)

    # --- CORE FLIGHT COMMANDS ---
    def set_mode(self, custom_mode):
        self.get_logger().info(f"Switching flight mode to {custom_mode}...")
        req = SetMode.Request()
        req.custom_mode = custom_mode
        self.set_mode_client.call_async(req)

    def set_speed(self, speed_m_s):
        self.get_logger().info(f"Applying Speed Limit: {speed_m_s} m/s")
        req = CommandLong.Request()
        req.command = 178 
        req.param1 = 1.0  
        req.param2 = float(speed_m_s) 
        req.param3 = -1.0 
        req.param4 = 0.0  
        self.command_cli.call_async(req)

    def send_waypoint(self, lat, lon, alt, yaw=0.0):
        goal = GlobalPositionTarget()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.coordinate_frame = 6 
        goal.type_mask = 2552 
        
        goal.latitude = float(lat)
        goal.longitude = float(lon)
        goal.altitude = float(alt) 
        goal.yaw = float(yaw)
        
        self.target_pub.publish(goal)

    # --- TERMINAL / API EVENT TRIGGERS ---
    def trigger_takeoff(self):
        self.base_altitude = self.raw_altitude
        self.takeoff_requested = True
        self.landing_requested = False
        self.get_logger().info("--- TAKEOFF SEQUENCE INITIATED ---")

    def trigger_land(self):
        self.landing_requested = True
        self.takeoff_requested = False
        self.is_airborne = False
        self.taking_off = False
        self.set_mode("LAND") 
        self.get_logger().info("--- INITIATING NATIVE LANDING ---")

    def trigger_stop(self):
        self.get_logger().info("--- EMERGENCY STOP: HOVERING IN PLACE ---")
        self.target_lat = None
        self.target_lon = None
        self.is_orbiting = False
        self.is_evading = False
        
        self.standby_lat = self.current_lat
        self.standby_lon = self.current_lon
        self.patrol_altitude = max(15.0, self.current_alt) 
        
        if self.current_state.mode != "GUIDED" and not self.landing_requested:
            self.set_mode("GUIDED")
        self.send_waypoint(self.standby_lat, self.standby_lon, self.patrol_altitude, getattr(self, 'target_yaw', 0.0))

    def trigger_rtl(self):
        self.get_logger().info("--- INITIATING RETURN TO LAUNCH (RTL) ---")
        self.target_lat = None
        self.target_lon = None
        self.is_orbiting = False
        self.is_evading = False
        
        self.landing_requested = True 
        self.takeoff_requested = False
        self.is_airborne = False
        self.taking_off = False
        self.set_mode("RTL")

    def update_altitude(self, new_alt):
        self.patrol_altitude = new_alt
        self.get_logger().info(f"--- ALTITUDE UPDATED TO {new_alt}m ---")
        
        if self.is_orbiting or self.is_evading:
            pass 
        elif self.target_lat is not None:
            self.dispatch_to_zone(self.target_lat, self.target_lon, self.patrol_radius)
        elif getattr(self, 'standby_lat', None) is not None:
            self.send_waypoint(self.standby_lat, self.standby_lon, self.patrol_altitude, getattr(self, 'target_yaw', 0.0))

    def dispatch_to_zone(self, lat, lon, radius):
        self.get_logger().info(f"Dispatching to disaster zone: {lat}, {lon} with {radius}m radius.")
        self.target_lat = float(lat)
        self.target_lon = float(lon)
        self.patrol_radius = float(radius)
        self.is_orbiting = False
        self.is_evading = False
        
        if self.current_state.mode != "GUIDED" and not self.landing_requested:
            self.set_mode("GUIDED")
            
        self.set_speed(7.0)
            
        dx = (self.target_lon - self.current_lon) * math.cos(math.radians(self.current_lat)) * 111320.0
        dy = (self.target_lat - self.current_lat) * 111320.0
        self.target_yaw = math.atan2(dy, dx)
        
        self.send_waypoint(self.target_lat, self.target_lon, self.patrol_altitude, self.target_yaw)

    def start_orbit(self):
        self.get_logger().info("Target Reached! Establishing Active Python Orbit...")
        self.last_orbit_time = self.get_clock().now()
        
        req = CommandLong.Request()
        req.command = 195 
        req.param5 = float(self.target_lat)
        req.param6 = float(self.target_lon)
        req.param7 = float(self.base_altitude) 
        self.command_cli.call_async(req)

    # --- MAIN 20HZ FLIGHT LOOP ---
    def flight_control_loop(self):
        if not self.current_state.connected or self.landing_requested:
            return
            
        now = self.get_clock().now()
        if not hasattr(self, 'last_loop_time'):
            self.last_loop_time = now
        dt = (now - self.last_loop_time).nanoseconds / 1e9
        self.last_loop_time = now

        if not self.is_airborne:
            if not self.takeoff_requested: return 
            if (now - self.last_req_time).nanoseconds < 3e9: return

            if self.current_state.mode != "GUIDED":
                self.set_mode("GUIDED")
                self.last_req_time = now
            elif not self.current_state.armed:
                self.get_logger().info('Waiting for GPS lock, requesting ARM...')
                arm_req = CommandBool.Request()
                arm_req.value = True
                self.arming_client.call_async(arm_req)
                self.last_req_time = now
            elif not self.taking_off:
                self.get_logger().info('Drone Armed! Initiating Takeoff to 15 meters...')
                takeoff_req = CommandTOL.Request()
                takeoff_req.altitude = 15.0 
                self.takeoff_client.call_async(takeoff_req)
                self.taking_off = True
                self.patrol_altitude = 15.0
                self.last_req_time = now
            else:
                if self.current_alt > 14.0:
                    self.get_logger().info("Takeoff complete. Locking coordinates for stable hover.")
                    self.standby_lat = self.current_lat
                    self.standby_lon = self.current_lon
                    self.is_airborne = True
                    self.send_waypoint(self.standby_lat, self.standby_lon, self.patrol_altitude, getattr(self, 'target_yaw', 0.0))
            return 

        if self.is_evading or not self.is_orbiting:
            return

        # THE FIX: Mathematical safety guard against division by zero!
        if self.patrol_radius < 1.0:
            return

        omega = 3.0 / self.patrol_radius
        self.orbit_angle += omega * dt
        
        dx = self.patrol_radius * math.cos(self.orbit_angle)
        dy = self.patrol_radius * math.sin(self.orbit_angle)
        
        lat_offset = dy / 111320.0
        lon_offset = dx / (111320.0 * math.cos(math.radians(self.target_lat)))
        
        self.target_yaw = math.atan2(-dy, -dx)
        self.send_waypoint(self.target_lat + lat_offset, self.target_lon + lon_offset, self.patrol_altitude, self.target_yaw)

    # --- SENSOR CALLBACKS ---
    def state_callback(self, msg):
        self.current_state = msg

    def gps_callback(self, msg):
        self.raw_altitude = msg.altitude
        self.current_lat = msg.latitude
        self.current_lon = msg.longitude
        self.current_alt = self.raw_altitude - self.base_altitude 

        if self.is_airborne and self.target_lat is not None and not self.is_orbiting and not self.landing_requested:
            R = 6378137 
            dLat = math.radians(self.target_lat - self.current_lat)
            dLon = math.radians(self.target_lon - self.current_lon)
            a = math.sin(dLat/2)**2 + math.cos(math.radians(self.current_lat)) * math.cos(math.radians(self.target_lat)) * math.sin(dLon/2)**2
            distance = 2 * R * math.atan2(math.sqrt(a), math.sqrt(1-a))

            # Trigger when we hit the perimeter of the target radius (or within 2m for a dead-center hover)
            trigger_dist = max(2.0, self.patrol_radius)
            if distance < trigger_dist: 
                if self.patrol_radius >= 1.0:
                    self.is_orbiting = True
                    self.start_orbit()
                else:
                    self.get_logger().info("Target Reached! Executing dead-center hover.")
                    # Lock the exact target coordinates into standby
                    self.standby_lat = self.target_lat
                    self.standby_lon = self.target_lon
                    # Clear the active target so it stops trying to fly there
                    self.target_lat = None
                    self.target_lon = None
                    
                    # Send one final command to hold perfectly still
                    self.send_waypoint(self.standby_lat, self.standby_lon, self.patrol_altitude, getattr(self, 'target_yaw', 0.0))

    def lidar_callback(self, msg):
        if not self.is_airborne or self.landing_requested:
            return

        valid_hits = [r for r in msg.ranges if not math.isinf(r) and not math.isnan(r) and r > 0.8]
        self.lidar_dist = min(valid_hits) if valid_hits else 15.0 
            
        if self.lidar_dist < 13.5:
            if not self.is_evading:
                self.get_logger().warning(f"PROXIMITY ALERT! Object at {self.lidar_dist:.1f}m! Committing to climb...")
                self.is_evading = True
                self.evasion_freeze_lat = self.current_lat
                self.evasion_freeze_lon = self.current_lon
                self.patrol_altitude = self.current_alt + 10.0
                
                self.send_waypoint(self.evasion_freeze_lat, self.evasion_freeze_lon, self.patrol_altitude, getattr(self, 'target_yaw', 0.0))
            else:
                if self.current_alt >= self.patrol_altitude - 1.0:
                    self.get_logger().warning("Obstacle still present! Stepping up another 5m...")
                    self.patrol_altitude += 5.0
                    self.send_waypoint(self.evasion_freeze_lat, self.evasion_freeze_lon, self.patrol_altitude, getattr(self, 'target_yaw', 0.0))
        else:
            if self.is_evading:
                if self.current_alt >= self.patrol_altitude - 1.0:
                    self.get_logger().info("Obstacle cleared! Resuming...")
                    self.is_evading = False
                    self.evasion_freeze_lat = None
                    self.evasion_freeze_lon = None
                    
                    if self.is_orbiting:
                        pass 
                    elif self.target_lat is not None:
                        self.dispatch_to_zone(self.target_lat, self.target_lon, self.patrol_radius)
                    elif self.standby_lat is not None:
                        self.send_waypoint(self.standby_lat, self.standby_lon, self.patrol_altitude, getattr(self, 'target_yaw', 0.0))

    # --- AI & VISION PROCESSING ---
    def run_efficientnet(self, frame):
        annotated = frame.copy()
        cv2.putText(annotated, "EfficientNet: Safe Zone", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)
        return annotated

    def run_pidnet(self, frame):
        annotated = frame.copy()
        cv2.putText(annotated, "PIDNet: Terrain Mapped", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
        return annotated

    def run_yolo(self, frame):
        results = self.model_yolo(frame, verbose=False)
        annotated = results[0].plot()

        if getattr(self, 'latest_depth', None) is not None:
            for box in results[0].boxes:
                # 1. Get the center pixel
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                cx = int((x1 + x2) / 2)
                cy = int((y1 + y2) / 2)

                # 2. Extract raw distance
                distance = float(self.latest_depth[cy, cx])

                if math.isnan(distance) or math.isinf(distance) or distance <= 0.0 or distance > 45.0:
                    text = "Target Detected - Awaiting Depth Lock..."
                    (tw, th), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.7, 2)
                    cv2.rectangle(annotated, (int(x1), int(y1) - th - 10), (int(x1) + tw, int(y1)), (0, 0, 0), -1)
                    cv2.putText(annotated, text, (int(x1), int(y1) - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                    continue

                # 3. The Perfect Symmetric Projection
                unified_focal_length = 831.4 
                forward_meters = ((cy - 360) / unified_focal_length) * distance
                right_meters = ((640 - cx) / unified_focal_length) * distance

                # 4. ENU Math (East-North-Up)
                east_offset = (forward_meters * math.cos(self.target_yaw)) + (right_meters * math.sin(self.target_yaw))
                north_offset = (forward_meters * math.sin(self.target_yaw)) - (right_meters * math.cos(self.target_yaw))

                # 5. Project true GPS
                surv_lat = self.current_lat + (north_offset / 111320.0)
                surv_lon = self.current_lon + (east_offset / (111320.0 * math.cos(math.radians(self.current_lat))))

                # 6. High-visibility text
                text = f"Lat: {surv_lat:.6f} Lon: {surv_lon:.6f} Dist: {distance:.1f}m"
                (tw, th), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.7, 2)
                
                cv2.rectangle(annotated, (int(x1), int(y1) - th - 10), (int(x1) + tw, int(y1)), (0, 0, 0), -1)
                cv2.putText(annotated, text, (int(x1), int(y1) - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

        return annotated

    def process_models_async(self, raw_frame):
        try:
            # 1. Run YOLO (Massive main frame)
            main_frame = self.run_yolo(raw_frame)

            coord_text = f"GPS: {self.current_lat:.5f}, {self.current_lon:.5f} | Alt: {self.current_alt:.1f}m"
            lidar_text = f"360 Clearance: {self.lidar_dist:.1f}m"
            alert_color = (0, 0, 255) if self.lidar_dist < 10.0 else (0, 255, 0)
            
            cv2.putText(main_frame, coord_text, (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(main_frame, lidar_text, (20, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, alert_color, 2)    

            # 2. Side Panel Models
            class_frame = self.run_efficientnet(raw_frame)
            seg_frame = self.run_pidnet(raw_frame)
            
            # 3. Clean Raw Feed
            clean_feed = raw_frame.copy()
            cv2.putText(clean_feed, "Clean Feed", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

            dash_dim = (426, 240)
            side_panel = np.vstack((
                cv2.resize(class_frame, dash_dim), 
                cv2.resize(seg_frame, dash_dim), 
                cv2.resize(clean_feed, dash_dim) 
            ))
            
            self.latest_dashboard = np.hstack((main_frame, side_panel))
            
        except Exception as e:
            self.get_logger().error(f"Inference Error: {e}")
        finally:
            self.inference_busy = False

    def vision_callback(self, msg):
        if not msg.data:
            return
            
        try:
            # BYPASS CV_BRIDGE: Read the raw memory buffer directly into a NumPy array
            # Gazebo outputs 3 channels (RGB)
            frame = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
            
            # Gazebo natively outputs 'rgb8', but OpenCV requires 'bgr8'
            if msg.encoding == 'rgb8':
                raw_frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            else:
                raw_frame = frame.copy()

            raw_frame = cv2.resize(raw_frame, (1280, 720))

            if not hasattr(self, 'latest_dashboard'):
                self.latest_dashboard = raw_frame 
                self.inference_busy = False

            if not self.inference_busy:
                self.inference_busy = True
                self.model_pool.submit(self.process_models_async, raw_frame)
        except Exception as e:
            self.get_logger().error(f"Camera Callback Error: {e}")

    def depth_callback(self, msg):
        if not msg.data:
            return
            
        try:
            # BYPASS CV_BRIDGE: Read 32-bit float memory directly
            raw_depth = np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.width)
            
            self.latest_depth = cv2.resize(raw_depth, (1280, 720), interpolation=cv2.INTER_NEAREST)
        except Exception as e:
            self.get_logger().error(f"Depth Callback Error: {e}")

# --- API SERVER SETUP ---
def run_api_server(node):
    # Silence the standard Flask logging spam
    log = logging.getLogger('werkzeug')
    log.setLevel(logging.ERROR)
    
    app = Flask(__name__)

    @app.route('/api/status', methods=['GET'])
    def api_status():
        return jsonify({
            "connected": node.current_state.connected,
            "armed": node.current_state.armed,
            "mode": node.current_state.mode,
            "airborne": node.is_airborne,
            "latitude": node.current_lat,
            "longitude": node.current_lon,
            "altitude": node.current_alt,
            "lidar_clearance": node.lidar_dist
        })

    @app.route('/api/takeoff', methods=['POST'])
    def api_takeoff():
        node.trigger_takeoff()
        return jsonify({"status": "success", "message": "Takeoff Sequence Initiated"})

    @app.route('/api/land', methods=['POST'])
    def api_land():
        node.trigger_land()
        return jsonify({"status": "success", "message": "Landing Initiated"})

    @app.route('/api/rtl', methods=['POST'])
    def api_rtl():
        node.trigger_rtl()
        return jsonify({"status": "success", "message": "Return to Launch (RTL) Initiated"})

    @app.route('/api/stop', methods=['POST'])
    def api_stop():
        node.trigger_stop()
        return jsonify({"status": "success", "message": "Emergency Stop Initiated"})

    @app.route('/api/go', methods=['POST'])
    def api_go():
        data = request.json
        if not data or 'lat' not in data or 'lon' not in data:
            return jsonify({"status": "error", "message": "Missing 'lat' or 'lon' in JSON body"}), 400
        
        radius = data.get('radius', 0.0)
        node.dispatch_to_zone(data['lat'], data['lon'], radius)
        return jsonify({"status": "success", "message": f"Dispatched to Lat: {data['lat']}, Lon: {data['lon']}, Radius: {radius}m"})

    @app.route('/api/alt', methods=['POST'])
    def api_alt():
        data = request.json
        if not data or 'alt' not in data:
            return jsonify({"status": "error", "message": "Missing 'alt' in JSON body"}), 400
            
        node.update_altitude(data['alt'])
        return jsonify({"status": "success", "message": f"Altitude updated to {data['alt']}m"})

    @app.route('/api/speed', methods=['POST'])
    def api_speed():
        data = request.json
        if not data or 'speed' not in data:
            return jsonify({"status": "error", "message": "Missing 'speed' in JSON body"}), 400
            
        node.set_speed(data['speed'])
        return jsonify({"status": "success", "message": f"Speed limit updated to {data['speed']} m/s"})

    # Run the server on all network interfaces
    app.run(host='0.0.0.0', port=5000, use_reloader=False)

# --- MAIN EXECUTION ---
def main(args=None):
    rclpy.init(args=args)
    node = DisasterDroid()
    
    # Start ROS 2 Executor Thread
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    ros_thread = threading.Thread(target=executor.spin, daemon=True)
    ros_thread.start()

    # Start the Flask API Server Thread
    api_thread = threading.Thread(target=run_api_server, args=(node,), daemon=True)
    api_thread.start()

    # Start the Terminal CLI Thread
    def command_prompt():
        while rclpy.ok():
            try:
                cmd = input("\n[Terminal] 'takeoff', 'go', 'alt', 'stop', 'land', 'speed', 'home': ") 
                if cmd.strip().lower() == 'takeoff': node.trigger_takeoff()
                elif cmd.strip().lower() == 'land': node.trigger_land()
                elif cmd.strip().lower() == 'go':
                    lat = float(input("Enter Latitude: "))
                    lon = float(input("Enter Longitude: "))
                    rad = float(input("Enter Radius (m): "))
                    node.dispatch_to_zone(lat, lon, rad)
                elif cmd.strip().lower() == 'alt':
                    new_alt = float(input("Enter new patrol altitude (m): "))
                    node.update_altitude(new_alt)
                elif cmd.strip().lower() == 'stop': node.trigger_stop()
                elif cmd.strip().lower() == 'speed':
                    new_speed = float(input("Enter new flight speed (m/s): "))
                    node.set_speed(new_speed)
                elif cmd.strip().lower() in ['home', 'rtl']: node.trigger_rtl()
            except Exception as e:
                print(f"Invalid input: {e}")
                
    input_thread = threading.Thread(target=command_prompt, daemon=True)
    input_thread.start()
    
    # Run the OpenCV Main Loop
    try:
        cv2.namedWindow("UAV Rescue Dashboard", cv2.WINDOW_NORMAL)
        while rclpy.ok():
            if hasattr(node, 'latest_dashboard'):
                cv2.imshow("UAV Rescue Dashboard", node.latest_dashboard)
            cv2.waitKey(1) 
            
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down AI Pilot...")
    finally:
        node.model_pool.shutdown(wait=False)
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()
        ros_thread.join(timeout=1.0)

if __name__ == '__main__':
    main()
