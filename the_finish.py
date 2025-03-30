import cv2
import numpy as np
import tkinter as tk
from tkinter import ttk
from PIL import Image, ImageTk
from collections import deque
import math
import threading
import time
import queue
import serial

class PathFinderApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Autonomous Buggy Navigation System")

        # Initialize UI frames first
        self.setup_ui_frames()
        
        # Create status variables first - ADDED THIS LINE
        self.serial_status_var = tk.StringVar(value="Serial: Not connected")
        self.status_var = tk.StringVar(value="Initializing...")

        # Thread synchronization
        self.frame_lock = threading.Lock()
        self.running = False
        self.frame_queue = queue.Queue(maxsize=2)

        # Camera and marker detection
        self.initialize_camera()
        self.setup_aruco_detector()

        # Navigation variables
        self.initialize_navigation_variables()
        self.load_calibration()

        # Serial communication
        self.initialize_serial()

        # Complete UI setup
        self.setup_ui_controls()
        self.start_stream()
        
        # Bind key press events
        self.root.bind("<KeyPress>", self.on_key_press)

    def setup_ui_frames(self):
        """Initialize all frame containers"""
        self.image_frame = ttk.Frame(self.root)
        self.image_frame.pack(side=tk.LEFT, padx=5, pady=5)
        
        self.control_frame = ttk.Frame(self.root)
        self.control_frame.pack(side=tk.RIGHT, padx=5, pady=5)
        
        self.canvas = tk.Canvas(self.image_frame, width=640, height=480)
        self.canvas.pack()

    def setup_ui_controls(self):
        """Setup all interactive controls"""
        # Control sliders
        self.setup_sliders()
        
        # Action buttons
        ttk.Button(self.control_frame, text="Find Path to Pick-up", 
                  command=self.find_path_to_pickup).grid(row=7, column=0, columnspan=2, pady=5, sticky=tk.EW)
        
        ttk.Button(self.control_frame, text="Simplify Path", 
                  command=self.simplify_path).grid(row=8, column=0, columnspan=2, pady=5, sticky=tk.EW)
        
        ttk.Button(self.control_frame, text="Generate Instructions", 
                  command=self.generate_driving_instructions).grid(row=9, column=0, columnspan=2, pady=5, sticky=tk.EW)
        
        ttk.Button(self.control_frame, text="Send Next Instruction (P)", 
                  command=self.send_next_instruction).grid(row=10, column=0, columnspan=2, pady=5, sticky=tk.EW)

        # Status displays
        self.setup_status_displays()

    def setup_sliders(self):
        """Configure all control sliders"""
        # Gap Threshold
        ttk.Label(self.control_frame, text="Gap Threshold:").grid(row=1, column=0, sticky=tk.W)
        self.gap_slider = ttk.Scale(self.control_frame, from_=1, to=30, value=10,
                                   command=lambda v: self.update_label(self.gap_label, v))
        self.gap_slider.grid(row=1, column=1, sticky=tk.EW)
        self.gap_label = ttk.Label(self.control_frame, text="10 px")
        self.gap_label.grid(row=2, column=1, sticky=tk.E)

        # Simplify Threshold
        ttk.Label(self.control_frame, text="Simplify Threshold:").grid(row=3, column=0, sticky=tk.W)
        self.simplify_slider = ttk.Scale(self.control_frame, from_=1, to=20, value=5,
                                        command=lambda v: self.update_label(self.simplify_label, v))
        self.simplify_slider.grid(row=3, column=1, sticky=tk.EW)
        self.simplify_label = ttk.Label(self.control_frame, text="5 px")
        self.simplify_label.grid(row=4, column=1, sticky=tk.E)

        # Leeway Radius
        ttk.Label(self.control_frame, text="Leeway Radius:").grid(row=5, column=0, sticky=tk.W)
        self.leeway_slider = ttk.Scale(self.control_frame, from_=5, to=50, value=20,
                                      command=lambda v: self.update_label(self.leeway_label, v))
        self.leeway_slider.grid(row=5, column=1, sticky=tk.EW)
        self.leeway_label = ttk.Label(self.control_frame, text="20 px")
        self.leeway_label.grid(row=6, column=1, sticky=tk.E)

    def setup_status_displays(self):
        """Configure status displays"""
        self.status_var = tk.StringVar(value="Initializing...")
        ttk.Label(self.control_frame, textvariable=self.status_var).grid(row=18, column=0, columnspan=2, pady=5)
        
        # Marker status
        self.marker_text = tk.Text(self.control_frame, height=4, width=40, state='disabled')
        self.marker_text.grid(row=14, column=0, columnspan=2, pady=5)
        
        # Instructions
        self.instructions_text = tk.Text(self.control_frame, height=10, width=40, state='normal')
        self.instructions_text.grid(row=16, column=0, columnspan=2, pady=5)
        
        # Serial status
        self.serial_status_var = tk.StringVar(value="Serial: Not connected")
        ttk.Label(self.control_frame, textvariable=self.serial_status_var).grid(row=19, column=0, columnspan=2, pady=5)

    def initialize_serial(self):
        """Initialize serial connection"""
        self.serial_connected = False
        self.ser = None
        self.instruction_queue = deque()
        
        try:
            self.ser = serial.Serial('COM11', 115200, timeout=1)
            time.sleep(2)  # Wait for connection to establish
            self.serial_connected = True
            self.serial_status_var.set("Serial: Connected")
        except Exception as e:
            self.serial_status_var.set(f"Serial error: {str(e)}")

    def initialize_camera(self):
        """Initialize video capture"""
        try:
            self.cap = cv2.VideoCapture("rtsp://10.206.157.80:8554/hack")
            if not self.cap.isOpened():
                raise RuntimeError("Camera not available")
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            self.original_frame = np.zeros((480, 640, 3), dtype=np.uint8)
        except Exception as e:
            self.status_var.set(f"Camera error: {str(e)}")
            self.cap = None

    def setup_aruco_detector(self):
        """Configure ArUco marker detection"""
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
        self.parameters = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.parameters)
        self.marker_positions = {69: None, 13: None, 420: None, 31: None}

    def initialize_navigation_variables(self):
        """Initialize pathfinding variables"""
        self.start_point = None
        self.end_point = None
        self.path = []
        self.simplified_path = []
        self.driving_instructions = []
        self.instruction_strings = []
        self.searching = False
        self.gap_threshold = 10
        self.simplify_threshold = 5
        self.leeway_radius = 20
        self.processed_frame = np.zeros((480, 640, 3), dtype=np.uint8)
        self.line_image = np.zeros((480, 640), dtype=np.uint8)

    def load_calibration(self):
        """Load camera calibration data"""
        try:
            with np.load('camera_params.npz') as data:
                self.mtx, self.dist = data['mtx'], data['dist']
            h, w = 480, 640
            self.newcameramtx, self.roi = cv2.getOptimalNewCameraMatrix(
                self.mtx, self.dist, (w,h), 1, (w,h))
        except Exception as e:
            self.status_var.set(f"Calibration error: {str(e)}")
            self.mtx = self.dist = None

    def start_stream(self):
        """Start video processing threads"""
        if self.running:
            return

        self.running = True
        threading.Thread(target=self.acquire_frames, daemon=True).start()
        threading.Thread(target=self.process_frames, daemon=True).start()
        self.update_ui_loop()

    def acquire_frames(self):
        """Thread for capturing frames"""
        while self.running and self.cap and self.cap.isOpened():
            try:
                ret, frame = self.cap.read()
                if not ret:
                    time.sleep(0.01)
                    continue

                if self.frame_queue.full():
                    self.frame_queue.get_nowait()
                
                self.frame_queue.put(frame, timeout=0.1)
                
            except Exception as e:
                print(f"Capture error: {e}")
                time.sleep(0.1)

    def process_frames(self):
        """Thread for processing frames"""
        while self.running:
            try:
                frame = self.frame_queue.get(timeout=0.5)
                frame = cv2.resize(frame, (640, 480))

                # Undistort if calibration available
                if self.mtx is not None:
                    frame = cv2.undistort(frame, self.mtx, self.dist, None, self.newcameramtx)
                    x, y, w, h = self.roi
                    frame = frame[y:y+h, x:x+w]

                # Process first frame for line detection
                if not hasattr(self, 'map_processed'):
                    self.process_map_frame(frame.copy())
                    self.map_processed = True

                # Detect markers
                self.detect_markers(frame)

                # Update display frame
                with self.frame_lock:
                    self.original_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

            except queue.Empty:
                continue
            except Exception as e:
                print(f"Processing error: {e}")

    def process_map_frame(self, frame):
        """Process the initial frame for line detection"""
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        self.processed_frame = cv2.adaptiveThreshold(
            gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, 
            cv2.THRESH_BINARY, 21, 20)
        self.line_image = cv2.bitwise_not(self.processed_frame)
        self.status_var.set("Map processed - ready for navigation")

    def detect_markers(self, frame):
        """Detect ArUco markers in frame"""
        corners, ids, _ = self.detector.detectMarkers(frame)
        if ids is not None:
            for i, marker_id in enumerate(ids.flatten()):
                if marker_id in self.marker_positions:
                    center = corners[i][0].mean(axis=0)
                    self.marker_positions[marker_id] = (int(center[0]), int(center[1]))
                    if marker_id == 31:  # Buggy
                        self.update_buggy_position((int(center[0]), int(center[1])))

    def update_buggy_position(self, position):
        """Handle buggy position updates"""
        self.start_point = position
        if self.marker_positions[13] is not None:  # If pick-up point exists
            self.find_nearest_line_points()

    def find_nearest_line_points(self):
        """Find nearest points on line for navigation"""
        if self.start_point:
            start_line_point = self.find_nearest_black_pixel(self.start_point)
            if start_line_point:
                self.start_point = start_line_point

        if self.marker_positions[13]:
            end_line_point = self.find_nearest_black_pixel(self.marker_positions[13])
            if end_line_point:
                self.end_point = end_line_point
                self.status_var.set("Ready to find path")

    def find_nearest_black_pixel(self, point):
        """Find nearest black pixel within leeway radius"""
        x, y = point
        for r in range(1, self.leeway_radius + 1):
            for angle in range(0, 360, 5):
                rad = math.radians(angle)
                nx = int(x + r * math.cos(rad))
                ny = int(y + r * math.sin(rad))
                
                if 0 <= nx < self.line_image.shape[1] and 0 <= ny < self.line_image.shape[0]:
                    if self.line_image[ny, nx] == 255:
                        return (nx, ny)
        return None

    def find_path_to_pickup(self):
        """Initiate path finding to pick-up point"""
        if self.marker_positions[31] is None:
            self.status_var.set("Buggy position not detected")
            return
        
        if self.marker_positions[13] is None:
            self.status_var.set("Pick-up point not detected")
            return
        
        self.find_nearest_line_points()
        self.find_path()

    def find_path(self):
        """Find path using BFS with gap jumping"""
        if not self.start_point or not self.end_point:
            self.status_var.set("Start or end point not set")
            return

        self.searching = True
        self.status_var.set("Searching for path...")
        
        # Convert points to (y,x) format for image processing
        start = (self.start_point[1], self.start_point[0])
        end = (self.end_point[1], self.end_point[0])

        self.path = self.bfs_with_gaps(start, end, int(self.gap_slider.get()))
        
        if self.path:
            self.simplified_path = self.path.copy()
            self.status_var.set(f"Path found ({len(self.path)} points)")
        else:
            self.status_var.set("No path found")
        
        self.searching = False
        self.update_display()

    def bfs_with_gaps(self, start, end, max_gap):
        """Breadth-first search with gap jumping"""
        directions = [(-1,-1), (-1,0), (-1,1), (0,-1), (0,1), (1,-1), (1,0), (1,1)]
        visited = set()
        queue = deque([(start, [start])])
        visited.add(start)

        while queue:
            (y, x), path = queue.popleft()

            if (y, x) == end:
                return path

            for dy, dx in directions:
                ny, nx = y + dy, x + dx
                
                if 0 <= ny < self.line_image.shape[0] and 0 <= nx < self.line_image.shape[1]:
                    if (ny, nx) not in visited:
                        if self.line_image[ny, nx] == 255:  # Black pixel
                            visited.add((ny, nx))
                            queue.append(((ny, nx), path + [(ny, nx)]))
                        else:  # White pixel - try to jump gap
                            for gap in range(1, max_gap + 1):
                                nny, nnx = ny + dy * gap, nx + dx * gap
                                if (0 <= nny < self.line_image.shape[0] and 
                                    0 <= nnx < self.line_image.shape[1]):
                                    if (self.line_image[nny, nnx] == 255 and 
                                        (nny, nnx) not in visited):
                                        visited.add((nny, nnx))
                                        new_path = path.copy()
                                        for g in range(1, gap + 1):
                                            new_path.append((ny + dy * g, nx + dx * g))
                                        queue.append(((nny, nnx), new_path))
                                        break
        return None

    def simplify_path(self):
        """Simplify path using Douglas-Peucker algorithm"""
        if not self.path:
            self.status_var.set("No path to simplify")
            return

        path_xy = [(p[1], p[0]) for p in self.path]  # Convert to (x,y)
        self.simplified_path = self.douglas_peucker(
            path_xy, float(self.simplify_slider.get()))
        self.simplified_path = [(p[1], p[0]) for p in self.simplified_path]  # Back to (y,x)
        
        self.status_var.set(f"Simplified from {len(self.path)} to {len(self.simplified_path)} points")
        self.update_display()

    def douglas_peucker(self, point_list, epsilon):
        """Line simplification algorithm"""
        if len(point_list) < 3:
            return point_list

        # Find point with maximum distance
        dmax = 0
        index = 0
        end = len(point_list) - 1

        for i in range(1, end):
            d = self.perpendicular_distance(point_list[i], point_list[0], point_list[end])
            if d > dmax:
                index = i
                dmax = d

        # Recursively simplify
        if dmax > epsilon:
            rec_results1 = self.douglas_peucker(point_list[:index+1], epsilon)
            rec_results2 = self.douglas_peucker(point_list[index:], epsilon)
            return rec_results1[:-1] + rec_results2
        else:
            return [point_list[0], point_list[end]]

    def perpendicular_distance(self, point, line_start, line_end):
        """Calculate distance from point to line segment"""
        if line_start == line_end:
            return math.sqrt((point[0]-line_start[0])**2 + (point[1]-line_start[1])**2)

        l2 = (line_end[0]-line_start[0])**2 + (line_end[1]-line_start[1])**2
        t = max(0, min(1, ((point[0]-line_start[0])*(line_end[0]-line_start[0]) + 
                           (point[1]-line_start[1])*(line_end[1]-line_start[1])) / l2))
        projection = (line_start[0] + t*(line_end[0]-line_start[0]), 
                     line_start[1] + t*(line_end[1]-line_start[1]))

        return math.sqrt((point[0]-projection[0])**2 + (point[1]-projection[1])**2)

    def generate_driving_instructions(self):
        """Convert path to driving instructions"""
        if not self.simplified_path or len(self.simplified_path) < 2:
            self.status_var.set("No simplified path available")
            return

        self.driving_instructions = []
        self.instruction_strings = []
        instructions = "Driving Instructions:\n"
        current_orientation = 0  # Facing right (0 degrees)

        for i in range(len(self.simplified_path) - 1):
            y1, x1 = self.simplified_path[i]
            y2, x2 = self.simplified_path[i+1]
            dx, dy = x2 - x1, y1 - y2  # Note: y increases downward

            desired_orientation = math.degrees(math.atan2(dy, dx))
            angle_diff = (desired_orientation - current_orientation + 180) % 360 - 180
            distance = math.sqrt(dx**2 + dy**2)

            if (angle_diff) > 1:
                self.driving_instructions.append(("rotate", angle_diff))
                instruction_code = f"{int(angle_diff)}2"
                self.instruction_strings.append(instruction_code)
                instructions += f"{instruction_code}\n"

            if (angle_diff) < -1:
                self.driving_instructions.append(("rotate", angle_diff))
                instruction_code = f"{int(abs(angle_diff))}3"
                self.instruction_strings.append(instruction_code)
                instructions += f"{instruction_code}\n"

            if distance > 1:
                self.driving_instructions.append(("move", distance))
                instruction_code = f"{int(distance*3.4)}1"
                self.instruction_strings.append(instruction_code)
                instructions += f"{instruction_code}\n"

            current_orientation = desired_orientation

        # Create a queue from the instructions
        self.instruction_queue = deque(self.instruction_strings)
        
        self.instructions_text.config(state='normal')
        self.instructions_text.delete(1.0, tk.END)
        self.instructions_text.insert(tk.END, instructions)
        self.instructions_text.config(state='disabled')
        self.status_var.set(f"Generated {len(self.driving_instructions)} instructions")

    def on_key_press(self, event):
        """Handle key press events"""
        if event.char.upper() == 'P':
            self.send_next_instruction()

    def send_next_instruction(self):
        """Send the next instruction over serial"""
        if not self.serial_connected:
            self.status_var.set("Serial not connected")
            return
            
        if not self.instruction_queue:
            self.status_var.set("No more instructions to send")
            return
            
        try:
            next_instruction = self.instruction_queue.popleft()
            self.ser.write(f"{next_instruction}".encode())
            self.status_var.set(f"Sent: {next_instruction} ({len(self.instruction_queue)} remaining)")
            
            # Highlight the sent instruction in the text
            self.highlight_sent_instruction(next_instruction)
            
        except Exception as e:
            self.status_var.set(f"Send error: {str(e)}")

    def highlight_sent_instruction(self, instruction):
        """Highlight the instruction that was just sent"""
        self.instructions_text.config(state='normal')
        content = self.instructions_text.get(1.0, tk.END)
        
        # Reset all tags
        self.instructions_text.tag_remove("highlight", "1.0", tk.END)
        
        # Find and highlight the instruction
        start_idx = content.find(instruction)
        if start_idx >= 0:
            line_idx = content[:start_idx].count('\n') + 1
            char_idx = start_idx - content[:start_idx].rfind('\n') - 1
            start_pos = f"{line_idx}.{char_idx}"
            end_pos = f"{line_idx}.{char_idx + len(instruction)}"
            
            self.instructions_text.tag_add("highlight", start_pos, end_pos)
            self.instructions_text.tag_config("highlight", background="yellow")
            
        self.instructions_text.config(state='disabled')

    def update_ui_loop(self):
        """Continuous UI update loop"""
        if not self.running:
            return

        self.update_marker_status()
        self.update_display()
        self.root.after(50, self.update_ui_loop)

    def update_marker_status(self):
        """Update marker status display"""
        status = ""
        for marker_id, pos in self.marker_positions.items():
            name = {69: "Drop-off", 13: "Pick-up", 420: "Start", 31: "Buggy"}.get(marker_id, str(marker_id))
            status += f"{name}: {pos if pos else 'Not detected'}\n"
        
        self.marker_text.config(state='normal')
        self.marker_text.delete(1.0, tk.END)
        self.marker_text.insert(tk.END, status)
        self.marker_text.config(state='disabled')

    def update_display(self):
        """Update the main display canvas"""
        try:
            with self.frame_lock:
                display_frame = self.original_frame.copy()

            # Draw path if available
            if self.path and not self.searching:
                for i in range(len(self.path) - 1):
                    pt1 = (self.path[i][1], self.path[i][0])
                    pt2 = (self.path[i+1][1], self.path[i+1][0])
                    cv2.line(display_frame, pt1, pt2, (255,0,0), 1)

            if self.simplified_path and not self.searching:
                for i in range(len(self.simplified_path) - 1):
                    pt1 = (self.simplified_path[i][1], self.simplified_path[i][0])
                    pt2 = (self.simplified_path[i+1][1], self.simplified_path[i+1][0])
                    cv2.line(display_frame, pt1, pt2, (0,255,0), 2)
                for pt in self.simplified_path:
                    cv2.circle(display_frame, (pt[1], pt[0]), 3, (0,255,255), -1)

            # Convert to PhotoImage
            img = Image.fromarray(display_frame)
            imgtk = ImageTk.PhotoImage(image=img)
            
            # Update canvas
            self.canvas.imgtk = imgtk
            self.canvas.create_image(0, 0, anchor=tk.NW, image=imgtk)
            
        except Exception as e:
            print(f"Display error: {e}")

    def update_label(self, label, value):
        """Update slider label text"""
        label.config(text=f"{int(float(value))} px")

    def cleanup(self):
        """Clean up resources"""
        self.running = False
        if hasattr(self, 'cap') and self.cap and self.cap.isOpened():
            self.cap.release()
        if self.serial_connected and self.ser:
            self.ser.close()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    root = tk.Tk()
    app = PathFinderApp(root)
    
    def on_closing():
        app.cleanup()
        root.destroy()
    
    root.protocol("WM_DELETE_WINDOW", on_closing)
    root.mainloop()
