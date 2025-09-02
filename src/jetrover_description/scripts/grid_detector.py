import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import numpy as np
import tkinter as tk
from tkinter import ttk
from threading import Thread
import os
import glob
from collections import defaultdict

class GridCellWindow:
    def __init__(self, detector_node):
        self.detector_node = detector_node
        self.root = tk.Tk()
        self.root.title('TicTacToe Cell Detector')
        self.root.geometry('600x500')
        
        # Status label
        self.label = tk.Label(self.root, text='Waiting for detection...', font=('Arial', 14), wraplength=350)
        self.label.pack(padx=20, pady=10)
        
        # Grid state display
        self.grid_frame = tk.Frame(self.root)
        self.grid_frame.pack(pady=10)
        
        self.grid_labels = []
        for i in range(3):
            row = []
            for j in range(3):
                cell_label = tk.Label(self.grid_frame, text='Empty', width=8, height=2, 
                                    relief='raised', borderwidth=2)
                cell_label.grid(row=i, column=j, padx=2, pady=2)
                row.append(cell_label)
            self.grid_labels.append(row)
        
        # Control buttons
        control_frame = tk.Frame(self.root)
        control_frame.pack(pady=10)
        
        tk.Button(control_frame, text='Calibrate from Files', 
                 command=self.calibrate_from_files).pack(side=tk.LEFT, padx=5)
        tk.Button(control_frame, text='Test Detection', 
                 command=self.test_detection).pack(side=tk.LEFT, padx=5)
        tk.Button(control_frame, text='Reset Grid', 
                 command=self.reset_grid).pack(side=tk.LEFT, padx=5)
        
        # Parameters frame
        params_frame = tk.Frame(self.root)
        params_frame.pack(pady=10, fill='x', padx=20)
        
        tk.Label(params_frame, text='Detection Parameters:').pack()
        
        # Threshold sliders
        self.white_thresh_var = tk.IntVar(value=200)
        tk.Label(params_frame, text='White Threshold:').pack()
        tk.Scale(params_frame, from_=150, to=255, orient='horizontal', 
                variable=self.white_thresh_var, command=self.update_params).pack(fill='x')
        
        self.black_thresh_var = tk.IntVar(value=80)
        tk.Label(params_frame, text='Black Threshold:').pack()
        tk.Scale(params_frame, from_=20, to=120, orient='horizontal', 
                variable=self.black_thresh_var, command=self.update_params).pack(fill='x')
        
        # Status text
        self.status_text = tk.Text(params_frame, height=8, width=70)
        self.status_text.pack(pady=5)
        scrollbar = tk.Scrollbar(params_frame)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        self.status_text.config(yscrollcommand=scrollbar.set)
        scrollbar.config(command=self.status_text.yview)
        
        Thread(target=self.root.mainloop, daemon=True).start()

    def update_params(self, val=None):
        if self.detector_node:
            self.detector_node.white_threshold = self.white_thresh_var.get()
            self.detector_node.black_threshold = self.black_thresh_var.get()

    def calibrate_from_files(self):
        if self.detector_node:
            self.detector_node.calibrate_from_saved_images()

    def test_detection(self):
        if self.detector_node:
            self.detector_node.test_detection_on_files()

    def reset_grid(self):
        for i in range(3):
            for j in range(3):
                self.grid_labels[i][j].config(text='Empty', bg='SystemButtonFace', fg='black')

    def update_cell(self, row, col, ball_type, confidence=0.0):
        self.label.config(text=f'{ball_type.title()} ball detected in cell ({row}, {col}) - Confidence: {confidence:.2f}')
        if 0 <= row < 3 and 0 <= col < 3:
            color = 'white' if ball_type == 'white' else 'gray'
            text_color = 'black' if ball_type == 'white' else 'white'
            self.grid_labels[row][col].config(text=f'{ball_type.title()}\n{confidence:.2f}', 
                                            bg=color, fg=text_color)

    def update_status(self, message):
        self.status_text.insert(tk.END, message + '\n')
        self.status_text.see(tk.END)

class GridDetector(Node):
    def __init__(self):
        super().__init__('grid_detector')
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.listener_callback,
            10
        )
        self.bridge = CvBridge()
        self.window = GridCellWindow(self)
        
        # Detection parameters (adjustable via UI)
        self.white_threshold = 200
        self.black_threshold = 80
        
        # Grid detection parameters
        self.grid_corners = None
        self.grid_initialized = False
        self.reference_empty_grid = None
        
        # File paths
        self.image_folder = "/tmp/camera_save_tutorial"
        self.grid_templates = {'empty': [], 'white': [], 'black': []}
        
        # Detection statistics
        self.detection_history = defaultdict(list)
        
        self.get_logger().info('Grid Detector Node Initialized')
        
        # Auto-calibrate if files exist
        if os.path.exists(self.image_folder):
            self.window.update_status(f"Found image folder: {self.image_folder}")
            # Auto-calibrate in a separate thread to avoid blocking
            Thread(target=self.auto_calibrate_delayed, daemon=True).start()

    def auto_calibrate_delayed(self):
        """Delayed auto-calibration to let the UI initialize"""
        import time
        time.sleep(2)  # Wait for UI to be ready
        self.calibrate_from_saved_images()

    def listener_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            if not self.grid_initialized:
                self.initialize_grid_detection(cv_image)
            
            detections = self.detect_balls_in_grid(cv_image)
            
            for detection in detections:
                row, col, ball_type, confidence = detection
                print(f'{ball_type.title()} ball detected in grid cell: ({row}, {col}) - Confidence: {confidence:.2f}')
                self.window.update_cell(row, col, ball_type, confidence)
                
        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')

    def calibrate_from_saved_images(self):
        """Calibrate detection parameters using saved images"""
        try:
            image_files = glob.glob(os.path.join(self.image_folder, "*.png"))
            if not image_files:
                self.window.update_status("No PNG files found in folder")
                return
            
            self.window.update_status(f"Found {len(image_files)} images for calibration")
            
            # Load and analyze multiple images
            grid_detections = []
            sample_images = image_files[:10]  # Use first 10 images for calibration
            
            for img_path in sample_images:
                try:
                    image = cv2.imread(img_path)
                    if image is not None:
                        if not self.grid_initialized:
                            self.initialize_grid_detection(image)
                        
                        # Store reference empty grid if not set
                        if self.reference_empty_grid is None and self.grid_initialized:
                            self.reference_empty_grid = self.extract_grid_region(image)
                            self.window.update_status("Reference empty grid captured")
                        
                        grid_detections.append(image)
                        
                except Exception as e:
                    self.window.update_status(f"Error loading {img_path}: {e}")
            
            self.window.update_status(f"Calibration completed with {len(grid_detections)} images")
            
        except Exception as e:
            self.window.update_status(f"Calibration error: {e}")
            self.get_logger().error(f'Calibration error: {e}')

    def test_detection_on_files(self):
        """Test detection on saved files and show results"""
        try:
            image_files = glob.glob(os.path.join(self.image_folder, "*.png"))
            if not image_files:
                self.window.update_status("No PNG files found for testing")
                return
            
            # Test on latest few images
            test_images = sorted(image_files)[-5:]  # Last 5 images
            
            for img_path in test_images:
                try:
                    image = cv2.imread(img_path)
                    if image is not None:
                        filename = os.path.basename(img_path)
                        detections = self.detect_balls_in_grid(image)
                        
                        if detections:
                            for row, col, ball_type, confidence in detections:
                                self.window.update_status(
                                    f"{filename}: {ball_type} at ({row},{col}) conf={confidence:.2f}")
                        else:
                            self.window.update_status(f"{filename}: No objects detected")
                            
                except Exception as e:
                    self.window.update_status(f"Error testing {img_path}: {e}")
                    
        except Exception as e:
            self.window.update_status(f"Test error: {e}")

    def initialize_grid_detection(self, image):
        """Enhanced grid detection using saved images"""
        try:
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            blurred = cv2.GaussianBlur(gray, (5, 5), 0)
            
            # Use adaptive threshold for better edge detection
            edges = cv2.adaptiveThreshold(blurred, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, 
                                        cv2.THRESH_BINARY, 11, 2)
            
            # Find contours
            contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            # Look for the grid frame (largest rectangular contour)
            best_contour = None
            max_area = 0
            
            for contour in contours:
                area = cv2.contourArea(contour)
                if area > max_area and area > 50000:
                    # Check if it's roughly rectangular
                    epsilon = 0.02 * cv2.arcLength(contour, True)
                    approx = cv2.approxPolyDP(contour, epsilon, True)
                    if len(approx) >= 4:  # At least quadrilateral
                        best_contour = contour
                        max_area = area
            
            if best_contour is not None:
                x, y, w, h = cv2.boundingRect(best_contour)
                padding = 20
                
                self.grid_corners = {
                    'x_start': max(0, x + padding),
                    'y_start': max(0, y + padding),
                    'x_end': min(image.shape[1], x + w - padding),
                    'y_end': min(image.shape[0], y + h - padding),
                    'width': w - 2 * padding,
                    'height': h - 2 * padding
                }
                
                self.grid_initialized = True
                self.window.update_status(f'Grid auto-detected: {w}x{h} at ({x},{y})')
                
            else:
                # Fallback
                h, w = image.shape[:2]
                center_x, center_y = w // 2, h // 2
                grid_size = min(w, h) // 2
                
                self.grid_corners = {
                    'x_start': center_x - grid_size // 2,
                    'y_start': center_y - grid_size // 2,
                    'x_end': center_x + grid_size // 2,
                    'y_end': center_y + grid_size // 2,
                    'width': grid_size,
                    'height': grid_size
                }
                self.grid_initialized = True
                self.window.update_status('Using fallback grid detection')
                
        except Exception as e:
            self.window.update_status(f'Grid initialization error: {e}')

    def extract_grid_region(self, image):
        """Extract just the grid region from the full image"""
        if not self.grid_initialized:
            return None
        
        return image[self.grid_corners['y_start']:self.grid_corners['y_end'],
                    self.grid_corners['x_start']:self.grid_corners['x_end']]

    def detect_balls_in_grid(self, image):
        """Enhanced ball detection with confidence scoring"""
        if not self.grid_initialized:
            return []
        
        detections = []
        
        try:
            grid_roi = self.extract_grid_region(image)
            if grid_roi is None:
                return []
            
            hsv = cv2.cvtColor(grid_roi, cv2.COLOR_BGR2HSV)
            gray = cv2.cvtColor(grid_roi, cv2.COLOR_BGR2GRAY)
            
            # Detect white and black balls with confidence
            white_detections = self.detect_white_balls_enhanced(hsv, gray, grid_roi)
            black_detections = self.detect_black_balls_enhanced(hsv, gray, grid_roi)
            
            detections.extend(white_detections)
            detections.extend(black_detections)
            
        except Exception as e:
            self.window.update_status(f'Ball detection error: {e}')
        
        return detections

    def detect_white_balls_enhanced(self, hsv, gray, grid_roi):
        """Enhanced white ball detection with confidence"""
        detections = []
        
        # Multiple detection methods
        # Method 1: Brightness threshold
        _, mask1 = cv2.threshold(gray, self.white_threshold, 255, cv2.THRESH_BINARY)
        
        # Method 2: HSV white range
        lower_white = np.array([0, 0, 180])
        upper_white = np.array([180, 40, 255])
        mask2 = cv2.inRange(hsv, lower_white, upper_white)
        
        # Combine masks
        mask = cv2.bitwise_or(mask1, mask2)
        
        # Clean up
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        for contour in contours:
            area = cv2.contourArea(contour)
            if 200 < area < 8000:  # Reasonable size range
                # Calculate confidence based on multiple factors
                confidence = self.calculate_ball_confidence(contour, mask, 'white')
                
                if confidence > 0.3:  # Minimum confidence threshold
                    M = cv2.moments(contour)
                    if M['m00'] != 0:
                        cx = int(M['m10'] / M['m00'])
                        cy = int(M['m01'] / M['m00'])
                        row, col = self.map_to_grid_cell(cx, cy)
                        if 0 <= row < 3 and 0 <= col < 3:
                            detections.append((row, col, 'white', confidence))
        
        return detections

    def detect_black_balls_enhanced(self, hsv, gray, grid_roi):
        """Enhanced black ball detection with confidence"""
        detections = []
        
        # Method 1: Dark threshold
        _, mask1 = cv2.threshold(gray, self.black_threshold, 255, cv2.THRESH_BINARY_INV)
        
        # Method 2: HSV black range
        lower_black = np.array([0, 0, 0])
        upper_black = np.array([180, 255, 50])
        mask2 = cv2.inRange(hsv, lower_black, upper_black)
        
        # Combine masks
        mask = cv2.bitwise_and(mask1, mask2)
        
        # Clean up
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        for contour in contours:
            area = cv2.contourArea(contour)
            if 200 < area < 8000:
                confidence = self.calculate_ball_confidence(contour, mask, 'black')
                
                if confidence > 0.3:
                    M = cv2.moments(contour)
                    if M['m00'] != 0:
                        cx = int(M['m10'] / M['m00'])
                        cy = int(M['m01'] / M['m00'])
                        row, col = self.map_to_grid_cell(cx, cy)
                        if 0 <= row < 3 and 0 <= col < 3:
                            detections.append((row, col, 'black', confidence))
        
        return detections

    def calculate_ball_confidence(self, contour, mask, ball_type):
        """Calculate confidence score for detected ball"""
        try:
            area = cv2.contourArea(contour)
            perimeter = cv2.arcLength(contour, True)
            
            # Circularity (0-1, 1 = perfect circle)
            if perimeter > 0:
                circularity = 4 * np.pi * area / (perimeter * perimeter)
            else:
                circularity = 0
            
            # Size score (penalize very small or very large objects)
            ideal_size = 1000  # Adjust based on your ball size
            size_score = 1.0 - abs(area - ideal_size) / ideal_size
            size_score = max(0, size_score)
            
            # Solidity (how filled the shape is)
            hull = cv2.convexHull(contour)
            hull_area = cv2.contourArea(hull)
            solidity = area / hull_area if hull_area > 0 else 0
            
            # Combine factors
            confidence = (circularity * 0.4 + size_score * 0.3 + solidity * 0.3)
            return min(1.0, max(0.0, confidence))
            
        except:
            return 0.0

    def map_to_grid_cell(self, x, y):
        """Map coordinates to grid cell"""
        if not self.grid_initialized:
            return (-1, -1)
        
        cell_width = self.grid_corners['width'] / 3
        cell_height = self.grid_corners['height'] / 3
        
        col = int(x // cell_width)
        row = int(y // cell_height)
        
        return (max(0, min(2, row)), max(0, min(2, col)))


def main(args=None):
    rclpy.init(args=args)
    node = GridDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()