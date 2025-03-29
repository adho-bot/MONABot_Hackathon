import cv2
import numpy as np
import tkinter as tk
from tkinter import filedialog, ttk
from PIL import Image, ImageTk

class BinarizationApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Image Binarization Tool")
        
        # Initialize variables
        self.image = None
        self.processed_image = None
        self.original_image = None
        self.current_method = "Binary"
        
        # Create UI elements
        self.create_widgets()
        
        # Default parameters
        self.threshold_value = 127
        self.max_value = 255
        self.block_size = 11
        self.c_value = 2
        self.otsu_threshold = False
        self.gaussian_block_size = 11
        self.gaussian_c_value = 2
        self.adaptive_method = cv2.ADAPTIVE_THRESH_MEAN_C
        
    def create_widgets(self):
        # Frame for image display
        self.image_frame = tk.Frame(self.root)
        self.image_frame.pack(side=tk.LEFT, padx=10, pady=10)
        
        # Original image label
        self.original_label = tk.Label(self.image_frame, text="Original Image")
        self.original_label.pack()
        self.original_panel = tk.Label(self.image_frame)
        self.original_panel.pack()
        
        # Processed image label
        self.processed_label = tk.Label(self.image_frame, text="Processed Image")
        self.processed_label.pack()
        self.processed_panel = tk.Label(self.image_frame)
        self.processed_panel.pack()
        
        # Control frame
        self.control_frame = tk.Frame(self.root)
        self.control_frame.pack(side=tk.RIGHT, padx=10, pady=10)
        
        # File open button
        self.open_button = tk.Button(self.control_frame, text="Open Image", command=self.open_image)
        self.open_button.pack(fill=tk.X, pady=5)
        
        # Method selection
        self.method_label = tk.Label(self.control_frame, text="Binarization Method:")
        self.method_label.pack()
        self.method_var = tk.StringVar(value="Binary")
        self.method_menu = ttk.Combobox(self.control_frame, textvariable=self.method_var, 
                                       values=["Binary", "Binary Inverted", "Truncate", "To Zero", 
                                               "To Zero Inverted", "Otsu's", "Adaptive Mean", 
                                               "Adaptive Gaussian"])
        self.method_menu.pack(fill=tk.X, pady=5)
        self.method_menu.bind("<<ComboboxSelected>>", self.method_changed)
        
        # Threshold value slider
        self.threshold_slider = tk.Scale(self.control_frame, from_=0, to=255, orient=tk.HORIZONTAL,
                                       label="Threshold Value", command=self.update_image)
        self.threshold_slider.set(127)
        self.threshold_slider.pack(fill=tk.X, pady=5)
        
        # Max value slider
        self.max_slider = tk.Scale(self.control_frame, from_=0, to=255, orient=tk.HORIZONTAL,
                                  label="Max Value", command=self.update_image)
        self.max_slider.set(255)
        self.max_slider.pack(fill=tk.X, pady=5)
        
        # Block size slider (for adaptive methods)
        self.block_slider = tk.Scale(self.control_frame, from_=3, to=101, orient=tk.HORIZONTAL,
                                    label="Block Size (must be odd)", command=self.update_image)
        self.block_slider.set(11)
        self.block_slider.pack(fill=tk.X, pady=5)
        
        # C value slider (for adaptive methods)
        self.c_slider = tk.Scale(self.control_frame, from_=0, to=20, orient=tk.HORIZONTAL,
                                label="C Value", command=self.update_image)
        self.c_slider.set(2)
        self.c_slider.pack(fill=tk.X, pady=5)
        
        # Otsu's threshold checkbox
        self.otsu_var = tk.BooleanVar()
        self.otsu_check = tk.Checkbutton(self.control_frame, text="Use Otsu's Threshold",
                                        variable=self.otsu_var, command=self.update_image)
        self.otsu_check.pack(fill=tk.X, pady=5)
        
        # Save button
        self.save_button = tk.Button(self.control_frame, text="Save Processed Image", 
                                   command=self.save_image, state=tk.DISABLED)
        self.save_button.pack(fill=tk.X, pady=5)
        
        # Initially hide adaptive method controls
        self.toggle_adaptive_controls(False)
    
    def open_image(self):
        file_path = filedialog.askopenfilename()
        if file_path:
            self.image = cv2.imread(file_path, cv2.IMREAD_GRAYSCALE)
            self.original_image = self.image.copy()
            self.display_images()
            self.save_button.config(state=tk.NORMAL)
            self.update_image()
    
    def method_changed(self, event=None):
        self.current_method = self.method_var.get()
        
        # Show/hide appropriate controls
        if self.current_method in ["Adaptive Mean", "Adaptive Gaussian"]:
            self.toggle_adaptive_controls(True)
            self.threshold_slider.pack_forget()
            self.max_slider.pack_forget()
            self.otsu_check.pack_forget()
        else:
            self.toggle_adaptive_controls(False)
            self.threshold_slider.pack(fill=tk.X, pady=5)
            self.max_slider.pack(fill=tk.X, pady=5)
            self.otsu_check.pack(fill=tk.X, pady=5)
        
        self.update_image()
    
    def toggle_adaptive_controls(self, show):
        if show:
            self.block_slider.pack(fill=tk.X, pady=5)
            self.c_slider.pack(fill=tk.X, pady=5)
        else:
            self.block_slider.pack_forget()
            self.c_slider.pack_forget()
    
    def update_image(self, event=None):
        if self.image is None:
            return
            
        # Get current parameter values
        self.threshold_value = self.threshold_slider.get()
        self.max_value = self.max_slider.get()
        self.block_size = self.block_slider.get()
        self.c_value = self.c_slider.get()
        self.otsu_threshold = self.otsu_var.get()
        
        # Ensure block size is odd and >= 3
        if self.block_size % 2 == 0:
            self.block_size += 1
            self.block_slider.set(self.block_size)
        
        # Apply selected binarization method
        if self.current_method == "Binary":
            _, self.processed_image = cv2.threshold(self.image, self.threshold_value, self.max_value, 
                                                    cv2.THRESH_BINARY | (cv2.THRESH_OTSU if self.otsu_threshold else 0))
        elif self.current_method == "Binary Inverted":
            _, self.processed_image = cv2.threshold(self.image, self.threshold_value, self.max_value, 
                                                    cv2.THRESH_BINARY_INV | (cv2.THRESH_OTSU if self.otsu_threshold else 0))
        elif self.current_method == "Truncate":
            _, self.processed_image = cv2.threshold(self.image, self.threshold_value, self.max_value, 
                                                    cv2.THRESH_TRUNC | (cv2.THRESH_OTSU if self.otsu_threshold else 0))
        elif self.current_method == "To Zero":
            _, self.processed_image = cv2.threshold(self.image, self.threshold_value, self.max_value, 
                                                    cv2.THRESH_TOZERO | (cv2.THRESH_OTSU if self.otsu_threshold else 0))
        elif self.current_method == "To Zero Inverted":
            _, self.processed_image = cv2.threshold(self.image, self.threshold_value, self.max_value, 
                                                    cv2.THRESH_TOZERO_INV | (cv2.THRESH_OTSU if self.otsu_threshold else 0))
        elif self.current_method == "Otsu's":
            _, self.processed_image = cv2.threshold(self.image, 0, self.max_value, 
                                                    cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        elif self.current_method == "Adaptive Mean":
            self.processed_image = cv2.adaptiveThreshold(self.image, self.max_value, 
                                                        cv2.ADAPTIVE_THRESH_MEAN_C, 
                                                        cv2.THRESH_BINARY, self.block_size, self.c_value)
        elif self.current_method == "Adaptive Gaussian":
            self.processed_image = cv2.adaptiveThreshold(self.image, self.max_value, 
                                                       cv2.ADAPTIVE_THRESH_GAUSSIAN_C, 
                                                       cv2.THRESH_BINARY, self.block_size, self.c_value)
        
        self.display_images()
    
    def display_images(self):
        if self.image is not None:
            # Convert original image for display
            original_display = cv2.cvtColor(self.image, cv2.COLOR_GRAY2RGB)
            original_display = Image.fromarray(original_display)
            original_display = ImageTk.PhotoImage(original_display)
            
            self.original_panel.config(image=original_display)
            self.original_panel.image = original_display
            
            # Convert processed image for display
            if self.processed_image is not None:
                processed_display = cv2.cvtColor(self.processed_image, cv2.COLOR_GRAY2RGB)
                processed_display = Image.fromarray(processed_display)
                processed_display = ImageTk.PhotoImage(processed_display)
                
                self.processed_panel.config(image=processed_display)
                self.processed_panel.image = processed_display
    
    def save_image(self):
        if self.processed_image is not None:
            file_path = filedialog.asksaveasfilename(defaultextension=".png", 
                                                    filetypes=[("PNG files", "*.png"), 
                                                               ("JPEG files", "*.jpg"), 
                                                               ("All files", "*.*")])
            if file_path:
                cv2.imwrite(file_path, self.processed_image)

if __name__ == "__main__":
    root = tk.Tk()
    app = BinarizationApp(root)
    root.mainloop()
