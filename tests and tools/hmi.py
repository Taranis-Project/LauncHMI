import tkinter as tk
from tkinter import PhotoImage
from PIL import Image, ImageDraw, ImageFont
import os

class HMIApp:
    def __init__(self, root,title,width,height,color):
        self.root = root
        self.root.title(title)
        self.root.geometry(str(height)+"x"+str(width))
        
        # Background frame (for 3D effect)
        self.background_frame = tk.Frame(self.root, width=width, height=height, bg=color)
        self.background_frame.pack(fill=tk.BOTH, expand=True)
        
        # Load transparent image (PNG)
        current_directory = os.path.dirname(os.path.abspath(__file__))  # Get current file path
        image_path = os.path.join(current_directory, "transparent_button.png")
        self.transparent_img = PhotoImage(file=image_path)
        
        # Create 3D-like buttons
        self.create_transparent_button(100, 50, "Button 1", self.on_button1_click)
        self.create_transparent_button(100, 150, "Button 2", self.on_button2_click)
        self.create_transparent_button(100, 250, "Button 3", self.on_button3_click)

    def create_3d_button(self, x, y, text, command):

        # Add 3D shadow effect (simulate depth with shading)
        shadow = tk.Button(self.background_frame, text=text, font=("Arial", 14), relief=tk.SUNKEN, bd=5, state=tk.DISABLED)
        shadow.place(x=x + 4, y=y + 4)  # Slightly offset the shadow

        """Create a button with 3D-like effects."""
        button = tk.Button(self.background_frame, text=text, font=("Arial", 14), relief=tk.RAISED, bd=5, command=command)
        button.place(x=x, y=y)

    def create_transparent_button(self, x, y, text, command):
        """Create a button with true transparency (using an image)."""
        button = tk.Button(self.background_frame, image=self.transparent_img, text=text, font=("Arial", 14),compound="center", relief="flat", command=command)
        button.place(x=x, y=y)

    def on_button1_click(self):
        print("Button 1 clicked!")
    
    def on_button2_click(self):
        print("Button 2 clicked!")
    
    def on_button3_click(self):
        print("Button 3 clicked!")

# Main window
root = tk.Tk()
app = HMIApp(root, "HMI App", 800, 600, "lightgray")
root.mainloop()