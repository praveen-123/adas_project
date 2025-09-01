# camera_simulator.py
import cv2
import numpy as np
import csv
from datetime import datetime

# Configuration
num_frames = 10
image_width = 640
image_height = 480
log_file = 'camera_log.csv'

# Create camera frames and log data
with open(log_file, 'w', newline='') as csvfile:
    log_writer = csv.writer(csvfile)
    log_writer.writerow(["Frame_ID", "Timestamp", "Status", "File_Path"])
    
    print(f"Generating {num_frames} frames and logging to {log_file}...")
    
    for frame_id in range(num_frames):
        # Create a blank image
        frame = np.zeros((image_height, image_width, 3), dtype=np.uint8)
        
        # Simulate status (OK for even frames, NO_SIGNAL for odd)
        if frame_id % 2 == 0:
            status = "OK"
            color = (0, 255, 0)  # Green
        else:
            status = "NO_SIGNAL"
            color = (0, 0, 255)  # Red
            
        # Add text to the frame
        cv2.putText(frame, f"Frame: {frame_id}", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, color, 2)
        cv2.putText(frame, f"Status: {status}", (50, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
        
        # Save the image
        image_filename = f'frame_{frame_id:03d}.png'
        cv2.imwrite(image_filename, frame)
        
        # Log the event
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
        log_writer.writerow([frame_id, timestamp, status, image_filename])
        
        print(f"Generated and logged: {image_filename}")

print("\nSimulation complete! ✅")
print(f"Generated {num_frames} images.")
print(f"Log file saved as: {log_file}")