from time import sleep
import subprocess
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import cv2
import sys
import os
import signal
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from std_msgs.msg import Header
from geometry_msgs.msg import TransformStamped

import os
import time
from watchdog.observers import Observer
from watchdog.events import FileSystemEventHandler

class FileHandler(FileSystemEventHandler):
    def on_modified(self, event):
        # Ignore if it's not a file
        if not event.is_directory:
            self.process(event.src_path)

    def process(self, file):
        # Read the file from bottom up to a certain number of lines and get the first line containing '0x0020:'
        with open(file, 'r+') as f:
            lines = f.readlines()
            for line in reversed(lines):
                if '0x0020:' in line:
                    last_line = line
                    break
            else:
                return
            
            # Erase the content from file after reading
            f.truncate(0)
            
        # Process the last line
        process_line(last_line)
            
def process_line(line):
    # Replace it with your own processing logic
    print(line, '\n')

def main():
    path = 'logfile.txt'  # replace it with your file path
    event_handler = FileHandler()

    observer = Observer()
    observer.schedule(event_handler, path, recursive=False)
    observer.start()

    try:
        while True:
            time.sleep(0.1)
    except KeyboardInterrupt:
        observer.stop()

    observer.join()

if __name__ == "__main__":
    main()
