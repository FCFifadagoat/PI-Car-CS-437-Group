#!/usr/bin/env python3c

import cv2
from picamera2 import Picamera2
import urllib.request
import os



def download_cascade():

    cascade_dir = "cascades"
    os.makedirs(cascade_dir, exist_ok=True)
    
    # Dog face Haar Cascade
    url = "https://raw.githubusercontent.com/opencv/opencv/master/data/haarcascades/haarcascade_frontalface_default.xml"
    
    # Alt
    dog_url = "https://raw.githubusercontent.com/opencv/opencv/master/data/haarcascades_cuda/haarcascade_fullbody.xml"
    
    cascade_path = os.path.join(cascade_dir, "haarcascade_fullbody.xml")
    
    if not os.path.exists(cascade_path):
        print("Downloading Haar Cascade...")
        urllib.request.urlretrieve(dog_url, cascade_path)
        print(f"Saved to {cascade_path}")
    
    return cascade_path

def main():
    print("Haar Cascade Dog Testing\n")
    
   
    cascade_path = download_cascade()
    
 
    print("Loading HC for dogs...")
    detector = cv2.CascadeClassifier(cascade_path)
    
    if detector.empty():
        print("Error loading cascade!")
        return
    
    # Initialize camera
    print("Setting up camera...")
    camera = Picamera2()
    config = camera.create_preview_configuration(main={"size": (640, 480)})
    camera.configure(config)
    camera.start()
    print("Camera on \n")
    
    print("Ctrl + C to quit\n")
    
    dog_count = 0
    
    try:
        while True:
         
            frame = camera.capture_array()
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            
         
            dog_detections = detector.detectMultiScale(
                gray,
                scaleFactor=1.1,
                minNeighbors=5,
                minSize=(30, 30)
            )
            
            # Draw dog detections
            for (x, y, w, h) in dog_detections:
                dog_count += 1
                cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
                cv2.putText(frame, "DOG", (x, y-10),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                print(f"Dog detected at ({x}, {y})")
            
            
            
            info_text = f"Dogs: {len(dog_detections)}"
            cv2.putText(frame, info_text, (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            
            # Display
            cv2.imshow('Dog Detector', frame)
            
     
            key = cv2.waitKey(30) & 0xFF
            if key == ord('q') or key == 27:  # 'q' or ESC
                print("\nQuitting...")
                break
            
            # Check if window was closed
            if cv2.getWindowProperty('Dog Detector', cv2.WND_PROP_VISIBLE) < 1:
                print("\nWindow closed")
                break
                
    except KeyboardInterrupt:
        print("\nStopped by Ctrl+C")
    except Exception as e:
        print(f"\nError: {e}")
    finally:
        print("Closing windows")
        camera.stop()
        cv2.destroyAllWindows()
        cv2.waitKey(1)  # Additional wait time to avoid crashes on previous window closing attempts
        print("Done")

if __name__ == "__main__":
    main()
