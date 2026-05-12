import cv2
from ultralytics import YOLO # install with: pip install ultralytics opencv-python

def main():
    print("Loading YOLO model...")
    model = YOLO('yolov8n.pt')

    video_path = './test_videos/IMG_4119.MOV'
    cap = cv2.VideoCapture(video_path)

    if not cap.isOpened():
        print("Error: Could not open video source.")
        return

    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps = cap.get(cv2.CAP_PROP_FPS)

    output_path = './output/annotated_IMG_4119.mp4'
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    
    writer = cv2.VideoWriter(output_path, fourcc, fps, (width, height))
    print(f"Saving output to: {output_path}")

    print("Starting video processing. Press 'q' to stop early.")

    while True:
        success, frame = cap.read()
        if not success:
            print("Video stream ended or interrupted.")
            break

        # COCO class ID for cat is 15. (16 is dog)
        results = model.predict(source=frame, classes=[15], conf=0.5, verbose=False)

        annotated_frame = results[0].plot()

        writer.write(annotated_frame)

        cv2.imshow("Cat Detector", annotated_frame)

        if cv2.waitKey(1) & 0xFF == ord("q"):
            print("Processing interrupted by user.")
            break

    cap.release()
    writer.release()
    cv2.destroyAllWindows()
    
    print("Processing complete!")

if __name__ == "__main__":
    main()