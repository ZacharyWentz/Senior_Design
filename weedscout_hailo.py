Created by Peter Russchenberg on 10/15/2025
from pathlib import Path
import os
import cv2
import hailo
import atexit
from datetime import datetime
import threading
import time
import csv
import queue
from hailo_apps.hailo_app_python.core.common.buffer_utils import get_caps_from_pad, get_numpy_from_buffer
from hailo_apps.hailo_app_python.core.gstreamer.gstreamer_app import app_callback_class
from hailo_apps.hailo_app_python.apps.detection.detection_pipeline import GStreamerDetectionApp
from gi.repository import Gst


class UserAppCallback(app_callback_class):
    # target object to detect and snapshot settings
    TARGET_CLASS = "tv"
    SNAPSHOT_INTERVAL_SEC = 1.0
    TARGET_VIDEO_FPS = 20.0

    def __init__(self):
        super().__init__()
        self.frame_count = 0
        self.total_objects_detected = 0
        self.csv_rows = []

        self.queue = queue.Queue(maxsize=50)  # queue for frames and detection info

        # setup storage folders
        usb_root = Path("/media/weedscout")
        drives = [d for d in usb_root.iterdir() if d.is_dir() and d.name != "System Volume Information"]
        if drives:
            drive_root = drives[0]
            self.base_dir = drive_root / "WEEDSCOUT_PHOTOS"
            self.base_dir.mkdir(parents=True, exist_ok=True)
            print(f"USB drive detected: {drive_root}")
        else:
            print("No writable USB drive detected, saving to Desktop")
            self.base_dir = Path.home() / "Desktop" / "WEEDSCOUT_PHOTOS"
            self.base_dir.mkdir(parents=True, exist_ok=True)

        # create date folder
        now = datetime.now().astimezone()
        date_str = now.strftime("%Y-%m-%d")
        time_str = now.strftime("%H-%M")
        self.date_folder = self.base_dir / date_str
        self.date_folder.mkdir(parents=True, exist_ok=True)

        # temporary folder for this run, will rename at the end
        self.output_dir = self.date_folder / f"{time_str}-temp"
        self.output_dir.mkdir(parents=True, exist_ok=True)

        # csv file inside the run folder with auto-increment if needed
        base_csv_name = f"Weedscout_{date_str}"
        csv_file = self.output_dir / f"{base_csv_name}.csv"
        counter = 1
        while csv_file.exists():
            csv_file = self.output_dir / f"{base_csv_name}_{counter}.csv"
            counter += 1
        self.csv_file = csv_file

        # write csv header
        with open(self.csv_file, mode='w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(["Snapshot #", "Timestamp", "Objects Detected", "Confidence (%)", "Snapshot Filename"])

        self.video_writer = None


# shared variables for preview window
preview_frame = None
preview_running = True


def preview_thread():
    # thread for showing the live preview
    global preview_frame, preview_running
    while preview_running:
        if preview_frame is not None:
            cv2.imshow("Preview", cv2.cvtColor(preview_frame, cv2.COLOR_RGB2BGR))
        if cv2.waitKey(1) & 0xFF == 27:
            preview_running = False
            Gst.main_quit()
            break
        time.sleep(0.01)


def app_callback(pad, info, user_data):
    # main callback called by the detection pipeline for each frame
    buffer = info.get_buffer()
    if buffer is None:
        return Gst.PadProbeReturn.OK

    user_data.frame_count += 1
    try:
        fmt, w, h = get_caps_from_pad(pad)
        frame = get_numpy_from_buffer(buffer, fmt, w, h)
    except Exception as e:
        print(f"Failed to read frame: {e}")
        return Gst.PadProbeReturn.OK

    frame_overlay = frame.copy()
    roi = hailo.get_roi_from_buffer(buffer)
    detections = roi.get_objects_typed(hailo.HAILO_DETECTION)

    detected_count = 0
    max_conf = 0.0

    # draw bounding boxes for detected objects
    if detections:
        for det in detections:
            if det.get_label().lower() != user_data.TARGET_CLASS.lower():
                continue
            detected_count += 1
            conf = det.get_confidence() * 100.0
            max_conf = max(max_conf, conf)

            bbox = det.get_bbox()
            x_min = int(bbox.xmin() * frame.shape[1])
            y_min = int(bbox.ymin() * frame.shape[0])
            x_max = int(bbox.xmax() * frame.shape[1])
            y_max = int(bbox.ymax() * frame.shape[0])
            cv2.rectangle(frame_overlay, (x_min, y_min), (x_max, y_max), (0, 255, 0), 2)
            cv2.putText(frame_overlay, f"{det.get_label()}:{conf:.1f}%", (x_min, max(0, y_min - 5)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

    user_data.total_objects_detected += detected_count

    # overlay info in top-left corner
    current_time_str = datetime.now().strftime("%H:%M:%S")
    overlay_text = f"Objects detected: {detected_count} | Time: {current_time_str}"
    cv2.putText(frame_overlay, overlay_text, (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

    # update preview
    global preview_frame
    preview_frame = frame_overlay.copy()

    # push frame and metadata to queue
    try:
        user_data.queue.put_nowait({
            "frame": frame_overlay,
            "detected_count": detected_count,
            "max_conf": max_conf,
            "frame_number": user_data.frame_count,
            "timestamp": time.time()
        })
    except queue.Full:
        pass  # drop frame if queue is full

    return Gst.PadProbeReturn.OK


def writer_thread(user_data: UserAppCallback):
    # thread for writing video frames and snapshots
    last_snapshot_time = 0.0
    video_start_time = None
    last_written_frame_number = -1
    snapshot_counter = 1  # sequential counter for snapshots

    while preview_running or not user_data.queue.empty():
        try:
            item = user_data.queue.get(timeout=0.1)
        except queue.Empty:
            continue

        frame_overlay = item["frame"]
        detected_count = item["detected_count"]
        max_conf = item["max_conf"]
        frame_timestamp = item["timestamp"]

        # initialize video writer if needed
        if user_data.video_writer is None:
            h, w = frame_overlay.shape[:2]
            fourcc = cv2.VideoWriter_fourcc(*'mp4v')
            user_data.video_writer = cv2.VideoWriter(
                str(user_data.output_dir / "video.mp4"),
                fourcc,
                user_data.TARGET_VIDEO_FPS,
                (w, h)
            )
            print("VideoWriter initialized")
            video_start_time = frame_timestamp

        # calculate how many frames to write based on elapsed time
        target_frame_number = int((frame_timestamp - video_start_time) * user_data.TARGET_VIDEO_FPS)
        while last_written_frame_number < target_frame_number:
            user_data.video_writer.write(cv2.cvtColor(frame_overlay, cv2.COLOR_RGB2BGR))
            last_written_frame_number += 1

        # save snapshot if interval passed and objects detected
        if detected_count > 0 and (frame_timestamp - last_snapshot_time >= user_data.SNAPSHOT_INTERVAL_SEC):
            timestamp_str = datetime.now().strftime("%H-%M-%S")
            snapshot_name = f"{snapshot_counter:03d}_{timestamp_str}.jpg"  # zero-padded (001, 002, etc.)
            snapshot_file = user_data.output_dir / snapshot_name
            snapshot_counter += 1

            cv2.imwrite(str(snapshot_file), cv2.cvtColor(frame_overlay, cv2.COLOR_RGB2BGR),
                        [int(cv2.IMWRITE_JPEG_QUALITY), 80])
            last_snapshot_time = frame_timestamp

            # save row to csv
            row = [
                snapshot_counter - 1,  # Snapshot number
                datetime.now().strftime("%H:%M:%S"),
                detected_count,
                round(max_conf, 1),
                snapshot_file.name
            ]
            user_data.csv_rows.append(row)
            try:
                with open(user_data.csv_file, mode='a', newline='') as f:
                    writer = csv.writer(f)
                    writer.writerow(row)
            except Exception as e:
                print(f"Failed to write CSV row: {e}")


def cleanup(user_data):
    # release resources and rename folder with current time and total weeds
    global preview_running
    preview_running = False

    if user_data.video_writer:
        user_data.video_writer.release()
        print("VideoWriter released")

    cv2.destroyAllWindows()

    now = datetime.now()
    run_time_str = now.strftime("%-I-%M%p").lower()  # e.g., 3-44pm
    final_folder = f"{run_time_str}_{user_data.total_objects_detected}weeds"
    final_path = user_data.output_dir.parent / final_folder
    try:
        if final_path != user_data.output_dir:
            user_data.output_dir.rename(final_path)
        print(f"Run complete. Saved to: {final_path}")
    except Exception as e:
        print(f"Error renaming folder: {e}")


if __name__ == "__main__":
    project_root = Path(__file__).resolve().parent.parent
    os.environ["HAILO_ENV_FILE"] = str(project_root / ".env")

    user_data = UserAppCallback()
    atexit.register(lambda: cleanup(user_data))

    threading.Thread(target=preview_thread, daemon=True).start()
    threading.Thread(target=writer_thread, args=(user_data,), daemon=True).start()

    app = GStreamerDetectionApp(app_callback, user_data)
    app.run()
