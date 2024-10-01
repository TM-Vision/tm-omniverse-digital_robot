import csv
import os
import shutil
import threading
import time
import tkinter as tk
from datetime import datetime

from PIL import Image, ImageTk
from virtual_camera_client import VirtualCameraClient

CAMERA_RESOLUTION = (2592, 1944)
WORLD_CAMERA = "world_camera"
ROBOT_CAMERA = "robot_01_tm12_camera"


class VirtualCameraUI:
    def __init__(self):
        self._current_script_dir = os.path.dirname(os.path.abspath(__file__))

    def _continuously_get_image_bytes(
        self, label_camera_set, camera_name, client: VirtualCameraClient
    ):
        start_time = time.time()
        total_images_received = 0

        while True:
            # Get image and update label
            image: Image = client.getImageBytes(camera_name)
            total_images_received += 1
            image_resized = image.resize((640, 480), Image.LANCZOS)
            tk_image = ImageTk.PhotoImage(image_resized)
            label_camera_set[0].config(image=tk_image)
            label_camera_set[0].image = tk_image

            now = time.time()
            elapsed_time = now - start_time
            frame_rate = total_images_received / elapsed_time

            label_camera_set[1].config(text=f"{frame_rate:.2f} fps")

            # Save image info to csv
            image_bytes = image.tobytes()
            csv_file_path = os.path.join(self._current_script_dir, "camera_images.csv")
            file_exists = os.path.isfile(csv_file_path)

            def save_image_info_to_csv(csv_file_path, camera_name, image_bytes):
                with open(csv_file_path, mode="a", newline="") as file:
                    writer = csv.writer(file)
                    if not file_exists:
                        writer.writerow(["camera_name", "image_bytes"])
                    writer.writerow([camera_name, len(image_bytes)])

            # Save image info to csv in a separate thread without blocking the UI
            # csv_thread = threading.Thread(
            #     target=save_image_info_to_csv,
            #     args=(csv_file_path, camera_name, image_bytes),
            # )
            # csv_thread.start()

            # Save image to tmp folder
            def save_image(image, image_path):
                image.save(image_path)

            image = Image.frombytes(
                mode="RGBA", size=CAMERA_RESOLUTION, data=image_bytes
            )
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
            image_name = f"{timestamp}.png"
            image_path = os.path.join(
                self._current_script_dir, "tmp", camera_name, image_name
            )
            # print(f"image_path: {image_path}")

            # Save image in a separate thread without blocking the UI
            save_thread = threading.Thread(target=save_image, args=(image, image_path))
            save_thread.start()


if __name__ == "__main__":
    ui = VirtualCameraUI()
    client = VirtualCameraClient()

    # Set Camera Parameters
    # client.setGain(WORLD_CAMERA, 10)
    # client.setGain(ROBOT_CAMERA, 50)

    # Clean tmp folder
    tmp_dir = os.path.join(ui._current_script_dir, "tmp")
    if os.path.exists(tmp_dir):
        shutil.rmtree(tmp_dir)

    # Create tmp folder
    os.makedirs(
        os.path.join(ui._current_script_dir, "tmp", WORLD_CAMERA), exist_ok=True
    )
    os.makedirs(
        os.path.join(ui._current_script_dir, "tmp", ROBOT_CAMERA), exist_ok=True
    )

    # Clean all images in tmp folder
    tmp_dir = os.path.join(ui._current_script_dir, "tmp")
    if os.path.exists(tmp_dir):
        for file_name in os.listdir(tmp_dir):
            if file_name.endswith(".png"):
                file_path = os.path.join(tmp_dir, file_name)
                try:
                    os.remove(file_path)
                    print(f"Deleted {file_path}")
                except Exception as e:
                    print(f"Error deleting {file_path}: {e}")

    # Create UI
    root = tk.Tk()
    root.title("Main Window")
    root.geometry("1280x500")

    frame1 = tk.Frame(root, width=400, height=300)
    frame1.pack(side="left", fill="both", expand=True)

    frame2 = tk.Frame(root, width=400, height=300)
    frame2.pack(side="right", fill="both", expand=True)

    frame3 = tk.Frame(root, width=400, height=50)
    frame3.pack(side="bottom", fill="both", expand=True)

    label_camera1 = tk.Label(frame1)
    label_camera1.pack()
    label_camera1_frame_rate = tk.Label(frame1, text="Virtual Camera 01")
    label_camera1_frame_rate.pack()
    label_camera1_set = (label_camera1, label_camera1_frame_rate)

    label_camera2 = tk.Label(frame2)
    label_camera2.pack()
    label_camera2_frame_rate = tk.Label(frame2, text="Virtual Camera 02")
    label_camera2_frame_rate.pack()
    label_camera2_set = (label_camera2, label_camera2_frame_rate)

    # Start threads to retrieve images
    view1_thread = threading.Thread(
        target=ui._continuously_get_image_bytes,
        args=(label_camera1_set, WORLD_CAMERA, client),
    )
    view1_thread.daemon = True
    view1_thread.start()

    view2_thread = threading.Thread(
        target=ui._continuously_get_image_bytes,
        args=(label_camera2_set, ROBOT_CAMERA, client),
    )
    view2_thread.daemon = True
    view2_thread.start()

    try:
        root.mainloop()
    except KeyboardInterrupt:
        print("Program stopped by user")
