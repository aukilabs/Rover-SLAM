import shutil
import argparse
import os
import cv2

def frame_name(frame_index, filename_prefix=""):
    return f"{filename_prefix}{frame_index:06d}.png"

def mp4_to_frames(mp4_path, frames_path, filename_prefix=""):
    capture = cv2.VideoCapture(mp4_path)
    frame_count = 0
    print("Unpacking mp4 to frames:", mp4_path, "->", frames_path)
    while capture.isOpened():
        ret, frame = capture.read()
        if not ret:
            break
        cv2.imwrite(f"{frames_path}/{frame_name(frame_count, filename_prefix)}", frame)
        frame_count += 1
    print(f"Unpacked {frame_count} frames from mp4")
    capture.release()


def ensure_folder_exists(folder):
    if not os.path.exists(folder):
        os.makedirs(folder)


def convert_dmt_to_euroc(input_folder, output_folder, output_timestamps=False):
    if not os.path.exists(input_folder):
        print(f'Error: Input folder {input_folder} does not exist.')
        return
    
    # Folder structure
    # FROM:
    # input_folder/
    # - Frames/
    # - Frames.csv
    # - Frames.mp4 (optional)
    # - gyro_accel.csv
    # TO:
    # output_folder/
    # - mav0/
    # --- cam0/
    # ----- data/
    # ----- data.csv
    # ----- sensor.yaml
    # --- imu0/
    # ----- data.csv
    # ----- sensor.yaml
    # --- CameraIntrinsics.csv (not euroc but we need for DMT)
    # --- Timestamps.txt (optional)
    
    frames_folder = os.path.join(output_folder, 'mav0', 'cam0', 'data')
    if os.path.exists(frames_folder):
        print(f"Output folder {frames_folder} already exists. Removing to start fresh.")
        for file in os.listdir(frames_folder):
            os.remove(os.path.join(frames_folder, file))

    ensure_folder_exists(output_folder)
    ensure_folder_exists(os.path.join(output_folder, 'mav0'))
    ensure_folder_exists(os.path.join(output_folder, 'mav0', 'cam0'))
    ensure_folder_exists(frames_folder)
    ensure_folder_exists(os.path.join(output_folder, 'mav0', 'imu0'))
    
    # FROM: 11426.895297,11426.895297.jpg
    # TO: 11426895297000,11426895297000.jpg
    img_input_csv = os.path.join(input_folder, 'Frames.csv') 
    mp4_input_path = os.path.join(input_folder, 'Frames.mp4')
    if os.path.exists(mp4_input_path):
        mp4_to_frames(mp4_input_path, frames_folder, filename_prefix="")
    else:
        print(f'Error: Frames folder {frames_folder} does not exist, and no Frames.mp4')
        return

    timestamps = []
    input_img_filenames = []
    output_img_filenames = []
    with open(img_input_csv, 'r') as f:
        lines = f.readlines()
        for frame_index, line in enumerate(lines):
            parts = line.split(',')
            input_img_filenames.append(frame_name(frame_index))
            timestamps.append(int(float(parts[0]) * 1e9))
            #img_extension = os.path.splitext(parts[1].strip())[1]
            img_extension = '.png'
            output_img_filenames.append(f'{timestamps[-1]}{img_extension}')
                
    print(f'Frames.csv loaded with {len(input_img_filenames)} images.')
    print(f'Frames folder contains {len(os.listdir(frames_folder))} images.')
    if len(input_img_filenames) != len(os.listdir(frames_folder)):
        print(f'Error: Frames.csv and Frames folder contain different numbers of images.')
        return
    
    cam0_output_file = os.path.join(output_folder, 'mav0', 'cam0', 'data.csv')
    print("Copying images...")
    with open(cam0_output_file, 'w') as f:
        f.write('#timestamp [ns],filename\n')
        for i in range(len(input_img_filenames)):
            f.write(f'{timestamps[i]},{output_img_filenames[i]}\n')
            
            # copy the image
            img_input = os.path.join(frames_folder, input_img_filenames[i])
            img_output = os.path.join(output_folder, 'mav0', 'cam0', 'data', output_img_filenames[i])
            
            os.system(f'mv {img_input} {img_output}')
            #os.system(f'ffmpeg -i {img_input} -preset ultrafast {img_output}')
            #im = Image.open(img_input)
            #im.save(img_output)
            
            prev_progress = (i - 1) / len(input_img_filenames)
            progress = i / len(input_img_filenames)
            if (int(progress * 20) > int(prev_progress * 20)):
                print(f'{int(progress * 100)}%')
    print(f'Images and timestamps written to mav0/cam0/data.csv (line count: {len(output_img_filenames)})') 
    
    # IMU
    imu_input_csv = os.path.join(input_folder, 'gyro_accel.csv')
    imu_output_file = os.path.join(output_folder, 'mav0', 'imu0', 'data.csv')
    with open(imu_input_csv, 'r') as f_in:
        with open(imu_output_file, 'w') as f_out:
            f_out.write("#timestamp [ns],w_RS_S_x [rad s^-1],w_RS_S_y [rad s^-1],w_RS_S_z [rad s^-1],a_RS_S_x [m s^-2],a_RS_S_y [m s^-2],a_RS_S_z [m s^-2]\n")
            lines = f_in.readlines()
            for line in lines[1:]:
                parts = line.split(',')
                timestamp = int(float(parts[0]) * 1e9)
                gx = parts[1]
                gy = parts[2]
                gz = parts[3]
                ax = parts[4]
                ay = parts[5]
                az = parts[6]
                f_out.write(f'{timestamp},{gx},{gy},{gz},{ax},{ay},{az}\n')
    print(f'IMU data written to mav0/imu0/data.csv (line count: {len(lines)})') 

    if output_timestamps:
        timestamps_file = os.path.join(output_folder, 'Timestamps.txt')
        with open(timestamps_file, 'w') as f:
            for timestamp in timestamps:
                f.write(f'{timestamp}\n')
        print(f'Timestamps written to {timestamps_file} (line count: {len(timestamps)})')
    
    # Camera intrinsics
    cam_intrinsics_csv = os.path.join(input_folder, 'CameraIntrinsics.csv')
    if os.path.exists(cam_intrinsics_csv):
        shutil.copy(cam_intrinsics_csv, os.path.join(output_folder, 'CameraIntrinsics.csv'))

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Convert DMT datasets to EuRoC, for easy use in slam algorithms')
    parser.add_argument('input_folder', type=str, help='DMT Recorder output folder, to be converted')
    parser.add_argument('output_folder', type=str, help='Output dataset folder in EuRoC format')
    parser.add_argument('--output-timestamps', action='store_true', help='Output timestamps file')
    parser.add_argument('--all-subfolders', action='store_true', help='Convert all subfolders in the input folder')

    args = parser.parse_args()

    if args.all_subfolders:
        for subfolder in os.listdir(args.input_folder):
            input_folder = os.path.join(args.input_folder, subfolder)
            output_folder = os.path.join(args.output_folder, subfolder)
            convert_dmt_to_euroc(input_folder, output_folder, args.output_timestamps)
    else:
        convert_dmt_to_euroc(args.input_folder, args.output_folder, args.output_timestamps)