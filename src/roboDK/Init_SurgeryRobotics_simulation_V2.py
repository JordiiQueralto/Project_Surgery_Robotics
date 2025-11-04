from robodk.robolink import *
from robodk.robomath import *
import time
import math
import tkinter as tk
import threading
import socket
import json
import os

# Define the relative and absolute path to the RoboDK project file
relative_path = "src/roboDK/SurgeryRobotics.rdk"
absolute_path = os.path.abspath(relative_path)
# Constants
UDP_IP = "0.0.0.0"
UDP_PORT = 12345
BUFFER_SIZE = 4096
ROBOT_NAME = 'UR5e'
ZERO_YAW_TOOL = 0
ZERO_YAW_GRIPPER = 0
READ_INTERVAL_S = 0.01

# Shared data
Endowrist_rpy = None
Gripper_rpy = None
Servo_torques = None  # will be a dict like {"Torque_roll1":..., "Torque_roll2":..., "Torque_pitch":..., "Torque_yaw":...}
data_lock = threading.Lock()  # semaphore to manage data from 2 threads

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))
# print(f"Listening on {UDP_IP}:{UDP_PORT}")

# Initialize RoboDK
def initialize_robodk(absolute_path):
    RDK = Robolink()
    time.sleep(2)  # wait for RoboDK to be ready
    RDK.AddFile(absolute_path)
    robot = RDK.Item(ROBOT_NAME)
    base = RDK.Item(f'{ROBOT_NAME} Base')
    endowrist = RDK.Item('Endowrist')
    gripper = RDK.Item('Gripper')
    needle = RDK.Item('Needle')
    Init_target = RDK.Item('Init')
    robot.setPoseFrame(base)
    robot.setPoseTool(endowrist)
    gripper_init = TxyzRxyz_2_Pose([0, 5, -105, 0, 0, 0])
    gripper.setParent(endowrist)
    gripper.setPose(gripper_init)
    needle_init = TxyzRxyz_2_Pose([0, 0, 0, 0, 0, 0])
    needle.setParent(gripper)
    needle.setPose(needle_init)
    robot.setSpeed(50)
    robot.MoveL(Init_target)
    return RDK, robot, base, gripper, needle

# Transformation Endowrist to base
def endowrist2base_orientation(roll, pitch, yaw):
    roll2 = (roll + 90) % 360
    pitch2 = pitch % 360
    yaw2 = yaw % 360
    return roll2, pitch2, yaw2

# Function to update the label with text
def update_text_label(label, tool_orientation, gripper_orientation, status_message, torque_values):
    full_text = f"Tool orientation: {tool_orientation}\nGripper orientation: {gripper_orientation}\n{status_message}\n\n{torque_values}"
    label.after(0, lambda: label.config(text=full_text))

# Update color indicator for torque (button/label)
def update_torque_indicator(widget, total_torque):
    """
    Color code:
      - low: green
      - medium: yellow
      - high: red
    Adjust thresholds as required.
    """
    low_thresh = 1.0
    med_thresh = 3.0

    if total_torque < low_thresh:
        color = "green"
    elif total_torque < med_thresh:
        color = "yellow"
    else:
        color = "red"
    # update widget color in the GUI thread
    widget.after(0, lambda: widget.config(bg=color, activebackground=color))

# Function to read UDP data and update the global variable
def read_data_UDP():
    global Endowrist_rpy, Gripper_rpy, Servo_torques, data_lock
    while True:
        try:
            data, addr = sock.recvfrom(BUFFER_SIZE)
            try:
                received_data = json.loads(data.decode())
                device_id = received_data.get("device")
                with data_lock:
                    if device_id == "G3_Endo":
                        Endowrist_rpy = received_data
                    elif device_id == "G3_Gri":
                        Gripper_rpy = received_data
                    elif device_id == "G3_Servos":
                        # Expect keys: Torque_roll1, Torque_roll2, Torque_pitch, Torque_yaw (some may be missing)
                        Servo_torques = {
                            "Torque_roll1": float(received_data.get("Torque_roll1", 0.0)),
                            "Torque_roll2": float(received_data.get("Torque_roll2", 0.0)),
                            "Torque_pitch": float(received_data.get("Torque_pitch", 0.0)),
                            "Torque_yaw": float(received_data.get("Torque_yaw", 0.0))
                        }
                    else:
                        # unknown device - ignore
                        pass
            except json.JSONDecodeError:
                print("Error decoding JSON data")
        except socket.error as e:
            # socket closed or error
            try:
                sock.close()
            except:
                pass
            print("Socket closed or error:", e)
            break

# Function to process the latest UDP data and move the robot
def move_robot(robot, gripper, needle, text_label, torque_value_label, torque_indicator_button):
    global ZERO_YAW_TOOL, ZERO_YAW_GRIPPER, Endowrist_rpy, Gripper_rpy, data_lock
    global e_roll, e_pitch, e_yaw, g_roll, g_pitch, g_yaw, s1, s2, s3, s4

    endowrist_orientation_msg = ""
    gripper_orientation_msg = ""
    status_message = ""
    servo_torques_msg = ""

    # local_vars for safety
    endo_roll = endo_pitch = endo_yaw = 0.0

    while True:
        with data_lock:
            current_Endowrist_rpy = Endowrist_rpy
            current_Gripper_rpy = Gripper_rpy
            current_Servo_torques = Servo_torques

        # ---- Endowrist / Tool movement ----
        if current_Endowrist_rpy:
            e_roll = current_Endowrist_rpy.get("roll", 0.0)
            e_pitch = current_Endowrist_rpy.get("pitch", 0.0)
            e_yaw = current_Endowrist_rpy.get("yaw", 0.0)
            s3 = current_Endowrist_rpy.get("s3", 1)
            s4 = current_Endowrist_rpy.get("s4", 1)
            endo_roll, endo_pitch, endo_yaw = endowrist2base_orientation(e_roll, e_pitch, e_yaw)
            # Move Endowrist (pose orientation only)
            try:
                endowrist_pose = robot.Pose()
                Xr, Yr, Zr, rr, pr, yr = Pose_2_TxyzRxyz(endowrist_pose)
                endowrist_pose_new = transl(Xr, Yr, Zr) * rotz(math.radians(ZERO_YAW_TOOL)) * rotz(math.radians(endo_yaw)) * roty(math.radians(endo_pitch)) * rotx(math.radians(endo_roll))
                if robot.MoveL_Test(robot.Joints(), endowrist_pose_new) == 0:
                    robot.MoveL(endowrist_pose_new, True)
                    endowrist_orientation_msg = f"R={round(endo_roll)} P={round(endo_pitch)} W={round((endo_yaw+ZERO_YAW_TOOL)%360)}"
                    status_message = ""
                else:
                    endowrist_orientation_msg = f"R={round(endo_roll)} P={round(endo_pitch)} W={round((endo_yaw+ZERO_YAW_TOOL)%360)}"
                    status_message = "Robot cannot reach the position"
            except Exception as e:
                status_message = f"Error moving Endowrist: {e}"

            # s3 s4 control Z
            if s3 == 0 or s4 == 0:
                try:
                    current_pose = robot.Pose()
                    Tz = transl(0, 0, 5) if s3 == 0 else transl(0, 0, -5)
                    new_pose = current_pose * Tz  # translation relative
                    status_message = "⬆ Botó S3 premut: pujant" if s3 == 0 else "⬇ Botó S4 premut: baixant"
                    if robot.MoveL_Test(robot.Joints(), new_pose) == 0:
                        robot.MoveL(new_pose, True)
                    else:
                        status_message = "❌ No es pot moure més en Z (relatiu)"
                except Exception as e:
                    status_message = f"Error Z move: {e}"

        # ---- Gripper orientation and needle logic ----
        if current_Gripper_rpy:
            try:
                g_roll = Gripper_rpy.get("roll")
                g_roll = g_roll - endo_roll  # Compensate for endowrist roll
                g_pitch = Gripper_rpy.get("pitch")
                g_pitch = g_pitch - endo_pitch  # Compensate for endowrist
                g_yaw = Gripper_rpy.get("yaw")
                g_yaw = g_yaw - endo_yaw  # Compensate for endowrist pitch
                s1 = current_Gripper_rpy.get("s1", 1)
                s2 = current_Gripper_rpy.get("s2", 1)

                gripper_pose = gripper.Pose()
                Xg, Yg, Zg, rg, pg, yg = Pose_2_TxyzRxyz(gripper_pose)
                gripper_pose_new = transl(Xg, Yg, Zg) * rotz(math.radians(ZERO_YAW_GRIPPER)) * rotz(math.radians(g_yaw)) * roty(math.radians(g_pitch)) * rotx(math.radians(g_roll))
                gripper.setPose(gripper_pose_new)
                gripper_orientation_msg = f"R={round(g_roll)} P={round(g_pitch)} W={round((g_yaw+ZERO_YAW_GRIPPER)%360)}"
                if s1 == 0:
                    needle.setParentStatic(base)
                    status_message = "🟢 S1 premut: agulla alliberada"
                elif s1 == 1:
                    needle.setParent(gripper)
                    needle.setPose(TxyzRxyz_2_Pose([0, 0, 0, 0, 0, 0]))
                    status_message = "🔵 S1 no premut: agulla agafada"
            except Exception as e:
                status_message = f"Error handling gripper: {e}"

        # ---- Servomotor torques: update GUI numeric and colored indicator ----
        servo_torques_msg = "No torques received yet."
        total_torque = 0.0
        if current_Servo_torques:
            tr1 = current_Servo_torques.get("Torque_roll1", 0.0)
            tr2 = current_Servo_torques.get("Torque_roll2", 0.0)
            tp = current_Servo_torques.get("Torque_pitch", 0.0)
            ty = current_Servo_torques.get("Torque_yaw", 0.0)
            total_torque = tr1 + tr2 + tp + ty
            servo_torques_msg = f"Torque_roll1: {tr1:.3f}  Torque_roll2: {tr2:.3f}\nTorque_pitch: {tp:.3f}  Torque_yaw: {ty:.3f}\nTotal torque: {total_torque:.3f}"

            # update numeric label in GUI (thread-safe)
            torque_value_label.after(0, lambda msg=servo_torques_msg: torque_value_label.config(text=msg))
            # update color indicator
            update_torque_indicator(torque_indicator_button, total_torque)
        else:
            # If no torque data, ensure numeric label shows placeholder
            torque_value_label.after(0, lambda: torque_value_label.config(text="No torques received yet."))

        # Update the main text label with the latest values
        update_text_label(text_label, endowrist_orientation_msg, gripper_orientation_msg, status_message, servo_torques_msg)

        time.sleep(READ_INTERVAL_S)  # define the reading interval

def on_closing():
    global root, sock
    print("Closing...")
    try:
        sock.close()
        print("Ending Socket")
    except Exception:
        pass
    try:
        root.destroy()
    except Exception:
        pass

# Update functions for sliders
def set_zero_yaw_tool(value):
    global ZERO_YAW_TOOL
    ZERO_YAW_TOOL = float(value)

def set_zero_yaw_gripper(value):
    global ZERO_YAW_GRIPPER
    ZERO_YAW_GRIPPER = float(value)

# Main function
def main():
    global root, ZERO_YAW_TOOL, ZERO_YAW_GRIPPER, robot, gripper, base, text_label, absolute_path

    RDK, robot, base, gripper, needle = initialize_robodk(absolute_path)

    root = tk.Tk()
    root.title("Suture Process")
    root.protocol("WM_DELETE_WINDOW", on_closing)  # Proper closing

    text_label = tk.Label(root, text="", wraplength=400, justify=tk.LEFT, anchor="w")
    text_label.pack(padx=10, pady=10, fill="both")

    # Torque numeric display and colored indicator
    torque_frame = tk.Frame(root)
    torque_frame.pack(padx=10, pady=5, fill="x")

    torque_value_label = tk.Label(torque_frame, text="No torques received yet.", justify=tk.LEFT, anchor="w")
    torque_value_label.pack(side="left", padx=(0,10))

    torque_indicator_button = tk.Button(torque_frame, text="Torque level", width=12)
    torque_indicator_button.pack(side="right")

    # Add sliders for ZERO_YAW_TOOL and ZERO_YAW_GRIPPER
    tool_yaw_slider = tk.Scale(root, from_=-180, to=180, orient=tk.HORIZONTAL, label="Tool Yaw",
                                    command=lambda value: set_zero_yaw_tool(float(value)), length=300)
    tool_yaw_slider.set(ZERO_YAW_TOOL)
    tool_yaw_slider.pack(padx=10, pady=5)

    gripper_yaw_slider = tk.Scale(root, from_=-180, to=180, orient=tk.HORIZONTAL, label="Gripper Yaw",
                                        command=lambda value: set_zero_yaw_gripper(float(value)), length=300)
    gripper_yaw_slider.set(ZERO_YAW_GRIPPER)
    gripper_yaw_slider.pack(padx=10, pady=5)

    # Start the UDP reading thread
    udp_thread = threading.Thread(target=read_data_UDP)
    udp_thread.daemon = True
    udp_thread.start()

    # Start the robot movement thread
    robot_thread = threading.Thread(target=move_robot, args=(robot, gripper, needle, text_label, torque_value_label, torque_indicator_button))
    robot_thread.daemon = True
    robot_thread.start()

    root.mainloop()
    print("Pop-up menu closed")
    try:
        RDK.CloseRoboDK()
        print("RoboDK closed")
    except Exception:
        pass

if __name__ == "__main__":
    main()
