#!/usr/bin/env python3

from typing import List, Optional, Dict
import rclpy
from rclpy.node import Node
from nao_lola_command_msgs.msg import JointStiffnesses as JointStiffnessesCMD
from nao_lola_sensor_msgs.msg import JointStiffnesses as JointStiffnessesSEN
from nao_lola_command_msgs.msg import JointIndexes
import tkinter as tk
from tkinter import ttk, messagebox
import threading


class JointStiffnessGUI(Node):
    """
    ROS 2 node with GUI interface for monitoring and controlling NAO robot joint stiffness.
    """

    def __init__(self):
        """Initialize the joint stiffness node and GUI."""
        super().__init__('joint_stiffness_gui_node')

        # Initialize ROS components
        self.joint_sub = self.create_subscription(
            JointStiffnessesSEN,
            '/sensors/joint_stiffnesses',
            self.joint_callback,
            10
        )
        self.joint_pub = self.create_publisher(
            JointStiffnessesCMD,
            '/effectors/joint_stiffnesses',
            10
        )

        # Initialize joint data
        self.joint_stiffnesses: Optional[List[float]] = None
        self.joint_indices: Dict[str, int] = {
            'HEADYAW': 0, 'HEADPITCH': 1,
            'LSHOULDERPITCH': 2, 'LSHOULDERROLL': 3, 'LELBOWYAW': 4, 'LELBOWROLL': 5, 'LWRISTYAW': 6,
            'LHIPYAWPITCH': 7, 'LHIPROLL': 8, 'LHIPPITCH': 9, 'LKNEEPITCH': 10, 'LANKLEPITCH': 11, 'LANKLEROLL': 12,
            'RHIPROLL': 13, 'RHIPPITCH': 14, 'RKNEEPITCH': 15, 'RANKLEPITCH': 16, 'RANKLEROLL': 17,
            'RSHOULDERPITCH': 18, 'RSHOULDERROLL': 19, 'RELBOWYAW': 20, 'RELBOWROLL': 21, 'RWRISTYAW': 22,
            'LHAND': 23, 'RHAND': 24
        }
        
        # Initialize GUI
        self.root = tk.Tk()
        self.root.title("NAO Joint Stiffness Controller")
        self.setup_gui()
        
        # Create timer for periodic updates
        self.create_timer(0.1, self.update_gui)

    def setup_gui(self):
        """Set up the GUI layout and widgets."""
        # Create main frame
        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))

        # Create notebook for different joint groups
        self.notebook = ttk.Notebook(main_frame)
        self.notebook.grid(row=0, column=0, padx=5, pady=5, sticky=(tk.W, tk.E, tk.N, tk.S))

        # Create frames for different joint groups
        self.sliders = {}
        self.labels = {}
        
        # Head joints
        head_frame = self.create_joint_group_frame(["HEADYAW", "HEADPITCH"], "Head")
        
        # Left arm joints
        left_arm_frame = self.create_joint_group_frame(
            ["LSHOULDERPITCH", "LSHOULDERROLL", "LELBOWYAW", "LELBOWROLL", "LWRISTYAW", "LHAND"],
            "Left Arm"
        )
        
        # Right arm joints
        right_arm_frame = self.create_joint_group_frame(
            ["RSHOULDERPITCH", "RSHOULDERROLL", "RELBOWYAW", "RELBOWROLL", "RWRISTYAW", "RHAND"],
            "Right Arm"
        )
        
        # Left leg joints
        left_leg_frame = self.create_joint_group_frame(
            ["LHIPYAWPITCH", "LHIPROLL", "LHIPPITCH", "LKNEEPITCH", "LANKLEPITCH", "LANKLEROLL"],
            "Left Leg"
        )
        
        # Right leg joints
        right_leg_frame = self.create_joint_group_frame(
            ["RHIPROLL", "RHIPPITCH", "RKNEEPITCH", "RANKLEPITCH", "RANKLEROLL"],
            "Right Leg"
        )

        # Create control buttons frame
        control_frame = ttk.Frame(main_frame)
        control_frame.grid(row=1, column=0, padx=5, pady=5, sticky=(tk.W, tk.E))

        # Add global control buttons
        ttk.Button(control_frame, text="Set All to Zero", 
                  command=lambda: self.set_all_stiffness(0.0)).grid(row=0, column=0, padx=5)
        ttk.Button(control_frame, text="Set All to Max", 
                  command=lambda: self.set_all_stiffness(1.0)).grid(row=0, column=1, padx=5)
        ttk.Button(control_frame, text="Set All to Half", 
                  command=lambda: self.set_all_stiffness(0.5)).grid(row=0, column=2, padx=5)

    def create_joint_group_frame(self, joint_names: List[str], group_name: str) -> ttk.Frame:
        """Create a frame for a group of joints with sliders and labels."""
        frame = ttk.Frame(self.notebook)
        self.notebook.add(frame, text=group_name)

        for i, joint in enumerate(joint_names):
            # Create label for joint name
            ttk.Label(frame, text=joint).grid(row=i, column=0, padx=5, pady=2, sticky=tk.W)
            
            # Create slider
            slider = ttk.Scale(frame, from_=0.0, to=1.0, orient=tk.HORIZONTAL,
                             command=lambda v, j=joint: self.on_slider_change(j, v))
            slider.grid(row=i, column=1, padx=5, pady=2, sticky=(tk.W, tk.E))
            self.sliders[joint] = slider
            
            # Create value label
            value_label = ttk.Label(frame, text="0.000")
            value_label.grid(row=i, column=2, padx=5, pady=2)
            self.labels[joint] = value_label

        frame.columnconfigure(1, weight=1)
        return frame

    def joint_callback(self, msg: JointStiffnessesSEN) -> None:
        """Handle incoming joint stiffness messages."""
        self.joint_stiffnesses = msg.stiffnesses

    def update_gui(self) -> None:
        """Update GUI with latest joint stiffness values."""
        if self.joint_stiffnesses is None:
            return

        for joint_name, index in self.joint_indices.items():
            if joint_name in self.labels:
                value = self.joint_stiffnesses[index]
                self.labels[joint_name].config(text=f"{value:.3f}")
                
                # Update slider without triggering callback
                self.sliders[joint_name].set(value)

    def on_slider_change(self, joint_name: str, value: str) -> None:
        """Handle slider value changes."""
        try:
            stiffness = float(value)
            self.set_joint_stiffness(joint_name, stiffness)
        except ValueError as e:
            self.get_logger().error(f"Invalid slider value: {e}")

    def set_joint_stiffness(self, joint_name: str, stiffness: float) -> None:
        """Set stiffness for a single joint."""
        msg = JointStiffnessesCMD()
        msg.indexes = [self.joint_indices[joint_name]]
        msg.stiffnesses = [stiffness]
        self.joint_pub.publish(msg)

    def set_all_stiffness(self, stiffness: float) -> None:
        """Set the same stiffness value for all joints."""
        msg = JointStiffnessesCMD()
        msg.indexes = list(range(len(self.joint_indices)))
        msg.stiffnesses = [stiffness] * len(self.joint_indices)
        self.joint_pub.publish(msg)

    def run(self):
        """Run the GUI and ROS node."""
        try:
            # Start ROS spinning in a separate thread
            ros_thread = threading.Thread(target=lambda: rclpy.spin(self))
            ros_thread.daemon = True
            ros_thread.start()

            # Run the GUI main loop
            self.root.mainloop()
        except KeyboardInterrupt:
            pass
        finally:
            self.destroy_node()


def main(args=None):
    """Main function to run the joint stiffness GUI node."""
    try:
        rclpy.init(args=args)
        node = JointStiffnessGUI()
        node.run()
    except Exception as e:
        messagebox.showerror("Error", str(e))
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()