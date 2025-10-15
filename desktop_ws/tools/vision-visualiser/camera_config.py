#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from rcl_interfaces.srv import SetParameters
import tkinter as tk
from tkinter import ttk
import configparser
import os
from ament_index_python.packages import get_package_share_directory

class CameraConfigUI(Node):
    def __init__(self):
        super().__init__('camera_config_ui')
        
        # Create service clients
        self.top_client = self.create_client(SetParameters, '/top_camera/set_parameters')
        self.bot_client = self.create_client(SetParameters, '/bot_camera/set_parameters')
        
        # Wait for services
        while not self.top_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Top camera service not available, waiting...')
        while not self.bot_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Bottom camera service not available, waiting...')
        
        # Load current config
        self.camera_params = self.load_current_config()
        
        # Create UI
        self.create_ui()

    def load_current_config(self):
        try:
            package_path = get_package_share_directory('vision')
            workspace_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(package_path))))
            cfg_path = os.path.join(
                os.path.dirname(workspace_root),
                'image',
                'home',
                'nao',  
                'data',
                'runswift.cfg'
            )
            
            self.get_logger().info(f'Reading config from: {cfg_path}')
            
            if not os.path.exists(cfg_path):
                self.get_logger().error(f'Config file not found: {cfg_path}')
                raise FileNotFoundError(f'Config file not found: {cfg_path}')
            
            config = configparser.ConfigParser()
            config.read(cfg_path)
            
            # Default parameters
            default_params = {
                'gain': 64,
                'exposure': 250,
                'saturation': 65,
                'contrast': 32,
                'brightness': 128,
                'focusabsolute': 0,
                'autowhitebalance': 0,
                'autofocus': 0,
                'exposureauto': 0
            }
            
            camera_params = {'top': default_params.copy(), 'bot': default_params.copy()}
            
            # Parameter mapping from config file to UI
            param_mapping = {
                'gain': 'gain',
                'exposure': 'exposure',
                'saturation': 'saturation',
                'contrast': 'contrast',
                'brightness': 'brightness',
                'focusabsolute': 'focusabsolute',
                'autowhitebalance': 'autowhitebalance',
                'autofocus': 'autofocus',
                'exposureauto': 'exposureauto'
            }
            
            if 'camera' in config:
                for key, value in config['camera'].items():
                    camera_pos, param = key.split('.')
                    if camera_pos in ['top', 'bot'] and param in param_mapping:
                        mapped_param = param_mapping[param]
                        try:
                            # Convert value to int, handling possible boolean values
                            if value.lower() in ['true', 'false']:
                                parsed_value = 1 if value.lower() == 'true' else 0
                            else:
                                parsed_value = int(value)
                            camera_params[camera_pos][mapped_param] = parsed_value
                            self.get_logger().info(f'Loaded {camera_pos}.{mapped_param}: {parsed_value}')
                        except ValueError as e:
                            self.get_logger().warn(f'Failed to parse value for {key}: {value}')
                            continue
            
            self.get_logger().info('Successfully loaded camera parameters from config')
            return camera_params
            
        except Exception as e:
            self.get_logger().error(f'Error loading config: {str(e)}')
            # Return default parameters if loading fails
            return {
                'top': default_params.copy(),
                'bot': default_params.copy()
            }

    def create_ui(self):
        self.window = tk.Tk()
        self.window.title("Camera Configuration")
        
        # Create notebook for tabs
        notebook = ttk.Notebook(self.window)
        notebook.pack(pady=10, expand=True)
        
        # Create frames for each camera
        top_frame = ttk.Frame(notebook)
        bot_frame = ttk.Frame(notebook)
        notebook.add(top_frame, text='Top Camera')
        notebook.add(bot_frame, text='Bottom Camera')
        
        # Create sliders for each camera
        self.top_sliders = self.create_camera_controls(top_frame, 'top')
        self.bot_sliders = self.create_camera_controls(bot_frame, 'bot')
        
        # Create submit button
        submit_btn = ttk.Button(self.window, text="Apply Settings", command=self.submit_parameters)
        submit_btn.pack(pady=10)

    def create_camera_controls(self, parent, camera_pos):
        controls = {}
        
        # Parameter ranges
        ranges = {
            'gain': (0, 10000),
            'exposure': (0, 2000),
            'saturation': (0, 10000),
            'contrast': (0, 10000),
            'brightness': (0, 10000),
            'focusabsolute': (0, 10000)
        }
        
        # Create a frame for numeric inputs
        numeric_frame = ttk.LabelFrame(parent, text="Camera Parameters")
        numeric_frame.pack(fill='x', padx=5, pady=5)
        
        # Create numeric entries for continuous values
        for param, (min_val, max_val) in ranges.items():
            frame = ttk.Frame(numeric_frame)
            frame.pack(fill='x', padx=5, pady=2)
            
            # Label with range
            label = ttk.Label(frame, text=f"{param.title()} ({min_val}-{max_val}):")
            label.pack(side='left')
            
            # Create StringVar to hold the numeric value
            value_var = tk.StringVar()
            value = self.camera_params[camera_pos].get(param, ranges[param][0])
            value_var.set(str(value))
            
            # Create and pack the numeric entry
            value_entry = ttk.Entry(frame, textvariable=value_var, width=8)
            value_entry.pack(side='right', padx=5)
            
            # Validation function
            def validate_entry(var=value_var, min_v=min_val, max_v=max_val):
                try:
                    val = int(var.get())
                    if val < min_v:
                        var.set(str(min_v))
                    elif val > max_v:
                        var.set(str(max_v))
                except ValueError:
                    var.set(str(min_v))
            
            # Bind validation
            value_entry.bind('<FocusOut>', lambda e, v=value_var: validate_entry(v))
            value_entry.bind('<Return>', lambda e, v=value_var: validate_entry(v))
            
            controls[param] = value_var
        
        # Create a frame for boolean parameters
        bool_frame = ttk.LabelFrame(parent, text="Auto Settings")
        bool_frame.pack(fill='x', padx=5, pady=5)
        
        # Create checkboxes for boolean values
        bool_params = ['autowhitebalance', 'autofocus', 'exposureauto']
        for param in bool_params:
            var = tk.BooleanVar(value=bool(self.camera_params[camera_pos].get(param, 0)))
            checkbox = ttk.Checkbutton(bool_frame, text=param.title(), variable=var)
            checkbox.pack(anchor='w', padx=5, pady=2)
            controls[param] = var
            
        return controls

    def create_parameter(self, name, value, type_):
        parameter_value = ParameterValue(type=type_)
        
        if type_ == ParameterType.PARAMETER_BOOL:
            parameter_value.bool_value = bool(value)
        elif type_ == ParameterType.PARAMETER_INTEGER:
            parameter_value.integer_value = int(value)
            
        return Parameter(name=name, value=parameter_value)

    def submit_parameters(self):
        # Get values from controls and create parameter lists
        for camera_pos, controls in [('top', self.top_sliders), ('bot', self.bot_sliders)]:
            try:
                parameters = [
                    self.create_parameter('gain', int(controls['gain'].get()), 
                                        ParameterType.PARAMETER_INTEGER),
                    self.create_parameter('exposure', int(controls['exposure'].get()), 
                                        ParameterType.PARAMETER_INTEGER),
                    self.create_parameter('saturation', int(controls['saturation'].get()), 
                                        ParameterType.PARAMETER_INTEGER),
                    self.create_parameter('contrast', int(controls['contrast'].get()), 
                                        ParameterType.PARAMETER_INTEGER),
                    self.create_parameter('brightness', int(controls['brightness'].get()), 
                                        ParameterType.PARAMETER_INTEGER),
                    self.create_parameter('focus', int(controls['focusabsolute'].get()), 
                                        ParameterType.PARAMETER_INTEGER),
                    self.create_parameter('white_balance_automatic', controls['autowhitebalance'].get(), 
                                        ParameterType.PARAMETER_BOOL),
                    self.create_parameter('autofocus', controls['autofocus'].get(), 
                                        ParameterType.PARAMETER_BOOL),
                    self.create_parameter('autoexposure', controls['exposureauto'].get(), 
                                        ParameterType.PARAMETER_BOOL),
                ]
                
                # TODO: Update config file
            
            except Exception as e:
                self.get_logger().error(f'Error creating parameters: {str(e)}')
                continue
            
            # Create and send request
            request = SetParameters.Request()
            request.parameters = parameters
            
            if camera_pos == 'top':
                future = self.top_client.call_async(request)
            else:
                future = self.bot_client.call_async(request)
            
            self.get_logger().info(f'Sent parameters for {camera_pos} camera')

    def update_config_file(self, camera_pos, sliders):
        try:
            package_path = get_package_share_directory('vision')
            workspace_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(package_path))))
            cfg_path = os.path.join(
                os.path.dirname(workspace_root),
                'data',
                'runswift.cfg'
            )
            
            config = configparser.ConfigParser()
            config.read(cfg_path)
            
            if 'camera' not in config:
                config['camera'] = {}
                
            # Update values in config
            for param, control in sliders.items():
                if isinstance(control, dict):  # Slider with numeric entry
                    value = int(control['slider'].get())
                else:  # Checkbox
                    value = control.get()
                    if isinstance(value, bool):
                        value = '1' if value else '0'
                config['camera'][f'{camera_pos}.{param}'] = str(value)
            
            # Write to file
            with open(cfg_path, 'w') as configfile:
                config.write(configfile)
                
            self.get_logger().info(f'Successfully updated config file for {camera_pos} camera')
            
        except Exception as e:
            self.get_logger().error(f'Error updating config file: {str(e)}')
            
    def run(self):
        # Start the UI main loop
        self.window.mainloop()

def main(args=None):
    rclpy.init(args=args)
    node = CameraConfigUI()
    
    # Create a separate thread for ROS spinning
    import threading
    ros_thread = threading.Thread(target=lambda: rclpy.spin(node))
    ros_thread.start()
    
    # Run the UI (this will block until window is closed)
    node.run()
    
    # Cleanup
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()