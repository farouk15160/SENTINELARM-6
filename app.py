import serial
import time

class RobotArm:
    """
    A class to control the 6-axis Arduino-based robot arm over a serial connection.
    This version sends commands without waiting for a response to prevent port hangs.
    """
    def __init__(self, port, baudrate=115200):
        """
        Initializes the connection to the robot arm.
        """
        try:
            self.ser = serial.Serial(port, baudrate, timeout=1)
            print(f"Successfully connected to robot arm on {port}.")
            print("Waiting for robot to initialize (2 seconds)...")
            time.sleep(2)
        except serial.SerialException as e:
            print(f"Error: Could not open serial port {port}. {e}")
            print("Please check the port name and ensure the robot is connected.")
            exit()

    def send_command(self, command):
        """
        Sends a command to the robot without waiting for a response.
        Args:
            command (str): The command string to send (e.g., 'H', 'S,1500').
        """
        if not self.ser.is_open:
            print("Error: Serial port is not open.")
            return

        # Commands must be sent as bytes and end with a newline character
        full_command = f"{command}\n".encode('utf-8')
        self.ser.write(full_command)
        self.ser.flush() # Ensure data is sent immediately
        
        # --- MODIFICATION: We no longer wait for a response from the robot ---
        # response = self.ser.readline().decode('utf-8').strip()
        # No print statements about responses here.

    def set_speed(self, duration_ms):
        """
        Sets the duration for subsequent movements.
        """
        print(f"Setting movement speed to {duration_ms} ms.")
        self.send_command(f"S,{duration_ms}")

    def go_home(self):
        """Commands the robot to move to its predefined HOME position."""
        print("Moving to HOME position.")
        self.send_command("H")

    def move_all_joints(self, angles):
        """
        Moves all 6 joints to the specified angles simultaneously.
        """
        if len(angles) != 6:
            print(f"Error: move_all_joints requires a list of 6 angles. Got {len(angles)}.")
            return
            
        angle_str = ",".join(map(str, angles))
        print(f"Moving all joints to: [{angle_str}]")
        self.send_command(f"M,{angle_str}")
        
    def close(self):
        """Closes the serial connection."""
        if self.ser.is_open:
            self.ser.close()
            print("Serial connection closed.")

# --- Main Program Execution ---
if __name__ == "__main__":
    # !!! IMPORTANT: CHANGE THIS TO YOUR ROBOT'S SERIAL PORT !!!
    SERIAL_PORT = '/dev/ttyUSB0'  # Or 'COM3' for Windows, etc.
    BAUD_RATE = 115200

    arm = RobotArm(SERIAL_PORT, BAUD_RATE)

    try:
        poses = [
            [80.00, 80.00, 9.99, 90.00, 49.95, 120.00],
            [115.01, 114.93, 9.99, 99.99, 0.00, 119.97],
            [118.07, 117.99, 9.99, 84.96, 0.00, 64.98],
            [118.07, 117.99, 9.99, 84.96, 0.00, 61],
            [90.00, 90.00, 20.00, 45.00, 90.00, 90.00]
        ]

        print("\n--- Starting Robot Control Sequence ---")

        arm.set_speed(1500)
        time.sleep(2) # Give the robot time to process the speed command

        for i, pose in enumerate(poses):
            print(f"\n--- Moving to Pose {i+1} ---")
            arm.move_all_joints(pose)
            # The sleep duration is now critical, as we aren't waiting for a response.
            # It should be slightly longer than the move duration set by set_speed().
            time.sleep(1.5) 

        print("\n--- Finishing Sequence ---")
        arm.set_speed(1500) # Set a slow speed for the final move
        time.sleep(1)
        arm.go_home()
        time.sleep(3.5) # Wait for the slow home move to finish

        print("\nSequence complete. 🤖")

    except KeyboardInterrupt:
        print("\nProgram interrupted by user. Returning to home position.")
        arm.set_speed(1500)
        arm.go_home()
        
    finally:
        arm.close()