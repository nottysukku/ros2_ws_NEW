import serial
import time

class LSCServoController:
    """
    API for controlling LSC Series Servo Controller via UART
    Communication Protocol V1.2 implementation
    """
    
    def __init__(self, port='/dev/ttyUSB0', baudrate=9600, timeout=1):
        """
        Initialize the servo controller
        """
        self.ser = serial.Serial(port, baudrate, timeout=timeout)
        time.sleep(2)  # Wait for serial port to initialize
        
    def _send_packet(self, cmd, params=None):
        """Send a packet to the servo controller"""
        if params is None:
            params = []
            
        packet = bytearray([0x55, 0x55])  # Header
        length = len(params) + 2
        packet.append(length)
        packet.append(cmd)
        packet.extend(params)
        
        self.ser.write(packet)
        time.sleep(0.05)
        
    def _read_response(self):
        """Read response from servo controller"""
        time.sleep(0.1)
        if self.ser.in_waiting > 0:
            response = bytearray()
            while self.ser.in_waiting > 0:
                byte = self.ser.read(1)
                if byte == b'\x55':
                    next_byte = self.ser.read(1)
                    if next_byte == b'\x55':
                        response.extend([0x55, 0x55])
                        length_byte = self.ser.read(1)
                        if length_byte:
                            length = ord(length_byte)
                            response.append(length)
                            remaining = self.ser.read(length - 1)
                            response.extend(remaining)
                            return response
            return response if len(response) > 0 else None
        return None

    def move_servos(self, servo_positions, time_ms):
        """Control multiple servos to move to specified positions"""
        if not servo_positions:
            raise ValueError("servo_positions cannot be empty")
            
        if time_ms > 65535 or time_ms < 0:
            raise ValueError("time_ms must be between 0 and 65535")
            
        time_low = time_ms & 0xFF
        time_high = (time_ms >> 8) & 0xFF
        
        params = [len(servo_positions), time_low, time_high]
        
        for servo_id, position in servo_positions:
            if position > 65535 or position < 0:
                raise ValueError(f"Position for servo {servo_id} must be between 0 and 65535")
            
            pos_low = position & 0xFF
            pos_high = (position >> 8) & 0xFF
            params.extend([servo_id, pos_low, pos_high])
            
        self._send_packet(3, params)

    def read_servo_positions(self, servo_ids):
        """Read current positions of multiple servos"""
        if not servo_ids:
            raise ValueError("servo_ids cannot be empty")
            
        params = [len(servo_ids)] + servo_ids
        self._send_packet(21, params)
        
        response = self._read_response()
        if response and len(response) >= 5:
            positions = {}
            data = response[4:]
            if len(data) >= 1:
                num_servos = data[0]
                idx = 1
                for _ in range(num_servos):
                    if idx + 2 < len(data):
                        servo_id = data[idx]
                        pos_low = data[idx + 1]
                        pos_high = data[idx + 2]
                        position = pos_low + (pos_high << 8)
                        positions[servo_id] = position
                        idx += 3
            return positions
        return {}

    def detect_servos(self, id_range=(1, 32)):
        """Detect which servos are connected"""
        detected_servos = []
        start_id, end_id = id_range
        
        batch_size = 6
        for i in range(start_id, end_id + 1, batch_size):
            batch_ids = list(range(i, min(i + batch_size, end_id + 1)))
            try:
                positions = self.read_servo_positions(batch_ids)
                detected_servos.extend(positions.keys())
                time.sleep(0.1)
            except Exception:
                for servo_id in batch_ids:
                    try:
                        pos = self.read_servo_positions([servo_id])
                        if pos:
                            detected_servos.append(servo_id)
                    except Exception:
                        pass
                    time.sleep(0.05)
        
        return sorted(detected_servos)

    def get_battery_voltage(self):
        """Get the controller's battery voltage in millivolts"""
        self._send_packet(15)
        
        response = self._read_response()
        if response and len(response) >= 6:
            data = response[4:]
            if len(data) >= 2:
                voltage_low = data[0]
                voltage_high = data[1]
                voltage = voltage_low + (voltage_high << 8)
                return voltage
        return None

    def close(self):
        """Close the serial connection"""
        if self.ser.is_open:
            self.ser.close()

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()


class ServoControllerMenu:
    """Interactive menu for LSC Servo Controller"""
    
    def __init__(self, port='/dev/ttyUSB0'):
        self.port = port
        self.controller = None
        self.detected_servos = []
        
    def connect(self):
        """Connect to the servo controller"""
        try:
            self.controller = LSCServoController(self.port)
            print(f"✓ Connected to servo controller on {self.port}")
            return True
        except Exception as e:
            print(f"✗ Failed to connect: {e}")
            return False
    
    def disconnect(self):
        """Disconnect from the servo controller"""
        if self.controller:
            self.controller.close()
            self.controller = None
            print("✓ Disconnected from servo controller")
    
    def detect_servos_menu(self):
        """Menu option to detect servos"""
        if not self.controller:
            print("✗ Not connected to controller!")
            return
            
        print("\n--- Detecting Servos ---")
        try:
            self.detected_servos = self.controller.detect_servos()
            if self.detected_servos:
                print(f"✓ Found {len(self.detected_servos)} servos: {self.detected_servos}")
            else:
                print("✗ No servos detected")
        except Exception as e:
            print(f"✗ Error detecting servos: {e}")
    
    def read_servo_by_id_menu(self):
        """Menu option to read servo position by ID"""
        if not self.controller:
            print("✗ Not connected to controller!")
            return
            
        print("\n--- Read Servo by ID ---")
        try:
            servo_id = int(input("Enter servo ID to read: "))
            positions = self.controller.read_servo_positions([servo_id])
            if positions:
                print(f"✓ Servo {servo_id} position: {positions[servo_id]}")
            else:
                print(f"✗ Could not read servo {servo_id} (not connected or invalid ID)")
        except ValueError:
            print("✗ Invalid servo ID entered")
        except Exception as e:
            print(f"✗ Error reading servo: {e}")
    
    def read_all_servos_menu(self):
        """Menu option to read all servo positions"""
        if not self.controller:
            print("✗ Not connected to controller!")
            return
            
        print("\n--- Read All Servos ---")
        if not self.detected_servos:
            print("No servos detected yet. Detecting now...")
            self.detected_servos = self.controller.detect_servos()
        
        if self.detected_servos:
            try:
                positions = self.controller.read_servo_positions(self.detected_servos)
                print("✓ All servo positions:")
                for servo_id, position in sorted(positions.items()):
                    print(f"   Servo {servo_id}: {position}")
            except Exception as e:
                print(f"✗ Error reading servos: {e}")
        else:
            print("✗ No servos available to read")
    
    def move_single_servo_menu(self):
        """Menu option to move a single servo"""
        if not self.controller:
            print("✗ Not connected to controller!")
            return
            
        print("\n--- Move Single Servo ---")
        try:
            servo_id = int(input("Enter servo ID: "))
            position = int(input("Enter position (0-65535): "))
            time_ms = int(input("Enter movement time (ms, default 1000): ") or "1000")
            
            if not (0 <= position <= 65535):
                print("✗ Position must be between 0 and 65535")
                return
                
            self.controller.move_servos([(servo_id, position)], time_ms)
            print(f"✓ Servo {servo_id} moving to position {position} over {time_ms}ms")
            
        except ValueError:
            print("✗ Invalid input values")
        except Exception as e:
            print(f"✗ Error moving servo: {e}")
    
    def move_multiple_servos_menu(self):
        """Menu option to move multiple servos"""
        if not self.controller:
            print("✗ Not connected to controller!")
            return
            
        print("\n--- Move Multiple Servos ---")
        try:
            num_servos = int(input("How many servos to move? "))
            servo_moves = []
            
            for i in range(num_servos):
                print(f"\nServo {i+1}:")
                servo_id = int(input("  Enter servo ID: "))
                position = int(input("  Enter position (0-65535): "))
                
                if not (0 <= position <= 65535):
                    print("✗ Position must be between 0 and 65535")
                    return
                    
                servo_moves.append((servo_id, position))
            
            time_ms = int(input("\nEnter movement time for all servos (ms, default 1000): ") or "1000")
            
            self.controller.move_servos(servo_moves, time_ms)
            print(f"✓ {len(servo_moves)} servos moving simultaneously over {time_ms}ms:")
            for servo_id, position in servo_moves:
                print(f"   Servo {servo_id} → position {position}")
                
        except ValueError:
            print("✗ Invalid input values")
        except Exception as e:
            print(f"✗ Error moving servos: {e}")
    
    def quick_test_menu(self):
        """Quick test menu with common positions"""
        if not self.controller:
            print("✗ Not connected to controller!")
            return
            
        print("\n--- Quick Test Menu ---")
        print("1. Move all servos to center (1500)")
        print("2. Move all servos to minimum (500)")
        print("3. Move all servos to maximum (2500)")
        print("4. Back to main menu")
        
        choice = input("Select option (1-4): ").strip()
        
        if choice == '4':
            return
            
        if not self.detected_servos:
            print("Detecting servos first...")
            self.detected_servos = self.controller.detect_servos()
        
        if not self.detected_servos:
            print("✗ No servos detected for quick test")
            return
        
        try:
            if choice == '1':
                position = 1500
                desc = "center"
            elif choice == '2':
                position = 500
                desc = "minimum"
            elif choice == '3':
                position = 2500
                desc = "maximum"
            else:
                print("✗ Invalid choice")
                return
            
            servo_moves = [(servo_id, position) for servo_id in self.detected_servos]
            self.controller.move_servos(servo_moves, 1500)
            print(f"✓ All {len(self.detected_servos)} servos moving to {desc} position ({position})")
            
        except Exception as e:
            print(f"✗ Error in quick test: {e}")
    
    def system_info_menu(self):
        """Show system information"""
        if not self.controller:
            print("✗ Not connected to controller!")
            return
            
        print("\n--- System Information ---")
        try:
            voltage = self.controller.get_battery_voltage()
            if voltage:
                print(f"Battery voltage: {voltage}mV ({voltage/1000:.1f}V)")
            else:
                print("Could not read battery voltage")
                
            if self.detected_servos:
                print(f"Detected servos: {self.detected_servos}")
            else:
                print("No servos detected yet (run detect first)")
                
        except Exception as e:
            print(f"✗ Error reading system info: {e}")
    
    def display_menu(self):
        """Display the main menu"""
        print("\n" + "="*50)
        print("    LSC SERVO CONTROLLER MENU")
        print("="*50)
        print("1.  Detect Servos")
        print("2.  Read Servo by ID")
        print("3.  Read All Servo Values")
        print("4.  Move Single Servo")
        print("5.  Move Multiple Servos")
        print("6.  Quick Test Menu")
        print("7.  System Information")
        print("8.  Reconnect Controller")
        print("9.  Exit")
        print("="*50)
    
    def run(self):
        """Run the interactive menu"""
        print("LSC Servo Controller Menu System")
        print("Connecting to controller...")
        
        if not self.connect():
            print("Failed to connect. Please check your connection and try again.")
            return
        
        try:
            while True:
                self.display_menu()
                choice = input("Enter your choice (1-9): ").strip()
                
                if choice == '1':
                    self.detect_servos_menu()
                elif choice == '2':
                    self.read_servo_by_id_menu()
                elif choice == '3':
                    self.read_all_servos_menu()
                elif choice == '4':
                    self.move_single_servo_menu()
                elif choice == '5':
                    self.move_multiple_servos_menu()
                elif choice == '6':
                    self.quick_test_menu()
                elif choice == '7':
                    self.system_info_menu()
                elif choice == '8':
                    self.disconnect()
                    if not self.connect():
                        print("Failed to reconnect!")
                elif choice == '9':
                    print("Exiting...")
                    break
                else:
                    print("✗ Invalid choice. Please select 1-9.")
                
                input("\nPress Enter to continue...")
                
        except KeyboardInterrupt:
            print("\n\nProgram interrupted by user")
        except Exception as e:
            print(f"\nUnexpected error: {e}")
        finally:
            self.disconnect()


def main():
    """Main function to start the menu system"""
    try:
        # You can change the port here if needed
        menu = ServoControllerMenu('/dev/ttyUSB0')
        menu.run()
    except Exception as e:
        print(f"Error starting menu system: {e}")

if __name__ == "__main__":
    main()

