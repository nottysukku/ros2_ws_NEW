from xarmapi import LSCServoController

def get_positions(servo_ids):
    """Prompt user for positions for each servo ID."""
    servo_moves = []
    for servo_id in servo_ids:
        while True:
            val = input(f"Enter position for Servo {servo_id} (200-750 or Enter to skip): ").strip()
            if val == "":
                break  # skip this servo
            if val.lower() == "exit":
                return None
            try:
                pos = int(val)
                if 200 <= pos <= 750:
                    servo_moves.append((servo_id, pos))
                    break
                else:
                    print("Value out of range (200-750). Try again.")
            except ValueError:
                print("Invalid input. Try again.")
    return servo_moves

def main():
    with LSCServoController('/dev/ttyUSB0') as controller:
        print("Detecting servos...")
        servo_ids = controller.detect_servos()
        if not servo_ids:
            print("No servos detected!")
            return
        print(f"Detected servos: {servo_ids}")

        print("\nType 'exit' at any prompt to quit.")
        while True:
            servo_moves = get_positions(servo_ids)
            if servo_moves is None:
                print("Exiting.")
                break
            if servo_moves:
                controller.move_servos(servo_moves, 300)
                print(f"Moved: {servo_moves}")
            else:
                print("No input provided; nothing moved.")

if __name__ == "__main__":
    main()

