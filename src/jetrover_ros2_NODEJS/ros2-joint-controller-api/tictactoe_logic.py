import random
import os
import requests
import time

class TicTacToe:
    def __init__(self):
        self.board = [' ' for _ in range(9)]  # 3x3 board represented as 1D list
        self.human = 'X'
        self.computer = 'O'
        
        # API configuration for robot arm control
        self.api_url = os.getenv('ROBOT_API_URL', 'https://ros2-joint-controller-api.onrender.com')
        
        # Joint positions for each tic-tac-toe tile (1-9)
        self.tile_positions = {
            1: {'joint1': 0.24, 'joint2': 0.61, 'joint3': 0.05, 'joint4': 0.30, 'joint5': -0.80},
            2: {'joint1': 0.05, 'joint2': 0.40, 'joint3': 0.34, 'joint4': 0.41, 'joint5': -0.50},
            3: {'joint1': -0.20, 'joint2': 0.40, 'joint3': 0.34, 'joint4': 0.41, 'joint5': -0.50},
            4: {'joint1': 0.23, 'joint2': 0.36, 'joint3': 0.31, 'joint4': 0.16, 'joint5': -0.26},
            5: {'joint1': 0.02, 'joint2': 0.36, 'joint3': 0.31, 'joint4': 0.11, 'joint5': -0.26},
            6: {'joint1': -0.20, 'joint2': 0.36, 'joint3': 0.31, 'joint4': 0.11, 'joint5': -0.26},
            7: {'joint1': 0.23, 'joint2': 0.36, 'joint3': 0.31, 'joint4': -0.22, 'joint5': -0.26},
            8: {'joint1': -0.02, 'joint2': 0.35, 'joint3': 0.31, 'joint4': -0.26, 'joint5': -0.26},
            9: {'joint1': -0.23, 'joint2': 0.35, 'joint3': 0.31, 'joint4': -0.26, 'joint5': -0.26}
        }
        
        # Predefined poses
        self.poses = {
            'home': {'joint1': 0.0, 'joint2': 0.0, 'joint3': 0.0, 'joint4': 0.0, 'joint5': 0.0},
            'pickup': {'joint1': 0.0, 'joint2': -0.5, 'joint3': 0.8, 'joint4': 0.5, 'joint5': 0.0},
            'waiting': {'joint1': 0.0, 'joint2': -1.0, 'joint3': 1.5, 'joint4': 0.0, 'joint5': 0.0}
        }
    
    def display_board(self):
        """Display the current board state"""
        os.system('clear' if os.name == 'posix' else 'cls')  # Clear screen
        print("\n" + "="*30)
        print("    TIC-TAC-TOE GAME")
        print("="*30)
        print("\nPosition numbers:")
        print(" 1 | 2 | 3 ")
        print("-----------")
        print(" 4 | 5 | 6 ")
        print("-----------")
        print(" 7 | 8 | 9 ")
        print("\nCurrent board:")
        print(f" {self.board[0]} | {self.board[1]} | {self.board[2]} ")
        print("-----------")
        print(f" {self.board[3]} | {self.board[4]} | {self.board[5]} ")
        print("-----------")
        print(f" {self.board[6]} | {self.board[7]} | {self.board[8]} ")
        print()
    
    def move_robot_arm(self, joint_positions, description="Moving arm"):
        """Send joint commands to robot arm via API"""
        try:
            print(f"🤖 {description}...")
            response = requests.post(f'{self.api_url}/api/joints/move', 
                                   json={'joints': joint_positions}, 
                                   timeout=10)
            if response.ok:
                print(f"✅ Arm moved successfully")
                return True
            else:
                print(f"❌ Failed to move arm: {response.text}")
                return False
        except requests.exceptions.RequestException as e:
            print(f"❌ Robot arm API error: {e}")
            return False
    
    def robot_sequence_to_tile(self, tile_number):
        """Execute robot sequence: pickup → move to tile → place"""
        print(f"\n🤖 Computer moving to tile {tile_number}...")
        
        # Step 1: Move to pickup position (simulate picking up piece)
        self.move_robot_arm(self.poses['pickup'], "Moving to pickup position")
        time.sleep(1.5)
        
        # Step 2: Move to target tile
        target_position = self.tile_positions[tile_number]
        self.move_robot_arm(target_position, f"Moving to tile {tile_number}")
        time.sleep(2)
        
        # Step 3: Return to waiting position (arm up)
        self.move_robot_arm(self.poses['waiting'], "Moving to waiting position")
        time.sleep(1)
        
        print(f"✅ Computer placed 'O' on tile {tile_number}")
    
    def human_turn_start(self):
        """Move arm to waiting position for human turn"""
        print("\n👤 Your turn - arm moving to waiting position...")
        self.move_robot_arm(self.poses['waiting'], "Moving arm up for human turn")
        time.sleep(1)
    
    def is_valid_move(self, position):
        """Check if the move is valid"""
        return 1 <= position <= 9 and self.board[position - 1] == ' '
    
    def make_move(self, position, player):
        """Make a move on the board"""
        if self.is_valid_move(position):
            self.board[position - 1] = player
            return True
        return False
    
    def check_winner(self):
        """Check if there's a winner"""
        # Winning combinations (indices)
        win_patterns = [
            [0, 1, 2], [3, 4, 5], [6, 7, 8],  # Rows
            [0, 3, 6], [1, 4, 7], [2, 5, 8],  # Columns
            [0, 4, 8], [2, 4, 6]              # Diagonals
        ]
        
        for pattern in win_patterns:
            if (self.board[pattern[0]] == self.board[pattern[1]] == 
                self.board[pattern[2]] != ' '):
                return self.board[pattern[0]]
        return None
    
    def is_board_full(self):
        """Check if the board is full"""
        return ' ' not in self.board
    
    def get_empty_positions(self):
        """Get list of empty positions"""
        return [i + 1 for i, spot in enumerate(self.board) if spot == ' ']
    
    def minimax(self, is_maximizing, depth=0):
        """Minimax algorithm for AI decision making"""
        winner = self.check_winner()
        
        # Terminal cases
        if winner == self.computer:
            return 10 - depth
        elif winner == self.human:
            return depth - 10
        elif self.is_board_full():
            return 0
        
        if is_maximizing:
            best_score = float('-inf')
            for pos in self.get_empty_positions():
                self.board[pos - 1] = self.computer
                score = self.minimax(False, depth + 1)
                self.board[pos - 1] = ' '
                best_score = max(score, best_score)
            return best_score
        else:
            best_score = float('inf')
            for pos in self.get_empty_positions():
                self.board[pos - 1] = self.human
                score = self.minimax(True, depth + 1)
                self.board[pos - 1] = ' '
                best_score = min(score, best_score)
            return best_score
    
    def computer_move(self):
        """AI makes the best move using minimax algorithm"""
        best_score = float('-inf')
        best_move = None
        
        for pos in self.get_empty_positions():
            self.board[pos - 1] = self.computer
            score = self.minimax(False)
            self.board[pos - 1] = ' '
            
            if score > best_score:
                best_score = score
                best_move = pos
        
        if best_move:
            self.make_move(best_move, self.computer)
            print(f"Computer chooses position {best_move}")
            # Execute robot arm sequence to place piece
            self.robot_sequence_to_tile(best_move)
    
    def human_move(self):
        """Handle human player's move"""
        while True:
            try:
                position = int(input("Enter your move (1-9): "))
                if self.is_valid_move(position):
                    self.make_move(position, self.human)
                    print(f"You placed 'X' on tile {position}")
                    break
                else:
                    print("Invalid move! Position already taken or out of range.")
            except ValueError:
                print("Please enter a number between 1 and 9.")
    
    def play_game(self):
        """Main game loop"""
        print("Welcome to Tic-Tac-Toe with Robot Arm!")
        print("You are 'X' and the computer is 'O'")
        print("Enter position numbers 1-9 to make your move")
        print("The robot arm will move when the computer plays!")
        
        # Initialize arm to home position
        print("\n🤖 Initializing robot arm...")
        self.move_robot_arm(self.poses['home'], "Moving to home position")
        time.sleep(2)
        
        input("Press Enter to start...")
        
        while True:
            # Display current board
            self.display_board()
            
            # Check for winner or tie
            winner = self.check_winner()
            if winner:
                if winner == self.human:
                    print("🎉 Congratulations! You won!")
                    # Move arm to celebrate
                    self.move_robot_arm(self.poses['waiting'], "Victory celebration!")
                else:
                    print("💻 Computer wins! Better luck next time!")
                    # Move arm to celebrate
                    self.move_robot_arm(self.poses['home'], "Computer celebration!")
                break
            
            if self.is_board_full():
                print("🤝 It's a tie! Good game!")
                self.move_robot_arm(self.poses['home'], "Game ended - returning home")
                break
            
            # Human turn - move arm up
            self.human_turn_start()
            
            # Human move
            print("Your turn:")
            self.human_move()
            
            # Check for winner after human move
            winner = self.check_winner()
            if winner == self.human:
                self.display_board()
                print("🎉 Congratulations! You won!")
                self.move_robot_arm(self.poses['waiting'], "Victory celebration!")
                break
            
            if self.is_board_full():
                self.display_board()
                print("🤝 It's a tie! Good game!")
                self.move_robot_arm(self.poses['home'], "Game ended - returning home")
                break
            
            # Computer move with robot arm sequence
            print("\nComputer's turn...")
            self.computer_move()
            input("Press Enter to continue...")
        
        # Ask to play again
        while True:
            play_again = input("\nWould you like to play again? (y/n): ").lower()
            if play_again == 'y':
                self.__init__()  # Reset the game
                self.play_game()
                break
            elif play_again == 'n':
                print("Thanks for playing! Goodbye! 👋")
                # Return arm to home position
                self.move_robot_arm(self.poses['home'], "Final return to home")
                break
            else:
                print("Please enter 'y' for yes or 'n' for no.")

def main():
    game = TicTacToe()
    game.play_game()

if __name__ == "__main__":
    main()