import tkinter as tk
import subprocess
import threading

def run_shell_command(cmd, text_widget):
    process = subprocess.Popen(cmd, shell=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    for line in process.stdout:
        text_widget.insert(tk.END, line)
        text_widget.see(tk.END)
    process.stdout.close()
    process.wait()

def main():
    root = tk.Tk()
    root.title("Grid Detector Terminal Output")
    text = tk.Text(root, wrap=tk.WORD, font=("Consolas", 12), width=100, height=30)
    text.pack(expand=True, fill=tk.BOTH)

    # Run grid_detector.py in a thread and show its output
    thread = threading.Thread(target=run_shell_command, args=("python3 ~/ros2_ws/src/jetrover_description/scripts/grid_detector.py", text))
    thread.start()

    root.mainloop()

if __name__ == "__main__":
    main()