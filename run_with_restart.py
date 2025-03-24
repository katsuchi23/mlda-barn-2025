#!/usr/bin/env python
import subprocess
import sys
import time

def main():
    # Get command line arguments to pass to the main script
    args = sys.argv[1:]
    
    max_restarts = 3
    restart_count = 0
    
    while restart_count < max_restarts:
        print(f"Run attempt {restart_count + 1}/{max_restarts}")
        
        # Run the main script with all arguments passed to this script
        command = ["python", "/jackal_ws/src/mlda-barn-2025/run_rviz_kul.py"] + args
        process = subprocess.Popen(command)
        
        # Wait for the process to complete
        return_code = process.wait()
        
        if return_code == 144:
            # Collision occurred, restart
            restart_count += 1
            print(f"Collision detected. Restarting program... ({restart_count}/{max_restarts})")
            time.sleep(2)  # Give system time to clean up
        elif return_code == 200:
            # Success, exit with success
            print("Navigation succeeded!")
            sys.exit(0)
        else:
            # Other error, exit with the same code
            print(f"Program exited with code {return_code}")
            sys.exit(return_code)
    
    print(f"Maximum restart attempts ({max_restarts}) reached. Exiting.")
    sys.exit(1)

if __name__ == "__main__":
    main()
