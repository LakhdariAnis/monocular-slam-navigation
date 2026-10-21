import subprocess
import time
import os
import sys

def main():
    # Make sure we are in the monitoring directory
    script_dir = os.path.dirname(os.path.abspath(__file__))
    os.chdir(script_dir)

    # pass monitoring/.env to the child scripts (docker compose reads it on its own)
    if os.path.exists(".env"):
        with open(".env") as f:
            for line in f:
                line = line.strip()
                if line and not line.startswith("#") and "=" in line:
                    key, value = line.split("=", 1)
                    os.environ.setdefault(key, value)
    if "INFLUXDB_TOKEN" not in os.environ:
        print("INFLUXDB_TOKEN not set: cp .env.example .env and fill it in.")
        sys.exit(1)

    print("Starting docker-compose services (Mosquitto and InfluxDB)...")
    try:
        subprocess.run(["docker-compose", "up", "-d"], check=True)
    except FileNotFoundError:
        print("docker-compose command not found, make sure it is installed.")
        # We might continue if they are already running or if the user uses a different docker compose command
    except subprocess.CalledProcessError as e:
        print(f"Failed to start docker-compose: {e}")
        # Proceed anyway just in case they are already running
        
    print("Waiting for services to be ready...")
    time.sleep(3)

    scripts = [
        "mock/simulator.py",
        "influx/writer.py",
        "anomaly/detector.py",
        "dashboard/backend/main.py"
    ]

    terminals = ["kitty", "alacritty", "gnome-terminal", "konsole", "xfce4-terminal", "terminator", "xterm"]
    term_cmd = None

    for t in terminals:
        if subprocess.call(["which", t], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL) == 0:
            term_cmd = t
            break

    if term_cmd is None:
        print("Error: Could not find a suitable terminal emulator.")
        print("You can manually run the following scripts:")
        for script in scripts:
            print(f"python3 {script}")
        sys.exit(1)

    print(f"Using terminal emulator: {term_cmd}")

    for script in scripts:
        print(f"Launching {script} in a new terminal...")
        if term_cmd == "kitty":
            subprocess.Popen([term_cmd, "bash", "-c", f"python3 {script}; exec bash"])
        elif term_cmd == "alacritty":
            subprocess.Popen([term_cmd, "-e", "bash", "-c", f"python3 {script}; exec bash"])
        elif term_cmd == "gnome-terminal":
            subprocess.Popen([term_cmd, "--", "bash", "-c", f"python3 {script}; exec bash"])
        else:
            subprocess.Popen([term_cmd, "-e", f"python3 {script}; bash"])
        
        # Give a slight delay to allow each process to start cleanly
        time.sleep(1)

    print("All components launched successfully.")

if __name__ == "__main__":
    main()
