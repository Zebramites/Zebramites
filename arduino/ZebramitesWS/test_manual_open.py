import websocket
import time
import threading

# Global variable to track the current servo angle
current_angle = 0

def on_open(ws):
    # Send initial commands to set servo position
    ws.send("s1;180")  # Start at 0 degrees
    print("Initial command sent.")

def send_angle(ws, angle):
    # Send the current angle to the server
    ws.send(f"s1;{angle}")

def angle_incrementer(ws):
    global current_angle
    while True:
        if current_angle <= 180:  # Limit the angle to 180 degrees
            send_angle(ws, current_angle)
            current_angle += 1  # Increment the angle
        time.sleep(0.02)  # Sleep for 20ms (50 Hz)

if __name__ == "__main__":
    websocket.enableTrace(True)
    print("Connecting to server...")
    ws = websocket.WebSocket()
    ws.connect("ws://192.168.4.1:9000/") # Replace with ESP32 IP
    ws.send("s1;0")
    time.sleep(2)
    for i in range(180):
        ws.send(f"s1;{i}")
        time.sleep(0.02)
