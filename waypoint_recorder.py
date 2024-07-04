import tkinter as tk
from tkinter import messagebox
from pymavlink import mavutil
import time
import math

class WaypointRecorderApp:
    def __init__(self, master):
        self.master = master
        master.title("Waypoint Recorder")

        self.is_recording = False
        self.waypoints = []

        self.label = tk.Label(master, text="Waypoint Recorder")
        self.label.pack()

        self.record_button = tk.Button(master, text="Record", command=self.start_recording)
        self.record_button.pack()

        self.stop_button = tk.Button(master, text="Stop Recording", command=self.stop_recording)
        self.stop_button.pack()

        self.result_label = tk.Label(master, text="")
        self.result_label.pack()

        # Connect to the flight controller
        self.master_connection = mavutil.mavlink_connection('COM9', baud=115200)  # Replace COM3 with your actual COM port
        self.master_connection.wait_heartbeat()

    def start_recording(self):
        if not self.is_recording:
            self.is_recording = True
            self.record_waypoints()

    def stop_recording(self):
        self.is_recording = False
        self.calculate_mission_facts()

    def record_waypoints(self):
        if self.is_recording:
            lat, lon, alt = self.get_gps_position()
            self.waypoints.append((lat, lon, alt))
            print(f"Waypoint recorded at lat: {lat}, lon: {lon}, alt: {alt}")

            # Continue recording waypoints every 5 seconds
            self.master.after(5000, self.record_waypoints)

    def get_gps_position(self):
        gps_msg = self.master_connection.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        lat = gps_msg.lat / 1e7
        lon = gps_msg.lon / 1e7
        alt = gps_msg.alt / 1000
        return lat, lon, alt

    def calculate_mission_facts(self):
        if len(self.waypoints) < 2:
            messagebox.showinfo("Mission Facts", "Not enough waypoints recorded.")
            return

        total_distance = 0
        total_altitude_change = 0
        previous_point = self.waypoints[0]
        
        for point in self.waypoints[1:]:
            total_distance += self.calculate_distance(previous_point, point)
            total_altitude_change += abs(point[2] - previous_point[2])
            previous_point = point
        
        estimated_flight_time = len(self.waypoints) * 5  # Assuming each waypoint is recorded every 5 seconds

        result_text = (f"Total Distance: {total_distance:.2f} meters\n"
                       f"Total Altitude Change: {total_altitude_change:.2f} meters\n"
                       f"Estimated Flight Time: {estimated_flight_time:.2f} seconds")

        self.result_label.config(text=result_text)
        messagebox.showinfo("Mission Facts", result_text)

    def calculate_distance(self, point1, point2):
        R = 6371000  # Radius of the Earth in meters
        lat1, lon1, _ = point1
        lat2, lon2, _ = point2

        phi1 = math.radians(lat1)
        phi2 = math.radians(lat2)
        delta_phi = math.radians(lat2 - lat1)
        delta_lambda = math.radians(lon2 - lon1)

        a = math.sin(delta_phi / 2.0) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(delta_lambda / 2.0) ** 2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

        distance = R * c
        return distance

if __name__ == "__main__":
    root = tk.Tk()
    app = WaypointRecorderApp(root)
    root.mainloop()
