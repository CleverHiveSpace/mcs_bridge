import time
import requests
import socketio
from .config import BACKEND_URL, WS_NAMESPACE


class MCSClient:
    def __init__(self):
        self.backend_url = BACKEND_URL
        self.namespace = WS_NAMESPACE
        self.token = None
        self.sio = None

    def login(self, username, password):
        url = f"{self.backend_url}/login"
        payload = {"username": username, "password": password}

        try:
            response = requests.post(url, json=payload)
            response.raise_for_status()
            data = response.json()
            self.token = data.get("accessToken")
            print(f"✓ Successfully logged in as {username}")
            return True
        except requests.exceptions.RequestException as e:
            print(f"✗ Login failed: {e}")
            return False

    def connect(self):
        self.sio = socketio.Client(
            logger=False,
            engineio_logger=False,
            ssl_verify=False,
        )

        @self.sio.event
        def connect():
            print(f"✓ Connected to WebSocket namespace {WS_NAMESPACE}")

        @self.sio.event
        def disconnect():
            print("✓ Disconnected from WebSocket")

        @self.sio.event
        def connect_error(data):
            print("✗ Connection error:")
            print(f"   Type: {type(data)}")
            print(f"   Data: {data}")

        try:
            self.sio.connect(
                self.backend_url,
                namespaces=[self.namespace],
                transports=["websocket"],
                wait_timeout=10,
            )

            if not self.sio.connected:
                time.sleep(2)

            return self.sio.connected
        except Exception as e:
            print(f"✗ Connection error: {e}")
            return False

    def send_position(self, rover_id, x, y, z, angle):
        if not self.sio or not self.sio.connected:
            return False

        position_data = {
            "roverId": rover_id,
            "ts_ns": int(time.time() * 1_000_000_000),
            "x": x,
            "y": y,
            "z": z,
            "angle": angle,
        }

        self.sio.emit("position", position_data, namespace=self.namespace)
        return True

    def send_sensor(self, rover_id, name, value, unit):
        if not self.sio or not self.sio.connected:
            return False

        sensor_data = {
            "roverId": rover_id,
            "ts_ns": int(time.time() * 1_000_000_000),
            "name": name,
            "value": value,
            "unit": unit,
        }

        self.sio.emit("sensor", sensor_data, namespace=self.namespace)
        return True

    def disconnect(self):
        if self.sio and self.sio.connected:
            self.sio.disconnect()
