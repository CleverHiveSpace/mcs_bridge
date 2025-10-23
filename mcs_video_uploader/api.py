import logging
import threading
import time
from typing import Any, Dict, Optional

import requests
import socketio

from .config import BACKEND_URL, VIDEO_WS_NAMESPACE, VIDEO_ACK_TIMEOUT

logger = logging.getLogger(__name__)


class VideoClient:
    """Client responsible for logging in and streaming video segments via WebSocket."""

    def __init__(
        self,
        backend_url: str = BACKEND_URL,
        namespace: str = VIDEO_WS_NAMESPACE,
    ):
        self.backend_url = backend_url
        self.namespace = namespace
        self.token: Optional[str] = None
        self.sio: Optional[socketio.Client] = None
        self._connected_event = threading.Event()

    def login(self, username: str, password: str) -> bool:
        url = f"{self.backend_url}/login"
        payload = {"username": username, "password": password}

        try:
            response = requests.post(url, json=payload, timeout=10)
            response.raise_for_status()
            data = response.json()
            self.token = data.get("accessToken")
            logger.info("Authenticated successfully as %s", username)
            return True
        except requests.exceptions.RequestException as exc:
            logger.error("Login failed: %s", exc)
            return False

    def connect(self) -> bool:
        self.sio = socketio.Client(
            logger=False,
            engineio_logger=False,
            ssl_verify=False,
        )

        @self.sio.event
        def connect():
            logger.debug("Connected to default namespace '/'")

        @self.sio.on("connect", namespace=self.namespace)
        def on_namespace_connect():
            logger.info("Connected to namespace %s", self.namespace)
            self._connected_event.set()

        @self.sio.on("disconnect", namespace=self.namespace)
        def on_namespace_disconnect():
            logger.info("Disconnected from namespace %s", self.namespace)
            self._connected_event.clear()

        @self.sio.on("connect_error", namespace=self.namespace)
        def on_namespace_connect_error(data):
            logger.error("Connection error on %s: %s", self.namespace, data)
            self._connected_event.clear()

        try:
            self.sio.connect(
                self.backend_url,
                namespaces=[self.namespace],
                transports=["websocket"],
                wait_timeout=10,
            )
        except Exception as exc:  # noqa: BLE001
            logger.error("Failed to establish WebSocket connection: %s", exc)
            self._connected_event.clear()
            return False

        if self._connected_event.wait(timeout=5):
            return True

        if self.sio.connected:
            logger.warning(
                "WebSocket session established but namespace %s did not confirm in time",
                self.namespace,
            )
            self._connected_event.set()
            return True

        logger.error("WebSocket connection did not become ready in time")
        return False

    def send_segment(self, payload: Dict[str, Any], timeout: float = VIDEO_ACK_TIMEOUT) -> Dict[str, Any]:
        if not self.sio or not self.sio.connected:
            raise RuntimeError("WebSocket client is not connected")

        ack_event = threading.Event()
        ack_response: Dict[str, Any] = {}

        def acknowledge(response: Any):
            ack_response["data"] = response
            ack_event.set()

        self.sio.emit("segment", payload, namespace=self.namespace, callback=acknowledge)

        if not ack_event.wait(timeout=timeout):
            raise TimeoutError("Timed out waiting for segment acknowledgement")

        return ack_response.get("data", {})

    def wait_until_connected(self, timeout: float = 10.0) -> bool:
        if self.sio and self.sio.connected:
            return True
        return self._connected_event.wait(timeout=timeout)

    def disconnect(self):
        if self.sio and self.sio.connected:
            self.sio.disconnect()
            # allow disconnect event to propagate
            time.sleep(0.1)
