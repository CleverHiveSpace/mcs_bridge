from abc import ABC, abstractmethod


class TelemetryBridge(ABC):
    """Abstract base class for telemetry bridge implementations."""

    @abstractmethod
    def spin(self):
        """Start the bridge and begin processing telemetry data."""
        pass

    @abstractmethod
    def cleanup(self):
        """Clean up resources when shutting down."""
        pass
