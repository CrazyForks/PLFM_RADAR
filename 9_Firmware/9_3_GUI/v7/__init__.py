"""
v7 — PLFM Radar GUI V7 (PyQt6 edition).

Re-exports all public classes and functions from sub-modules for convenient
top-level imports:

    from v7 import RadarDashboard, RadarTarget, RadarSettings, ...
"""

# Models / constants
from .models import (
    RadarTarget,
    RadarSettings,
    GPSData,
    ProcessingConfig,
    TileServer,
    DARK_BG, DARK_FG, DARK_ACCENT, DARK_HIGHLIGHT, DARK_BORDER,
    DARK_TEXT, DARK_BUTTON, DARK_BUTTON_HOVER,
    DARK_TREEVIEW, DARK_TREEVIEW_ALT,
    DARK_SUCCESS, DARK_WARNING, DARK_ERROR, DARK_INFO,
    USB_AVAILABLE, FTDI_AVAILABLE, SCIPY_AVAILABLE,
    SKLEARN_AVAILABLE, FILTERPY_AVAILABLE, CRCMOD_AVAILABLE,
)

# Hardware interfaces
from .hardware import (
    FT2232HQInterface,
    STM32USBInterface,
)

# Processing pipeline
from .processing import (
    RadarProcessor,
    RadarPacketParser,
    USBPacketParser,
    apply_pitch_correction,
)

# Workers and simulator
from .workers import (
    RadarDataWorker,
    GPSDataWorker,
    TargetSimulator,
    polar_to_geographic,
)

# Map widget
from .map_widget import (
    MapBridge,
    RadarMapWidget,
)

# Main dashboard
from .dashboard import (
    RadarDashboard,
    RangeDopplerCanvas,
)

__all__ = [
    # models
    "RadarTarget", "RadarSettings", "GPSData", "ProcessingConfig", "TileServer",
    "DARK_BG", "DARK_FG", "DARK_ACCENT", "DARK_HIGHLIGHT", "DARK_BORDER",
    "DARK_TEXT", "DARK_BUTTON", "DARK_BUTTON_HOVER",
    "DARK_TREEVIEW", "DARK_TREEVIEW_ALT",
    "DARK_SUCCESS", "DARK_WARNING", "DARK_ERROR", "DARK_INFO",
    "USB_AVAILABLE", "FTDI_AVAILABLE", "SCIPY_AVAILABLE",
    "SKLEARN_AVAILABLE", "FILTERPY_AVAILABLE", "CRCMOD_AVAILABLE",
    # hardware
    "FT2232HQInterface", "STM32USBInterface",
    # processing
    "RadarProcessor", "RadarPacketParser", "USBPacketParser",
    "apply_pitch_correction",
    # workers
    "RadarDataWorker", "GPSDataWorker", "TargetSimulator",
    "polar_to_geographic",
    # map
    "MapBridge", "RadarMapWidget",
    # dashboard
    "RadarDashboard", "RangeDopplerCanvas",
]
