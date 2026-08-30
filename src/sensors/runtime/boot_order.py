"""Architecture notes — MegaCollect → hik-sensors mapping."""

# Lifecycle order on a real cell (start → stop reverse):
BOOT_ORDER = [
    "bus",        # socat /tmp/ttyUR ↔ robot:54321 + by-id USB
    "arm",        # ZMQ server :6001 (ur|elite)
    "gello",      # Dynamixel leader client
    "gripper",    # DH AG95 (often opened inside arm server)
    "realsense",  # left / right / middle
    "pipeline",   # collection_flag + mosaic/joint SHM + remote :12345
]

# Explicitly out of MegaCollect main path (extension stubs):
EXTENSION_KINDS = ["ft", "tactile"]
