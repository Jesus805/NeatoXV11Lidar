import enum


class LidarParentOps(enum.Enum):
    """
    Opcodes sent to the LIDAR reader
    """
    # Turn the LIDAR on
    ON   = 0x01
    # Turn the LIDAR off
    OFF  = 0x02
    # Application shutdown
    KILL = 0xFF


class LidarChildOps(enum.Enum):
    """
    Opcodes received from the LIDAR reader
    """
    # LIDAR data ready
    DATA = 0x01
    # Error occured
    ERR  = 0xFF
