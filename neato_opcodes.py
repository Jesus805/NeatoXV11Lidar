import enum


class LidarParentOps(enum.Enum):
    '''
    Opcodes sent to the LIDAR reader
    '''
    ON   = 0x01 # Turn the LIDAR on
    OFF  = 0x02 # Turn the LIDAR off
    KILL = 0xFF # Application shutdown


class LidarChildOps(enum.Enum):
    '''
    Opcodes received from the LIDAR reader
    '''
    DATA = 0x01 # LIDAR data ready
    ERR  = 0xFF # Error occured
