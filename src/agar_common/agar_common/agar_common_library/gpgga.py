import enum
from datetime import datetime

class arr2data(object):
    def __init__(self, data):
        data = data.strip().split(b',')
        self.type               = data[0].decode()
        self.gps_status         = int(data[6])
        self.gps_status_label   = self.get_status_lbl(self.gps_status)
        self.utc_time           = datetime.strptime(data[1].decode(), "%H%M%S.%f").time() if self.gps_status > 0 else datetime.now().time()
        self.latitude           = self.convert_latitude(data[2]) if self.gps_status > 0 else 0
        self.latitude_hem       = data[3].decode()
        self.longitude          = self.convert_longitude(data[4]) if self.gps_status > 0 else 0
        self.longitude_hem      = data[5].decode()
        self.no_satellites      = int(data[7]) if self.gps_status > 0 else 0
        self.hdop_precision     = float(data[8]) if self.gps_status > 0 else 0
        self.altitude           = float(data[9]) if self.gps_status > 0 else 0

    # def decimal_degrees(self, x):
    #     degrees = int(x) // 100
    #     minutes = x - 100 * degrees
    #     return degrees + minutes / 60
    
    def safe_float(self, field):
        try:
            return float(field)
        except ValueError:
            return float('NaN')
    
    def convert_latitude(self, field):
        return self.safe_float(field[0:2]) + self.safe_float(field[2:]) / 60.0

    def convert_longitude(self, field):
        return self.safe_float(field[0:3]) + self.safe_float(field[3:]) / 60.0
        
    def get_status_lbl(self, status):
        try: 
            return gps_status(status).name
        except:
            return gps_status(0).name

class gps_status(enum.Enum):
    NO_POSITIONING                      = 0
    SINGLE_POINT_POSITIONING            = 1
    SBAS_DIFFERENTIAL_POSIIONING        = 2
    RTK_FIXED_SOLUTION                  = 4
    RTK_FLOATING_POINT_SOLUTION         = 5
    INTERTIAL_NAVIGATION_POSITIONING    = 6
    FIXED_POSITION                      = 7