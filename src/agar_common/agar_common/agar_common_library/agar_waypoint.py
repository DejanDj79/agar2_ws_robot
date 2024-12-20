csv_header = ['latitude', 'longitude']

class waypoint_csv:
    def __init__(self, row):
        self.latitude               = float(row.get('latitude'))
        self.longitude              = float(row.get('longitude'))

class waypoint:
    def __init__(self, idx, start_x, start_y, end_x, end_y):
        self.idx                    = idx
        self.start_x                = start_x
        self.start_y                = start_y 
        self.end_x                  = end_x
        self.end_y                  = end_y