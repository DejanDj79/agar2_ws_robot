csv_header = ['latitude', 'longitude', 'reverse', 'reverse_turn', 'tank', 'speed', 'agar_action', 'combo_value', 'combo_text', 'param_1', 'param_2', 'param_3', 'param_4', 'visited']

class waypoint_csv:
    def __init__(self, row):
        self.latitude               = float(row.get('latitude'))
        self.longitude              = float(row.get('longitude'))
        self.reverse                = True if row.get('reverse').lower() == 'true' else False
        self.reverse_turn           = True if row.get('reverse_turn').lower() == 'true' else False
        self.tank                   = True if row.get('tank').lower() == 'true' else False
        self.speed                  = float(row.get('speed'))
        self.agar_action            = int(row.get('agar_action'))
        self.combo_value            = int(row.get('combo_value'))
        self.combo_text             = row.get('combo_text')
        self.param_1                = int(row.get('param_1'))
        self.param_2                = int(row.get('param_2'))
        self.param_3                = int(row.get('param_3'))
        self.param_4                = int(row.get('param_4'))
        self.visited                = True if row.get('visited').lower() == 'true' else False

class waypoint:
    def __init__(self, idx, start_x, start_y, end_x, end_y, reverse, reverse_turn, tank, speed, agar_action, combo_value, combo_text,
                 param_1, param_2, param_3, param_4, visited, generated_wp):
        self.idx                    = idx
        self.start_x                = start_x
        self.start_y                = start_y 
        self.end_x                  = end_x
        self.end_y                  = end_y
        self.reverse                = reverse
        self.reverse_turn           = reverse_turn
        self.tank                   = tank
        self.speed                  = speed
        self.agar_action            = agar_action
        self.combo_value            = combo_value
        self.combo_text             = combo_text
        self.param_1                = param_1
        self.param_2                = param_2
        self.param_3                = param_3
        self.param_4                = param_4
        self.visited                = visited
        self.generated_wp           = generated_wp