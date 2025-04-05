import math
class MotorPositionTracker:
    def __init__(self):
        self.prev_pos = None       # last raw angle read in [-π, +π]
        self.unwrapped_pos = None  # continuous angle result

    def update(self, motor_pos):
        """
        Given the newest 'wrapped' angle (motor_pos) in radians, return
        the continuous 'unwrapped' angle.  The absolute encoder itself
        is always in the range [-π, +π], but we accumulate the net motion
        so you can move beyond ±π up to however many turns you want.
        """
        # On the very first call, just initialize:
        if self.prev_pos is None:
            self.prev_pos = motor_pos
            self.unwrapped_pos = motor_pos
            return self.unwrapped_pos
        
        # 1) Compute the naive difference:
        delta = motor_pos - self.prev_pos
        
        # 2) If the difference is bigger than +π, subtract 2π
        #    If the difference is smaller than -π, add 2π
        if delta > math.pi:
            delta -= 2 * math.pi
        elif delta < -math.pi:
            delta += 2 * math.pi
        
        # 3) Add that "shortest path" difference to the previously unwrapped angle
        self.unwrapped_pos += delta
        
        # 4) Store current raw position for next iteration
        self.prev_pos = motor_pos
        
        return self.unwrapped_pos
    
motor_tracker = MotorPositionTracker()