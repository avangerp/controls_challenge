from . import BaseController
import numpy as np

class Controller(BaseController):
    
    def __init__(self,):
        self.p = 0.3
        self.i = 0.05
        self.d = -0.1
        self.error_integral = 0
        self.prev_error = 0

        self.steer_factor = 12.0 # lat accel to steer command factor
        self.steer_sat_v = 20.0 # saturate v measurements for steering
        self.steer_command_sat = 0.5 # feedforward command magnitude saturation

    def update(self, target_lataccel, current_lataccel, state, future_plan):
        error = (target_lataccel - current_lataccel)
        self.error_integral += error
        error_diff = error - self.prev_error
        self.prev_error = error

        u_pid = self.p * error + self.i * self.error_integral + self.d * error_diff

        # estimate some steer command based on available measurements
        steer_accel_target = (target_lataccel - state.roll_lataccel)
        steer_command = (steer_accel_target * self.steer_factor /
            max(self.steer_sat_v, state.v_ego))

        # saturate command
        steer_command = max(-self.steer_command_sat,
                            min(steer_command, self.steer_command_sat))
        
        K_ff = 0.8 # feed forward input gain
        u_ff = K_ff * steer_command
        
        return u_pid + u_ff