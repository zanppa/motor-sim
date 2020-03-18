# -*- coding: utf-8 -*-
"""
Created on Mon Jul 10 21:01:52 2017

"""

# https://e2e.ti.com/blogs_/b/motordrivecontrol/archive/2013/11/08/generate-your-own-commutation-table-trapezoidal-control-3-phase-bldc-motors-using-hall-sensors
# https://e2e.ti.com/blogs_/b/motordrivecontrol/archive/2013/12/20/generate-your-own-commutation-table-part-2


from control import Control

class Control_BLDC_Hall(Control):

    # Controller parameters
    Kp = 0.05            # Proportional gain
    Ti = 10e-3          # Integration time [s]
    Umax = 50.0         # Maximum output voltage (+ and -)
    speed = 0.0         # Filtered rotation speed [1/s]
    speed_est = 0.0     # Last estimated speed value [1/s]
    last_comm = 0.0     # Time of last commutation [s]
    Sf = 1.0/500       # Speed filter time constant [s]
    error = 0.0         # Error of speed - reference
    integral = 0.0      # Integrator value
    intmax = 5.0        # Integrator anti-windup

    # Controller outputs
    Uout = 0.0          # Output voltage amplitude
    phase = 0           # Current commutation phase
    last_order = 0      # Hall sensor order

    # Motor parameters
    P = 2               # Motor polepairs    
    speed = 0.0         # Measured rotation speed
    
    # Commutation sequence
    sequence = [    \
        [float('NaN'), float('NaN'), float('NaN')],  # Zero (no position)  \
        [1, float('NaN'), -1],     # HA         \
        [-1, 1, float('NaN')],     # HB         \
        [float('NaN'), 1, -1],     # HA + HB    \
        [float('NaN'), -1, 1],     # HC         \
        [1, -1, float('NaN')],     # HC+HA      \
        [-1, float('NaN'), 1],     # HC+HB      \
        [float('NaN'), float('NaN'), float('NaN')]     # All positions 1    \
        ]

    # Positive rotation direction of hall sensors
    # If calculated A + 2*B + 4*C
    hall_order = [0, 1, 3, 2, 5, 6, 4, 0]

    def __init__(self, params):
        Control.__init__(self, params)

        self.params = { \
            'Kp' : 'Proportional gain',     \
            'Ki' : 'Integration gain',      \
            'Ti' : 'Integration time',      \
            'Imax' : 'Integrator maximum value',         \
            'P' : 'Motor polepairs',        \
            'Sf' : 'Speed filter time constant [s]',     \
            'Umax' : 'Maximum output voltage amplitude' \
            }

        self.set_parameters(params)
        
        self.output = [0.0, 0.0, 0.0]
        
        return

    def set_parameters(self, params):
        if 'kp' in params:
            self.Kp = params['Kp']

        if 'Ki' in params:
            self.Ti = self.Kp / params['Ki']

        if 'Ti' in params:
            self.Ti = params['Ti']

        if 'Imax' in params:
            self.intmax = params['Imax']
                        
        if 'P' in params:
            self.P = params['P']

        if 'Umax' in params:
            self.Umax = params['Umax']


        if 'Sf' in params:
            self.Sf = params['Sf']

        return
     
     
    def get_parameters(self):
        params = Control.get_parameters(self)
        
        params.update({     \
            'Kp' : self.Kp,     \
            'Ki' : self.Kp / self.Ki, \
            'Ti' : self.Ti,   \
            'P' : self.P,   \
            'Umax' : self.Umax,      \
            'Sf' : self.Sf      \
            })
        
        return params
        
    
    def set_motor(self, motor):
        
        if motor.get_config_word('Name') != 'BLDC':
            print('Motor type not BLDC')
            return False
        
        Control.set_motor(self, motor)

        return True
    
    
    def calculate(self, dt):
        # Get hall-sensor values
        hall = self.motor.get_hall()
        
        # Convert hall sensor bits to number 1...6
        # which then corresponds the commutation phase
        last_phase = self.phase
        self.phase = hall[0] + hall[1] * 2 + hall[2] * 4
        new_order = self.hall_order[self.phase]

        if self.phase != last_phase:
            # Speed estimation
            if self.last_comm > 0.0:# Ignore first edge
                self.speed_est = 1.0 / (6.0 * self.P * (self.Trun - self.last_comm))
            
            # Check rotation direction
            # Handle wrap-around cases separately
            if (new_order < self.last_order and not (new_order == 1 and self.last_order == 6)) or  \
                    (new_order == 6 and self.last_order == 1):
                self.speed_est = -self.speed_est

            self.last_order = new_order
            self.last_comm = self.Trun

        # Low-pass filter the actual speed value
        self.speed = self.speed_est * (dt / (self.Sf+dt)) + self.speed * (self.Sf/(self.Sf+dt))            
            

        # PID calculation of output voltage
        self.error = (self.reference / 60.0) - self.speed
        self.integral +=  self.error * dt / self.Ti

        # Limit integrator
        if self.integral > self.intmax:
            self.integral = self.intmax
        elif self.integral < -self.intmax:
            self.integral = -self.intmax
            
        self.Uout = self.Kp * (self.error + self.integral)
        
        # Limit output voltage
        if self.Uout > self.Umax:
            self.Uout = self.Umax
        elif self.Uout < -self.Umax:
            self.Uout = -self.Umax

        self.output = []
        for p in self.sequence[self.phase]:
        #for p in self.sequence[0]:
            self.output.append(self.Uout * p)

        self.motor.set_input(self.output)
        
        self.Trun += dt