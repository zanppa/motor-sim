# -*- coding: utf-8 -*-
"""
Base class for all motor models
"""

from math import pi

class Motor:
    
    # Common mechanical features
    J = 1e-6        # Moment of inertia [Nm?]
    F = 2e-7        # Friction coefficient [?]
    Text = 0.0      # External load torque [Nm]

    # Common output features
    T = 0.0         # Torque [Nm]
    Omega = 0.0     # Angular speed [rad/s]
    dOmega = 0.0    # Angular acceleration [rad/s^2]
    Theta = 0.0     # Angular position [rad]

    
    def __init__(self, params):
        # params is a dictionary
        self.config = { \
                'Name' : 'Motor base',         \
                'Phases': 0,             \
                'Views': 0               \
                }

        self.parameters = { \
            'J' : 'Moment of inertia',    \
            'F' : 'Friction coefficient',  \
            'V0' : 'Initial speed [rad/s]',   \
            'T0' : 'Initial position [rad]'   \
            }
            
        self.set_parameters(params)

        return

    def set_parameters(self, params):
        if 'J' in params:
            self.J = params['J']

        if 'F' in params:
            self.F = params['F']

        if 'V0' in params:
            self.Omega = params['V0']
                        
        if 'T0' in params:
            self.Theta = params['T0']
        
        return


    def get_config(self):
        return self.config

    def get_config_word(self, confword):
        if confword in self.config:
            return self.config[confword]

        return 0


    def get_parameters_list(self):
        return self.parameters

    def get_parameters(self):
        params = {        \
            'J' : self.J,   \
            'F' : self.F,   \
            'V0' : self.Omega,  \
            'T0' : self.Theta}

        return params

    def set_input(self, vin):
        # To be implemented in subclass
        return
        
    def set_load(self, T):
        self.Text = T
        return
        
    def get_voltage(self):
        return (0)
    
    def get_current(self):
        # To be implemented in subclass
        return (0)
    
    def get_mechanical(self):
        # Return torque, angular speed and rotor position
        # return [self.T, self.Omega, self.Theta]
        torque = self.T + self.F*self.Omega + self.Text
        return [torque, self.Omega, self.Theta]
        

    def calculate(self, dt):
        return
        
    def calculate_mechanical(self, dt):
        # Common method to calculate mechanical acceleration
        self.dOmega = (1.0/self.J) * (self.T - self.F*self.Omega - self.Text)
        self.Omega += self.dOmega * dt
        self.Theta += self.Omega * dt
        self.Theta = self.Theta % (2.0 * pi)
        
        return

