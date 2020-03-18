# -*- coding: utf-8 -*-
"""
DC PM motor
DC permanent magnet motor
"""

from motor import Motor

from math import isnan, sin

class Motor_PMDC(Motor):

    # Electrical parameters:
    L = 1.84e-3         # Coil inductance [H]
    R = 1.0             # Coil resistance [ohm]
    Ke = 0.00414        # Voltage coefficient [V/rad]

    def __init__(self, params):
        Motor.__init__(self, params)
        
        self.config = {             \
            'Name' : 'PMDC',        \
            'Phases' : 1,           \
            'Views' : 3             \
            }

        self.parameters.update({    \
            'L' : 'Coil inductance [H]',  \
            'R' : 'Coil resistance [ohm]',  \
            'Ke' : 'Voltage coefficient [V/rad]' \
            })

        self.Vin = float("Nan")
        self.I = 0.0            # Motor current [A]
        self.Bemf = 0.0         # Back-EMF voltage [A]

        self.set_parameters(params)
        
        return


    def set_parameters(self, params):
        Motor.set_parameters(self, params)

        if 'L' in params:
            self.L = params['L']

        if 'R' in params:
            self.R = params['R']
            
        if 'Ke' in params:
            self.Ke = params['Ke']
        
        return
    
    def get_parameters(self):
        params = Motor.get_parameters(self)
        params.update({  \
            'L' : self.L,   \
            'R' : self.R,   \
            'Ke' : self.Ke})
        
        return params
    
    
    def set_input(self, vin):
        if len(vin) != 1:
            return False
            
        self.Vin = vin[0]
        return True

    def get_voltage(self):
        # Returns phase voltage. If input voltage is nan, return back-emf
        # i.e. the phase is floating from control point of view
        vout = self.Vin
        if isnan(vout):
            vout = self.Bemf

        return [vout]
            
    def get_current(self):
        return (self.I)
        
    
    def calculate(self, dt):
        self.Bemf = self.Ke * self.Omega
        
        # Calculate current change
        if isnan(self.Vin):
            self.I = 0
        else:
            dI = (self.Vin - self.Bemf - self.I*self.R) / self.L     
            self.I += dI * dt
        
        self.T = self.Ke * self.I    # Losses not taken into account -> mechanical power = electrical power
        #self.P = self.Bemf * self.I
            
        # Calculate mechanical side
        Motor.calculate_mechanical(self, dt)
        
        return
