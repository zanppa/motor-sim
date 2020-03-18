# -*- coding: utf-8 -*-
"""
DC motor with field winding
"""

from motor import Motor

from math import isnan

class Motor_DC(Motor):

    # Electrical parameters:
    Lf = 1.84e-3        # Field coil inductance [H]
    Rf = 1.0            # Field coil resistance [ohm]
    Ra = 1.0            # Armature coil resistance [ohm]
    Ke = 0.00414        # Voltage coefficient [V/rad]

    def __init__(self, params):
        Motor.__init__(self, params)
        
        self.config = {             \
            'Name' : 'DC',        \
            'Phases' : 2,           \
            'Views' : 3             \
            }

        self.parameters.update({    \
            'Lf' : 'Field coil inductance [H]',  \
            'Rf' : 'Field coil resistance [ohm]',  \
            'Ra' : 'Armature coil resistance [ohm]', \
            'Ke' : 'Voltage coefficient [V/rad]' \
            })

        self.Vin = [float("Nan"), float("NaN")]
        self.I = [0.0, 0.0]            # Motor current [A]
        self.Bemf = 0.0         # Back-EMF voltage [A]

        self.set_parameters(params)
        
        return

    def set_parameters(self, params):
        Motor.set_parameters(self, params)        
        
        if params.has_key('Lf'):
            self.Lf = params['Lf']

        if params.has_key('Rf'):
            self.LRf = params['Rf']

        if params.has_key('Ra'):
            self.Ra = params['Ra']
            
        if params.has_key('Ke'):
            self.Ke = params['Ke']

        return

    def get_parameters(self):
        params = Motor.get_parameters(self)
        params.update({  \
            'Lf' : self.Lf, \
            'Rf' : self.Rf, \
            'Ra' : self.Ra, \
            'Ke' : self.Ke})
            
        return params
        
    def set_input(self, vin):
        # 1 = field coil voltage, 2 = armature voltage
        if len(vin) != 2:
            return False
            
        self.Vin = vin
        return True

    def get_voltage(self):
        # Returns 1=field voltage, 2=armature voltage.
        # If armature voltage is nan, returns back-emf
        # i.e. the phase is floating from control point of view
        vout = self.Vin

        if isnan(vout[1]):
            vout[1] = self.Bemf

        return vout
            
    def get_current(self):
        # 1=field coil current, 2=armature current
        return self.I
        
    
    def calculate(self, dt):
        self.Bemf = self.Ke * self.I[0] * self.Omega
        
        # Calculate current change in field coil
        if isnan(self.Vin[0]):
            self.I[0] = 0
        else:
            dIf = (self.Vin[0] - self.I[0]*self.Rf) / self.Lf     
            self.I[0] += dIf * dt
        
        # And in armature
        if isnan(self.Vin[1]):
            self.I[1] = 0
        else:
            #dIa = (self.Va - self.Bemf - self.Ia*self.Ra) / self.La
            #self.Ia += dIa * dt
            self.I[1] = (self.Vin[1] - self.Bemf) / self.Ra

        
        self.T = self.Ke * self.I[0] * self.I[1]    # Losses not taken into account -> mechanical power = electrical power
            
        # Calculate mechanical side
        Motor.calculate_mechanical(self, dt)

        #self.power = self.T * self.Omega
        
        return
