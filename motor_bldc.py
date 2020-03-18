# -*- coding: utf-8 -*-
"""
BLDC motor model
3-phase brushless permanent magnet motor
"""

from motor import Motor

from math import isnan, sin, pi, cos

class Motor_BLDC(Motor):

    # Electrical parameters:
    P = 2.0             # Number of pole pairs
    Ld = 1.84e-3        # D-axis inductance [H]
    Lq = 1.84e-3        # Q-axis inductance [H]
    R = 0.5             # Coil resistance [ohm]
    Ke = 0.00414        # Voltage coefficient [V/rad]

    def __init__(self, params):
        Motor.__init__(self, params)
        
        self.config = {             \
            'Name' : 'BLDC',        \
            'Phases' : 3,           \
            'Views' : 3             \
            }

        self.parameters.update({    \
            'P' : 'Number of polepairs', \
            'Ld' : 'D-axis inductance [H]',  \
            'Lq' : 'Q-axis inductance [H]', \
            'R' : 'Coil resistance [ohm]',  \
            'Ke' : 'Flux coefficient [B?/rad]' \
            })

        self.Vin = [float("Nan"), float("Nan"), float("Nan")]
        self.I = [0.0, 0.0, 0.0]            # Motor currents [A]
        self.Bemf = [0.0, 0.0, 0.0]         # Back-EMF voltage [A]
        self.hall = [0, 0, 0]               # Hall position sensors

        self.set_parameters(params)
        
        return
        
    def set_parameters(self, params):
        Motor.set_parameters(self, params)        
        
        if 'P' in params:
            self.P = params['P']

        if 'Ld' in params:
            self.Ld = params['Ld']

        if 'Lq' in params:
            self.Lq = params['Lq']

        if 'R' in params:
            self.R = params['R']
            
        if 'Ke' in params:
            self.Ke = params['Ke']
        
        return

        
    def get_parameters(self):
        params = Motor.get_parameters(self)
        params.update({  \
            'P' : self.P,   \
            'Ld' : self.Ld, \
            'Lq' : self.Lq, \
            'R' : self.R,   \
            'Ke' : self.Ke})        
        return params
        
    
    def set_input(self, vin):
        if len(vin) != 3:
            return False
            
        self.Vin = vin
        return True

    def get_voltage(self):
        # Returns phase voltage. If input voltage is nan, return back-emf
        # i.e. the phase is floating from control point of view
        vout = self.Vin
        
        for n, v in enumerate(vout):
            if isnan(v):
                vout[n] = self.Bemf[n] + self.Vcommon

        return vout            
            
    def get_current(self):
        return self.I
        
    
    def calculate(self, dt):
        alpha = 2.0*3.14159/3.0     # Coil interval in radians
        
        # Todo: calculate inductance variation due to salient poles, e.g.
        #La = self.Ld + self.Lq + (self.Lq - self.Ld) * cos(2*self.angle*self.polepairs + 3.14159/3.0)
        # todo: what are the phase angle offsets..?

        self.Vcommon = 0.0
        nact = 0.0
        for n in range(3):

            # Calculate common mode voltage
            if not isnan(self.Vin[n]):
                nact += 1.0
                self.Vcommon += self.Vin[n]

        if nact != 0.0:
            self.Vcommon /= nact

        # Calculate effect of phases
        self.B = [0.0, 0.0, 0.0]    
        dI = [0.0, 0.0, 0.0]
        self.T = 0.0
        #ang = self.P * self.Theta + pi/6.0       # First coil position (electrical) TODO: Was like this
        ang = self.P * self.Theta
        for n in range(3):
            # Calculate magnetic flux at coil positions
            self.B[n] = self.Ke * sin(ang)     # TODO: This was original...
            #self.B[n] = self.Ke * cos(ang)
            ang -= alpha                        # Next coil position (electrical)

            # Calculate back-emf at coil
            self.Bemf[n] = self.P * self.Omega * self.B[n]

            # If phase voltage is nan, then the phase is floating -> 
            # force phase current to zero
            if isnan(self.Vin[n]):
                # Decrease other coil from which the current is flowing
                for k in range(3):
                    if k != n and self.I[n] * self.I[k] < 0.0:
                        self.I[k] += self.I[n]

                self.I[n] = 0
            else:
                self.Vin[n] = self.Vin[n] - self.Vcommon
                dI[n] = (self.Vin[n] - self.Bemf[n] - self.R*self.I[n]) / self.Ld
                self.I[n] += dI[n] * dt
            

        # TODO: Force phase current sum to zero?
        isum = 0.0
        for n in range(3):
            isum += self.I[n]
        
        if isum != 0.0:
            for n in range(3):
                if not isnan(self.Vin[n]):
                    self.I[n] -= isum
                    break
            
            
        # Add torque generated by each phase together
        for n in range(3):
            self.T += self.I[n]*self.B[n]

        # Calculate final torque
        self.T *= self.P
        
        # And calculate mechanical side
        Motor.calculate_mechanical(self, dt)

        # Handle HALL sensor outputs depending on rotor electrical angle
        # Accordign to https://e2e.ti.com/blogs_/b/motordrivecontrol/archive/2013/11/08/generate-your-own-commutation-table-trapezoidal-control-3-phase-bldc-motors-using-hall-sensors
        # TODO: Check if pi/6 offset is correct!
        ang = (self.P * self.Theta - pi/6) % (2.0*pi)
        if ang < pi:
            self.hall[0] = 1
        else:
            self.hall[0] = 0

        if ang >= (2.0*pi/3.0) and ang < (5.0*pi/3.0):
            self.hall[1] = 1
        else:
            self.hall[1] = 0
        
        if ang < (pi/3.0) or ang >= (4.0*pi/3.0):
            self.hall[2] = 1
        else:
            self.hall[2] = 0
        
        return

    def get_hall(self):
        return self.hall
