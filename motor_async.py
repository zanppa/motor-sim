# -*- coding: utf-8 -*-
"""
3-phase asynchronous motor
Using the T model

# For some reason (euler integration?) this results
# in quite a large torque ripple...
"""

from motor import Motor

from math import isnan, sin, pi, cos, sqrt

def clarke(v1, v2, v3):
    """
    Transform 3-phase quantities v1, v2 and v3 to
    alpha-beta frame, i.e. do the Clarke transform.
    
    From Microchip application note AN1078 B.
    """
    a = v1
    b = (v1 + 2.0*v2) / sqrt(3)
    
    return (a, b)


def inverse_clarke(a, b):
    """
    Inverse Clarke transform, converts alpha-beta 
    frame back to 3-phase values.
    
    This is corrected version, original from
    Microchip application note AN1078 B was wrong.
    """
    v1 = a
    v2 = (b*sqrt(3) - a) * 0.5
    v3 = (-b*sqrt(3) - a) * 0.5
    
    return (v1, v2, v3)

class Motor_Async(Motor):

    # Electrical parameters:
    # Example is a 7.5 kW, 6-pole 220 V, 60 Hz motor
    P = 2.0             # Number of pole pairs
    Ls = 400.e-3        # Stator inductance [H]
    Rs = 14.            # Stator resistance [ohm]
    Lr = 412.8e-3       # Rotor inductance [H]
    Rr = 10.1           # Rotor resistance [ohm]
    Lm = 377e-3         # Mutual inductance [H]

    def __init__(self, params):
        Motor.__init__(self, params)
        
        self.config = {             \
            'Name' : 'Async',        \
            'Phases' : 3,           \
            'Views' : 3             \
            }

        self.parameters.update({    \
            'P' : 'Number of polepairs', \
            'Ls' : 'Stator inductance [H]',  \
            'Rs' : 'Stator resistance [ohm]', \
            'Lr' : 'Rotor inductance [H]', \
            'Rr' : 'Rotor resistenca [H]', \
            'Lm' : 'Mutual inductance [H]'
            })

        self.Vin = [float("Nan"), float("Nan"), float("Nan")]
        self.I = [0.0, 0.0, 0.0]            # Motor currents [A]
        self.Ia = 0.                         # Motor input currents in ab
        self.Ib = 0.
        
        self.Phia = 0.               # Motor flux in ab
        self.Phib = 0.
        self.J = 0.01           # Default test

        self.set_parameters(params)
        
        # Precalc something
        self.K = 1./(self.Ls*self.Lr - self.Lm*self.Lm)
        
        return
        
    def set_parameters(self, params):
        Motor.set_parameters(self, params)        
        
        if params.has_key('P'):
            self.P = params['P']

        if params.has_key('Ls'):
            self.Ls = params['Ls']

        if params.has_key('Rs'):
            self.Rs = params['rs']

        if params.has_key('Rr'):
            self.Rr = params['Rr']
            
        if params.has_key('Lr'):
            self.Lr = params['Lr']

        if params.has_key('Lm'):
            self.Lm = params['Lm']
        
        return

        
    def get_parameters(self):
        params = Motor.get_parameters(self)
        params.update({  \
            'P' : self.P,   \
            'Ls' : self.Ls, \
            'Lr' : self.Lr, \
            'Rs' : self.Rs,   \
            'Rr' : self.Rr,   \
            'Lm' : self.Lm})        
        return params
        
    
    def set_input(self, vin):
        if len(vin) != 3:
            return False
            
        self.Vin = vin
        return True

    def get_voltage(self):
        # Returns phase voltage.
        return self.Vin            
            
    def get_current(self):
        return self.I
        
    
    def calculate(self, dt):
        # Modeling according to
        # On the  Discrete–Time Modelling  and  Control of  Induction  Motors with Sliding  Modes
        # B. Castillo–Toledo, S. Di Gennaro, A. G. Loukianov and J. Rivera
        # Proceeding of the 2004 American Control Conference
        # p. 2598-2602

        # Convert voltages to ab
        (Ua, Ub) = clarke(self.Vin[0], self.Vin[1], self.Vin[2])
        self.Ua = Ua
        self.Ub = Ub

        # Calculate helpers
        # TODO: Precalc in init?
        sigma = self.Ls - self.Lm*self.Lm/self.Lr
        mu = 3./2. * self.Lm*self.P/self.Lr
        alpha = self.Rr/self.Lr
        beta = self.Lm/(sigma*self.Lr)
        gamma = self.Lm*self.Lm*self.Rr/(sigma*self.Lr*self.Lr)+self.Rs/sigma

        # Calculate derivates
        dPhia = -alpha*self.Phia - self.P*self.Omega*self.Phib + alpha*self.Lm*self.Ia
        dPhib = -alpha*self.Phib + self.P*self.Omega*self.Phia + alpha*self.Lm*self.Ib
        dIa = alpha*beta*self.Phia + self.P*beta*self.Omega*self.Phib - gamma*self.Ia + Ua/sigma
        dIb = alpha*beta*self.Phib - self.P*beta*self.Omega*self.Phia - gamma*self.Ib + Ub/sigma

        # Calculate generated torque
        self.T = mu*(self.Phia*self.Ib - self.Phib-self.Ia)

        dOmega = (self.T - self.Text + self.F*self.Omega)/self.J
        
        self.Phia += dPhia*dt
        self.Phib += dPhib*dt
        self.Ia += dIa*dt
        self.Ib += dIb*dt
        self.Omega += dOmega*dt
        self.Theta += self.Omega*dt
        self.Theta = self.Theta % (2.0 * pi)

        # Calculate currents back to 3-phase
        (self.I[0], self.I[1], self.I[2]) = inverse_clarke(self.Ia, self.Ib)

        # Mechanical calculation is done by this function
        #Motor.calculate_mechanical(self, dt)

        return
