# -*- coding: utf-8 -*-
"""
Created on Wed Jul 12 19:17:53 2017

"""

# TI SPRABQ3: Sensorless Field Oriented Control of 3-Phase Permanent Magnet Synchronous Motors
# Microchip AN1078: Sensorless Field Oriented Control of a PMSM
# Atmel AVR32723: Sensor Field Oriented Control for Brushless DC motors with AT32UC3B0256

from control import Control

from math import sin, cos, sqrt, atan2, pi


def clarke(v1, v2, v3):
    """
    Transform 3-phase quantities v1, v2 and v3 to
    alpha-beta frame, i.e. do the Clarke transform.
    
    From Microchip application note AN1078 B.
    """
    a = v1
    b = (v1 + 2.0*v2) / sqrt(3)
    
    return (a, b)
    

def park(a, b, theta):
    """
    Rotate alpha-beta frame to direct-quadrature
    frame at angle theta, i.e. do the Park transform.
    
    From Microchip application note AN1078 B.
    """
    d = a * cos(theta) + b * sin(theta)
    q = -a * sin(theta) + b * cos(theta)
    return (d, q)


def inverse_park(d, q, theta):
    """
    Inverse Park transform, converts direct-quadrature
    frame at angle theta back to alpha-beta frame.
    
    From Microchip application note AN1078 B.
    """   
    a = d * cos(theta) - q * sin(theta)
    b = d * sin(theta) + q * cos(theta)
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


def limit(val, mini, maxi):
    """ Limit value between min and max """
    if val < mini:
        return mini
    elif val > maxi:
        return maxi
    return val


class Model_FOC:
    """ Motor model base for field oriented control """

    theta = 0.0     # Electrical angle
    omega = 0.0     # Electrical speed
    
    def __init__(self):
        self.voltage = [0.0]*3
        self.current = [0.0]*3
        self.params = {}            # Parameter list
        self.name = 'Base model'    # Model name
        return
        
    def set_current(self, current):
        """ Set measured phase/ab/dq currents """
        self.current = current
    
    def set_voltage(self, voltage):
        """ Set motor phase/ab/dq voltages """
        self.voltage = voltage
    
    def get_theta(self):
        """ Return estimated rotor electrical angle """
        return self.theta
        
    def get_omega(self):
        return self.omega

    def calculate(self, dt):
        """ Calculate the motor model """
        return



class Model_BLDC_Slide(Model_FOC):
    """
    BLDC motor model for field oriented control, using
    sliding mode controller for back-EMF estimation.
    Estimation is done in alpha-beta frame without normalization.
    """

    Ts = 2.e-4       # Model run interval (control cycle) [s]
 
    # Motor model parameters
    L = 1.84e-3         # Winding inductance (phase to neutral) [H]
    R = 1.0             # Winding resistance (phase to neutral) [ohm]
    
    Ks = 10.0           # Sliding mode controller gain
    Xs = 30.0           # Sliding mode controller max limit

    Tsf = 1.e-3        # Speed filter
   
    def __init__(self, Ts=0.0002):
        Model_FOC.__init__(self)

        self.name = 'BLDC_Slide'
        self.description = 'BLDC model with sliding-mode estimator'    # Model name
        self.params = {     \
                'Ks' : 'Sliding mode controller gain',              \
                'Xs' : 'Sliding mode controller limit',             \
                'L' : 'Motor phase to neutral inductance [H]',     \
                'R' : 'Motor phase to neutral resistance [ohm]',   \
                'Tsf' : 'Speed low pass filter time constant [s]'     \
                }

        self.iest = [0.0] * 2           # Estimated alpha-beta currents
        self.emfest = [0.0] * 2         # Estimated back EMF
        self.emfestf = [0.0] * 2        # Filtered back EMF estimate

        self.Ts = Ts

        # Constants to simplify derivates etc
        self.F = 1.0 - self.Ts*self.R/self.L
        self.G = self.Ts/self.L

        return
    
    def calculate(self, dt):
        """ Run the motor model """
        
        # Current observer with sliding-mode controller
        # Runs in ab coordinates
        for n in range(2):

            error = self.iest[n] - self.current[n]

            # Use a sliding mode controller
            self.emfest[n] = 10.0 * limit(error, -30, 30)
            self.iest[n] = self.F * self.iest[n] + self.G * (self.voltage[n] - self.emfest[n])

            # Calculate back-emf filter time constant based on the current speed estimate
            tfilt = 0.001
            if self.omega != 0.0:
                tfilt = limit(abs(1. / self.omega), 1.0e-6, 0.1)

            # Low-pass filter the estimate
            self.emfestf[n] = self.emfest[n] * (self.Ts / (tfilt+self.Ts)) + self.emfestf[n] * (tfilt/(tfilt+self.Ts))

        # Estimate position from the phase shift of back-emf voltages
        # 90 degrees is added due to the filtering (?)
        thetaold = self.theta
        self.theta = atan2(self.emfestf[1], self.emfestf[0]) + (pi/2)

        # Calculate electrical speed from angular position change        
        # Unwrap theta by selecting smallest theta change after
        # adding full rotations to either direction (atan2 wraps from -pi to +pi)
        dtheta = self.theta - thetaold
        dtheta2 = dtheta + 2.0*pi
        dtheta3 = dtheta - 2.0*pi        
        if abs(dtheta2) < abs(dtheta):
            dtheta = dtheta2
        elif abs(dtheta3) < abs(dtheta):
            dtheta = dtheta3
        
        speedest = dtheta / self.Ts

        # Low-pass filter estimated electrical speed
        self.omega = speedest * (self.Ts / (self.Tsf+self.Ts)) + self.omega * (self.Tsf/(self.Tsf+self.Ts))



class Model_BLDC_SlideNorm(Model_FOC):
    """
    BLDC motor model for field oriented control, using
    normalized sliding mode controller for back-EMF estimation.
    Estimation is done in alpha-beta frame
    """

    Ts = 2.e-4       # Model run interval (control cycle) [s]
 
    # Motor model parameters
    L = 1.84e-3         # Winding inductance (phase to neutral) [H]
    R = 1.0             # Winding resistance (phase to neutral) [ohm]

    Ks = 2.0            # Sliding mode controller gain

    Tsf = 1.e-3        # Speed filter
   
    def __init__(self, Ts=0.0002):
        Model_FOC.__init__(self)

        self.name = 'BLDC_SlideNorm'
        self.description = 'BLDC model with normalized sliding-mode estimator'    # Model name
        self.params = {     \
                'Ks' : 'Sliding mode controller gain',              \
                'L' : 'Motor phase to neutral inductance [H]',     \
                'R' : 'Motor phase to neutral resistance [ohm]',   \
                'Tsf' : 'Speed low pass filter time constant [s]'     \
                }

        self.iest = [0.0] * 2           # Estimated alpha-beta currents
        self.emfest = [0.0] * 2         # Estimated back EMF
        self.emfestf = [0.0] * 2        # Filtered back EMF estimate

        self.Ts = Ts

        # Constants to simplify derivates etc
        self.F = 1.0 - self.Ts*self.R/self.L
        self.G = self.Ts/self.L

        return
    
    def calculate(self, dt):
        """ Run the motor model """

        # Sliding mode control
        # Normalize vsupply and current
        lvolt = sqrt(self.voltage[0]**2 + self.voltage[1]**2)
        if lvolt == 0.0:
            lvolt = 1.0
        lcurr = sqrt(self.current[0]**2 + self.current[1]**2)
        if lcurr == 0.0:
            lcurr = 1.0
            
        if lvolt > lcurr:
            lcurr = lvolt
        vnorm = [self.voltage[0] / lcurr, self.voltage[1] / lcurr]
        cnorm = [self.current[0] / lcurr, self.current[1] / lcurr]

        # Current observer
        # Runs in ab coordinates
        for n in range(2):
            # Sliding mode control with normalized voltage & current
            # Difference of estimated and actual phase current
            error = self.iest[n] - cnorm[n]

            # TEST: Use a sliding mode controller
            self.emfest[n] = self.Ks * limit(error, -1, 1)
            self.iest[n] = self.F * self.iest[n] + self.G * (vnorm[n] - self.emfest[n])

#            tfilt = 0.001
#            if self.omega != 0.0:
#                tfilt = limit(abs(1.0 / self.omega), 1.0e-5, 0.1)
            
#            self.emfestf[n] = self.emfest[n] * (self.Ts / (tfilt+self.Ts)) + self.emfestf[n] * (tfilt/(tfilt+self.Ts))
            self.emfestf[n] = self.emfest[n]

         # Estimate position from the phase shift of back-emf voltages
        # 90 degrees is added due to the filtering (?)
        thetaold = self.theta
        self.theta = atan2(self.emfestf[1], self.emfestf[0]) + pi/2

        # Calculate electrical speed from angular position change        
        # Unwrap theta by selecting smallest theta change after
        # adding full rotations to either direction (atan2 wraps from -pi to +pi)
        dtheta = self.theta - thetaold
        dtheta2 = dtheta + 2.0*pi
        dtheta3 = dtheta - 2.0*pi        
        if abs(dtheta2) < abs(dtheta):
            dtheta = dtheta2
        elif abs(dtheta3) < abs(dtheta):
            dtheta = dtheta3
        
        speedest = dtheta / self.Ts

        # Low-pass filter estimated electrical speed
        self.omega = speedest * (self.Ts / (self.Tsf+self.Ts)) + self.omega * (self.Tsf/(self.Tsf+self.Ts))




class Model_BLDC_PI(Model_FOC):
    """
    BLDC motor model for field oriented control, using
    PI controller for back-EMF estimation.
    Estimation is done in alpha-beta frame.
    """

    Ts = 1.0 / 5.0e3       # Model run interval (control cycle) [s]
 
    # Motor model parameters
    L = 1.84e-3         # Winding inductance (phase to neutral) [H]
    R = 1.0             # Winding resistance (phase to neutral) [ohm]

    Kpe = 2.0           # EMF-estimate proportional gain
    Tie = 0.6e-3        # EMF-estimate integration time [s]
    Kie = 0.0           # EMF-estimate integrator gain

    Kpemax = 5.0        # Maximum Kpe value
    Kpemin = 0.5        # Minimum Kpe value
    Kperef = 240.0      # Reference speed

    Tsf = 1.0 / 1000.0        # Speed filter
   
    def __init__(self, Ts=0.0002):
        Model_FOC.__init__(self)

        self.name = 'BLDC_PI'
        self.description = 'BLDC model with PI estimator'    # Model name
        self.params = {     \
                'Kpe' : 'Estimator PI controller gain',              \
                'Tie' : 'Estimator PI controller integration time',             \
                'Kie' : 'Estimator PI controller integrator gain',             \
                'L' : 'Motor phase to neutral inductance [H]',     \
                'R' : 'Motor phase to neutral resistance [ohm]',   \
                'Tsf' : 'Speed low pass filter time constant [s]',     \
                'Kpemax' : 'Maximum Kpe value for adaptation',         \
                'Kpemin' : 'Minimum Kpe value for adaptation',         \
                'Kperef' : 'Reference electrical speed for Kpe adaptation [rad/s]'            \
                }


        self.iest = [0.0] * 2           # Estimated alpha-beta currents
        self.emfest = [0.0] * 2         # Estimated back EMF
        self.emfestf = [0.0] * 2        # Filtered back EMF estimate

        self.emfint = [0.0]*2       # Integrator value for EMF estimator

        self.Ts = Ts

        # Constants to simplify derivates etc
        self.F = 1.0 - self.Ts*self.R/self.L
        self.G = self.Ts/self.L

        self.Kie = self.Ts / self.Tie

        return

    
    def calculate(self, dt):
        """ Run the motor model """

        # Current observer with PI controller
        # Runs in ab coordinates
        for n in range(2):
            # Estimated EMF from current error with PI controller
            # Difference of estimated and actual phase current
            error = self.iest[n] - self.current[n]
            
            # Tune gain depending on the rotation speed
            Kp = limit((self.Kpe/self.Kperef) * self.omega, self.Kpemin, self.Kpemax)

            self.emfint[n] += error * self.Ts / self.Tie
            self.emfest[n] = Kp * (error + self.emfint[n])

            # TODO: Low-pass filter EMF estimate?
#            tfilt = 0.001
#            if self.omega != 0.0:
#                tfilt = limit(abs(1.0 / self.omega), 1.0e-5, 0.1)
#
#            self.emfestf[n] = self.emfest[n] * (self.Ts / (tfilt+self.Ts)) + self.emfestf[n] * (tfilt/(tfilt+self.Ts))
            self.emfestf[n] = self.emfest[n]

            # Calculate current models estimating the next cycle currents
            self.iest[n] = self.F * self.current[n] + self.G * (self.voltage[n] - self.emfest[n])



        # Estimate position from the phase shift of back-emf voltages
        # 90 degrees is added due to the filtering (?)
        thetaold = self.theta
        self.theta = atan2(self.emfestf[1], self.emfestf[0]) + pi/2

        # Calculate electrical speed from angular position change        
        # Unwrap theta by selecting smallest theta change after
        # adding full rotations to either direction (atan2 wraps from -pi to +pi)
        dtheta = self.theta - thetaold
        dtheta2 = dtheta + 2.0*pi
        dtheta3 = dtheta - 2.0*pi        
        if abs(dtheta2) < abs(dtheta):
            dtheta = dtheta2
        elif abs(dtheta3) < abs(dtheta):
            dtheta = dtheta3
        
        speedest = dtheta / self.Ts

        # Low-pass filter estimated electrical speed
        self.omega = speedest * (self.Ts / (self.Tsf+self.Ts)) + self.omega * (self.Tsf/(self.Tsf+self.Ts))



class Control_BLDC_FOC(Control):

    # Common parameters
    inhibit = True      # Inhibit controller, i.e. set outputs floating
    mode = 0            # Controller operation mode, see set_mode(...)
    Umax = 30.0         # Maximum output voltage (+ and -)
    intmax = 10.0       # Integrator anti-windup
    P = 1.0             # Motor polepairs
    Ts = 1.0 / 5.0e3    # Model run interval (control cycle) [s]


    # Speed controller parameters
    Kp = 1.0            # Speed controller proportional gain
    Ti = 10e-3          # Speed controller integration time [s]
    Ki = 0.0            # Speed controller integrator gain
    serror = 0.0        # Speed error from reference [rad/s]
    sint = 0.0          # Speed PI controller integrator value
    speed = 0.0         # Actual (estimated) speed
    theta = 0.0         # Actual (estimated) electrical angle of rotor
    
    # Torque controller parameters
    Kpt = 5.0           # Torque controller proportional gain
    Tit = 5e-3         # Torque controller integration time [s]
    Kit = 0.0           # Torque controller integrator gain
    qreference = 0.0    # Torque reference
    dreference = 0.0    # Flux reference
    qint = 0.0          # Q axis (torque) integrator value
    dint = 0.0          # D axis (flux) integrator value

    # Outputs from controller
    Ia = 0.0            # Alpha-axis current value [A]
    Ib = 0.0            # Beta-axis current value [A]
    Iq = 0.0            # Q-axis (torque) current value [A]
    Id = 0.0            # D-axis (flux) current value [A]
    qout = 0.0          # Q-axis (torque) voltage output [V]
    dout = 0.0          # D-axis (flux) voltage output [V]


    def __init__(self, params):
        Control.__init__(self, params)

        self.params = { \
            'Kp' : 'Proportional gain of speed controller',     \
            'Ki' : 'Integration gain of speed controller',      \
            'Ti' : 'Integration time of speed controller',      \
            'Kpt' : 'Proportional gain of torque controller',     \
            'Kit' : 'Integration gain of torque controller',      \
            'Tit' : 'Integration time of torque controller',      \
            'P' : 'Motor polepairs',        \
            'Sf' : 'Speed filter time constant [s]',     \
            'Umax' : 'Maximum output voltage amplitude [V]', \
            'Ts' : 'Controller execution interval [s]'
            }


        self.Ts = 1.0 / 5.0e3       # Controller run interval (control cycle) [s]
        self.Tnext = self.Ts        # Time to next run if calculation step is shorter than control cycle
        
        self.output = [0.0]*3       # Output voltages in 3-phase system
        self.vsupply = [0.0]*2      # Output (motor supply) voltages in alpha-beta frame
        self.current = [0.0]*2      # Output (motor) currents in alpha-beta frame

        # Motor model, this is default
        self.model = Model_BLDC_Slide()

        self.set_parameters(params)
        
        return


    def set_parameters(self, params):
        if params.has_key('Kp'):
            self.Kp = params['Kp']

        if params.has_key('Ki'):
            self.Ti = self.Kp / params['Ki']

        if params.has_key('Ti'):
            self.Ti = params['Ti']

        if params.has_key('Kpt'):
            self.Kpt = params['Kpt']

        if params.has_key('Kit'):
            self.Tit = self.Kpt / params['Kit']

        if params.has_key('Tit'):
            self.Tit = params['Tit']
            
        if params.has_key('P'):
            self.P = params['P']

        if params.has_key('Umax'):
            self.Umax = params['Umax']

        if params.has_key('Ts'):
            self.Ts = params['Ts']

        # Motor model selection
        if params.has_key('Model'):
            mod = params['Model']
            if mod == 'BLDC_Slide':
                self.model = Model_BLDC_Slide(self.Ts)
            elif mod == 'BLDC_SlideNorm':
                self.model = Model_BLDC_SlideNorm(self.Ts)
            elif mod == 'BLDC_PI':
                self.model = Model_BLDC_PI(self.Ts)
            else:
                print 'Unknown motor model, using default (Slide)'

        # Update model run period
        self.model.Ts = self.Ts

        # Re-calculate any constants that have changed
        self.do_precalc()

        return
     

    def do_precalc(self):     
        """ Pre-calculate several constants to optimize controller """

        # PI controllers
        self.Ki = self.Ts / self.Ti
        self.Kit = self.Ts / self.Tit


     
    def get_parameters(self):
        params = Control.get_parameters(self)
        
        params.update({     \
            'Kp' : self.Kp,     \
            'Ki' : self.Ki, \
            'Ti' : self.Ti,   \
            'Kpt' : self.Kpt,     \
            'Kit' : self.Kit, \
            'Tit' : self.Tit,   \
            'P' : self.P,   \
            'Umax' : self.Umax,      \
            'Ts' : self.Ts,      \
            'Model' : self.model.name   \
            })
        
        params.update(self.model.params)
        
        return params
        
    
    def set_motor(self, motor):
        
        if motor.get_config_word('Name') != 'BLDC':
            print 'Motor type not BLDC'
            return False
        
        Control.set_motor(self, motor)

        return True
    
    def set_mode(self, mode):
        # Modes:
        # 0 = Control disabled, model still runs
        # 1 = Open-loop start-up, ramp to reference
        # 2 = Closed-loop torque control, reference = torque
        # 3 = Closed-loop speed control, reference = speed
    
        self.mode = mode
        
        if mode == 0:
            self.inhibit = True
        else:
            self.inhibit = False
        return


    def calculate(self, dt):
        # Run calculation at selected time intervals only
        # self.Ts should be multiple of dt for accurate results
        self.Tnext -= dt
        if self.Tnext > 0.0:
            return

        self.Tnext = self.Ts

        # Read currents and voltages
        self.do_measurements()

        # Run controllers
        if self.mode == 3:
            self.run_speed_control()
        
        if self.mode > 1:
            if self.mode == 2:
                # In torque mode reference = torque, speed mode sets the reference itself
                self.qreference = self.reference

            # Run the torque controller always
            self.run_torque_control()


        # Output voltages are then given to motor inputs
        if self.inhibit:
            self.motor.set_input([float("NaN"), float("NaN"), float("NaN")])
        else:
            self.motor.set_input(self.output)

        # Increase controller time
        self.Trun += self.Ts

        # Run motor model
        self.model.set_current(self.current)
        self.model.set_voltage(self.vsupply)
        self.model.calculate(self.Ts)

        # Get model results
        self.theta = self.model.get_theta()

        # Calculate actual rotation speed of the shaft
        self.speed = self.model.get_omega() / (2.0 * pi * self.P)


        return


    def run_speed_control(self):
         # PI control of d and q terms
        serror = self.reference - self.speed
        self.sint +=  serror * self.Ts / 100e-3
        self.sint = limit(self.sint, -self.intmax, self.intmax)
        self.qreference = 2.0 * (serror + self.sint)
        return


    def do_measurements(self):
        # Get actual output currents
        (iu, iv, iw) = self.motor.get_current()

        # Convert currents to ab coordinates; real ones are not used anymore
        (self.Ia, self.Ib) = clarke(iu, iv, iw)
        self.current = [self.Ia, self.Ib]        

        # Get actual motor voltage, this is used if controller is inhibited
        # Controller overrides vsupply if necessary
        vin = self.motor.get_voltage()
        (Ua, Ub) = clarke(vin[0], vin[1], vin[2])
        self.vsupply = [Ua, Ub]      # This is used in model calculation
        
        return
        

    def run_torque_control(self):

        # Convert currents to d-q coordinates rotating with the rotor        
        (self.Id, self.Iq) = park(self.Ia, self.Ib, self.theta)

        # TODO: Correct the terms etc...
        # PI control of d and q terms
        qerror = self.qreference - self.Iq
        self.qint +=  qerror * self.Ts / self.Ti

        derror = self.dreference - self.Id
        self.dint +=  derror * self.Ts / self.Ti

        # Limit integrators
        self.qint = limit(self.qint, -self.intmax, self.intmax)
        self.dint = limit(self.dint, -self.intmax, self.intmax)

        # Final controller values
        self.qout = self.Kp * (qerror + self.qint)
        self.dout = self.Kp * (derror + self.dint)

        # Limit the vector length to maximum voltage vector
        # Otherwise when DC voltage is exceeded, the controller uses
        # wrong vector in estimation and thus gives horribly wrong
        # results
        qdlen = sqrt(self.qout**2 + self.dout**2)
        scale = 1.0
        if qdlen > 0.0:
            scale = limit(qdlen, 0, self.Umax) / qdlen
        self.qout *= scale
        self.dout *= scale 

        # Convert controller values to non-rotating coordinates
        (Ua, Ub) = inverse_park(self.dout, self.qout, self.theta)
                    
        # And finally back to real phase voltage values for modulator
        (uu, uv, uw) = inverse_clarke(Ua, Ub)

        # "Electrical" limitation due to DC link voltage
        uu = limit(uu, -self.Umax, self.Umax)
        uv = limit(uv, -self.Umax, self.Umax)
        uw = limit(uw, -self.Umax, self.Umax)
        self.output = [uu, uv, uw]

        # Motor input voltages for motor model also, in ab frame
        if not self.inhibit:
            self.vsupply = [Ua, Ub]      # This is used in model calculation

        return

