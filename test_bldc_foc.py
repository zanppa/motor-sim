# -*- coding: utf-8 -*-
"""
Test the BLDC motor with FOC control
"""

from math import sqrt, pi
import matplotlib.pyplot as plt

from motor_bldc import Motor_BLDC

from control_bldc_foc import Control_BLDC_FOC

scale = 50000
Ts = 1. / scale

motor = Motor_BLDC({'F':2.0e-7, 'Ke':0.004, 'P':2.0, 'R':1.0, 'V0':2.0, 'T0':pi/2, 'J':0.00002, 'Ld':1.84e-3, 'Lq':1.84e-3})
control = Control_BLDC_FOC({'P':2.0, 'Model':'BLDC_Slide', 'Ts':Ts})
#control = Control_BLDC_FOC({'P':2.0, 'Model':'BLDC_SlideNorm'})
#control = Control_BLDC_FOC({'P':2.0, 'Model':'BLDC_PI'})


control.set_motor(motor)
control.set_reference(8000.0)           # rpm


#fin = 100.0
uinf = 3.0
uina = 6.0


time = []

EMF = [[], [], [], []]
Curr = [[], [], [], []]
Spd = [[], [], []]
Torque = [[], [], []]

Pos = [[], []]
motor.set_load(0.00)

for t in range(scale):
    time.append(float(t) / scale)

    if t <= 0.05*scale:
        control.set_mode(0)
    elif t > 0.05*scale and t < 0.1*scale:
        control.set_mode(2)
        control.set_reference(0.0)
    elif t > 0.1*scale and t < 0.5*scale:
        control.set_reference(-5.0)
    elif t > 0.7*scale and t < 0.8*scale:
        control.set_reference(20.0)
    else:
        control.set_reference(0.0)

    if t > 0.6*scale and t < 0.65*scale:
        motor.set_load(0.05)
    else:
        motor.set_load(0.0)
        
    control.calculate(1.0 / scale)
    motor.calculate(1.0 / scale)

    # Cheat a bit and use direct model variables for plotting
    # Back emf in ab frame
    EMF[0].append(motor.Bemf[0])
    EMF[1].append((motor.Bemf[0] + 2.0*motor.Bemf[1]) / sqrt(3))
    # Estimated back-emf
    EMF[2].append(control.model.emfestf[0])
    EMF[3].append(control.model.emfestf[1])

    (u1, v1, w1) = motor.get_current()
    # Motor currents in ab frame
    Curr[0].append(control.Ia)
    Curr[1].append(control.Ib)
    # Model estimated currents in ab frame
    Curr[2].append(control.model.iest[0])
    Curr[3].append(control.model.iest[1])

    (trq, speed, angle) = motor.get_mechanical()
    Spd[0].append(speed / (2.0 * 3.14159) * 60.0)   # rpm
    Spd[1].append(control.speed * 60.0) # rpm
    ref = float("NaN")
    if (control.mode == 1 or control.mode == 3):
        ref = control.reference
    Spd[2].append(ref)    # rpm (if in speed mode)

    Torque[0].append(-trq * 1e2)
    Torque[1].append(control.Iq)
    Torque[2].append(control.qreference)

    Pos[0].append(motor.Theta*motor.P % (2*3.14159))
    Pos[1].append(control.theta % (2*3.14159))


plt.clf()
plt.figure(1)                # the first figure

ax1 = plt.subplot(511)             # the first subplot in the first figure
ax1.plot(time, EMF[0], label="EMF A")
ax1.plot(time, EMF[1], label="EMF B")
ax1.plot(time, EMF[2], label="EMF est A")
ax1.plot(time, EMF[3], label="EMF est B")
ax1.legend()

ax2 = plt.subplot(512, sharex=ax1)             # the second subplot in the first figure
ax2.plot(time, Curr[0], label="I A")
ax2.plot(time, Curr[1], label="I B")
ax2.plot(time, Curr[2], label="I est A")
ax2.plot(time, Curr[3], label="I est B")
ax2.legend()


ax3 = plt.subplot(513, sharex=ax1)             # the second subplot in the first figure
ax3.plot(time, Spd[0], label="Act. speed [rpm]")
ax3.plot(time, Spd[1], label="Est. speed [rpm]")
ax3.plot(time, Spd[2], label="Speed ref [rpm]")
ax3.legend()

ax4 = plt.subplot(514, sharex=ax1)
#ax4.plot(time, Torque[0], label="Act. torque")
ax4.plot(time, Torque[1], label="Est. torque")
ax4.plot(time, Torque[2], label="Torque ref")
ax4.legend()

ax5 = plt.subplot(515, sharex=ax1)
ax5.plot(time, Pos[0], label="Act. rotor pos")
ax5.plot(time, Pos[1], label="Est. rotor pos")
ax5.legend()

plt.show()
