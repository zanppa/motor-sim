# -*- coding: utf-8 -*-
"""
Test BLDC motor with hall effect sensors
"""

from math import sqrt, pi
import matplotlib.pyplot as plt

from motor_bldc import Motor_BLDC

from control_bldc_hall import Control_BLDC_Hall

#motor = Motor_PMDC({'J':70.0e-9, 'F':2.0e-7, 'V0':-200.0})
motor = Motor_BLDC({'F':2.0e-7, 'Ke':0.004, 'P':2.0, 'R':1.0, 'V0':2.0, 'T0':pi/2, 'J':2e-5})
control = Control_BLDC_Hall({'P':2.0, 'Sf':0.001, 'Kp':5.0, 'Ti':0.1})
control.set_motor(motor)

control.set_reference(0.0)           # rpm
motor.set_load(0.0)

# Time scale
scale = 50000       # dt = 1. / scale

# Arrays for plotting
time = []
EMF = [[], [], []]      # Motor back-EMF
Hall = [[], [], [], []]      # Hall sensor outputs
Curr = [[], [], []]     # Phase currents
Speed = [[], [], []]                # Motor speed (actual, measured, reference)
Uout = [[], [], [], []]     # Controller output voltages 
Torque = []


for t in range(scale):
    time.append(float(t) / scale)

    if t > 0.7*scale:
        control.set_reference(-2000.0)
    elif t > 0.2*scale:
        control.set_reference(5000.0)
    elif t > 0.05*scale:
        control.set_reference(3000.0)

    if t > 0.3*scale and t < 0.5*scale:
        motor.set_load(0.05)
    elif t > 0.5*scale:
        motor.set_load(0.0)
    elif t > 0.1*scale:
        motor.set_load(-0.03)
        
    control.calculate(1.0 / scale)
    motor.calculate(1.0 / scale)

    # Store several values for plotting
    EMF[0].append(motor.Bemf[0])
    EMF[1].append(motor.Bemf[1])
    EMF[2].append(motor.Bemf[2])

    Hall[0].append(motor.hall[0])
    Hall[1].append(motor.hall[1])
    Hall[2].append(motor.hall[2])
    #Hall[3].append(control.last_order)

    (u1, v1, w1) = motor.get_current()
    Curr[0].append(u1)
    Curr[1].append(v1)
    Curr[2].append(w1)

    (trq, omega, angle) = motor.get_mechanical()
    Speed[0].append(omega*60/(2*pi))
    Speed[1].append(control.speed*60)  # Filtered speed
    Speed[2].append(control.reference)
    Torque.append(trq)
    
    Uout[0].append(control.sequence[control.phase][0])
    Uout[1].append(control.sequence[control.phase][1])
    Uout[2].append(control.sequence[control.phase][2])
    Uout[3].append(control.Uout)



plt.clf()
plt.figure(1)                # the first figure

ax1 = plt.subplot(511)             # the first subplot in the first figure
ax1.plot(time, EMF[0], label="EMF A")
ax1.plot(time, EMF[1], label="EMF B")
ax1.plot(time, EMF[2], label="EMF C")
ax1.legend()

ax2 = plt.subplot(512, sharex=ax1)             # the second subplot in the first figure
ax2.plot(time, Curr[0], label="I A")
ax2.plot(time, Curr[1], label="I B")
ax2.plot(time, Curr[2], label="I C")
ax2.legend()


ax3 = plt.subplot(513, sharex=ax1)             # the second subplot in the first figure
ax3.plot(time, Hall[0], label="Hall 1")
ax3.plot(time, Hall[1], label="Hall 2")
ax3.plot(time, Hall[2], label="Hall 3")
#ax3.plot(time, Hall[3], label="Hall order")
ax3.plot(time, Torque, label="Torque")
ax3.legend()

ax4 = plt.subplot(514, sharex=ax1)
ax4.plot(time, Speed[0], label="Act. speed [rpm]")
ax4.plot(time, Speed[1], label="Est. speed [rpm]")
ax4.plot(time, Speed[2], label="Speed ref [rpm]")
ax4.legend()

ax5 = plt.subplot(515, sharex=ax1)
ax5.plot(time, Uout[0], label="Uout A")
ax5.plot(time, Uout[1], label="Uout B")
ax5.plot(time, Uout[2], label="Uout C")
ax5.plot(time, Uout[3], label="Uout mag")
ax5.legend()

plt.show()