# -*- coding: utf-8 -*-
"""
Test the BLDC motor with FOC control
"""

from math import sqrt, pi, sin
import matplotlib.pyplot as plt

from motor_async import Motor_Async

scale = 50000
Ts = 1. / scale

motor = Motor_Async({})

time = []

EMF = [[],[],[]]
Curr = [[], [], [], []]
Spd = [[], [], []]
Torque = [[], [], []]

Pos = [[], []]
motor.set_load(0.00)

for t in range(scale):
    time.append(float(t) / scale)

    if t > 0.8*scale:
        motor.set_load(-25.0)
    elif t > 0.5*scale:
        motor.set_load(15.0)

    Uact = t*5.0/scale
    if Uact > 1.0:
        Uact = 1.0
    fact = 60.0
    Uact *= 1.414*400.0

    Ua = Uact*sin(2.*pi*fact*t/scale)
    Ub = Uact*sin(2.*pi*fact*t/scale - 2.*pi/3.)
    Uc = Uact*sin(2.*pi*fact*t/scale - 4.*pi/3.)
        
    motor.set_input([Ua, Ub, Uc])
    motor.calculate(1.0 / scale)

    # Cheat a bit and use direct model variables for plotting
    # Back emf in ab frame
    (uu, uv, uw) = motor.get_voltage()
    EMF[0].append(motor.Phia)
    EMF[1].append(motor.Phib)
    #EMF[2].append(uw)
    EMF[2].append(0)

    (iu, iv, iw) = motor.get_current()
    # Motor currents in ab frame
    Curr[0].append(-motor.Ia*motor.Phib)
    Curr[1].append(motor.Ib*motor.Phia)
    Curr[2].append(0)

    (trq, speed, angle) = motor.get_mechanical()
    Spd[0].append(speed / (2.0 * 3.14159))   # Hz


plt.clf()
plt.figure(1)                # the first figure

ax1 = plt.subplot(511)             # the first subplot in the first figure
ax1.plot(time, EMF[0], label="UA")
ax1.plot(time, EMF[1], label="UB")
ax1.plot(time, EMF[2], label="UC")
ax1.legend()

ax2 = plt.subplot(512, sharex=ax1)             # the second subplot in the first figure
ax2.plot(time, Curr[0], label="IA")
ax2.plot(time, Curr[1], label="IB")
ax2.plot(time, Curr[2], label="IC")
ax2.legend()


ax3 = plt.subplot(513, sharex=ax1)             # the second subplot in the first figure
ax3.plot(time, Spd[0], label="Rotor speed [Hz]")
#ax3.plot(time, Spd[1], label="Est. speed [rpm]")
#ax3.plot(time, Spd[2], label="Speed ref [rpm]")
ax3.legend()

#ax4 = plt.subplot(514, sharex=ax1)
##ax4.plot(time, Torque[0], label="Act. torque")
#ax4.plot(time, Torque[1], label="Est. torque")
#ax4.plot(time, Torque[2], label="Torque ref")
#ax4.legend()
#
#ax5 = plt.subplot(515, sharex=ax1)
#ax5.plot(time, Pos[0], label="Act. rotor pos")
#ax5.plot(time, Pos[1], label="Est. rotor pos")
#ax5.legend()

plt.show()