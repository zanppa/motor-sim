# -*- coding: utf-8 -*-
"""
Created on Sun Jul 02 21:34:28 2017
"""

from math import sqrt, pi
import matplotlib.pyplot as plt

from motor_bldc import Motor_BLDC
from motor_dc import Motor_DC
from motor_pmdc import Motor_PMDC

from control_bldc_hall import Control_BLDC_Hall
from control_bldc_foc import Control_BLDC_FOC



#motor = Motor_PMDC({'J':70.0e-9, 'F':2.0e-7, 'V0':-200.0})
motor = Motor_BLDC({'J':70.0e-9, 'F':2.0e-7, 'Ke':0.004, 'P':2.0, 'R':1.0, 'V0':2.0, 'T0':pi/2, 'J':0.00002})
control = Control_BLDC_FOC({'P':2.0, 'Model':'BLDC_Slide'})
control.set_motor(motor)
control.set_reference(8000.0)           # rpm


#fin = 100.0
uinf = 3.0
uina = 6.0

scale = 50000
time = []

EMF = [[], [], [], [], [], []]
Curr = [[], [], [], [], [], []]
Spd = [[], [], [], [], [], [], [], [], []]

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

    #(a1, b1, c1) = motor.get_voltage()
    EMF[0].append(motor.Bemf[0])
    #EMF[1].append(motor.Bemf[1])
    EMF[1].append((motor.Bemf[0] + 2.0*motor.Bemf[1]) / sqrt(3))
    #EMF[2].append(motor.Bemf[2])
    EMF[3].append(control.model.emfestf[0])
    #EMF[4].append(control.emfest[1])
    #EMF[4].append(control.Z[0])
    EMF[5].append(control.model.emfestf[1])

    (u1, v1, w1) = motor.get_current()
    #Curr[0].append(u1)
    #Curr[1].append(v1)
    #Curr[2].append(w1)
    Curr[0].append(control.Ia)
    Curr[1].append(control.Ib)
    Curr[2].append(u1)
    Curr[3].append(control.model.iest[0])
    Curr[4].append(control.model.iest[1])
    #Curr[5].append(control.iest[2])

    (x1,y1,z1) = motor.get_mechanical()
    Spd[0].append(y1 / (2.0 * 3.14159) * 60.0)
    Spd[1].append(control.speed * 60.0)
    Spd[2].append(motor.Theta*motor.P % (2*3.14159))
    Spd[3].append(control.theta % (2*3.14159))

    Spd[4].append(x1*100.0)
    #Spd[5].append(control.dReference)
    #Spd[6].append(control.Trun)
    Spd[5].append(motor.Vin[0])
    Spd[6].append(motor.Vin[1])
    Spd[7].append(control.Id)
    Spd[8].append(control.Iq)



plt.clf()
plt.figure(1)                # the first figure

ax1 = plt.subplot(511)             # the first subplot in the first figure
ax1.plot(time, EMF[0], label="EMF A")
ax1.plot(time, EMF[1], label="EMF B")
#ax1.plot(time, EMF[2], label="EMF C")
ax1.plot(time, EMF[3], label="EMFestF A")
#ax1.plot(time, EMF[4], label="Z A")
ax1.plot(time, EMF[5], label="EMFestF B")
ax1.legend()

ax2 = plt.subplot(512, sharex=ax1)             # the second subplot in the first figure
ax2.plot(time, Curr[0], label="I A")
ax2.plot(time, Curr[1], label="I B")
#ax2.plot(time, Curr[2], label="I C")
ax2.plot(time, Curr[3], label="Iest A")
ax2.plot(time, Curr[4], label="Iest B")
#ax2.plot(time, Curr[5], label="Iest C")
ax2.legend()


ax3 = plt.subplot(513, sharex=ax1)             # the second subplot in the first figure
#ax3.plot(time, u, label="Ia")
#ax3.plot(time, v, label="Iest_a")
#ax3.plot(time, w, label="Z")
#ax3.plot(time, Spd[0], label="Act. speed [rpm]")
#ax3.plot(time, Spd[1], label="Est. speed [rpm]")
ax3.plot(time, Spd[5], label="UU")
ax3.plot(time, Spd[6], label="UV")
ax3.legend()

ax4 = plt.subplot(514, sharex=ax1)
#ax4.plot(time, torque, label="Torque*1e5")
#ax4.plot(time, Spd[0], label="Act. speed [rpm]")
#ax4.plot(time, Spd[1], label="Est. speed [rpm]")
ax4.plot(time, Spd[2], label="Theta")
ax4.plot(time, Spd[3], label="Theta est")
ax4.legend()

ax5 = plt.subplot(515, sharex=ax1)
ax5.plot(time, Spd[4], label="Act torquex100")
#ax5.plot(time, Spd[5], label="D")
#ax5.plot(time, Spd[6], label="Q")
ax5.plot(time, Spd[7], label="ID")
ax5.plot(time, Spd[8], label="IQ")
ax5.legend()
