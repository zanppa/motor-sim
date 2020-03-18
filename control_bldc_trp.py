# -*- coding: utf-8 -*-
"""
Created on Sun Jul 09 11:18:10 2017

"""

def BLDC_com():
    phase = -1
    next_phase = 0
    next_delay = 0
    prev_bemf = uin
    com_delay = 0
    
    for t in range(scale/3):
        time.append(float(t) / scale)
        
        if phase == 0:
            motor.set_input(uin, float("Nan"), -uin)
        elif phase == 1:
            motor.set_input(uin, -uin, float("Nan"))
        elif phase == 2:
            motor.set_input(float("Nan"), -uin, uin)
        elif phase == 3:
            motor.set_input(-uin, float("Nan"), uin)
        elif phase == 4:
            motor.set_input(-uin, uin, float("Nan"))
        elif phase == 5:
            motor.set_input(float("Nan"), uin, -uin)    
        else:
            motor.set_input(float("NaN"), float("NaN"), float("NaN"))
        
        ph.append(phase)    
        
        if t > 0.25*scale:
            motor.set_load(-0.036)
        elif t > 0.1*scale:
            motor.set_load(-0.018)
        else:
            motor.set_load(0)
    
        motor.calculate(1.0/scale)
    
        if isnan(motor.Va):
            a.append(motor.Bemfa)
        else:
            a.append(motor.Va)
            
        if isnan(motor.Vb):
            b.append(motor.Bemfb)
        else:
            b.append(motor.Vb)
            
        if isnan(motor.Vc):
            c.append(motor.Bemfc)
        else:
            c.append(motor.Vc)
        
        (g,h,j) = motor.get_currents()
        u.append(g)
        v.append(h)
        w.append(j)
        
        (g,h,j) = motor.get_bemf()
        bu.append(g)
        bv.append(h)
        bw.append(j)
    
        # Calculate commutation if back emf changes sign
        if phase == 0 and next_delay <= 0:
            if motor.Bemfb * prev_bemf < 0:
                next_phase = phase + 1
                next_delay = 2.0 * 3.14159 / (abs(motor.speed) * motor.polepairs * 2.0 * 12.0) * scale
            prev_bemf = motor.Bemfb
        elif phase == 1 and next_delay <= 0:
            if motor.Bemfc * prev_bemf < 0:
                next_phase = phase + 1
                next_delay = 2.0 * 3.14159 / (abs(motor.speed) * motor.polepairs * 2.0 * 12.0) * scale
            prev_bemf = motor.Bemfc
        elif phase == 2 and next_delay <= 0:
            if motor.Bemfa * prev_bemf < 0:
                next_phase = phase + 1
                next_delay = 2.0 * 3.14159 / (abs(motor.speed) * motor.polepairs * 2.0 * 12.0) * scale
            prev_bemf = motor.Bemfa
        elif phase == 3 and next_delay <= 0:
            if motor.Bemfb * prev_bemf < 0:
                next_phase = phase + 1
                next_delay = 2.0 * 3.14159 / (abs(motor.speed) * motor.polepairs * 2.0 * 12.0) * scale
            prev_bemf = motor.Bemfb
        elif phase == 4 and next_delay <= 0:
            if motor.Bemfc * prev_bemf < 0:
                next_phase = phase + 1
                next_delay = 2.0 * 3.14159 / (abs(motor.speed) * motor.polepairs * 2.0 * 12.0) * scale
            prev_bemf = motor.Bemfc
        elif next_delay <= 0:
            if motor.Bemfa * prev_bemf < 0:
                next_phase = 0
                next_delay = 2.0 * 3.14159 / (abs(motor.speed) * motor.polepairs * 2.0 * 12.0) * scale
            prev_bemf = motor.Bemfa
        
        if next_delay > 0:
            next_delay -= 1
        else:
            phase = next_phase
            next_delay = 0
        
        torque.append(motor.torque*100000.0)
        #torque.append((motor.F*motor.speed/motor.J))  # Friction loss
        spd.append(60*motor.speed/(2*3.14159))
        position.append(motor.angle)
        #position.append(phase)