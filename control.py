# -*- coding: utf-8 -*-
"""
Created on Mon Jul 10 21:01:03 2017
"""


class Control:
    reference = 0.0
    Trun = 0.0          # Run time [s]    
    
    def __init__(self, params):
        self.params = {}
        self.Trun = 0.0
        
        self.config = {                 \
                'Name' : 'Controller base',         \
                'Phases': 0,             \
                'Views': 0               \
                }
        return
    
    def set_parameters(self, params):
        return
        
    def get_parameters_list(self):
        return self.params
    
    def get_parameters(self):
        return {}
    
    def set_motor(self, motor):
        self.motor = motor
        return
    
    def set_reference(self, ref):
        self.reference = ref
    
    def calculate(self, dt):
        return
        
        