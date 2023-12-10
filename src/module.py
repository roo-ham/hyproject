#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from abc import ABC, abstractmethod

from basement import Basement

class AbstractModule(ABC):
    @abstractmethod
    def __init__(self, base:Basement, name:str):
        pass

    @abstractmethod
    def update(self):
        pass

class IOModule(AbstractModule):
    @abstractmethod
    def __init__(self, base:Basement, name:str):
        print("I'm %s the IOModule!"%name)
        self.basement = base
        self.timeout = 300

    def callback(self, data):
        self.timeout = 60
        
    def update(self):
        self.timeout -= 1

class TaskModule(AbstractModule):
    @abstractmethod
    def __init__(self, base:Basement, name:str):
        print("I'm %s the TaskModule!"%name)
        base.taskmodules[name] = self
        self.basement = base
        self.x = 0.0
        self.z = 0.0
        self.weight_x = 0.0
        self.weight_z = 0.0

    def getDataset(self):
        return self.x, self.z, self.weight_x, self.weight_z