from basement import Basement
class Submodule:
    def __init__(self, base:Basement, name:str):
        print("I'm %s"%name)
        self.basement = base
        self.timeout = 300

    def callback(self, data):
        self.timeout = 60
        
    def update(self):
        self.timeout -= 1