import numpy as np

from ..basement import Basement
from ..module import TaskModule

class Sign(TaskModule):
    def __init__(self, base:Basement) -> None:
        super().__init__(base, "Sign")

class Arrow(TaskModule):
    def __init__(self, base:Basement) -> None:
        super().__init__(base, "Arrow")

class TPark(TaskModule):
    def __init__(self, base:Basement) -> None:
        super().__init__(base, "TPark")

class Ramp(TaskModule):
    def __init__(self, base:Basement) -> None:
        super().__init__(base, "Ramp")