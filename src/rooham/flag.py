from typing import Callable

__flags = dict()

def __is_flag(flags, key) -> bool:
    if key in flags:
        return flags[key]
    return False

def __set_flag(timer, key, value:bool):
    timer[key] = value
    if type(value) != bool:
        timer[key] = False

def is_flag(key):
    global __flags
    return __is_flag(__flags, key)

def set_flag(key, value:bool):
    global __flags
    return __set_flag(__flags, key, value)

def set_flag_with_callback(key, value, callback:Callable, *args, **kw_args):
    if is_flag(key) != value:
        callback(*args, **kw_args)
    set_flag(key, value)

def debug_flags():
    global __flags
    debug_text = ""
    for k, v in __flags.items():
        debug_text += "%s : %s\n"%(k, v)
    return debug_text