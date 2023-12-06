import rospy
from functools import partial

__timer = dict()

def __is_timer_running(timer, key) -> bool:
    if key in timer:
        return rospy.get_time() < timer[key]
    return False

is_timer_running = partial(__is_timer_running, __timer)

def __set_timer(timer, key, seconds, force=False):
    if is_timer_running(key) and (not force):
        return
    timer[key] = rospy.get_time() + seconds

set_timer = partial(__set_timer, __timer)

def debugTimers():
    global __timer
    debug_text = ""
    now = rospy.get_time()
    for k, v in __timer.items():
        time = v-now
        if time > 0:
            time = "%10.3f"%time
        else:
            time = "---"
        debug_text += "%s\n : %s\n"%(k, time)
        debug_text += "\n"
    return debug_text