import rospy

__timer = dict()

def __is_timer_on(timer, key) -> bool:
    if key in timer:
        return rospy.get_time() < timer[key]
    return False

def __set_timer(timer, key, seconds, force=False):
    if __is_timer_on(timer, key) and (not force):
        return
    timer[key] = rospy.get_time() + seconds

def is_timer_on(key):
    global __timer
    return __is_timer_on(__timer, key)

def is_timer_off(key):
    return not is_timer_on(key)

def set_timer(key, seconds, force):
    global __timer
    return __set_timer(__timer, key, seconds, force)

def debug_timers():
    global __timer
    debug_text = ""
    now = rospy.get_time()
    for k, v in __timer.items():
        time = v-now
        if time > 0:
            time = "%10.3f"%time
        else:
            time = "---"
        debug_text += "%s : %s\n"%(k, time)
    return debug_text