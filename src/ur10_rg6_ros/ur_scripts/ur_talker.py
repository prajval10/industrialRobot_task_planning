#!/usr/bin/env python
# license removed for brevity
import time
from ur_driver.io_interface import *

if __name__ == "__main__":
    print "testing io-interface"
    get_states()
    print "listener has been activated"
    set_states()
    print "service-server has been started"
    while(True):
        set_digital_out(0, True)
        print "set"
        time.sleep(1)
        # set_digital_out(0, False)
        print(Digital_Out_States[0])
        # time.sleep(1)