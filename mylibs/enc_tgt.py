#!/usr/bin/python
#
# File:  enc_tgt
#
# Doc:   encoder target thread management
#
import threading
import thread
import time

debugLevel = 0

# ### encoder thread management
#
enc_tgt_thread = None     # holder for encoder target thread
enc_tgt_thread_state = 0  # holder for thread state (OFF,STARTING,RUNNNING,STOPPING)
tgt_l_r = [0,0]           #   [right,left]
status_l_r = [0,0]        #  0 = reached target
enabled_l_r = [0,0]       #  1 = enabled




# enc_tgt_action()
#
# run until (enabled and count < tgt) or do_run goes false

def enc_tgt_action():
    global enc_tgt_thread_state, debugLevel, tgt_l_r, status_l_r, enabled_l_r, encoder_count_l_r
    
    if (debugLevel): print "enc_tgt.enc_tgt_action() alive"
    enc_tgt_thread_state = 2     # set encoder thread state to running first time
    if (debugLevel): print "enc_tgt.enc_tgt_action: enc_tgt_thread_state:%d" % enc_tgt_thread_state
    t = threading.currentThread()
    while getattr(t, "do_run", True):
        if (debugLevel): print "enc_tgt.enc_tgt_action() checking counts"
        if (tgt_l_r[0] <= encoder_count_l_r[0]):  status_l_r[0] = 0
        if (tgt_l_r[1] <= encoder_count_l_r[1]):  status_l_r[1] = 0
        if (not status_l_r[0] and not status_l_r[1] == True):
            if (debugLevel): print "enc_tgt.enc_tgt_action()  both targets reached"
            break
        time.sleep(.05)
        
    enc_tgt_thread_state = 3
    if (debugLevel): print "enc_tgt.enc_tgt_action() end reached"
    return 0

def start_enc_tgt_thread():
    global enc_tgt_thread, enc_tgt_thread_state, encoder_count_l_r

    encoder_count_l_r = [0,0]        # reset encoder count    
    
    if (enc_tgt_thread == None):     # no thread, so create and start
            if (debugLevel): print "enc_tgt.start_enc_tgt_thread: starting new enc_tgt thread"
            enc_tgt_thread_state = 1
            enc_tgt_thread = threading.Thread(target=enc_tgt_action,args=())
            enc_tgt_thread.daemon = True   # to be sure the thread goes away when main ends
            enc_tgt_thread.start()
    else:
        if (debugLevel): print "enc_tgt.start_enc_tgt_thread: running thread exists, use it"
        if (enc_tgt_thread_state == 3):
            if (debugLevel):
                print "enc_tgt.start_enc_tgt_thread: stopped thread exists"
                print "enc_tgt.start_enc_tgt_thread: cancel old thread, re-enter to start"
            cancel()
            start_enc_tgt_thread()

def cancel():
    global enc_tgt_thread_state, enc_tgt_thread
    
    if (debugLevel):
        print "enc_tgt.cancel() called"
        print "enc_tgt.cancel: enc_tgt_thread.is_alive(): %s" % enc_tgt_thread.is_alive()
        print "enc_tgt.cancel: setting do_run False"
    enc_tgt_thread.do_run = False
    if (debugLevel): print "enc_tgt.cancel: enc_tgt_thread_state: %d" % enc_tgt_thread_state  
    enc_tgt_thread.join(1)
    if (debugLevel): print "enc_tgt.cancel: enc_tgt_thread_state: %d" % enc_tgt_thread_state
    enc_tgt_thread = None
    enc_tgt_thread_state = 0
    
def wait_for_tgt_w_timeout():
    



        
# ### TEST MAIN
def main():
    global debugLevel, tgt_l_r, status_l_r, enabled_l_r, encoder_count_l_r

    debugLevel = 1
    print "enc_tgt.main: starting"
    encoder_count_l_r = [0,0]   # start off with count of 0
    

    # ### normal run (start, run till count, vanish)
    print "\nenc_tgt.main: NORMAL RUN"
    print "enc_tgt.main: setting target and parms"
    tgt_l_r = [32,32]
    enabled_l_r = [1,1]   
    status_l_r = [1,1]
    print "enc_tgt.main: starting enc_tgt(1,1,32)"
    start_enc_tgt_thread()
    print "enc_tgt.main: encoder_count_l_r:",encoder_count_l_r
    # print "enc_tgt.main: enc_tgt_thread_state:", enc_tgt_thread_state
    print "enc_tgt.main: begin drive simulation"
    for c in range(50):
        encoder_count_l_r[:]=[i+1 for i in encoder_count_l_r]
        # print "enc_tgt.main: encoder_count_l_r:",encoder_count_l_r
        time.sleep(0.1)
        if (not status_l_r[0] and not status_l_r[1] == True):
            print "enc_tgt.main: stopping motor"    
            break
        
    print "enc_tgt.main: cancelling enc_tgt_thread"    
    cancel()    
    print "enc_tgt.main: status_l_r:", status_l_r
    print "enc_tgt.main: encoder_count_l_r:",encoder_count_l_r
    # print "enc_tgt.main: enc_tgt_thread_state:", enc_tgt_thread_state
    # print "enc_tgt.main: enc_tgt_thread == None?", (enc_tgt_thread == None)
    print "enc_tgt.main: normal run test complete"

    

    # ### cancel early test (start, run for while, cancel)
    print "\nenc_tgt.main: CANCEL EARLY"
    print "enc_tgt.main: setting target and parms"
    tgt_l_r = [32,32]
    enabled_l_r = [1,1]   
    status_l_r = [1,1]
    print "enc_tgt.main: starting enc_tgt(1,1,32)"
    start_enc_tgt_thread()
    print "enc_tgt.main: encoder_count_l_r:",encoder_count_l_r
    # print "enc_tgt.main: enc_tgt_thread_state:", enc_tgt_thread_state
    print "enc_tgt.main: begin drive simulation"
    for c in range(5):
        encoder_count_l_r[:]=[i+1 for i in encoder_count_l_r]
        # print "enc_tgt.main: encoder_count_l_r:",encoder_count_l_r
        time.sleep(0.1)
        if (not status_l_r[0] and not status_l_r[1] == True):
            print "enc_tgt.main: stopping motor"    
            break
        
    print "enc_tgt.main: cancelling enc_tgt_thread"    
    cancel()
    print "enc_tgt.main: status_l_r:", status_l_r
    print "enc_tgt.main: encoder_count_l_r:",encoder_count_l_r
    # print "enc_tgt.main: enc_tgt_thread_state:", enc_tgt_thread_state
    # print "enc_tgt.main: enc_tgt_thread == None?", (enc_tgt_thread == None)
    print "enc_tgt.main: cancel early test complete"

    # ### repeated call test (start, then restart, cancel)
    print "\nenc_tgt.main: RESTART TEST"
    print "enc_tgt.main: setting target and parms"
    tgt_l_r = [32,32]
    enabled_l_r = [1,1]   
    status_l_r = [1,1]
    print "enc_tgt.main: starting enc_tgt(1,1,32)"
    start_enc_tgt_thread()
    print "enc_tgt.main: encoder_count_l_r:",encoder_count_l_r
    # print "enc_tgt.main: enc_tgt_thread_state:", enc_tgt_thread_state
    print "enc_tgt.main: begin drive simulation"
    for c in range(5):
        encoder_count_l_r[:]=[i+1 for i in encoder_count_l_r]
        # print "enc_tgt.main: encoder_count_l_r:",encoder_count_l_r
        time.sleep(0.1)
        if (not status_l_r[0] and not status_l_r[1] == True):
            print "enc_tgt.main: stopping motor"    
            break
    print "enc_tgt.main: status_l_r:", status_l_r
    print "enc_tgt.main: encoder_count_l_r:",encoder_count_l_r,"\n"


    tgt_l_r = [16,16]
    enabled_l_r = [1,1]   
    status_l_r = [1,1]
    print "enc_tgt.main: starting enc_tgt(1,1,16)"
    start_enc_tgt_thread()
    print "enc_tgt.main: encoder_count_l_r:",encoder_count_l_r
    # print "enc_tgt.main: enc_tgt_thread_state:", enc_tgt_thread_state
    print "enc_tgt.main: begin drive simulation"
    for c in range(50):
        encoder_count_l_r[:]=[i+1 for i in encoder_count_l_r]
        # print "enc_tgt.main: encoder_count_l_r:",encoder_count_l_r
        time.sleep(0.1)
        if (not status_l_r[0] and not status_l_r[1] == True):
            print "enc_tgt.main: stopping motor"    
            break
        
    print "enc_tgt.main: cancelling enc_tgt_thread"    
    cancel()
    print "enc_tgt.main: status_l_r:", status_l_r
    print "enc_tgt.main: encoder_count_l_r:",encoder_count_l_r
    # print "enc_tgt.main: enc_tgt_thread_state:", enc_tgt_thread_state
    # print "enc_tgt.main: enc_tgt_thread == None?", (enc_tgt_thread == None)
    print "enc_tgt.main: restart test complete"

    # ### END MAIN
    print "enc_tgt.main: done ********\n"

if __name__ == "__main__":
    main()	    
            
