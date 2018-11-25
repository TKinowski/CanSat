"""This module provides the battery_daemon class, which reads voltage from ADS1115"""
from time import sleep
import threading
import ADS1x15


# Choose a gain of 1 for reading voltages from 0 to 4.09V.
# Or pick a different gain to change the range of voltages that are read:
#  - 2/3 = +/-6.144V
#  -   1 = +/-4.096V
#  -   2 = +/-2.048V
#  -   4 = +/-1.024V
#  -   8 = +/-0.512V
#  -  16 = +/-0.256V
# See table 3 in the ADS1015/ADS1115 datasheet for more info on gain.

class battery_daemon(threading.Thread):

    def __init__(self, name, threadID, gain = 2.0/3, delay = 0.1):
        threading.Thread.__init__(self)
        self.adc = ADS1x15.ADS1115()
        self.name = name
        self.threadID = threadID
        self._end = False
        self.gain = gain
        self.volts_per_step = (4.096/(2**15*self.gain))
        if self.gain < 1:
            self.gain = 0
        self.delay = delay
        self.lock = threading.RLock()
        self.v1 = 0
        self.v2 = 0
        
        

    def endThread(self):
		self._end = True


    def run(self):
        while not self._end:
            #try:
            with self.lock:
                self.v1 = 0.8*self.v1 + 0.2*self.adc.read_adc(0, gain=self.gain)*self.volts_per_step
                self.v2 = 0.8*self.v2 + 0.2*self.adc.read_adc(1, gain=self.gain)*self.volts_per_step
            sleep(self.delay)



# Main loop.
if __name__=='__main__':
    import sys
    bat = battery_daemon("battery", 1,  delay = 0.1)
    bat.start()
    try:
        while True:
            with bat.lock:
                sys.stdout.write('\033[2J' + str(bat.v1) + 'V' )
                sys.stdout.write('\n' + str(bat.v2) + 'V\n')
            sys.stdout.flush()
            sleep(0.1)
    except KeyboardInterrupt:
        pass
    finally:
        bat.endThread()
        bat.join()