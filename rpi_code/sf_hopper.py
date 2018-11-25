print("Loading...")

import SX127x
import time
import sys
import os

print("Loaded.")

debug = False

def stob (string):
    return [ord(c) for c in ''.join(list(string))]

class transciever(SX127x.LoRa):
    def __init__ (self):
        SX127x.BOARD.setup()
        SX127x.LoRa.__init__(self, False)
        self.set_dio_mapping([0,0,0,0,0,0])
        self.set_mode(SX127x.MODE.SLEEP)
        self.set_freq(433.0)
        self.set_spreading_factor(12)
        self.set_agc_auto_on(True)
        self.set_pa_config(pa_select=1, output_power=15)
        self.packet_size = 16
        self._end = True
        self.hop = 0
        self.lastSendComplete = 0


    def on_rx_done(self):
        payload = self.read_payload(nocheck=True)
        print("RCV: " + str(payload))
        if (payload[0] == 0x08 and payload[1] == 0x07):
            self.hop = payload[2] - 3
            self.transmit([0x08, 0x07, payload[2]])
            

    def on_tx_done(self):
        if (self.hop != 0):
            self.set_spreading_factor(self.get_spreading_factor()+self.hop)
            self.hop = 0
        self.lastSendComplete = time.time()

    def send_packet(self):
        self.transmit(stob(os.urandom(self.packet_size)))


radio = transciever()
lastSendOrder = 0

try:
    radio.lastSendComplete = time.time()
    while True:
        if (time.time()-radio.lastSendComplete>2 and lastSendOrder<radio.lastSendComplete):
            print("Sending")
            radio.send_packet()
            lastSendOrder = time.time()
        time.sleep(0.1)
        
        
except KeyboardInterrupt:
    pass
finally:
    sys.stdout.write("\n\r")
    sys.stdout.flush()