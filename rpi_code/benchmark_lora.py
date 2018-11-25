print("Loading...")

import SX127x.LoRa
#from SX127x.constants import *
from SX127x.board_config import BOARD
import time
import sys
import argparse
import os

print("Loaded.")

#argument parsing
parser = argparse.ArgumentParser()
parser.add_argument("-d", "--data", help="amount of data to transfer in KB", type=int, default=8)
parser.add_argument("-p", "--size", help="packet size in bytes", type=int, default=255)
parser.add_argument("-s", "--sf", help="spreading factor (7-12)", type=int, choices=[6,7,8,9,10,11,12], default=7)
args = parser.parse_args()


sf = args.sf
data = args.data
size = args.size


class transciever(SX127x.LoRa):
    def __init__ (self):
        SX127x.BOARD.setup()
        SX127x.LoRa.__init__(self, False)
        self.set_dio_mapping([0,0,0,0,0,0])
        self.set_mode(SX127x.MODE.SLEEP)
        self.set_freq(433.0)
        self.set_agc_auto_on(True)
        self.set_pa_config(pa_select=1, output_power=15)
        self.packet_size = 255
        self.tx_buffer = []
        self.rx_buffer = []
        self._end = True

    def negotiate_speed(self):
        self.send_bytes([0x01])

    def handle_sf_increase(self):
        pass
        
    def handle_sf_decrease(self):
        pass

    def on_rx_done(self):
        #self.rx_buffer.append(self.read_payload(nocheck=True))
        payload = self.read_payload(nocheck=True)
        snr = self.get_pkt_snr_value()
        sf = self.get_spreading_factor()

        received_text = "".join(map(chr, payload))
        print("RCV: " + received_text + ", SNR: " + str(snr) + ", at sf: " + str(sf))
        self.set_mode(SX127x.MODE.RXCONT)

    def on_tx_done(self):
        if len(self.tx_buffer) > 0:
            self.send_one_packet()

    def send_one_packet(self):
        self.transmit(self.tx_buffer.pop(0))

    def start(self):
        number_of_packets = ((data*1024)-1)//self.packet_size + 1
        self.send_bytes([sf, number_of_packets])
        time.sleep(0.5)
        self.set_spreading_factor(sf)
        


    def send_bytes(self, data):
        data_size = len(data)
        number_of_packets = (data_size-1)//self.packet_size + 1

        for i in range(number_of_packets - 1):
            self.tx_buffer.append(data[i*self.packet_size:(i+1)*self.packet_size])
            
        self.tx_buffer.append(data[(number_of_packets-1)*self.packet_size:(len(data))])
        self.send_one_packet()

radio = transciever()

def stob (string):
    return [ord(c) for c in ''.join(list(string))]

try:
    radio.packet_size = size
    radio.start()
    radio.send_bytes(stob(os.urandom(1024*data)))
    while(len(radio.tx_buffer)>0):
        time.sleep(0.5)
        
        
except KeyboardInterrupt:
    pass
finally:
    sys.stdout.write("\n\r")
    sys.stdout.flush()