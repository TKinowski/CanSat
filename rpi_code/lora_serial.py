import argparse

#argument parsing
parser = argparse.ArgumentParser()
parser.add_argument("-c", "--cr", help="coding rate", type=int, choices=[5,6,7,8], default=5)
parser.add_argument("-s", "--sf", help="spreading factor (7-12)", type=int, choices=[7,8,9,10,11,12], default=7)
args = parser.parse_args()


sf = args.sf
cr = args.cr

print("Loading...")

import SX127x
import time
import sys


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
        self.set_spreading_factor(sf)
        self.set_coding_rate(cr)
        self.set_agc_auto_on(True)
        self.set_pa_config(pa_select=1, output_power=15)
        self.packet_size = 255
        self.tx_buffer = []
        self.rx_buffer = []
        self._end = True

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
        self.send_bytes(stob("LoRa serial active."))


    def send_bytes(self, data):
        data_size = len(data)
        number_of_packets = (data_size-1)//self.packet_size + 1
        
        if (debug):
            print("size: " + str(data_size) + ", " + str(number_of_packets) + " packets.")

        for i in range(number_of_packets - 1):
            self.tx_buffer.append(data[i*self.packet_size:(i+1)*self.packet_size])

        self.tx_buffer.append(data[(number_of_packets-1)*self.packet_size:(len(data))])
        if(debug):
            print(self.tx_buffer)
        self.send_one_packet()

radio = transciever()

try:
    radio.start()
    while True:
        c = raw_input("")
        if c != "":
            sys.stdout.write("\r\033[1ASND: " + c + "\n")
            radio.send_bytes(stob(c))
        
        
except KeyboardInterrupt:
    pass
finally:
    sys.stdout.write("\n\r")
    sys.stdout.flush()