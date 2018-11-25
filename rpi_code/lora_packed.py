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
import daemons.gps_daemon as gps_daemon
from compressed_num import compressed_num

print("Loaded.")

gps = gps_daemon.gpsReader(1, "gps")
gps.daemon = True
gps.start()

temperature = compressed_num(low = -15,   high = 30,     bits = 11)
pressure =    compressed_num(low = 60000, high = 110000, bits = 16)
humidity =    compressed_num(low = 0,     high = 100,    bits = 7)

latitude =    compressed_num(low = 52.164,high = 52.165,  bits = 16)
longitude =   compressed_num(low = 21.050,high = 21.060,  bits = 16)
altitude =    compressed_num(low = 0,     high = 3500,   bits = 15)

roll =        compressed_num(low = 0,     high = 360,    bits = 12)
pitch =       compressed_num(low = 0,     high = 360,    bits = 12)
heading =     compressed_num(low = 0,     high = 360,    bits = 12)

def concentrate_data():
    bits = []
    bits += temperature.get_bits()
    bits += pressure.get_bits()
    bits += humidity.get_bits()
    bits += latitude.get_bits()
    bits += longitude.get_bits()
    bits += altitude.get_bits()
    bits += roll.get_bits()
    bits += pitch.get_bits()
    bits += heading.get_bits()
    bits += [0] * (8-len(bits)%8)
    return [int("".join(map(str, bits[i:i+8])), 2) for i in range(0, len(bits), 8)]

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
        pass

    def start(self):
        #self.transmit([0x05] + stob("Lora tx on."))
        pass

radio = transciever()

lastSend = time.time()

temperature.set_value(22)
pressure.set_value(1E5)
humidity.set_value(70)

try:
    radio.start()
    while True:
        if time.time()-lastSend>1.0:
            altitude.set_value(gps.altitude)
            latitude.set_value(gps.latitude)
            longitude.set_value(gps.longitude)

            radio.transmit(concentrate_data())
            print (concentrate_data())
            lastSend = time.time()
        
        
except KeyboardInterrupt:
    pass
finally:
    sys.stdout.write("\n\r")
    sys.stdout.flush()