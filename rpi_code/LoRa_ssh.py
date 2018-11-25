import sys
import os
import subprocess
import SX127x
import time

def stob (string):
    return [ord(c) for c in ''.join(list(string))]

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

    def on_rx_done(self):
        payload = self.read_payload(nocheck=True)
        text = "".join(map(chr, payload))
        print("#> " + text)
        result = execute(text)
        self.send_bytes(stob(result))


    def on_tx_done(self):
        print ("tx done, tx_buffer: ")
        print (self.tx_buffer)
        if len(self.tx_buffer) > 0:
            self.send_one_packet()
        else:
            self.set_mode(SX127x.MODE.RXCONT)

    def send_one_packet(self):
        self.transmit(self.tx_buffer.pop(0))

    def start(self):
        self.send_bytes(stob("Raspberry pi ssh over LoRa"))
        time.sleep(1)

    def send_bytes(self, data):
        packet_number = (len(data)-1)//self.packet_size + 1
        print("packets: " + str(packet_number))
        for i in range(packet_number - 1):
            self.tx_buffer.append(data[i*self.packet_size:(i+1)*self.packet_size])
        self.tx_buffer.append(data[(packet_number-1)*self.packet_size:(len(data))])
        print(self.tx_buffer)
        self.send_one_packet()
        
            

radio = transciever()


def execute(command):  
    try:
        tokenized = command.split(" ")
        if len(tokenized) == 1:
            result = subprocess.check_output(tokenized)
        else:
            result = subprocess.check_output(tokenized)
    except:
        result="err"
    finally:
        return result

try:
    radio.start()
    while True:
        time.sleep(1)
        
        
except KeyboardInterrupt:
    pass
finally:
    sys.stdout.write("\n\r")
    sys.stdout.flush()