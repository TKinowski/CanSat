class arduino_comm:
    def __init__(self):
        self.ser = serial.Serial("/dev/ttyAMA0", baudrate = 38400, timeout = 0)
        self.rcv = ""

    def send(self, message):
        if not message.endswith("\n"):
            message += "\n"
        self.ser.write(message)
    
    def process(self):
        while self.ser.in_waiting():
            c = self.ser.read()
            if (c == "\n"):
                self.handle_message(self.rcv)
                self.rcv = ""
            else:
                self.rcv += c
    
    def handle_message(self, message):
        if (not message[0]=="$"):
            return
        if(message[1]=="E"):
            #landed, do landing stuff
            print("was ordered to shut down")
            pass
        if(message[1]=="D"):
            #landed, do landing stuff
            out("Ardu_debug: " + message[2:], save_log=True)
            pass
        if(message[1]=="F"):
            #the flight is starting
            out("Received command to enter flight mode", save_log=True)
            pass
        if(message[1]=="T"):
            #test subsystems
            out("test")

            pass
        if(message[1]=="G"):
            #parse gps data
            print("gps data arrived")
            pass