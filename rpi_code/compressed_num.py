class compressed_num ():
    def __init__(self, low, high, bits):
        assert(high > low)
        assert(bits<=32)
        self.low = low
        self.high = high
        self.bits = bits
        self.value = low

    def set_value (self, value):
        self.value = float(value)
        if (value > self.high):
            value -= (self.high-self.low)
            return 1
        return 0


    def get_bits (self):
        comp = int(round((self.value-self.low)/(self.high-self.low)*(2**self.bits-1), 0))
        b = [comp >> i & 1 for i in range(self.bits - 1,-1,-1)]
        return b

    def from_bitlist(self, bl):
        val = 0
        for bit in bl:
            val = (val << 1) | bit
        self.value = self.low + val*(self.high-self.low)/(2**self.bits-1)
        print(val)
        return self.value