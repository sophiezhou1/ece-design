import math

def thermistor_conv(raw):
    vout = raw
    # ADC_MAX = 4095
    rfixed = 100000
    vcc = 3.295
    # vout = raw * vcc / ADC_MAX

    A = -0.1943 * 10 ** -3
    B = 3.4023 * 10 ** -4
    C = -2.3843 * 10 ** -7
    
    rth = vout * rfixed / (vcc - vout)
    Tinv = A + B * math.log(rth) + C * (math.log(rth)) ** 3
    T = Tinv ** -1
    T -= 273.15
    return T

if __name__ == "__main__":
    voltage = 1.75
    T = thermistor_conv(voltage);
    print(T)