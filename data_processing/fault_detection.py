"""
over temp monitor w timer latch
dropout detection: range checks
    probe dc (adc == 4095)
    thermocouple dc (tc == 0 and internal == 0)
sensor fault: if fault (oc | scg | scv) persistent == dropout
servo safe shutdown -- event detection markers for overtemp
"""

