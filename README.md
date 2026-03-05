```
serial_read.py
├── class sensor_data
├── serial params: BAUD, TIMEOUT_S, PORT_HINTs
├
├── find_port() -> port name (str)
├── open_serial() -> serial object (serial)
├
├── parse_line(line) -> t_us, adc, raw, tc, fault, fault_oc, fault_scg, fault_scv
├
└── main:
    ├── line = ser_conn.readline()
    ├── parsed = parse_line(text)
    ├── thermistor_conv()
    └── print, update plot
.
plot.py
├── MAX_POINTS
├── time, probe (raw), tc buffers (queue: deque)
├── initial time t0
├── def axes, labels
├
└── update_plot(t_us, raw, tc)
    └── auto rescale y, sliding x
```
