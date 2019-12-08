from serial.tools import list_ports
ports = list(list_ports.comports())
for p in ports:
    print(p)