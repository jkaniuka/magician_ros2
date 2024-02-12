# Source: https://stackoverflow.com/questions/12090503/listing-available-com-ports-with-python
import serial.tools.list_ports
ports = serial.tools.list_ports.comports()

for port, desc, hwid in sorted(ports):
        print("{}: {} [{}]".format(port, desc, hwid))