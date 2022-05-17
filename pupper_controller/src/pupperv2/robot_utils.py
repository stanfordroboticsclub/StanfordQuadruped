from serial.tools import list_ports


def get_teensy_serial_port():
    for device in list_ports.grep(".*"):
        if device.manufacturer == "Teensyduino":
            return device.device
