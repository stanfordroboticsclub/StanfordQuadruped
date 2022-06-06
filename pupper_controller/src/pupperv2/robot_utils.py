from serial.tools import list_ports


def get_teensy_serial_port():
    for device in list_ports.grep(".*"):
        if device.manufacturer == "Teensyduino":
            print("found device: ", device.device)
            return device.device
    return "COM5" #Connor's Port
