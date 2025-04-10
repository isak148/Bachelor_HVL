import smbus2
import time

VL6180X_I2C_ADDR = 0x29  # Standard I2C-adresse for VL6180X

# Registeradresser
RESULT_RANGE_STATUS = 0x04D
RESULT_RANGE_VAL = 0x062
SYSTEM_INTERRUPT_CLEAR = 0x015
SYSRANGE_START = 0x018
SYSRANGE_RANGE_CHECK_ENABLES = 0x02D
SYSRANGE_RANGE_IGNORE_THRESHOLD = 0x026

# Opprett I2C-bussen
bus = smbus2.SMBus(1)

def read_register(reg):
    return bus.read_byte_data(VL6180X_I2C_ADDR, reg)

def write_register(reg, value):
    bus.write_byte_data(VL6180X_I2C_ADDR, reg, value)

def configure_sensor():
    """ Deaktiver Range Ignore Threshold """
    print("?? Deaktiverer Range Ignore Threshold...")
    write_register(SYSRANGE_RANGE_CHECK_ENABLES, 0x00)  # Deaktiverer ECE og SNR
    write_register(SYSRANGE_RANGE_IGNORE_THRESHOLD, 0x00)  # Fjern terskelen

def take_single_measurement():
    """ Tar en enkelt avstandsm�ling fra VL6180X """
    print("?? Starter en enkel m�ling...")

    write_register(SYSRANGE_START, 0x01)  # Start m�ling

    for _ in range(100):  # Maks ventetid 1 sekund
        status = read_register(RESULT_RANGE_STATUS)
        if status & 0x01:  # Bit 0: M�ling klar
            break
        time.sleep(0.01)

    # Les m�lt avstand
    range_val = read_register(RESULT_RANGE_VAL)
    print(f"?? M�lt avstand: {range_val} mm")

    # Nullstill avbrudd
    write_register(SYSTEM_INTERRUPT_CLEAR, 0x07)

    return range_val

# **Konfigurer sensoren med oppdaterte verdier**
configure_sensor()

# **Test �n m�ling**
distance = take_single_measurement()
print(RESULT_RANGE_VAL)