import smbus2
import time

VL6180X_ADDRESS = 0x29  # Sensorens I2C-adresse
bus = smbus2.SMBus(1)

# VL6180X registerkonfigurasjon
def factory_reset():
    print("?? Tilbakestiller VL6180X...")
    try:
        bus.write_byte_data(VL6180X_ADDRESS, 0x016, 0x00)  # Nullstiller SYSTEM_FRESH_OUT_OF_RESET
        time.sleep(0.1)

        register_settings = [
            (0x0207, 0x01),
            (0x0208, 0x01),
            (0x0096, 0x00),
            (0x0097, 0xfd),
            (0x00e3, 0x00),
            (0x00e4, 0x04),
            (0x00e5, 0x02),
            (0x00e6, 0x01),
            (0x00e7, 0x03),
            (0x00f5, 0x02),
            (0x00d9, 0x05),
            (0x00db, 0xce),
            (0x00dc, 0x03),
            (0x00dd, 0xf8),
            (0x009f, 0x00),
            (0x00a3, 0x3c),
            (0x00b7, 0x00),
            (0x00bb, 0x3c),
            (0x00b2, 0x09),
            (0x00ca, 0x09),
            (0x0198, 0x01),
            (0x01b0, 0x17),
            (0x01ad, 0x00),
            (0x00ff, 0x05),
            (0x0100, 0x05),
            (0x0199, 0x05),
            (0x01a6, 0x1b),
            (0x01ac, 0x3e),
            (0x01a7, 0x1f),
            (0x0030, 0x00),
        ]

        for reg, value in register_settings:
            bus.write_byte_data(VL6180X_ADDRESS, reg, value)
            time.sleep(0.002)  # Kort pause for stabil skriving

        print("? Sensoren er nullstilt og klar!")
    except Exception as e:
        print(f"? Feil under nullstilling: {e}")

# Kj�r nullstillingen
factory_reset()
bus.close()



