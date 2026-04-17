import time
import smbus2

I2C_BUS  = 1
ADDR     = 0x29
CMD      = 0x80

BLACK_MAX    = 25       # Black is ~12
WHITE_MIN    = 155      # White is ~168-178
SILVER_MIN   = 110      # Silver is ~131-140
SILVER_MAX   = 150      
RED_MIN      = 0.50     # Red ratio is ~0.66
BLUE_MIN     = 0.40     # Blue ratio is ~0.43-0.44

class ColorHandler:
    def __init__(self):
        self.bus = smbus2.SMBus(I2C_BUS)
        self.bus.write_byte_data(ADDR, CMD | 0x00, 0x01)   
        time.sleep(0.003)
        self.bus.write_byte_data(ADDR, CMD | 0x00, 0x03)   

        self.bus.write_byte_data(ADDR, CMD | 0x01, 0xFF)   
        self.bus.write_byte_data(ADDR, CMD | 0x0F, 0x01)   
        time.sleep(0.01)

        print("TCS3472 ready. Detecting BLACK | RED | BLUE | WHITE | SILVER ...\n")

    def read_word(self, reg):
        low  = self.bus.read_byte_data(ADDR, CMD | reg)
        high = self.bus.read_byte_data(ADDR, CMD | (reg + 1))
        return (high << 8) | low

    def check_tile_color(self):
        try:
            c = self.read_word(0x14)   # clear
            r = self.read_word(0x16)   # red
            g = self.read_word(0x18)   # green
            b = self.read_word(0x1A)   # blue

            total = r + g + b

            if total == 0:
                color = "UNKNOWN"
                
            elif c < BLACK_MAX:
                color = "BLACK"
                return "black"
            elif (r / total) >= RED_MIN and r > g and r > b:
                color = "RED"
                return "red"
            elif (b / total) >= BLUE_MIN and b > r and b > g:
                color = "blue"
                return "blue"
            elif c > WHITE_MIN:
                color = "WHITE"
                return "white"
            elif c > SILVER_MIN and c <= SILVER_MAX:
                color = "SILVER"
                return "silver"
            else:
                color = "UNKNOWN"

            print(f"C={c:5d}  R={r:5d}  G={g:5d}  B={b:5d}  → {color}")
            return None

        except Exception as e:
            print(f"Color sensor read failed: {e}")
            return None

    def close(self):
        try:
            self.bus.write_byte_data(ADDR, CMD | 0x00, 0x00)  
            self.bus.close()
        except:
            pass

_color_instance = None

def init_color_sensor():
    global _color_instance
    try:
        _color_instance = ColorHandler()
    except Exception as e:
        print(f"[COLOR] Initialization failed: {e}")
        _color_instance = None
    return _color_instance

def check_tile_color():
    if _color_instance:
        return _color_instance.check_tile_color()
    return None
