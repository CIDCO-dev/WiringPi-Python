
HIGH = 1
LOW = 0
MSBFIRST = 1
LSBFIRST = 0
WPI_MODE_PINS = 0
WPI_MODE_GPIO = 1
WPI_MODE_SYS = 2
MODE_PINS = 0
MODE_GPIO = 1
MODE_SYS = 2
INPUT = 0
OUTPUT = 1
PWM_OUTPUT = 2
PUD_OFF = 0
PUD_DOWN = 1
PUD_UP = 2

class NES(object):
    def setupNesJoystick(self,*args):
        return setupNesJoystick(*args)

    def readNesJoystick(self,*args):
        return readNesJoystick(*args)

class Serial(object):
    device = '/dev/ttyAMA0'
    baud = 9600
    serial_id = 0
    def printf(self,*args):
        return serialPrintf(self.serial_id,*args)

    def dataAvail(self,*args):
        return serialDataAvail(self.serial_id,*args)

    def getchar(self,*args):
        return serialGetchar(self.serial_id,*args)

    def putchar(self,*args):
        return serialPutchar(self.serial_id,*args)

    def puts(self,*args):
        return serialPuts(self.serial_id,*args)

    def __init__(self,device,baud):
        self.device = device
        self.baud = baud
        self.serial_id = serialOpen(self.device,self.baud)

    def __del__(self):
        serialClose(self.serial_id)

class Pin(object):
    PIN_ID = -1
    PIN_MODE = OUTPUT
    def __init__(self,pin_id,pin_mode=OUTPUT):
        self.PIN_ID = pin_id
        self.PIN_MODE = pin_mode
        pinMode(self.PIN_ID, self.PIN_MODE)

    def read(self):
        return digitalRead(self.PIN_ID)

    def write(self,value):
        return digitalWrite(self.PIN_ID,value)

    def pwmWrite(self,value):
        return pwmWrite(self.PIN_ID,value)

    def mode(self,pin_mode):
        self.PIN_MODE = pin_mode
        pinMode(self.PIN_ID,self.PIN_MODE)

class Shift(object):
    DATA_PIN = 0
    CLOCK_PIN = 0
    LATCH_PIN = 0
    def __init__(self,data_pin,clock_pin,latch_pin):
        self.DATA_PIN = data_pin
        self.CLOCK_PIN = clock_pin
        self.LATCH_PIN = latch_pin
        self.DATA_PIN.mode(OUTPUT)
        self.CLOCK_PIN.mode(OUTPUT)
        self.LATCH_PIN.mode(OUTPUT)

    def write(self,value,byte_order = MSBFIRST):
        digitalWrite(self.LATCH_PIN.PIN_ID,LOW)
        shiftOut(self.DATA_PIN.PIN_ID,self.CLOCK_PIN.PIN_ID,byte_order,value)
        digitalWrite(self.LATCH_PIN.PIN_ID,HIGH)

class GPIO(object):
    HIGH = 1
    LOW = 0
    MSBFIRST = 1
    LSBFIRST = 0
    WPI_MODE_PINS = 0
    WPI_MODE_GPIO = 1
    WPI_MODE_SYS = 2
    MODE_PINS = 0
    MODE_GPIO = 1
    MODE_SYS = 2
    INPUT = 0
    OUTPUT = 1
    PWM_OUTPUT = 2
    PUD_OFF = 0
    PUD_DOWN = 1
    PUD_UP = 2
    MODE = 0
    def __init__(self,pinmode=WPI_MODE_PINS):
        self.MODE = pinmode
        if pinmode == 0:
            wiringPiSetup()
        elif pinmode == 1:
            wiringPiSetupGpio()
        elif pinmode == 2:
            wiringPiSetupSys()

    def delay(self,*args):
        delay(*args)

    def delayMicroseconds(self,*args):
        delayMicroseconds(*args)

    def millis(self):
        return millis()

    def pinMode(self,*args):
        pinMode(*args)

    def digitalWrite(self,*args):
        digitalWrite(*args)

    def pwmWrite(self,*args):
        pwmWrite(*args)

    def digitalRead(self,*args):
        return digitalRead(*args)

    def shiftOut(self,*args):
        shiftOut(*args)

    def shiftOutWithDelay(self,*args):
        shiftOutWithDelay(*args)

    def shiftIn(self,*args):
        return shiftIn(*args)

    def pullUpDnControl(self,*args):
        return pullUpDnControl(*args)

    def softPwmCreate(*args):
        return softPwmCrate(*args)

    def softPwmWrite(*args):
        return sofPwmWrite(*args)
