
"""

* Adafruit's Register library:
  https://github.com/adafruit/Adafruit_CircuitPython_Register

# unzip it and copy over the subfolder (  adafruit_register/ ) into
# the lib folder of your CircuitPython device.

# This driver tested on Raspberry Pi Pico microcontroller, requiring use of
# busio library to select SCL and SDA pins as Pico does not have SCL and SDA
# board pins defined.
# No testing of proximity functionality or interrupts.

"""


# Author:  Landon Ousley   Feb 22, 2022
# Silicon Labs SI1145 PROXIMITY/UV/AMBIENT LIGHT SENSOR IC WITH I2C INTERFACE
# CircuitPython port of Python SI1145 driver by Joe Gutting.

# Liberal re-purposing of CircuitPython code for VEML7700 high precision I2C
# ambient light sensor by  Kattni Rembor.
#
# Sincere apologies to all object-oriented programmers.
#
# With use of Adafruit SI1145 library for Arduino, Adafruit_GPIO.I2C & BMP Library by Tony DiCola
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.


import time
import board 

from busio import I2C

import adafruit_bus_device.i2c_device as i2cdevice



from adafruit_register.i2c_struct import UnaryStruct, ROUnaryStruct
from adafruit_register.i2c_bits import RWBits,ROBits
from adafruit_register.i2c_bit import RWBit, ROBit



# COMMANDS
SI1145_PARAM_QUERY                      = 0x80
SI1145_PARAM_SET                        = 0xA0
SI1145_NOP                              = 0x0
SI1145_RESET                            = 0x01
SI1145_BUSADDR                          = 0x02
SI1145_PS_FORCE                         = 0x05
SI1145_ALS_FORCE                        = 0x06
SI1145_PSALS_FORCE                      = 0x07
SI1145_PS_PAUSE                         = 0x09
SI1145_ALS_PAUSE                        = 0x0A
SI1145_PSALS_PAUSE                      = 0xB
SI1145_PS_AUTO                          = 0x0D
SI1145_ALS_AUTO                         = 0x0E
SI1145_PSALS_AUTO                       = 0x0F
SI1145_GET_CAL                          = 0x12

# Parameters
SI1145_PARAM_I2CADDR                    = 0x00
SI1145_PARAM_CHLIST                     = 0x01
SI1145_PARAM_CHLIST_ENUV                = 0x80
SI1145_PARAM_CHLIST_ENAUX               = 0x40
SI1145_PARAM_CHLIST_ENALSIR             = 0x20
SI1145_PARAM_CHLIST_ENALSVIS            = 0x10
SI1145_PARAM_CHLIST_ENPS1               = 0x01
SI1145_PARAM_CHLIST_ENPS2               = 0x02
SI1145_PARAM_CHLIST_ENPS3               = 0x04

SI1145_PARAM_PSLED12SEL                 = 0x02
SI1145_PARAM_PSLED12SEL_PS2NONE         = 0x00
SI1145_PARAM_PSLED12SEL_PS2LED1         = 0x10
SI1145_PARAM_PSLED12SEL_PS2LED2         = 0x20
SI1145_PARAM_PSLED12SEL_PS2LED3         = 0x40
SI1145_PARAM_PSLED12SEL_PS1NONE         = 0x00
SI1145_PARAM_PSLED12SEL_PS1LED1         = 0x01
SI1145_PARAM_PSLED12SEL_PS1LED2         = 0x02
SI1145_PARAM_PSLED12SEL_PS1LED3         = 0x04

SI1145_PARAM_PSLED3SEL                  = 0x03
SI1145_PARAM_PSENCODE                   = 0x05
SI1145_PARAM_ALSENCODE                  = 0x06

SI1145_PARAM_PS1ADCMUX                  = 0x07
SI1145_PARAM_PS2ADCMUX                  = 0x08
SI1145_PARAM_PS3ADCMUX                  = 0x09
SI1145_PARAM_PSADCOUNTER                = 0x0A
SI1145_PARAM_PSADCGAIN                  = 0x0B
SI1145_PARAM_PSADCMISC                  = 0x0C
SI1145_PARAM_PSADCMISC_RANGE            = 0x20
SI1145_PARAM_PSADCMISC_PSMODE           = 0x04

SI1145_PARAM_ALSIRADCMUX                = 0x0E
SI1145_PARAM_AUXADCMUX                  = 0x0F

SI1145_PARAM_ALSVISADCOUNTER            = 0x10
SI1145_PARAM_ALSVISADCGAIN              = 0x11
SI1145_PARAM_ALSVISADCMISC              = 0x12
SI1145_PARAM_ALSVISADCMISC_VISRANGE     = 0x20
SI1145_PARAM_ALSVISADCMISC_INDOOR       = 0x00


SI1145_PARAM_ALSIRADCOUNTER             = 0x1D
SI1145_PARAM_ALSIRADCGAIN               = 0x1E
SI1145_PARAM_ALSIRADCMISC               = 0x1F
SI1145_PARAM_ALSIRADCMISC_RANGE         = 0x20
SI1145_PARAM_ALSIRADCMISC_INDOOR        = 0x00

SI1145_PARAM_ADCCOUNTER_511CLK          = 0x70

SI1145_PARAM_ADCMUX_SMALLIR             = 0x00
SI1145_PARAM_ADCMUX_LARGEIR             = 0x03



# REGISTERS
SI1145_REG_PARTID                       = 0x00
SI1145_REG_REVID                        = 0x01
SI1145_REG_SEQID                        = 0x02

SI1145_REG_INTCFG                       = 0x03
SI1145_REG_INTCFG_INTOE                 = 0x01
SI1145_REG_INTCFG_INTMODE               = 0x02

SI1145_REG_IRQEN                        = 0x04
SI1145_REG_IRQEN_ALSEVERYSAMPLE         = 0x01
SI1145_REG_IRQEN_PS1EVERYSAMPLE         = 0x04
SI1145_REG_IRQEN_PS2EVERYSAMPLE         = 0x08
SI1145_REG_IRQEN_PS3EVERYSAMPLE         = 0x10


SI1145_REG_IRQMODE1                     = 0x05
SI1145_REG_IRQMODE2                     = 0x06

SI1145_REG_HWKEY                        = 0x07
SI1145_REG_MEASRATE0                    = 0x08
SI1145_REG_MEASRATE1                    = 0x09
SI1145_REG_PSRATE                       = 0x0A
SI1145_REG_PSLED21                      = 0x0F
SI1145_REG_PSLED3                       = 0x10
SI1145_REG_UCOEFF0                      = 0x13
SI1145_REG_UCOEFF1                      = 0x14
SI1145_REG_UCOEFF2                      = 0x15
SI1145_REG_UCOEFF3                      = 0x16
SI1145_REG_PARAMWR                      = 0x17
SI1145_REG_COMMAND                      = 0x18
SI1145_REG_RESPONSE                     = 0x20
SI1145_REG_IRQSTAT                      = 0x21
SI1145_REG_IRQSTAT_ALS                  = 0x01

SI1145_REG_ALSVISDATA0                  = 0x22
SI1145_REG_ALSVISDATA1                  = 0x23
SI1145_REG_ALSIRDATA0                   = 0x24
SI1145_REG_ALSIRDATA1                   = 0x25
SI1145_REG_PS1DATA0                     = 0x26
SI1145_REG_PS1DATA1                     = 0x27
SI1145_REG_PS2DATA0                     = 0x28
SI1145_REG_PS2DATA1                     = 0x29
SI1145_REG_PS3DATA0                     = 0x2A
SI1145_REG_PS3DATA1                     = 0x2B
SI1145_REG_UVINDEX0                     = 0x2C
SI1145_REG_UVINDEX1                     = 0x2D
SI1145_REG_PARAMRD                      = 0x2E
SI1145_REG_CHIPSTAT                     = 0x30

# I2C Address
SI1145_ADDR                             = 0x60

class SI1145:  

    # board hardware identifiers
    seq_id =  ROBits(8, SI1145_REG_SEQID , 0, register_width=1)
    rev_id =  ROBits(8, SI1145_REG_REVID , 0, register_width=1)
    part_id = ROBits(8, SI1145_REG_PARTID, 0, register_width=1)

    # Per SI1145 Silicon Labs datasheet: The system must write the value 0x17
    # to this register for proper Si114x operation 
    hw_key =  RWBits(8, SI1145_REG_HWKEY, 0, register_width=1)

    # UV index calculation co-efficients per Silicon Labs SI1145 datasheet
    ucoef0 =  RWBits(8, SI1145_REG_UCOEFF0, 0, register_width=1)
    ucoef1 =  RWBits(8, SI1145_REG_UCOEFF1, 0, register_width=1)
    ucoef2 =  RWBits(8, SI1145_REG_UCOEFF2, 0, register_width=1)
    ucoef3 =  RWBits(8, SI1145_REG_UCOEFF3, 0, register_width=1)
    

    psled21   =  RWBits(8, SI1145_REG_PSLED21, 0, register_width=1)
    intcfg    =  RWBits(8, SI1145_REG_INTCFG, 0, register_width=1)
    irqen     =  RWBits(8, SI1145_REG_IRQEN, 0, register_width=1)


    # parameter values are written to this single byte register
    # prior to writing command register.  Command register value is
    # logical OR :   parameter |  SI1145_PARAM_SET
    param_wr =  RWBits(8, SI1145_REG_PARAMWR, 0, register_width=1)
    
    # write command register with command SI1145_PARAM_QUERY logical OR
    # with parameter (ie SI1145_PARAM_QUERY | SI1145_PARAM_CHLIST ).
    # value will be returned in this single byte register.
    param_rd =  ROBits(8, SI1145_REG_PARAMRD, 0, register_width=1)

    # The COMMAND Register is the primary mailbox register into the internal sequencer.
    # Writing to the COMMAND register is the only I2C operation that wakes
    # the device from standby mode.  
    command = RWBits(8, SI1145_REG_COMMAND, 0, register_width=1)

    # response register.  Per Silicon Labs SI1145 datasheet
    # The Response register is used in conjunction with command processing.
    # When an error is encountered, the response register will be loaded with
    # an error code. All error codes will have the MSB is set.
    # 0x00–0x0F: No Error. Bits 3:0 form an incrementing roll-over counter.
    # The roll over counter in bit 3:0 increments when a command has been
    # executed by the Si114x.  The host software must make use of the rollover
    # counter to ensure that commands are processed.
    response = ROBits(8, SI1145_REG_RESPONSE, 0, register_width=1)

    # By default place the SI1145 into FORCED MEASUREMENT mode for lowest power usage
    auto_run_disabled : bool = True
    

## Supported size/byte order prefixes:
##    Char    byte order
##      @     native
##      <     little-endian
##      >     big-endian
##      !     network (= big-endian)
##
    
## Supported format codes:
## Format   C Type            Python type             Standard Size
##   b      signed char       integer                 1
##   B      unsigned char     integer                 1
##   x      pad byte          no value
##   h      short             integer                 2
##   H      unsigned short    integer                 2
##   i      int               integer                 4
##   I      unsigned int      integer                 4
##   l      long              integer                 4
##   L      unsigned long     integer                 4
##   q      long long         integer                 8
##   Q      unsigned long long  integer               8
##   s      char[]            bytes
##   P      void*             integer
##   f
##   d ( depending on the floating-point support).

    vis = ROUnaryStruct(SI1145_REG_ALSVISDATA0 , "<H")

    ir = ROUnaryStruct(SI1145_REG_ALSIRDATA0 , "<H")
    
    uv = ROUnaryStruct(SI1145_REG_UVINDEX0 , "<H")
    
    measrate  =  ROUnaryStruct(SI1145_REG_MEASRATE0, "<H")


    def __init__(self, i2c_bus, address =  SI1145_ADDR ):    
        self.i2c_device = i2cdevice.I2CDevice(i2c_bus, address)

        #reset device
        self._reset()
        self._load_calibration()
    
    def _reset(self):
        # device reset    
        self.command =  SI1145_RESET
        time.sleep(.01)
        self.hw_key =  0x17
        time.sleep(.01)

    def readParam(self, p):
        # read Param    
        self.command =  p | SI1145_PARAM_QUERY
        return hex(self.param_rd)


    def writeParam(self, p, v):
        # write Param    
        self.param_wr = v
        self.command =  p | SI1145_PARAM_SET
        return hex(self.response)


    # load calibration to sensor
    def _load_calibration(self):
        # Enable UVindex measurement coefficients!

        self.ucoef0 =  0x29
        self.ucoef1 =  0x89
        self.ucoef2 =  0x02
        self.ucoef3 =  0x00

        # Enable UV,IR,vis
        self.writeParam(SI1145_PARAM_CHLIST, SI1145_PARAM_CHLIST_ENUV | SI1145_PARAM_CHLIST_ENALSIR | SI1145_PARAM_CHLIST_ENALSVIS | SI1145_PARAM_CHLIST_ENPS1)


        # /****************************** Prox Sense 1 */

        # Program LED current
        self.psled21 = 0x03   # 20mA for LED 1 only
        self.writeParam(SI1145_PARAM_PS1ADCMUX, SI1145_PARAM_ADCMUX_LARGEIR)

        # Prox sensor #1 uses LED #1
        self.writeParam(SI1145_PARAM_PSLED12SEL, SI1145_PARAM_PSLED12SEL_PS1LED1)

        # Fastest clocks, clock div 1
        self.writeParam(SI1145_PARAM_PSADCGAIN, 0)

        # Take 511 clocks to measure
        self.writeParam(SI1145_PARAM_PSADCOUNTER, SI1145_PARAM_ADCCOUNTER_511CLK)

        # in prox mode, high range
        self.writeParam(SI1145_PARAM_PSADCMISC, SI1145_PARAM_PSADCMISC_RANGE | SI1145_PARAM_PSADCMISC_PSMODE)
        self.writeParam(SI1145_PARAM_ALSIRADCMUX, SI1145_PARAM_ADCMUX_SMALLIR)

        # Fastest clocks, clock div 1
        self.writeParam(SI1145_PARAM_ALSIRADCGAIN, 0)

        # Take 511 clocks to measure
        self.writeParam(SI1145_PARAM_ALSIRADCOUNTER, SI1145_PARAM_ADCCOUNTER_511CLK)

        # in high range mode
        self.writeParam(SI1145_PARAM_ALSIRADCMISC, SI1145_PARAM_ALSIRADCMISC_RANGE)

        # fastest clocks, clock div 1
        self.writeParam(SI1145_PARAM_ALSVISADCGAIN, 0)

        # Take 511 clocks to measure
        self.writeParam(SI1145_PARAM_ALSVISADCOUNTER, SI1145_PARAM_ADCCOUNTER_511CLK)

        # in high range mode (not normal signal)
        self.writeParam(SI1145_PARAM_ALSVISADCMISC, SI1145_PARAM_ALSVISADCMISC_VISRANGE)


        
    def enableInterrupts(self):
        # Enable interrupt on every sample
        self.intcfg  =  SI1145_REG_INTCFG_INTOE
        self.irqen =  SI1145_REG_IRQEN_ALSEVERYSAMPLE
        

    def modeOutdoor(self):
        # Switch to High Signal mode, suitable for direct sunlight
        # This is the default mode 
        self.writeParam(SI1145_PARAM_ALSIRADCMISC,SI1145_PARAM_ALSIRADCMISC_RANGE)
        self.writeParam(SI1145_PARAM_ALSVISADCMISC, SI1145_PARAM_ALSVISADCMISC_VISRANGE)
        
    def modeIndoor(self):
        # Switch to Normal Signal mode, suitable for indoor lighting
        self.writeParam(SI1145_PARAM_ALSIRADCMISC,SI1145_PARAM_ALSIRADCMISC_INDOOR)
        self.writeParam(SI1145_PARAM_ALSVISADCMISC, SI1145_PARAM_ALSVISADCMISC_INDOOR)
        

    def auto_run_on(self):
        # auto run, continuous readings enabled
        # measurement rate for auto
        self.measrate = 0xFF   # 255 * 31.25uS = 8ms

        self.command = SI1145_PSALS_AUTO

        self.auto_run_disabled = False
        return hex(self.response)

    def auto_run_off(self):
        # auto run off, continuous readings disabled
        # measurements taken on demand (lower power usage) 
        self.command = PSALS_PAUSE
        self.auto_run_disabled = True
        return hex(self.response)
        
    def ForceRead(self):
        # single measurement mode, reduces power consumption
        if self.auto_run_disabled:
            self.command = SI1145_ALS_FORCE
        
    def readUV(self):
        # UV index is a calculation from vis/IR,
        # SI1145 has no actual UV sensor
        self.ForceRead()
        return self.uv / 100

    #returns visible light levels
    def readVisible(self):
        self.ForceRead()
        return self.vis
    
    #returns IR light levels
    def readIR(self):
        self.ForceRead()
        return self.ir
    

# check to see if default SCL and SDA pins are defined
# for the board and return busio_i2c default if found.
# Otherwise need to explicitly define SCL and SDA pins
if 'SCL' in dir(board):
    print ("Default SCL,SDA  I2C pins")
    busio_i2c = I2C(board.SCL,board.SDA)
else:
    # No default SCL,SDA for  Raspberry Pi Pico RP2040
    # 22 of the Picos 40 pins can be configured for SCL,SDA I2C pins
    # SI1145 sensor SCL to Pico pin 22 (board.GP17)
    # SI1145 sensor SDA to Pico pin 21 (board.GP16) 
    busio_i2c = I2C(board.GP17, board.GP16)    


sensor_si1145 = SI1145(busio_i2c)



while True:
    print("vis,ir,uv: ",sensor_si1145.readVisible() ,sensor_si1145.readIR() ,sensor_si1145.readUV()  )

    time.sleep(3)

