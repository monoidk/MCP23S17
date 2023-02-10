/*
 * Copyright (c) 2014-2021, Majenko Technologies
 *               2022-2023, Ivan Labáth
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 *  1. Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *
 *  2. Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *
 *  3. Neither the name of Majenko Technologies nor the names of its contributors may be used
 *     to endorse or promote products derived from this software without
 *     specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */


#ifndef _MCP23S_H
#define _MCP23S_H

#if (ARDUINO >= 100)
# include <Arduino.h>
#else
# include <WProgram.h>
#endif

#ifdef __PIC32MX__
#include <DSPI.h>
#else
#include <SPI.h>
#endif

template <typename UNIT>
class MCP23S {
    public:
#ifdef __PIC32MX__
        /*! SPI object created from the chipKIT DSPI library. */
        typedef DSPI _SPIClass;
#else
        /*! SPI object created from the Arduino compatible SPI library. */
        typedef SPIClass _SPIClass;
#endif
        typedef UNIT unit_t;
        const uint8_t PIN_COUNT = sizeof(unit_t) * 8;

        // MCP23XXX IOCON settings - initialized as 0 on POR
        static const uint8_t IOCON_BANK   = 7;  // Split registers to two banks
        static const uint8_t IOCON_MIRROR = 6;  // Mirror INT on both pins
        static const uint8_t IOCON_SEQOP  = 5;  // Disable R/W sequential addressing
        static const uint8_t IOCON_DISSLW = 4;  // Disable SDA slew rate control (I2C only)
        static const uint8_t IOCON_HAEN   = 3;  // Use addressing from hardware address pins (vs. addr 0)
        static const uint8_t IOCON_ODR    = 2;  // INT output is open-drain
        static const uint8_t IOCON_INTPOL = 1;  // INT output pin active-high (vs. low), ignored if open-drain
        static const uint8_t IOCON_INTCC  = 0;  // Interrupt cleared on reading INTCAP (not GPIO) register (MCP23XX8 only?)

        // default IOCON settings for MCP23S17
        static const uint8_t DEFAULT_IOCON_SINGLE = (1 << IOCON_DISSLW) | (1 << IOCON_HAEN);
        static const uint16_t DEFAULT_IOCON_FULL = (((uint16_t) DEFAULT_IOCON_SINGLE) << 8) || DEFAULT_IOCON_SINGLE;

    private:
        _SPIClass *_spi; /*! This points to a valid SPI object */
        uint8_t _cs;    /*! Chip select pin */
        uint8_t _chip_addr;  /*! 3-bit chip address */

        enum {
            MCP_IODIR,
            MCP_IPOL,
            MCP_GPINTEN,
            MCP_DEFVAL,
            MCP_INTCON,
            MCP_IOCON,
            MCP_GPPU,
            MCP_INTF,
            MCP_INTCAP,
            MCP_GPIO,
            MCP_OLAT,
            MCP_REG_COUNT
        };

        unit_t _reg[MCP_REG_COUNT];   /*! Local mirrors of the 22 internal registers of the MCP23Sxx chip, little-endian */

        void spi_begin();
        void spi_end();

        uint8_t get_byte(unit_t data, uint8_t offset) { return (data >> offset) & 0xff; }
        void set_byte(unit_t & data, uint8_t offset, uint8_t byte) {
            unit_t mask = ~((unit_t) 0xff << offset);
            data = (data & mask) | ((unit_t) byte << offset);
        }

        void readRegisters(uint8_t addr, uint8_t size = 1);
        /* Get cached value of register (1 byte) */
        uint8_t getRegister8(uint8_t addr, uint8_t offset) { return get_byte(_reg[addr], offset); }
        /* Get cached value of register (little-endian) */
        unit_t getRegister(uint8_t addr) { return _reg[addr]; }
        /* Read register and return value (1 byte) */
        uint8_t readRegister8(uint8_t addr, uint8_t offset);
        /* Read register and return value (little-endian) */
        unit_t readRegister(uint8_t addr) { readRegisters(addr, 1); return getRegister(addr); }
        void writeRegisters(uint8_t addr, uint8_t size = 1);
        void writeRegister8(uint8_t addr, uint8_t offset, uint8_t val) {
            set_byte(_reg[addr], offset, val);
            writeRegisterOnly8(addr, offset);
        }
        void writeRegisterOnly8(uint8_t addr, uint8_t offset);
        void writeRegister(uint8_t addr, unit_t val) {
            _reg[addr] = val; writeRegisters(addr, 1);
        }
        void readAll();
        void writeAll();
        void write_iocon_default();

        SPISettings spiSettings = SPISettings((uint32_t)8000000, MSBFIRST, SPI_MODE0);

    public:
        /*! The constructor takes three parameters.  The first is an SPI class
         *  pointer.  This is the address of an SPI object (either the default
         *  SPI object on the Arduino, or an object made using the DSPIx classes
         *  on the chipKIT).  The second parameter is the chip select pin number
         *  to use when communicating with the chip.  The third is the internal
         *  address number of the chip.  This is controlled by the three Ax pins
         *  on the chip.
         *
         *  Example:
         *
         *      MCP23S17 myExpander(&SPI, 10, 0);
         *
         */
        MCP23S(_SPIClass *spi, uint8_t cs, uint8_t addr) {
            _spi = spi;
            _cs = cs;
            _chip_addr = addr;

            _reg[MCP_IODIR] = (unit_t) -1;
            _reg[MCP_IPOL] = 0x0000;
            _reg[MCP_GPINTEN] = 0x0000;
            _reg[MCP_DEFVAL] = 0x0000;
            _reg[MCP_INTCON] = 0x0000;
            _reg[MCP_IOCON] = DEFAULT_IOCON_FULL;
            _reg[MCP_GPPU] = 0x0000;
            _reg[MCP_INTF] = 0x0000;
            _reg[MCP_INTCAP] = 0x0000;
            _reg[MCP_GPIO] = 0x0000;
            _reg[MCP_OLAT] = 0x0000;
        }
        MCP23S(_SPIClass &spi, uint8_t cs, uint8_t addr): MCP23S(&spi, cs, addr) {};

        void begin_tree();
        void begin_light(bool preserve_vals);
        void pinMode(uint8_t pin, uint8_t mode);
        void digitalWrite(uint8_t pin, uint8_t value);
        uint8_t digitalRead(uint8_t pin);


        /*! This function returns the entire 8-bit value of a GPIO port.  Note that
         *  only the bits which correspond to a GPIO pin set to INPUT are valid.
         *  Other pins should be ignored.  The only parameter defines which port (A/B)
         *  to retrieve: 0 is port A and 1 (or anything other than 0) is port B.
         *
         *  Example:
         *
         *      byte portA = myExpander.readPort(0);
         */
        uint8_t readPort(uint8_t port) {
            if (port >= sizeof(unit_t)) {
                return -1;
            }
            return readRegister8(MCP_GPIO, port);
        }

        /*! This is a full version of the parameterised readPort function.  This
         *  version reads the value of all ports, combining them into a single value.
         *
         *  Example:
         *
         *      unsigned int value = myExpander.readPort();
         */
        unit_t readPort() {
            return readRegister(MCP_GPIO);
        }

        /*! This writes an 8-bit value to one of the IO port banks on the chip.
         *  The value is output direct to any pins on that bank that are set as OUTPUT. Any
         *  bits that correspond to pins set to INPUT are ignored.  As with the readPort
         *  function the first parameter defines which bank to use (0 = A, 1 = B, ..).
         *
         *  Example:
         *
         *      myExpander.writePort(0, 0x55);
         */
        void writePort(uint8_t port, uint8_t val) {
            if (port >= sizeof(unit_t))
                return;
            writeRegister8(MCP_OLAT, port, val);
        }

        /*! This is the 16-bit version of the writePort function.  This takes a single
         *  16-bit value and splits it between the two IO ports, the upper half going to
         *  port B and the lower to port A.
         *
         *  Example:
         *
         *      myExpander.writePort(0x55AA);
         */
        void writePort(uint16_t val) {
            writeRegister(MCP_OLAT, val);
        }

        void enableInterrupt(uint8_t pin, uint8_t type);
        void disableInterrupt(uint8_t pin);
        void setMirror(bool m);
        unit_t getInterruptPins();
        uint8_t getInterruptPins(uint8_t port);
        unit_t getInterruptValue();
        uint8_t getInterruptValue(uint8_t port);
        void setInterruptLevel(uint8_t level);
        void setInterruptOD(bool openDrain);

        void setDir(uint8_t pin, uint8_t mode);
        unit_t getDirsInput() { return getRegister(MCP_IODIR); }
        unit_t readDirsInput() { return getRegister(MCP_IODIR); }
        void enablePullup(uint8_t pin, bool enable);
        bool getEnabledPullup(uint8_t pin) { return !!bitRead(getEnabledPullups(), pin); }
        unit_t getEnabledPullups() { return getRegister(MCP_GPPU); }
};

/*
class MCP23S08: public MCP23S<uint8_t> {
    public:
        MCP23S08(_SPIClass *spi, uint8_t cs, uint8_t addr): MCP23S(spi, cs, addr) {};
        MCP23S08(_SPIClass &spi, uint8_t cs, uint8_t addr): MCP23S(&spi, cs, addr) {};
};
*/

class MCP23S17: public MCP23S<uint16_t> {
    public:
        MCP23S17(_SPIClass *spi, uint8_t cs, uint8_t addr): MCP23S(spi, cs, addr) {};
        MCP23S17(_SPIClass &spi, uint8_t cs, uint8_t addr): MCP23S(&spi, cs, addr) {};
};
#endif
