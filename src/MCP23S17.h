/*
 * Copyright (c) , Majenko Technologies
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


#ifndef _MCP23S17_H
#define _MCP23S17_H

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

class MCP23S17 {
    public:
#ifdef __PIC32MX__
        /*! SPI object created from the chipKIT DSPI library. */
        typedef DSPI _SPIClass;
#else
        /*! SPI object created from the Arduino compatible SPI library. */
        typedef SPIClass _SPIClass;
#endif
        typedef uint16_t unit_t;
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

        unit_t _reg[MCP_REG_COUNT];   /*! Local mirrors of the 22 internal registers of the MCP23S17 chip, little-endian */

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
        MCP23S17(_SPIClass *spi, uint8_t cs, uint8_t addr);
        MCP23S17(_SPIClass &spi, uint8_t cs, uint8_t addr): MCP23S17(&spi, cs, addr) {};

        void begin_tree();
        void begin_light(bool preserve_vals);
        void pinMode(uint8_t pin, uint8_t mode);
        void digitalWrite(uint8_t pin, uint8_t value);
        uint8_t digitalRead(uint8_t pin);

        uint8_t readPort(uint8_t port);
        unit_t readPort();
        void writePort(uint8_t port, uint8_t val);
        void writePort(uint16_t val);
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
#endif
