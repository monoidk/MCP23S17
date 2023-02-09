/*
 * Copyright (c) 2014-2021, Majenko Technologies
 *               2022, Ivan Labáth
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

#include <MCP23S17.h>

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
MCP23S17::MCP23S17(_SPIClass *spi, uint8_t cs, uint8_t addr) {
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

void MCP23S17::spi_begin() {
    _spi->beginTransaction(spiSettings);
    ::digitalWrite(_cs, LOW);
}

void MCP23S17::spi_end() {
    ::digitalWrite(_cs, HIGH);
    _spi->endTransaction();
}

/*! The begin_tree function performs the initial configuration of the IO expander chip.
 *  Not only does it set up the SPI communications, but it also configures the chip
 *  for address-based communication and sets the default parameters and registers
 *  to sensible values.
 *
 *  Example:
 *
 *      myExpander.begin_tree();
 *
 */
void MCP23S17::begin_tree() {
    // Should we be initializing SPI?
    // It is documented in doc, but seems inappropriate.
    _spi->begin();
    ::pinMode(_cs, OUTPUT);
    ::digitalWrite(_cs, HIGH);
    // first enable HAEN
    write_iocon_default();
    writeAll();
}

void MCP23S17::write_iocon_default() {
    spi_begin();
    uint8_t cmd = 0b01000000;
    _spi->transfer(cmd);
    _spi->transfer(MCP_IOCON * sizeof(unit_t));
    _spi->transfer(DEFAULT_IOCON_SINGLE);
    spi_end();
}

/*! The begin_light function performs the initial configuration of the MCP IO expander chip only.
 *  It configures the chip for address-based communication and sets the default parameters and registers,
 *  optionally preserving GPIO output configuration - direction, pullup and output latches.
 *
 *  It does not configure SPI nor does it configure the CS pin, as the begin_full()
 *
 *  Example:
 *
 *      myExpander.begin_tree();
 *
 */
void MCP23S17::begin_light(bool preserve_vals) {
    write_iocon_default();
    if (preserve_vals) {
        readRegister(MCP_IODIR);
        readRegister(MCP_GPPU);
        readRegister(MCP_OLAT);
    }
    writeAll();
}

/*! This private function reads from the register array on the chip
 * a number of unit-size values, and stores them in the _reg array for later usage.
 */
void MCP23S17::readRegisters(uint8_t addr, uint8_t size) {
    if ((uint32_t) addr + size > MCP_REG_COUNT) {
        return;
    }
    uint8_t cmd = 0b01000001 | ((_chip_addr & 0b111) << 1);
    spi_begin();
    _spi->transfer(cmd);
    _spi->transfer(addr * sizeof(unit_t));
    for (uint32_t i = 0; i < size; ++i) {
        unit_t reg = 0;
        for (uint32_t j = 0; j < sizeof reg; ++j) {
            uint8_t byte = _spi->transfer(0xFF);
            reg = reg | (byte << (8 * j));
        }
        _reg[addr++] = reg;
    }
    spi_end();
}

/*! This private function reads a byte-size value from the specified register on the chip and
 *  stores it in the _reg array for later usage.
 */
uint8_t MCP23S17::readRegister8(uint8_t addr, uint8_t offset) {
    if ((uint32_t) addr > MCP_REG_COUNT) {
        return -1;
    }
    if (offset >= sizeof (unit_t)) {
        return -1;
    }
    uint8_t cmd = 0b01000001 | ((_chip_addr & 0b111) << 1);
    spi_begin();
    _spi->transfer(cmd);
    _spi->transfer(addr * sizeof(unit_t) + offset);
    uint8_t byte = _spi->transfer(0xFF);
    set_byte(_reg[addr], offset, byte);
    spi_end();
    return byte;
}

/*! This private function writes the current value of a unit-size register
 * (as stored in the _reg array) out to the register in the chip.
 */
void MCP23S17::writeRegisters(uint8_t addr, uint8_t size) {
    if ((uint32_t) addr + size > MCP_REG_COUNT) {
        return;
    }
    uint8_t cmd = 0b01000000 | ((_chip_addr & 0b111) << 1);
    spi_begin();
    _spi->transfer(cmd);
    _spi->transfer(addr * sizeof(unit_t));
    for (uint32_t i = 0; i < size; ++i) {
        unit_t reg = _reg[addr++];
        for (uint32_t j = 0; j < sizeof(reg); ++j) {
            _spi->transfer(reg & 0xff);
            reg = reg >> 8;
        }
    }
    spi_end();
}

/*! This private function writes the current value of a byte-size register
 * (as stored in the _reg array) out to the register in the chip.
 */
void MCP23S17::writeRegisterOnly8(uint8_t addr, uint8_t offset) {
    if ((uint32_t) addr > MCP_REG_COUNT) {
        return;
    }
    if (offset >= sizeof(unit_t)) {
        return;
    }
    uint8_t cmd = 0b01000000 | ((_chip_addr & 0b111) << 1);
    spi_begin();
    _spi->transfer(cmd);
    _spi->transfer(addr * sizeof(unit_t) + offset);
    uint8_t byte = get_byte(_reg[addr], offset);
    _spi->transfer(byte);
    spi_end();
}

/*! This private function performs a bulk read on all the registers in the chip to
 *  ensure the _reg array contains all the correct current values.
 */
void MCP23S17::readAll() {
    readRegisters(0, MCP_REG_COUNT);
}

/*! This private function performs a bulk write of all the data in the _reg array
 *  out to all the registers on the chip.  It is mainly used during the initialisation
 *  of the chip.
 */
void MCP23S17::writeAll() {
    writeRegisters(0, MCP_REG_COUNT);
}

/*! Just like the pinMode() function of the Arduino API, this function sets the
 *  direction of the pin.  The first parameter is the pin nimber (0-15) to use,
 *  and the second parameter is the direction of the pin.  There are standard
 *  Arduino macros for different modes which should be used.  The supported macros are:
 *
 *  * OUTPUT
 *  * INPUT
 *  * INPUT_PULLUP
 *
 *  The INPUT_PULLUP mode enables the weak pullup that is available on any pin.
 *
 *  Example:
 *
 *      myExpander.pinMode(5, INPUT_PULLUP);
 */
void MCP23S17::pinMode(uint8_t pin, uint8_t mode) {
    if (pin >= PIN_COUNT) {
        return;
    }
    switch (mode) {
        case OUTPUT:
            setDir(pin, OUTPUT);
            enablePullup(pin, false);
            break;

        case INPUT:
        case INPUT_PULLUP:
            setDir(pin, INPUT);
            enablePullup(pin, (mode == INPUT_PULLUP));
            break;
    }
}

/*! Like the Arduino API's namesake, this function will set an output pin to a specific
 *  value, either HIGH (1) or LOW (0).  If the pin is currently set to an INPUT instead of
 *  an OUTPUT, then this function acts like the old way of enabling / disabling the pullup
 *  resistor, which pre-1.0.0 versions of the Arduino API used - i.e., set HIGH to enable the
 *  pullup, or LOW to disable it.
 *
 *  Example:
 *
 *      myExpander.digitalWrite(3, HIGH);
 */
void MCP23S17::digitalWrite(uint8_t pin, uint8_t value) {
    if (pin >= PIN_COUNT) {
        return;
    }

    unit_t dir = getRegister(MCP_IODIR);
    // mode: INPUT : OUTPUT  =>  set: pullup vs. output latch
    uint8_t reg = bitRead(dir, pin) ? MCP_GPPU : MCP_OLAT;

    unit_t regval = getRegister(reg);
    bitWrite(regval, pin, value);
    writeRegister(reg, regval);
}

void MCP23S17::enablePullup(uint8_t pin, bool enable) {
    if (pin >= PIN_COUNT) {
        return;
    }
    unit_t pu = getRegister(MCP_GPPU);
    bitWrite(pu, pin, enable);
    writeRegister(MCP_GPPU, pu);
}

/*! Set pin direction as INPUT or OUTPUT.
 *
 *  The first parameter is the pin nimber (0-15) to use,
 *  and the second parameter is the direction of the pin.
 *  Supported values are
 *
 *  * OUTPUT
 *  * INPUT
 *
 *  Example:
 *
 *      myExpander.setDir(5, INPUT);
 */
void MCP23S17::setDir(uint8_t pin, uint8_t mode) {
    if (pin >= PIN_COUNT) {
        return;
    }
    uint16_t dir = getRegister(MCP_IODIR);
    switch (mode) {
        case OUTPUT: bitClear(dir, pin); break;
        case INPUT:  bitSet(dir, pin);  break;
        default: return;
    }
    writeRegister(MCP_IODIR, dir);
}

/*! This will return the current state of a pin set to INPUT, or the last
 *  value written to a pin set to OUTPUT.
 *
 *  Example:
 *
 *      byte value = myExpander.digitalRead(4);
 */
uint8_t MCP23S17::digitalRead(uint8_t pin) {
    if (pin >= PIN_COUNT) {
        return 0;
    }

    uint8_t mode = (getRegister(MCP_IODIR) & (1<<pin)) == 0 ? OUTPUT : INPUT;

    switch (mode) {
        case OUTPUT:
            return bitRead(getRegister(MCP_OLAT), pin) ? HIGH : LOW;
        case INPUT:
            return bitRead(readRegister(MCP_GPIO), pin) ? HIGH : LOW;
    }
    return 0;
}

/*! This function returns the entire 8-bit value of a GPIO port.  Note that
 *  only the bits which correspond to a GPIO pin set to INPUT are valid.
 *  Other pins should be ignored.  The only parameter defines which port (A/B)
 *  to retrieve: 0 is port A and 1 (or anything other than 0) is port B.
 *
 *  Example:
 *
 *      byte portA = myExpander.readPort(0);
 */
uint8_t MCP23S17::readPort(uint8_t port) {
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
MCP23S17::unit_t MCP23S17::readPort() {
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
void MCP23S17::writePort(uint8_t port, uint8_t val) {
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
void MCP23S17::writePort(uint16_t val) {
    writeRegister(MCP_OLAT, val);
}

/*! This enables the interrupt functionality of a pin.  The interrupt type can be one of:
 *
 *  * CHANGE
 *  * RISING
 *  * FALLING
 *
 *  When an interrupt occurs the corresponding port's INT pin will be driven to it's configured
 *  level, and will remain there until either the port is read with a readPort or digitalRead, or the
 *  captured port status at the time of the interrupt is read using getInterruptValue.
 *
 *  Example:
 *
 *      myExpander.enableInterrupt(4, RISING);
 */
void MCP23S17::enableInterrupt(uint8_t pin, uint8_t type) {
    if (pin >= PIN_COUNT) {
        return;
    }
    uint16_t intcon = getRegister(MCP_INTCON);
    uint16_t defval = getRegister(MCP_DEFVAL);
    uint16_t gpinten = getRegister(MCP_GPINTEN);

    switch (type) {
        case CHANGE:
            bitClear(intcon, pin);
            break;
        case RISING:
            bitSet(intcon, pin);
            bitClear(defval, pin);
            break;
        case FALLING:
            bitSet(intcon, pin);
            bitSet(defval, pin);
            break;
    }

    bitSet(gpinten, pin);

    writeRegister(MCP_INTCON, intcon);
    writeRegister(MCP_DEFVAL, defval);
    writeRegister(MCP_GPINTEN, gpinten);
}

/*! This disables the interrupt functionality of a pin.
 *
 *  Example:
 *
 *      myExpander.disableInterrupt(4);
 */
void MCP23S17::disableInterrupt(uint8_t pin) {
    if (pin >= PIN_COUNT) {
        return;
    }
    uint16_t gpinten = getRegister(MCP_GPINTEN);

    bitClear(gpinten, pin);
    writeRegister(MCP_GPINTEN, gpinten);
}

/*! The two IO banks can have their INT pins connected together.
 *  This enables you to monitor both banks with just one interrupt pin
 *  on the host microcontroller.  Calling setMirror with a parameter of
 *  *true* will enable this feature.  Calling it with *false* will disable
 *  it.
 *
 *  Example:
 *
 *      myExpander.setMirror(true);
 */
void MCP23S17::setMirror(bool m) {
    if (m) {
        bitSet(_reg[MCP_IOCON], IOCON_MIRROR);
        bitSet(_reg[MCP_IOCON], IOCON_MIRROR + 8);
    } else {
        bitClear(_reg[MCP_IOCON], IOCON_MIRROR);
        bitClear(_reg[MCP_IOCON], IOCON_MIRROR + 8);
    }
    writeRegisters(MCP_IOCON);
}

/*! This function returns a 16-bit bitmap of the the pin or pins that have cause an interrupt to fire.
 *
 *  Example:
 *
 *      unsigned int pins = myExpander.getInterruptPins();
 */
MCP23S17::unit_t MCP23S17::getInterruptPins() {
    return readRegister(MCP_INTF);
}

/*! This function returns an 8-bit bitmap of the given port pins that have caused an interrupt to fire.
 *
 *  Example:
 *
 *      unsigned int pins = myExpander.getInterruptPins(0);
 */
uint8_t MCP23S17::getInterruptPins(uint8_t port) {
    return readRegister8(MCP_INTF, port);
}

/*! This returns a snapshot of the IO pin states at the moment the last interrupt occured.  Reading
 *  this value clears the interrupt status (and hence the INT pins) for the whole chip.
 *  Until this value is read (or the current live port value is read) no further interrupts can
 *  be indicated.
 *
 *  Example:
 *
 *      unsigned int pinValues = myExpander.getInterruptValue();
 */
MCP23S17::unit_t MCP23S17::getInterruptValue() {
    return readRegister(MCP_INTCAP);
}

/*! This returns a snapshot of the given port pin states at the moment the last interrupt occured.
 *
 *  Reading this getInterruptValue (MCP_INTCAP) or readPort (MCP_GPIO) clears the interrupt status
 *  (and hence INT pins) for the port, depending on IOCON_INTCC setting and its support on given
 *  device type. Iff IOCON_INTCC is not supported, both reads clear the interrupt.
 *
 *  Until the interrupt is cleared, no further interrupts can be indicated (on the given port?).
 *
 *  Example:
 *
 *      unsigned int pinValues = myExpander.getInterruptValue(0);
 */
uint8_t MCP23S17::getInterruptValue(uint8_t port) {
    return readRegister8(MCP_INTCAP, port);
}

/*! This sets the "active" level for an interrupt.  HIGH means the interrupt pin
 *  will go HIGH when an interrupt occurs, LOW means it will go LOW.
 *
 *  Example:
 *
 *      myExpander.setInterruptLevel(HIGH);
 */
void MCP23S17::setInterruptLevel(uint8_t level) {
    if (level == LOW) {
        bitClear(_reg[MCP_IOCON], IOCON_INTPOL);
        bitClear(_reg[MCP_IOCON], IOCON_INTPOL + 8);
    } else {
        bitSet(_reg[MCP_IOCON], IOCON_INTPOL);
        bitSet(_reg[MCP_IOCON], IOCON_INTPOL + 8);
    }
    writeRegisters(MCP_IOCON, 1);
}

/*! Using this function it is possible to configure the interrupt output pins to be open
 *  drain.  This means that interrupt pins from multiple chips can share the same interrupt
 *  pin on the host MCU.  This causes the level set by setInterruptLevel to be ignored.  A
 *  pullup resistor will be required on the host MCU's interrupt pin.
 *
 *  Example:
 *
 *      myExpander.setInterruptOD(true);
 */
void MCP23S17::setInterruptOD(bool openDrain) {
    if (openDrain) {
        bitSet(_reg[MCP_IOCON], IOCON_ODR);
        bitSet(_reg[MCP_IOCON], IOCON_ODR + 8);
    } else {
        bitClear(_reg[MCP_IOCON], IOCON_ODR);
        bitClear(_reg[MCP_IOCON], IOCON_ODR + 8);
    }
    writeRegisters(MCP_IOCON, 1);
}
