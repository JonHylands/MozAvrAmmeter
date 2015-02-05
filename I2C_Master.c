
// Port for the I2C
#define I2C_DDR DDRB
#define I2C_PIN PINB
#define I2C_PORT PORTB

// Pins to be used in the bit banging
#define I2C_CLK 2
#define I2C_DAT 1

#define I2C_CLK_MASK (1 << I2C_CLK)
#define I2C_DAT_MASK (1 << I2C_DAT)

// Set PORT to input, tristate
#define I2C_DATA_HI()\
    I2C_DDR &= ~I2C_DAT_MASK;

// Set PORT to output, low
#define I2C_DATA_LO()\
    I2C_DDR |= I2C_DAT_MASK;\
    I2C_PORT &= ~I2C_DAT_MASK;

// Set PORT to input, tristate
#define I2C_CLOCK_HI()\
    I2C_DDR &= ~I2C_CLK_MASK;

// Set PORT to output, low
#define I2C_CLOCK_LO()\
    I2C_DDR |= I2C_CLK_MASK;\
    I2C_PORT &= ~I2C_CLK_MASK;

#define bit_delay() us_spin(5)

void I2C_WriteBit(unsigned char c) {

    if (c > 0) {
        I2C_DATA_HI();
    } else {
        I2C_DATA_LO();
    }

    bit_delay();
    I2C_CLOCK_HI();
    bit_delay();
    I2C_CLOCK_LO();
    bit_delay();

//     if (c > 0) {
//         I2C_DATA_LO();
//     }

//     bit_delay();
}

unsigned char I2C_ReadBit() {

    I2C_DATA_HI();
    I2C_CLOCK_HI();
    bit_delay();

    unsigned char c = I2C_PIN;

    I2C_CLOCK_LO();
    bit_delay();

    return (c >> I2C_DAT) & 1;
}

// Inits bitbanging port, must be called before using the functions below
//
void I2C_Init() {

//    I2C_PORT &= ~ ((1 << I2C_DAT) | (1 << I2C_CLK));

    I2C_CLOCK_HI();
    I2C_DATA_HI();

    bit_delay();
}

void I2C_Test() {

//    I2C_PORT &= ~ ((1 << I2C_DAT) | (1 << I2C_CLK));

    I2C_DDR &= ~I2C_DAT_MASK;
    // I2C_PORT |= I2C_DAT_MASK;

}

// Send a START Condition
//
void I2C_Start() {

    // set both to high at the same time
    I2C_DDR &= ~ ((1 << I2C_DAT) | (1 << I2C_CLK));
    bit_delay();

    I2C_DATA_LO();
    bit_delay();

    I2C_CLOCK_LO();
    bit_delay();
}

// Send a STOP Condition
//
void I2C_Stop() {

    I2C_CLOCK_HI();
    bit_delay();

    I2C_DATA_HI();
    bit_delay();
}

// write a byte to the I2C slave device
//
unsigned char I2C_Write(unsigned char c) {

    for (char i = 0; i < 8; i++) {
        I2C_WriteBit(c & 128);
        c <<= 1;
    }

    return I2C_ReadBit();
    //return 0;
}


// read a byte from the I2C slave device
//
unsigned char I2C_Read(unsigned char ack) {

    unsigned char res = 0;
    for (char i = 0; i < 8; i++) {
        res <<= 1;
        res |= I2C_ReadBit();
    }

    if (ack > 0) {
        I2C_WriteBit(0);
    } else {
        I2C_WriteBit(1);
    }

    bit_delay();
    return res;
}
