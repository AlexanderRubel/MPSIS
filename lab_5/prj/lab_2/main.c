#include <msp430.h>
#include <stdint.h>
//#include <math.h>
// CONSTANTS
#define MCLK 25000000
#define TICKSPERUS (MCLK / 1000000)
// PORT DEFINITIONS
#define ACCEL_INT_IN P2IN
#define ACCEL_INT_OUT P2OUT
#define ACCEL_INT_DIR P2DIR
#define ACCEL_SCK_SEL P2SEL
#define ACCEL_INT_IE P2IE
#define ACCEL_INT_IES P2IES
#define ACCEL_INT_IFG P2IFG
#define ACCEL_INT_VECTOR PORT2_VECTOR
#define ACCEL_OUT P3OUT
#define ACCEL_DIR P3DIR
#define ACCEL_SEL P3SEL
// PIN DEFINITIONS
#define ACCEL_INT BIT5
#define ACCEL_CS BIT5
#define ACCEL_SIMO BIT3
#define ACCEL_SOMI BIT4
#define ACCEL_SCK BIT7
#define ACCEL_PWR BIT6
// ACCELEROMETER REGISTER DEFINITIONS
#define REVID 0x01
#define CTRL 0x02
#define MODE_400 0x04 // Measurement mode 400 Hz ODR
#define MODE_400_M 0x0C
#define DOUTX 0x06
#define DOUTY 0x07
#define DOUTZ 0x08
#define G_RANGE_2 0x80 // 2g range
#define I2C_DIS 0x10 // I2C disabled
int8_t accelData;
int8_t RevID;
int8_t Cma3000_xAccel;

int8_t Cma3000_yAccel;
int8_t Cma3000_zAccel;
// Screen size
#define DOGS102x6_X_SIZE 102 // Display Size in dots: X-Axis
#define DOGS102x6_Y_SIZE 64 // Display Size in dots: Y-Axis
// Screen printing styles
#define DOGS102x6_DRAW_NORMAL 0x00 // Display dark pixels on a light background
#define DOGS102x6_DRAW_INVERT 0x01 // Display light pixels on a dark background
// Screen printing mode
#define DOGS102x6_DRAW_IMMEDIATE 0x01 // Display update done immediately
#define DOGS102x6_DRAW_ON_REFRESH 0x00 // Display update done only with refresh
#define SET_COLUMN_ADDRESS_MSB 0x10 //Set SRAM col. addr. before write, last 4 bits =
// ca4-ca7
#define SET_COLUMN_ADDRESS_LSB 0x00 //Set SRAM col. addr. before write, last 4 bits =
// ca0-ca3
#define SET_POWER_CONTROL 0x2F //Set Power control - booster, regulator, and follower
// on
#define SET_SCROLL_LINE 0x40 //Scroll image up by SL rows (SL = last 5 bits),
// range:0-63
#define SET_PAGE_ADDRESS 0xB0 //Set SRAM page addr (pa = last 4bits), range:0-8
#define SET_VLCD_RESISTOR_RATIO 0x27 //Set internal resistor ratio Rb/Ra to adjust contrast
#define SET_ELECTRONIC_VOLUME_MSB 0x81 //Set Electronic Volume "PM" to adjust contrast
#define SET_ELECTRONIC_VOLUME_LSB 0x0F //Set Electronic Volume "PM" to adjust contrast (PM =
// last 5 bits)
#define SET_ALL_PIXEL_ON 0xA4 //Disable all pixel on (last bit 1 toturn on all pixels
// - does not affect memory)
#define SET_INVERSE_DISPLAY 0xA6 //Inverse display off (last bit 1 toinvert display -
// does not affect memory)
#define SET_DISPLAY_ENABLE 0xAF //Enable display (exit sleep mode &restore power)
#define SET_SEG_DIRECTION 0xA1 //Mirror SEG (column) mapping (setbit0 to mirror
// display)
#define SET_COM_DIRECTION 0xC8 //Mirror COM (row) mapping (set bit3to mirror display)
#define SET_SEG_DIRECTION_1 0xA5 //Mirror SEG (column) mapping (setbit0 to mirror
// display)
#define SET_COM_DIRECTION_1 0xC4 //Mirror COM (row) mapping (set bit3 to mirror display)
#define SYSTEM_RESET 0xE2 //Reset the system. Control regs reset, memory not
// affected
#define NOP 0xE3 //No operation
#define SET_LCD_BIAS_RATIO 0xA2 //Set voltage bias ratio (BR = bit0)
#define SET_CURSOR_UPDATE_MODE 0xE0 //Column address will increment wit write operation
// (but no wrap around)

#define RESET_CURSOR_UPDATE_MODE 0xEE //Return cursor to column address from before cursor
// update mode was set
#define SET_ADV_PROGRAM_CONTROL0_MSB 0xFA //Set temp. compensation curve to - 0.11%/C
#define SET_ADV_PROGRAM_CONTROL0_LSB 0x90
// Pins from MSP430 connected to LCD
#define CD BIT6
#define CS BIT4
#define RST BIT7
#define BACKLT BIT6
#define SPI_SIMO BIT1
#define SPI_CLK BIT3
// Ports
#define CD_RST_DIR P5DIR
#define CD_RST_OUT P5OUT
#define CS_BACKLT_DIR P7DIR
#define CS_BACKLT_OUT P7OUT
#define CS_BACKLT_SEL P7SEL
#define SPI_SEL P4SEL
#define SPI_DIR P4DIR
uint8_t dogs102x6Memory[816 + 2];
uint8_t currentPage = 0, currentColumn = 0;
uint8_t colSize = 6;
int CURRENT_ORIENTATION = 0;
uint8_t backlight = 8;
uint8_t contrast = 0x0F;
uint8_t drawmode = DOGS102x6_DRAW_IMMEDIATE;

uint8_t Dogs102x6_initMacro[] = {
    SET_SCROLL_LINE,
    SET_SEG_DIRECTION,
    SET_COM_DIRECTION,
    SET_ALL_PIXEL_ON,
    SET_INVERSE_DISPLAY,
    SET_LCD_BIAS_RATIO,
    SET_POWER_CONTROL,
    SET_VLCD_RESISTOR_RATIO,
    SET_ELECTRONIC_VOLUME_MSB,
    SET_ELECTRONIC_VOLUME_LSB,
    SET_ADV_PROGRAM_CONTROL0_MSB,
    SET_ADV_PROGRAM_CONTROL0_LSB,
    SET_DISPLAY_ENABLE,
    SET_PAGE_ADDRESS,
    SET_COLUMN_ADDRESS_MSB,
    SET_COLUMN_ADDRESS_LSB
};
uint8_t Dogs102x6_inv1Macro[] = {
    SET_SEG_DIRECTION,
    SET_COM_DIRECTION
};
uint8_t Dogs102x6_inv2Macro[] = {
    SET_SEG_DIRECTION_1,
    SET_COM_DIRECTION_1
};
void Dogs102x6_writeCommand(uint8_t *sCmd, uint8_t i);
int parseProjectionByte(uint8_t projectionByte);

int8_t Cma3000_readRegister(int8_t Address);
int8_t Cma3000_writeRegister(uint8_t Address, int8_t accelData);
void ShowNumber(int num);
void SetupButtons();
double atan(double x, int n);
int MAPPING_VALUES[] = { 1142, 571, 286, 143, 71, 36, 18 };
uint8_t BITx[] = { BIT6, BIT5, BIT4, BIT3, BIT2, BIT1, BIT0 };
int num_z=1, num_y=1;
int pos_on_screen = 0;
static const uint8_t FONT6x8[] = {
    /* 6x8 font, each line is a character each byte is a one pixel wide column
    * of that character. MSB is the top pixel of the column, LSB is the bottom
    * pixel of the column. 0 = pixel off. 1 = pixel on. */
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // space
    0x00, 0x00, 0xFA, 0x00, 0x00, 0x00, // !
    0x00, 0xE0, 0x00, 0xE0, 0x00, 0x00, // "
    0x28, 0xFE, 0x28, 0xFE, 0x28, 0x00, // #
    0x24, 0x54, 0xFE, 0x54, 0x48, 0x00, // $
    0xC4, 0xC8, 0x10, 0x26, 0x46, 0x00, // %
    0x6C, 0x92, 0x6A, 0x04, 0x0A, 0x00, // &
    0x00, 0x10, 0xE0, 0xC0, 0x00, 0x00, // '
    0x00, 0x38, 0x44, 0x82, 0x00, 0x00, // (
    0x00, 0x82, 0x44, 0x38, 0x00, 0x00, // )
    0x54, 0x38, 0xFE, 0x38, 0x54, 0x00, // *
    0x10, 0x10, 0x7C, 0x10, 0x10, 0x00, // +
    0x00, 0x02, 0x1C, 0x18, 0x00, 0x00, // ,
    0x10, 0x10, 0x10, 0x10, 0x10, 0x00, // -
    0x00, 0x00, 0x06, 0x06, 0x00, 0x00, // .
    0x04, 0x08, 0x10, 0x20, 0x40, 0x00, // /
    //96 Bytes
    0x7C, 0x8A, 0x92, 0xA2, 0x7C, 0x00, // 0
    0x00, 0x42, 0xFE, 0x02, 0x00, 0x00, // 1
    0x42, 0x86, 0x8A, 0x92, 0x62, 0x00, // 2
    0x84, 0x82, 0x92, 0xB2, 0xCC, 0x00, // 3
    0x18, 0x28, 0x48, 0xFE, 0x08, 0x00, // 4
    0xE4, 0xA2, 0xA2, 0xA2, 0x9C, 0x00, // 5
    0x3C, 0x52, 0x92, 0x92, 0x0C, 0x00, // 6
    0x82, 0x84, 0x88, 0x90, 0xE0, 0x00, // 7
    0x6C, 0x92, 0x92, 0x92, 0x6C, 0x00, // 8
    0x60, 0x92, 0x92, 0x94, 0x78, 0x00, // 9
    0x00, 0x7C, 0xA2, 0x92, 0x8A, 0x7C, // 0 inv
    //0x7C, 0x8A, 0x92, 0xA2, 0x7C, 0x00, // 0
    0x00, 0x02, 0x02, 0xFE, 0x42, 0x00, // 1 inv
    //0x00, 0x42, 0xFE, 0x02, 0x00, 0x00, // 1
    0x00, 0x62, 0x92, 0x8A, 0x86, 0x42, // 2 inv
    //0x42, 0x86, 0x8A, 0x92, 0x62, 0x00, // 2
    0x00, 0xCC, 0xB2, 0xB9, 0x82, 0x84, // 3 inv
    //0x84, 0x82, 0x92, 0xB2, 0xCC, 0x00, // 3
    0x00, 0x08, 0xFE, 0x48, 0x28, 0x18, // 4 inv
    //0x18, 0x28, 0x48, 0xFE, 0x08, 0x00, // 4
    0x00, 0x9C, 0xA2, 0xA2, 0xA2, 0xE4, // 5 inv
    //0xE4, 0xA2, 0xA2, 0xA2, 0x9C, 0x00, // 5 inv
    0x00, 0x0C, 0x92, 0x92, 0x52, 0x3C, // 6 inv
    //0x3C, 0x52, 0x92, 0x92, 0x0C, 0x00, // 6 inv
    0x00, 0xE0, 0x90, 0x88, 0x84, 0x82, // 7 inv
    //0x82, 0x84, 0x88, 0x90, 0xE0, 0x00, // 7 inv
    0x00, 0x6C, 0x92, 0x92, 0x92, 0x6C, // 8 inv
    //0x6C, 0x92, 0x92, 0x92, 0x6C, 0x00, // 8 inv
    0x00, 0x78, 0x94, 0x92, 0x92, 0x60, // 9 inv
    //0x60, 0x92, 0x92, 0x94, 0x78, 0x00, // 9 inv
    0x00, 0x10, 0x10, 0x7C , 0x10, 0x10, // + inv
    // 0x10, 0x10, 0x7C, 0x10, 0x10, 0x00, // +
    0x00, 0x10, 0x10, 0x10, 0x10, 0x10, // - inv
    //0x10, 0x10, 0x10, 0x10, 0x10, 0x00, // -
};

void Dogs102x6_init(void)
{
    // Port initialization for LCD operation
    CD_RST_DIR |= RST;
    // Reset is active low
    CD_RST_OUT &= RST;
    // Reset is active low
    CD_RST_OUT |= RST;
    // Chip select for LCD
    CS_BACKLT_DIR |= CS;
    // CS is active low
    CS_BACKLT_OUT &= ~CS;
    // Command/Data for LCD
    CD_RST_DIR |= CD;
    // CD Low for command
    CD_RST_OUT &= ~CD;
    // P4.1 option select SIMO
    SPI_SEL |= SPI_SIMO;
    SPI_DIR |= SPI_SIMO;
    // P4.3 option select CLK
    SPI_SEL |= SPI_CLK;
    SPI_DIR |= SPI_CLK;
    // Initialize USCI_B1 for SPI Master operation
    // Put state machine in reset
    UCB1CTL1 |= UCSWRST;
    //3-pin, 8-bit SPI master
    UCB1CTL0 = UCCKPH + UCMSB + UCMST + UCMODE_0 + UCSYNC;
    // Clock phase - data captured first edge, change second edge
    // MSB
    // Use SMCLK, keep RESET
    UCB1CTL1 = UCSSEL_2 + UCSWRST;
    UCB1BR0 = 0x02;
    UCB1BR1 = 0;
    // Release USCI state machine
    UCB1CTL1 &= ~UCSWRST;
    UCB1IFG &= ~UCRXIFG;
    Dogs102x6_writeCommand(Dogs102x6_initMacro, 13);
    // Deselect chip
    CS_BACKLT_OUT |= CS;
    dogs102x6Memory[0] = 102;
    dogs102x6Memory[1] = 8;
}
void Dogs102x6_writeCommand(uint8_t *sCmd, uint8_t i)
{
    // Store current GIE state
    uint16_t gie = __get_SR_register() & GIE;
    // Make this operation atomic
    __disable_interrupt();
    // CS Low
    P7OUT &= ~CS;
    // CD Low
    P5OUT &= ~CD;

    while (i)    {
        // USCI_B1 TX buffer ready?
        while (!(UCB1IFG & UCTXIFG)) ;
        // Transmit data
        UCB1TXBUF = *sCmd;
        // Increment the pointer on the array
        sCmd++;
        // Decrement the Byte counter
        i--;
    }
    // Wait for all TX/RX to finish
    while (UCB1STAT & UCBUSY) ;
    // Dummy read to empty RX buffer and clear any overrun conditions
    UCB1RXBUF;
    // CS High
    P7OUT |= CS;
    // Restore original GIE state
    __bis_SR_register(gie);
}
void Dogs102x6_writeData(uint8_t *sData, uint8_t i)
{
    // Store current GIE state
    uint16_t gie = __get_SR_register() & GIE;
    // Make this operation atomic
    __disable_interrupt();
    if (drawmode == DOGS102x6_DRAW_ON_REFRESH)
    {
        while (i) {
            dogs102x6Memory[2 + (currentPage * 102) + currentColumn] =
            (uint8_t)*sData++;
            currentColumn++;
            // Boundary check
            if (currentColumn > 101)
            {
            currentColumn = 101;
            }
            // Decrement the Byte counter
            i--;
        }
    }  else   {
        // CS Low
        P7OUT &= ~CS;
        //CD High
        P5OUT |= CD;

        while (i)
        {
            dogs102x6Memory[2 + (currentPage * 102) + currentColumn] =
            (uint8_t)*sData;
            currentColumn++;
            // Boundary check
            if (currentColumn > 101)
            {
            currentColumn = 101;
            }
            // USCI_B1 TX buffer ready?
            while (!(UCB1IFG & UCTXIFG)) ;
            // Transmit data and increment pointer
            UCB1TXBUF = *sData++;
            // Decrement the Byte counter
            i--;
        }
        // Wait for all TX/RX to finish
        while (UCB1STAT & UCBUSY) ;
        // Dummy read to empty RX buffer and clear any overrun conditions
        UCB1RXBUF;
        // CS High
        P7OUT |= CS;
    }
    // Restore original GIE state
    __bis_SR_register(gie);
}

void Dogs102x6_setAddress(uint8_t pa, uint8_t ca)
{
    uint8_t cmd[1];
    // Page boundary check
    if (pa > 7)
    {
        pa = 7;
    }
    // Column boundary check
    if (ca > 101)
    {
        ca = 101;
    }
    // Page Address Command = Page Address Initial Command + Page Address
    cmd[0] = SET_PAGE_ADDRESS + (7 - pa);
    uint8_t H = 0x00;
    uint8_t L = 0x00;
    uint8_t ColumnAddress[] = { SET_COLUMN_ADDRESS_MSB, SET_COLUMN_ADDRESS_LSB };
    currentPage = pa;
    currentColumn = ca;
    if (drawmode == DOGS102x6_DRAW_ON_REFRESH) return; // exit if drawmode on     refresh
    // Separate Command Address to low and high
    L = (ca & 0x0F);
    H = (ca & 0xF0);
    H = (H >> 4);
    // Column Address CommandLSB = Column Address Initial Command
    // + Column Address bits 0..3
    ColumnAddress[0] = SET_COLUMN_ADDRESS_LSB + L;
    // Column Address CommandMSB = Column Address Initial Command
    // + Column Address bits 4..7
    ColumnAddress[1] = SET_COLUMN_ADDRESS_MSB + H;
    // Set page address
    Dogs102x6_writeCommand(cmd, 1);
    // Set column address
    Dogs102x6_writeCommand(ColumnAddress, 2);
}

void Dogs102x6_clearScreen(void)
{
    uint8_t LcdData[] = {0x00};
    uint8_t p, c;
    // 8 total pages in LCD controller memory
    for (p = 0; p < 8; p++)
    {
        Dogs102x6_setAddress(p, 0);
        // 102 total columns in LCD controller memory
        for (c = 0; c < 102; c++)
        {
            Dogs102x6_writeData(LcdData, 1);
        }
    }
}

void Dogs102x6_charDraw(uint8_t row, uint8_t col, uint16_t f, uint8_t style)
{
    // Each Character consists of 6 Columns on 1 Page
    // Each Page presents 8 pixels vertically (top = MSB)
    uint8_t b;
    uint16_t h;
    uint8_t inverted_char[6];
    // Row boundary check
    if (row > 7)
    {
        row = 7;
    }
    // Column boundary check
    if (col > 101)
    {
        col = 101;
    }
    // handle characters not in our table
    if (f < 32 || f > 129)
    {
        // replace the invalid character with a '.'
        f = '.';
    }
    // subtract 32 because FONT6x8[0] is "space" which is ascii 32,
    // multiply by 6 because each character is columns wide
    h = (f - 32) * 6;
    Dogs102x6_setAddress(row, col);
    if (style == DOGS102x6_DRAW_NORMAL)
    {
        // write character
        Dogs102x6_writeData((uint8_t *)FONT6x8 + h, 6);
    }
    else
    {
        for (b = 0; b < 6; b++)
        {
            // invert the character
            inverted_char[b] = FONT6x8[h + b] ^ 0xFF;
        }
    // write inverted character
        Dogs102x6_writeData(inverted_char, 6);
    }
}


void Dogs102x6_backlightInit(void)
{
    // Turn on Backlight
    CS_BACKLT_DIR |= BACKLT;
    CS_BACKLT_OUT |= BACKLT;
    // Uses PWM to control brightness
    CS_BACKLT_SEL |= BACKLT;
    // start at full brightness (8)
    TB0CCTL4 = OUTMOD_7;
    TB0CCR4 = TB0CCR0 >> 1;
    TB0CCR0 = 50;
    TB0CTL = TBSSEL_1 + MC_1;
}

void Dogs102x6_setInverseDisplay(uint8_t dir)
{
    uint8_t cmd;
    if (dir == 1) {
        cmd = SET_COM_DIRECTION_1;
    } else {
        cmd = SET_COM_DIRECTION;
    }
    Dogs102x6_writeCommand(&cmd, 1);
}

void Dogs102x6_setBacklight(uint8_t brightness)
{
    unsigned int dutyCycle = 0, i, dummy;
    if (brightness > 0)
    {
        TB0CCTL4 = OUTMOD_7;
        dummy = (TB0CCR0 >> 4);
        dutyCycle = 12;
        for (i = 0; i < brightness; i++)
            dutyCycle += dummy;
        TB0CCR4 = dutyCycle;
        //If the backlight was previously turned off, turn it on.
        if (!backlight)
            TB0CTL |= MC0;
    }
    else
    {
        TB0CCTL4 = 0;
        TB0CTL &= ~MC0;
    }
    backlight = brightness;
}

void Cma3000_init(void)
{
    do {
        // Set P3.6 to output direction high
        ACCEL_OUT |= ACCEL_PWR;
        ACCEL_DIR |= ACCEL_PWR;
        // P3.3,4 option select
        ACCEL_SEL |= ACCEL_SIMO + ACCEL_SOMI;
        // P2.7 option select
        ACCEL_SCK_SEL |= ACCEL_SCK;
        ACCEL_INT_DIR &= ~ACCEL_INT;
        // Generate interrupt on Lo to Hi edge
        ACCEL_INT_IES &= ~ACCEL_INT;
        // Clear interrupt flag
        ACCEL_INT_IFG &= ~ACCEL_INT;
        // Unselect acceleration sensor
        ACCEL_OUT |= ACCEL_CS;
        ACCEL_DIR |= ACCEL_CS;
        // **Put state machine in reset**
        UCA0CTL1 |= UCSWRST;
        // 3-pin, 8-bit SPI master Clock polarity high, MSB
        UCA0CTL0 = UCMST + UCSYNC + UCCKPH + UCMSB;
        // Use SMCLK, keep RESET
        UCA0CTL1 = UCSWRST + UCSSEL_2;
        // /0x30
        UCA0BR0 = 0x30;
        // 0
        UCA0BR1 = 0;
        // No modulation
        UCA0MCTL = 0;
        // **Initialize USCI state machine**
        UCA0CTL1 &= ~UCSWRST;
        // Read REVID register
        RevID = Cma3000_readRegister(REVID);
        __delay_cycles(50 * TICKSPERUS);
        // Activate measurement mode: 2g/400Hz
        accelData = Cma3000_writeRegister(CTRL, G_RANGE_2 | I2C_DIS | MODE_400
        /*MODE_400_M*/);
        // Settling time per DS = 10ms
        __delay_cycles(1000 * TICKSPERUS);
        // INT pin interrupt disabled
        ACCEL_INT_IE &= ~ACCEL_INT;
        // Repeat till interrupt Flag is set to show sensor is working
    } while (!(ACCEL_INT_IN & ACCEL_INT));
}

int8_t Cma3000_writeRegister(uint8_t Address, int8_t accelData)
{
    uint8_t Result;
    // Address to be shifted left by 2
    Address <<= 2;
    // RW bit to be set
    Address |= 2;

    // Select acceleration sensor
    ACCEL_OUT &= ~ACCEL_CS;
    // Read RX buffer just to clear interrupt flag
    Result = UCA0RXBUF;
    // Wait until ready to write
    while (!(UCA0IFG & UCTXIFG)) ;
    // Write address to TX buffer
    UCA0TXBUF = Address;
    // Wait until new data was written into RX buffer
    while (!(UCA0IFG & UCRXIFG)) ;
    // Read RX buffer just to clear interrupt flag
    Result = UCA0RXBUF;
    // Wait until ready to write
    while (!(UCA0IFG & UCTXIFG)) ;
    // Write data to TX buffer
    UCA0TXBUF = accelData;
    // Wait until new data was written into RX buffer
    while (!(UCA0IFG & UCRXIFG)) ;
    // Read RX buffer
    Result = UCA0RXBUF;
    // Wait until USCI_A0 state machine is no longer busy
    while (UCA0STAT & UCBUSY) ;
    // Deselect acceleration sensor
    ACCEL_OUT |= ACCEL_CS;
    return Result;
}

uint8_t CMA3000_writeCommand(uint8_t firstByte, uint8_t secondByte) {
    char indata;
    P3OUT &= ~BIT5;
    indata = UCA0RXBUF;
    while(!(UCA0IFG & UCTXIFG));
    UCA0TXBUF = firstByte;
    while(!(UCA0IFG & UCRXIFG));
    indata = UCA0RXBUF;
    while(!(UCA0IFG & UCTXIFG));
    UCA0TXBUF = secondByte;
    while(!(UCA0IFG & UCRXIFG));
    indata = UCA0RXBUF;
    while(UCA0STAT & UCBUSY);
    P3OUT |= BIT5;

    return indata;
}

int8_t Cma3000_readRegister(int8_t Address)
{
    int8_t Result;
    // Address to be shifted left by 2 and RW bit to be reset
    Address <<= 2;
    // Select acceleration sensor
    ACCEL_OUT &= ~ACCEL_CS;
    // Read RX buffer just to clear interrupt flag
    Result = UCA0RXBUF;
    // Wait until ready to write
    while (!(UCA0IFG & UCTXIFG)) ;
    // Write address to TX buffer
    UCA0TXBUF = Address;
    // Wait until new data was written into RX buffer
    while (!(UCA0IFG & UCRXIFG)) ;
    // Read RX buffer just to clear interrupt flag
    Result = UCA0RXBUF;
    // Wait until ready to write
    while (!(UCA0IFG & UCTXIFG)) ;
    // Write dummy data to TX buffer
    UCA0TXBUF = 0;
    // Wait until new data was written into RX buffer
    while (!(UCA0IFG & UCRXIFG)) ;
    // Read RX buffer
    Result = UCA0RXBUF;
    // Wait until USCI_A0 state machine is no longer busy
    while (UCA0STAT & UCBUSY) ;
    // Deselect acceleration sensor
    ACCEL_OUT |= ACCEL_CS;
    // Return new data from RX buffer
    return Result;
}

int parseProjectionByte(uint8_t projectionByte) {
    int i = 0;
    int projectionValue = 0;
    int isNegative = projectionByte & BIT7;
    for (; i < 7; i++) {
        if (isNegative) {
            projectionValue += (BITx[i] & projectionByte) ? 0 : MAPPING_VALUES[i];
        }
        else {
            projectionValue += (BITx[i] & projectionByte) ? MAPPING_VALUES[i] : 0;

        }
    }
    projectionValue *= isNegative ? -1 : 1;
    return projectionValue;
}

void Cma3000_readAccel(void)
{
    // Read DOUTX register
    Cma3000_xAccel = Cma3000_readRegister(DOUTX);
    __delay_cycles(50 * TICKSPERUS);
    // Read DOUTY register
    Cma3000_yAccel = Cma3000_readRegister(DOUTY);
    __delay_cycles(50 * TICKSPERUS);
    // Read DOUTZ register
    Cma3000_zAccel = Cma3000_readRegister(DOUTZ);
    num_y = parseProjectionByte(Cma3000_yAccel);
    ShowNumber(num_y);
}

#pragma vector=WDT_VECTOR
__interrupt void WDTINT()
{
    SFRIE1 &=~ WDTIE;
    Cma3000_readAccel();
    SFRIE1 |= WDTIE;
}

int invFlg = 0;
#pragma vector = PORT2_VECTOR
__interrupt void buttonS1(void)
{
    volatile int i = 0;
    for (i = 0; i < 2000; i++);
    if ((P2IN & BIT2) == 0) {
        if (invFlg == 0) {
            invFlg = 1;
            Dogs102x6_setInverseDisplay(1);
        }
        else {
            invFlg = 0;
            Dogs102x6_setInverseDisplay(0);
        }
        Dogs102x6_clearScreen();
        for (i = 0; i < 2000; i++);
    }
    P2IFG = 0;
}

double atan(double x, int n) {
    double a = x;
    double sum = a;
    double b = a;
    double E = 1. / n;
    int i = 1;
    for (i = 1; a > E; i++) {
        b *= -x * x;
        a *= b / (2 * i + 1);
        sum += a;
    }
    return sum;
}
int fabs_s(int num) {
    if(num > 0){
        return num;
    }
    else{
        return num*(-1);
    }
}
//void ShowNumber(int num)
//{
//    Dogs102x6_clearScreen();
//    volatile int length = 1;
//    volatile int digit = 0;
//    volatile int j = 0;
//    volatile int i = 10;
//
//    while(1)
//    {
//        if(num / i != 0)
//        {
//            i *= 10;
//            length++;
//        }
//        else
//        {
//            break;
//        }
//    }
//
//    int temp = (int)num;
//    temp = fabs_s(num);
//    if(pos_on_screen == 0){
//        for(j = 0; j < length; j++)
//        {
//            digit = (int)(temp % 10);
//            temp = temp / 10;
//            if (digit < 10){
//                Dogs102x6_charDraw(7, length*colSize-j*colSize, 48 + digit,0);
//            }
//        }
//        if (num < 0)
//        {
//            Dogs102x6_charDraw(7, 0, 45 ,0);
//        }
//        else
//        {
//            Dogs102x6_charDraw(7, 0, 43 ,0);
//        }
//    }
//    else{
//        if (num < 0)
//        {
//            Dogs102x6_charDraw(7, 102-colSize, 48 + 9 + 10 + 2 ,0);
//        }
//        else
//        {
//            Dogs102x6_charDraw(7, 102-colSize, 48 + 9 + 10 + 1 ,0);
//        }
//        for(j = 1; j < length+1; j++)
//        {
//            digit = (int)(temp % 10);
//
//            temp = temp / 10;
//            if (digit < 10){
//                Dogs102x6_charDraw(7, 102 - (length*colSize-j*colSize)-
//                colSize-colSize, 48 + 10 + digit ,0);
//            }
//        }
//    }
//}
void ShowNumber(int num)
{
    Dogs102x6_clearScreen();
    volatile int length = 1;
    volatile int digit = 0;
    volatile int j = 0;
    volatile int i = 10;

    while(1)
    {
        if(num / i != 0)
        {
            i *= 10;
            length++;
        }
        else
        {
            break;
        }
    }

    int temp = (int)num;
    temp = fabs_s(num);
    if(pos_on_screen == 0){
        for(j = 0; j < length; j++)
        {
            digit = (int)(temp % 10);
            temp = temp / 10;
            if (digit < 10){
                Dogs102x6_charDraw(7, 102 - (length*colSize-j*colSize), 48 + digit, 0);
            }
        }
        if (num < 0)
        {
            Dogs102x6_charDraw(7, 102-(length*colSize) - colSize, 45, 0);
        }
    }
    else{
        if (num < 0)
        {
            Dogs102x6_charDraw(7, 102-(length*colSize) - colSize, 45,0);
        }
        for(j = 1; j < length+1; j++)
        {
            digit = (int)(temp % 10);

            temp = temp / 10;
            if (digit < 10){
                Dogs102x6_charDraw(7, 102 - (length*colSize-j*colSize), 48 + digit,0);
            }
        }
    }
}



void SetupButtons()
{
    P4SEL |= BIT2;
    P4DIR |= BIT2;
    //����� � �����
    P4SEL |= BIT1;
    P4DIR |= BIT1;
    //����� � �����
    P4SEL |= BIT0;
    P4DIR |= BIT0;
    P4SEL |= BIT3;
    P4DIR |= BIT3;
    P3DIR |= BIT7;
    //����� � �����
    P3SEL |= BIT7;
    // set buttons for reading
    //P1DIR &= ~BIT7;
    P2DIR &= ~BIT2;
    // Set up pull up resistors
    //P1REN |= BIT7;
    P2REN |= BIT2;
    // initialize buttons with inactive level
    //P1OUT |= BIT7;
    P2OUT |= BIT2;
    // set up interrupts for S1
    //P1IE |= BIT7;
    //P1IES |= BIT7;
    //P1IFG = 0;
    P2IE |= BIT2;
    P2IES |= BIT2;
    P2IFG = 0;
}
    /*
    * main.c
    */
int main(void) {
    WDTCTL = WDT_ADLY_250; // Stop watchdog timer
    SFRIE1 |= WDTIE;
    SetupButtons();
    __bis_SR_register(GIE);
    Cma3000_init();
    Dogs102x6_init();
    Dogs102x6_backlightInit();
    Dogs102x6_setBacklight(5);
    Dogs102x6_clearScreen();
    __no_operation();
    return 0;
}
