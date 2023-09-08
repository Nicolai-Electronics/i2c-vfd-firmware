#define SYSTEM_CORE_CLOCK 48000000

#include "ch32v003fun.h"
#include "i2c_slave.h"
#include <stdio.h>
#include <string.h>

#define VFD_DCRAM_WR    0x10		// ccccaaaa dddddddd dddddddd ..
#define VFD_CGRAM_WR    0x20		// "
#define VFD_ADRAM_WR    0x30		// ccccaaaa ******dd ******dd ..
#define VFD_DUTY        0x50		// ccccdddd
#define VFD_NUMDIGIT    0x60		// "
#define VFD_LIGHTS      0x70		// cccc**dd

#define LINORM          0x00		// normal display
#define LIOFF           0x01		// lights OFF
#define LION            0x02		//        ON

#define NUMDIGITS       12			// display digit width
#define BUFSIZE         100			// display/scroll buffer

volatile uint8_t i2c_registers[32] = {0};

void enable_timer() {
    // Enable GPIO port A and timer 1
    RCC->APB2PCENR |= RCC_APB2Periph_GPIOA | RCC_APB2Periph_TIM1;

    // Configure GPIO A1 as alternate function output (CLK_A)
    GPIOA->CFGLR &= ~(0xf<<(4*1));
    GPIOA->CFGLR |= (GPIO_Speed_10MHz | GPIO_CNF_OUT_PP_AF)<<(4*1);

    // Configure GPIO A2 as alternate function output (CLK_B)
    GPIOA->CFGLR &= ~(0xf<<(4*2));
    GPIOA->CFGLR |= (GPIO_Speed_10MHz | GPIO_CNF_OUT_PP_AF)<<(4*2);

    // Reset the peripheral
    RCC->APB2PRSTR |= RCC_APB2Periph_TIM1;
    RCC->APB2PRSTR &= ~RCC_APB2Periph_TIM1;

    // Prescaler
    TIM1->PSC = 0x000A;

    // Configure target and maximum value
    TIM1->ATRLR = 255; // Period
    TIM1->CH2CVR = 128; // Duty cycle

    // Reload immediately
    TIM1->SWEVGR |= TIM_UG;

    // Enable both outputs of channel 2
    TIM1->CCER |= TIM_CC2E | TIM_CC2NE | TIM_CC2P | TIM_CC2NP;

    // CH1 Mode is output, PWM1 (CC1S = 00, OC1M = 110)
    TIM1->CHCTLR1 |= TIM_OC2M_2 | TIM_OC2M_1;

    // Enable TIM1 outputs
    TIM1->BDTR |= TIM_MOE;

    // Enable TIM1
    TIM1->CTLR1 |= TIM_CEN;
}

void disable_timer() {
    // Enable GPIO port A and timer 1
    RCC->APB2PCENR |= RCC_APB2Periph_GPIOA | RCC_APB2Periph_TIM1;

    // Configure GPIO A1 as output (CLK_A)
    GPIOA->CFGLR &= ~(0xf<<(4*1));
    GPIOA->CFGLR |= (GPIO_Speed_10MHz | GPIO_CNF_OUT_PP)<<(4*1);
    GPIOA->BSHR  |= 1 << (1); // Set pin high, pulling filament to GND

    // Configure GPIO A2 as output (CLK_B)
    GPIOA->CFGLR &= ~(0xf<<(4*2));
    GPIOA->CFGLR |= (GPIO_Speed_10MHz | GPIO_CNF_OUT_PP)<<(4*2);
    GPIOA->BSHR  |= 1 << (2); // Set pin high, pulling filament to GND

    // Reset the peripheral
    RCC->APB2PRSTR |= RCC_APB2Periph_TIM1;
    RCC->APB2PRSTR &= ~RCC_APB2Periph_TIM1;
}

void spi_initialize() {
    // Enable GPIO port C and SPI
    RCC->APB2PCENR |= RCC_APB2Periph_GPIOC | RCC_APB2Periph_SPI1;

    // Configure GPIO C0 as output (CS)
    GPIOC->CFGLR &= ~(0xf<<(4*0));
    GPIOC->CFGLR |= (GPIO_Speed_10MHz | GPIO_CNF_OUT_PP)<<(4*0);
    GPIOC->BSHR  |= 1 << (0); // Pull CS high

    // Configure GPIO C5 as alternate function output (CLK)
    GPIOC->CFGLR &= ~(0xf<<(4*5));
    GPIOC->CFGLR |= (GPIO_Speed_10MHz | GPIO_CNF_OUT_PP_AF)<<(4*5);

    // Configure GPIO C6 as alternate function output (MOSI)
    GPIOC->CFGLR &= ~(0xf<<(4*6));
    GPIOC->CFGLR |= (GPIO_Speed_10MHz | GPIO_CNF_OUT_PP_AF)<<(4*6);

    // Configure SPI
    SPI1->CTLR1 =
        SPI_NSS_Soft |
        SPI_CPHA_2Edge |
        SPI_CPOL_High |
        SPI_DataSize_8b |
        SPI_Mode_Master |
        SPI_Direction_1Line_Tx |
        SPI_BaudRatePrescaler_128;

    // Enable SPI
    SPI1->CTLR1 |= CTLR1_SPE_Set;
}

uint8_t reverse(uint8_t b) {
   b = (b & 0xF0) >> 4 | (b & 0x0F) << 4;
   b = (b & 0xCC) >> 2 | (b & 0x33) << 2;
   b = (b & 0xAA) >> 1 | (b & 0x55) << 1;
   return b;
}

void spi_send(const uint8_t* data, uint16_t length) {
    //printf("SPI sending %u bytes %02x\n", length, data[0]);
    GPIOC->BSHR |= 1 << (0 + 16); // Pull CS low
    Delay_Us(8);
    for (uint16_t position = 0; position < length; position++) {
        while(!(SPI1->STATR & SPI_STATR_TXE));
        SPI1->DATAR = reverse(data[position]);
        Delay_Us(32);
    }
    while(SPI1->STATR & SPI_STATR_BSY);
    Delay_Us(8);
    GPIOC->BSHR |= 1 << (0); // Pull CS high
    Delay_Us(8);
}

void spi_send_command(const uint8_t command) {
    spi_send(&command, 1);
}

void display_enable() {
    GPIOC->BSHR  |= 1 << (4 + 16); // Disable controller (pull RESET low)
    GPIOC->BSHR  |= 1 << 3; // Enable 5V
    GPIOD->BSHR  |= 1 << 0; // Enable 24V
    enable_timer();
    //test_dc();
    Delay_Ms(10);
    GPIOC->BSHR  |= 1 << 4; // Enable controller (pull RESET high)
    Delay_Ms(10);
    spi_send_command(VFD_NUMDIGIT | 12);
    spi_send_command(VFD_DUTY | 4);
    spi_send_command(VFD_LIGHTS | LINORM);
    Delay_Ms(1);
    //printf("Display enabled\n");
}

void display_disable() {
    GPIOC->BSHR  |= 1 << (4 + 16); // Disable controller (pull RESET low)
    GPIOD->BSHR  |= 1 << (0 + 16); // Disable 24V
    GPIOC->BSHR  |= 1 << (3 + 16); // Disable 5V
    disable_timer();

    //printf("Display disabled\n");
}

uint8_t convert_char(char c) {
    if(c>='@' && c<='_')				// 64.. -> 16..
        c -= 48;
    else if(c>=' ' && c<='?')			// 32.. -> 48..
        c += 16;
    else if(c>='a' && c<='z')			// 97.. -> 17..
        c -= 80;
    else								// unvalid -> ?
        c = 79;
    return c;
}

void display_update(volatile uint8_t* data) {
    uint8_t buffer[13];
    buffer[0] = VFD_DCRAM_WR;
    for (uint8_t i = 0; i < 12; i++) {
        buffer[i + 1] = convert_char(data[11-i]);
    }
    spi_send(buffer, sizeof(buffer));
}

int main() __attribute__((optimize("O0")));
int main() {
    SystemInit();
    SetupI2CSlave(0xA, i2c_registers, sizeof(i2c_registers));

    // Enable GPIO ports C and D
    RCC->APB2PCENR |= RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD;

    // Configure GPIO C3 as output (5V_ENABLE)
    GPIOC->CFGLR &= ~(0xf<<(4*3));
    GPIOC->CFGLR |= (GPIO_Speed_10MHz | GPIO_CNF_OUT_PP)<<(4*3);

    // Configure GPIO C4 as output (RST)
    GPIOC->CFGLR &= ~(0xf<<(4*4));
    GPIOC->CFGLR |= (GPIO_Speed_10MHz | GPIO_CNF_OUT_PP)<<(4*4);

    // Configure GPIO D0 as output (24V_ENABLE)
    GPIOD->CFGLR &= ~(0xf<<(4*0));
    GPIOD->CFGLR |= (GPIO_Speed_10MHz | GPIO_CNF_OUT_PP)<<(4*0);

    spi_initialize();
    display_disable();

    uint8_t curr_display_state = 0;

    while (1) {
        if (i2c_registers[0] != curr_display_state) {
            if (i2c_registers[0] == 0) {
                display_disable();
            } else if (curr_display_state == 0) {
                    display_enable();
            }
            if (i2c_registers[0] == 1) {
                display_update(&i2c_registers[1]);
            } else if (i2c_registers[0] == 2) {
                display_update(&i2c_registers[13]);
            }
            curr_display_state = i2c_registers[0];
        }
    }
}
