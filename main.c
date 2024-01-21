#include "ch32v003fun.h"
#include "i2c_slave.h"
#include <stdio.h>
#include <string.h>
#include <stdbool.h>

// Board revision
#define HW_REV 2

// VFD registers
#define VFD_DCRAM_WR    0x10  // ccccaaaa dddddddd dddddddd ..
#define VFD_CGRAM_WR    0x20  // "
#define VFD_ADRAM_WR    0x30  // ccccaaaa ******dd ******dd ..
#define VFD_DUTY        0x50  // ccccdddd
#define VFD_NUMDIGIT    0x60  // "
#define VFD_LIGHTS      0x70  // cccc**dd

// VFD register values
#define VFD_LIGHTS_NORMAL 0x00 // Normal operation
#define VFD_LIGHTS_OFF    0x01 // Turn off all segments
#define VFD_LIGHTS_ON     0x02 // Turn on all segments

#define VFD_DIGITS 12

#define DATA_OFFSET 10
#define DATA_LENGTH (sizeof(i2c_registers) - DATA_OFFSET)

uint8_t i2c_registers[255] = {0};

uint8_t curr_i2c_registers[sizeof(i2c_registers)] = {0};
uint8_t prev_i2c_registers[sizeof(i2c_registers)] = {0};

uint16_t ac_counter = 0;
bool ac_value = false;

void ac_step() {
    ac_counter++;
    if (ac_counter > 10) {
        ac_counter = 0;
        if (ac_value) {
            TIM1->CCER |= TIM_CC2E | TIM_CC2P;
            TIM1->CCER &= ~(TIM_CC2NE | TIM_CC2NP);
            ac_value = false;
        } else {
            TIM1->CCER |= TIM_CC2NE | TIM_CC2NP;
            TIM1->CCER &= ~(TIM_CC2E | TIM_CC2P);
            ac_value = true;
        }
    }
}

void enable_timer() {
    uint8_t deadtime = 255;

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
    TIM1->PSC = 0x0300;

    // Configure target and maximum value
    TIM1->ATRLR = 255; // Period
    TIM1->CH2CVR = 255 - 50; // Duty cycle

    // Reload immediately
    TIM1->SWEVGR |= TIM_UG;

    // Enable both outputs of channel 2
    TIM1->CCER |= TIM_CC2E | TIM_CC2P; //| TIM_CC2NE | TIM_CC2NP;

    // CH1 Mode is output, PWM1 (CC1S = 00, OC1M = 110)
    TIM1->CHCTLR1 |= TIM_OC2M_2 | TIM_OC2M_1;

    // Enable TIM1 outputs
    TIM1->BDTR |= TIM_MOE | (deadtime & 0xFF);

    // Enable TIM1
    TIM1->CTLR1 |= TIM_CEN;
}


void TIM1_UP_IRQHandler(void) __attribute__((interrupt));
void TIM1_UP_IRQHandler() {

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

void display_enable(bool test) {
    GPIOC->BSHR  |= 1 << (4 + 16); // Disable controller (pull RESET low)
    GPIOC->BSHR  |= 1 << 3; // Enable 5V
    GPIOD->BSHR  |= 1 << 0; // Enable 24V
    enable_timer();
    Delay_Ms(10);
    GPIOC->BSHR  |= 1 << 4; // Enable controller (pull RESET high)
    Delay_Ms(10);
    spi_send_command(VFD_NUMDIGIT | VFD_DIGITS);
    spi_send_command(VFD_DUTY | 4);
    spi_send_command(VFD_LIGHTS | (test ? VFD_LIGHTS_ON : VFD_LIGHTS_NORMAL));
    Delay_Ms(1);
}

void display_disable() {
    GPIOC->BSHR  |= 1 << (4 + 16); // Disable controller (pull RESET low)
    GPIOD->BSHR  |= 1 << (0 + 16); // Disable 24V
    GPIOC->BSHR  |= 1 << (3 + 16); // Disable 5V
    disable_timer();
}

uint8_t convert_char(char c) {
    if (c>='@' && c<='_') {
        // 64.. -> 16..
        c -= 48;
    } else if(c>=' ' && c<='?') {
        // 32.. -> 48..
        c += 16;
    } else if(c>='a' && c<='z') {
        // 97.. -> 17..
        c -= 80;
    } else { // Other characters (?)
        c = 79;
    }
    return c;
}

void display_update(char* data) {
    uint8_t buffer[13];
    buffer[0] = VFD_DCRAM_WR;
    for (uint8_t i = 0; i < VFD_DIGITS; i++) {
        buffer[i + 1] = convert_char(data[11-i]);
    }
    spi_send(buffer, sizeof(buffer));
    char str_buffer[VFD_DIGITS + 1] = {0};
    memcpy(str_buffer, data, VFD_DIGITS);
}

uint8_t read_i2c_address() {
#if HW_REV > 1
    return 0x10 + ((~(GPIOD->INDR >> 2)) & 0x1F);
#else
    return 0x10;
#endif
}

bool i2c_changed = false;
void i2c_callback() {
    i2c_changed = true;
}

int main() __attribute__((optimize("O0")));
int main() {
    SystemInit();

    // Enable GPIO ports A, C and D
    RCC->APB2PCENR |= RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD;

    // Configure GPIO C3 as output (5V_/*ENABLE*/)
    GPIOC->CFGLR &= ~(0xf<<(4*3));
    GPIOC->CFGLR |= (GPIO_Speed_10MHz | GPIO_CNF_OUT_PP)<<(4*3);

    // Configure GPIO C4 as output (RST)
    GPIOC->CFGLR &= ~(0xf<<(4*4));
    GPIOC->CFGLR |= (GPIO_Speed_10MHz | GPIO_CNF_OUT_PP)<<(4*4);

    // Configure GPIO D0 as output (24V_ENABLE)
    GPIOD->CFGLR &= ~(0xf<<(4*0));
    GPIOD->CFGLR |= (GPIO_Speed_10MHz | GPIO_CNF_OUT_PP)<<(4*0);

    // Configure GPIO D7 as output (LED)
    GPIOD->CFGLR &= ~(0xf<<(4*7));
    GPIOD->CFGLR |= (GPIO_Speed_10MHz | GPIO_CNF_OUT_PP)<<(4*7);

    // Configure GPIO D2, D3, D4, D5 and D6 as inputs with pullup (I2C_ADDR)
    for (uint8_t i = 2; i <= 6; i++) {
        GPIOD->CFGLR &= ~(0xf<<(4*i));
        GPIOD->CFGLR |= (GPIO_Speed_In | GPIO_CNF_IN_PUPD)<<(4*i);
        GPIOD->OUTDR |= 1 << i; // Pull-up
    }

    SetupI2CSlave(read_i2c_address(), 0, i2c_registers, sizeof(i2c_registers), i2c_callback);

    spi_initialize();
    display_disable();

    i2c_registers[25] = 1; // Turn on LED at startup

    uint8_t scroll_mode = 0;
    uint32_t scroll_interval = 1000 * DELAY_MS_TIME;
    uint32_t previousCounter = 0;
    uint8_t scroll_position = 0;
    uint8_t scroll_length = 0;
    bool force_update = false;

    strcpy(&i2c_registers[DATA_OFFSET], "          v2");
    i2c_registers[0] = 1; // Enable screen
    i2c_registers[6] = 110; // Default brightness
    i2c_changed = true;


    while (1) {
        if (i2c_changed) {
            memcpy(curr_i2c_registers, i2c_registers, sizeof(i2c_registers));
            i2c_changed = false;
        }

        ac_step();

        // Register 0, bit 0: display power control
        // Register 0, bit 1: display test mode
        uint8_t display = (curr_i2c_registers[0] >> 0) & 3;
        uint8_t prev_display = (prev_i2c_registers[0] >> 0) & 3;
        if (display != prev_display) {
            if (display & 1) {
                display_enable((display >> 1) & 1);
            } else {
                display_disable();
            }
            force_update = true;
        }

        // Register 0, bit 2: LED control
        bool led = (curr_i2c_registers[0] >> 2) & 1;
        GPIOD->BSHR  |= 1 << (7 + (led ? 0 : 16));

        // Register 1: displayed offset / start
        uint8_t offset = curr_i2c_registers[1];
        if (offset > DATA_LENGTH - VFD_DIGITS) offset = DATA_LENGTH - VFD_DIGITS;
        uint8_t prev_offset = prev_i2c_registers[1];
        if (prev_offset > DATA_LENGTH - VFD_DIGITS) prev_offset = DATA_LENGTH - VFD_DIGITS;

        // Register 2: scroll length
        scroll_length = curr_i2c_registers[2];
        if (scroll_length > DATA_LENGTH - VFD_DIGITS) scroll_length = DATA_LENGTH - VFD_DIGITS;

        // Register 3: mode (bits 0-3: scroll mode)
        scroll_mode = curr_i2c_registers[3];

        // Registers 4-5: scroll speed
        uint16_t* scroll_interval_ptr = (uint16_t*) &curr_i2c_registers[4];
        scroll_interval = (*scroll_interval_ptr) * DELAY_MS_TIME;

        // Register 6: filament current
        TIM1->CH2CVR = 0xFF - curr_i2c_registers[6];

        // Registers 7-9: reserved

        // Registers 10 - 255: data
        bool data_changed = (memcmp(&prev_i2c_registers[DATA_OFFSET], &curr_i2c_registers[DATA_OFFSET], DATA_LENGTH) != 0);

        // Update screen if offset or data changes
        if (offset != prev_offset || data_changed || force_update) {
            display_update((char*) &curr_i2c_registers[DATA_OFFSET + offset + scroll_position]);
            force_update = false;
        }

        memcpy(prev_i2c_registers, curr_i2c_registers, sizeof(i2c_registers));

        uint32_t currentCounter = SysTick->CNT;
        if (currentCounter - previousCounter >= scroll_interval && scroll_interval > 0 && scroll_length > 0) {
            previousCounter = currentCounter;
            if ((scroll_mode & 0x0F) == 1) {
                scroll_position++;
                if (scroll_position > DATA_LENGTH - VFD_DIGITS) {
                    scroll_position = DATA_LENGTH - VFD_DIGITS;
                }
                if (scroll_position > scroll_length) {
                    scroll_position = scroll_length;
                    if ((scroll_mode >> 4) & 1) {
                        scroll_position = 0;
                    }
                }
            } else if ((scroll_mode & 0x0F) == 2) {
                if (scroll_position > 0) {
                    scroll_position--;
                } else if ((scroll_mode >> 4) & 1) {
                    scroll_position = scroll_length - 1;
                }
            }
            force_update = true;
        } else if (scroll_interval == 0 || scroll_mode == 0) {
            scroll_position = 0;
        }
    }
}
