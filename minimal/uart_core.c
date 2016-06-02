#include <unistd.h>
#include "py/mpconfig.h"

/*
 * Core UART functions to implement for a port
 */

//#if MICROPY_MIN_USE_STM32_MCU
//typedef struct {
//    volatile uint32_t SR;
//    volatile uint32_t DR;
//} periph_uart_t;
//#define USART1 ((periph_uart_t*)0x40011000)
//#endif


typedef struct {
    volatile uint32_t DR;
    volatile uint32_t RSR_ECR;
    volatile uint32_t _1[4];
    volatile uint32_t FR;
    volatile uint32_t _2;
    volatile uint32_t IRLPR;
    volatile uint32_t IBRD;
    volatile uint32_t FBRD;
    volatile uint32_t LCRH;
    volatile uint32_t CTL;
    volatile uint32_t IFLS;
    volatile uint32_t IM;
    volatile uint32_t RIS;
    volatile uint32_t MIS;
    volatile uint32_t ICR;
    volatile uint32_t DMACTL;
    volatile uint32_t _3[22];
    volatile uint32_t ADDR9BIT;
    volatile uint32_t AMASK9BIT;
} periph_uart_t;
#define UART0  ((periph_uart_t*) 0x4000C000)



// Receive single character
int mp_hal_stdin_rx_chr(void) {
    unsigned char c = 0;
#if MICROPY_MIN_USE_STDOUT
    int r = read(0, &c, 1);
    (void)r;
#elif MICROPY_MIN_USE_STM32_MCU
    // wait for RXNE
    while (UART0->FR & (1 << 4)) {}
    c = UART0->DR;
#endif
    return c;
}

// Send string of given length
void mp_hal_stdout_tx_strn(const char *str, mp_uint_t len) {
#if MICROPY_MIN_USE_STDOUT
    int r = write(1, str, len);
    (void)r;
#elif MICROPY_MIN_USE_STM32_MCU
    while (len--) {
        // wait for TXE
        while (UART0->FR & (1 << 5)) {}
        UART0->DR = *str++;
    }
#endif
}
