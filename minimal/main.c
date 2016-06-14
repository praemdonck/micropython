#include <stdint.h>
#include <stdio.h>
#include <string.h>


#include "py/mpconfig.h"
#include "py/nlr.h"
#include "py/compile.h"
#include "py/runtime.h"
#include "py/repl.h"
#include "py/gc.h"
#include "lib/utils/pyexec.h"

#include "inc/hw_memmap.h"
#include "pybpin.h"

#include "gccollect.h"
#include "gchelper.h"

void clock_init(void);


void do_str(const char *src, mp_parse_input_kind_t input_kind) {
    mp_lexer_t *lex = mp_lexer_new_from_str_len(MP_QSTR__lt_stdin_gt_, src, strlen(src), 0);
    if (lex == NULL) {
        printf("MemoryError: lexer could not allocate memory\n");
        return;
    }

    nlr_buf_t nlr;
    if (nlr_push(&nlr) == 0) {
        qstr source_name = lex->source_name;
        mp_parse_tree_t parse_tree = mp_parse(lex, input_kind);
        mp_obj_t module_fun = mp_compile(&parse_tree, source_name, MP_EMIT_OPT_NONE, true);
        mp_call_function_0(module_fun);
        nlr_pop();
    } else {
        // uncaught exception
        mp_obj_print_exception(&mp_plat_print, (mp_obj_t)nlr.ret_val);
    }
}

//static char *stack_top;
//static char heap[2048];
static char heap[24 * 1024];

int main(int argc, char **argv) {
    //int stack_dummy;
    //stack_top = (char*)&stack_dummy;

    uint32_t sp = gc_helper_get_sp();
    gc_collect_init (sp);

    #if MICROPY_ENABLE_GC
    gc_init(heap, heap + sizeof(heap));
    #endif
    mp_init();



    pin_init0();
    #if MICROPY_REPL_EVENT_DRIVEN
    pyexec_event_repl_init();
    for (;;) {
        int c = mp_hal_stdin_rx_chr();
        if (pyexec_event_repl_process_char(c)) {
            break;
        }
    }
    #else
    pyexec_friendly_repl();
    #endif
    //do_str("print('hello world!', list(x+1 for x in range(10)), end='eol\\n')", MP_PARSE_SINGLE_INPUT);
    //do_str("for i in range(10):\r\n  print(i)", MP_PARSE_FILE_INPUT);
    mp_deinit();
    return 0;
}

//void gc_collect(void) {
//    // WARNING: This gc_collect implementation doesn't try to get root
//    // pointers from CPU registers, and thus may function incorrectly.
//    void *dummy;
//    gc_collect_start();
//    gc_collect_root(&dummy, ((mp_uint_t)stack_top - (mp_uint_t)&dummy) / sizeof(mp_uint_t));
//    gc_collect_end();
//    gc_dump_info();
//}

mp_lexer_t *mp_lexer_new_from_file(const char *filename) {
    return NULL;
}

mp_import_stat_t mp_import_stat(const char *path) {
    return MP_IMPORT_STAT_NO_EXIST;
}

mp_obj_t mp_builtin_open(uint n_args, const mp_obj_t *args, mp_map_t *kwargs) {
    return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_KW(mp_builtin_open_obj, 1, mp_builtin_open);

void nlr_jump_fail(void *val) {
}

void NORETURN __fatal_error(const char *msg) {
    while (1);
}

#ifndef NDEBUG
void MP_WEAK __assert_func(const char *file, int line, const char *func, const char *expr) {
    printf("Assertion '%s' failed, at file %s:%d\n", expr, file, line);
    __fatal_error("Assertion failed");
}
#endif

#if MICROPY_MIN_USE_CORTEX_CPU

// this is a minimal IRQ and reset framework for any Cortex-M CPU


extern uint32_t _estack, _sidata, _sdata, _edata, _sbss, _ebss;

void Reset_Handler(void) __attribute__((naked));
void Reset_Handler(void) {
    // set stack pointer
    asm volatile ("ldr sp, =_estack");
    // copy .data section from flash to RAM
    for (uint32_t *src = &_sidata, *dest = &_sdata; dest < &_edata;) {
        *dest++ = *src++;
    }
    // zero out .bss section
    for (uint32_t *dest = &_sbss; dest < &_ebss;) {
        *dest++ = 0;
    }
    // jump to board initialisation
    void _start(void);
    _start();
}

void Default_Handler(void) {
    for (;;) {
    }
}

extern void GPIOPortA_Handler(void);
extern void GPIOPortB_Handler(void);
extern void GPIOPortC_Handler(void);
extern void GPIOPortD_Handler(void);
extern void GPIOPortE_Handler(void);
extern void GPIOPortF_Handler(void);

uint32_t isr_vector[] __attribute__((section(".isr_vector"))) = {
    (uint32_t)&_estack,
    (uint32_t)&Reset_Handler,
    (uint32_t)&Default_Handler,   // NMI_Handler
    (uint32_t)&Default_Handler,   // HardFault_Handler
    (uint32_t)&Default_Handler,   // MemManage_Handler
    (uint32_t)&Default_Handler,   // BusFault_Handler
    (uint32_t)&Default_Handler,   // UsageFault_Handler
    0,
    0,
    0,
    0,
    (uint32_t)&Default_Handler,   // SVC_Handler
    (uint32_t)&Default_Handler,   // DebugMon_Handler
    0,
    (uint32_t)&Default_Handler,   // PendSV_Handler
    (uint32_t)&Default_Handler,   // SysTick_Handler
    (uint32_t)&GPIOPortA_Handler, // GPIO Port A
    (uint32_t)&GPIOPortB_Handler, // GPIO Port B
    (uint32_t)&GPIOPortC_Handler, // GPIO Port C
    (uint32_t)&GPIOPortD_Handler, // GPIO Port D
    (uint32_t)&GPIOPortE_Handler, // GPIO Port E
    0,                            // UART0 Rx and Tx
    0,                            // UART1 Rx and Tx
    0,                            // SSI0 Rx and Tx
    0,                            // I2C0 Master and Slave
    0,                            // PWM 0 Fault
    0,                            // PWM 0 Generator 0
    0,                            // PWM 0 Generator 1
    0,                            // PWM 0 Generator 2
    0,                            // Quadrature Encoder 0
    0,                            // ADC0 Sequence 0
    0,                            // ADC0 Sequence 1
    0,                            // ADC0 Sequence 2
    0,                            // ADC0 Sequence 3
    0,                            // Watchdog
    0,                            // Timer 0 subtimer A
    0,                            // Timer 0 subtimer B
    0,                            // Timer 1 subtimer A
    0,                            // Timer 1 subtimer B
    0,                            // Timer 2 subtimer A
    0,                            // Timer 2 subtimer B
    0,                            // Analog Comp 0
    0,                            // Analog Comp 1
    0,                            // Analog Comp 2
    0,                            // System Control
    0,                            // Flash Control
    (uint32_t)&GPIOPortF_Handler, // GPIO Port F
};

void _start(void) {
    // when we get here: stack is initialised, bss is clear, data is copied

soft_reset:
    // SCB->CCR: enable 8-byte stack alignment for IRQ handlers, in accord with EABI
    *((volatile uint32_t*)0xe000ed14) |= 1 << 9;

    // initialise the cpu and peripherals
    #if MICROPY_MIN_USE_STM32_MCU
    void stm32_init(void);
    stm32_init();
    #endif

    // now that we have a basic system up and running we can call main
    main(0, NULL);

    goto soft_reset;

    // we must not return
    for (;;) {
    }
}

#endif

#if MICROPY_MIN_USE_STM32_MCU

// this is minimal set-up code for an STM32 MCU

typedef struct {
    volatile uint32_t CR;
    volatile uint32_t PLLCFGR;
    volatile uint32_t CFGR;
    volatile uint32_t CIR;
    uint32_t _1[8];
    volatile uint32_t AHB1ENR;
    volatile uint32_t AHB2ENR;
    volatile uint32_t AHB3ENR;
    uint32_t _2;
    volatile uint32_t APB1ENR;
    volatile uint32_t APB2ENR;
} periph_rcc_t;

//typedef struct {
//    volatile uint32_t MODER;
//    volatile uint32_t OTYPER;
//    volatile uint32_t OSPEEDR;
//    volatile uint32_t PUPDR;
//    volatile uint32_t IDR;
//    volatile uint32_t ODR;
//    volatile uint16_t BSRRL;
//    volatile uint16_t BSRRH;
//    volatile uint32_t LCKR;
//    volatile uint32_t AFR[2];
//} periph_gpio_t;


typedef struct {
    volatile uint32_t _1[255];        // 0x40004000 - 0x400043FB 
    volatile uint32_t DATA;           // 0x400043FC - 0x400043FF 
    volatile uint32_t DIR;            // 0x40004400 - 0x40004403 
    volatile uint32_t IS;             // 0x40004404 - 0x40004407 
    volatile uint32_t IBE;            // 0x40004408 - 0x4000440B 
    volatile uint32_t IEV;            // 0x4000440C - 0x4000440F
    volatile uint32_t IM;             // 0x40004410 - 0x40004413
    volatile uint32_t RIS;            // 0x40004414 - 0x40004417
    volatile uint32_t MIS;            // 0x40004418 - 0x4000441B
    volatile uint32_t ICLR;           // 0x4000441C - 0x4000441F
    volatile uint32_t AFSEL;          // 0x40004420 - 0x40004423

    volatile uint32_t _2[55];         // 0x40004424 - 0x400044FF 

    volatile uint32_t DR2R;           // 0x40004500 - 0x40004503
    volatile uint32_t DR4R;           // 0x40004504 - 0x40004507
    volatile uint32_t DR8R;           // 0x40004508 - 0x4000450B
    volatile uint32_t ODR;            // 0x4000450C - 0x4000450F
    volatile uint32_t PUR;            // 0x40004510 - 0x40004513
    volatile uint32_t PDR;            // 0x40004514 - 0x40004517
    volatile uint32_t SLR;            // 0x40004518 - 0x4000451B
    volatile uint32_t DEN;            // 0x4000451C - 0x4000451F
    volatile uint32_t LOCK;           // 0x40004520 - 0x40004523
    volatile uint32_t CR;             // 0x40004524 - 0x40004527
    volatile uint32_t AMSEL;          // 0x40004528 - 0x4000452B
    volatile uint32_t PCTL;           // 0x4000452C - 0x4000452F
    volatile uint32_t ADCCTL;         // 0x40004530 - 0x40004533
    volatile uint32_t DMACTL;         // 0x40004534 - 0x40004537
//    volatile uint32_t AMSEL;          // 0x40004538 - 0x4000453B
//    volatile uint32_t PCTL;           // 0x4000453C - 0x4000453F
} periph_gpio_t;

//typedef struct {
//    volatile uint32_t SR;
//    volatile uint32_t DR;
//    volatile uint32_t BRR;
//    volatile uint32_t CR1;
//} periph_uart_t;


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

#define GPIOA  ((periph_gpio_t*) 0x40004000)
#define GPIOB  ((periph_gpio_t*) 0x40005000)
#define GPIOC  ((periph_gpio_t*) 0x40006000)
#define GPIOD  ((periph_gpio_t*) 0x40007000)
#define GPIOE  ((periph_gpio_t*) 0x40024000)
#define GPIOF  ((periph_gpio_t*) 0x40025000)


#define RCGC0  ((unsigned int*)  0x400FE100)
#define RCGC1  ((unsigned int*)  0x400FE104)
#define RCGC2  ((unsigned int*)  0x400FE108)


#define RCC    ((periph_rcc_t*)  0x40023800)

// simple GPIO interface
#define GPIO_MODE_IN (0)
#define GPIO_MODE_OUT (1)
#define GPIO_MODE_ALT (2)
#define GPIO_PULL_NONE (0)
#define GPIO_PULL_UP (0)
#define GPIO_PULL_DOWN (1)
void gpio_init(periph_gpio_t *gpio, int pin, int mode, int pull, int alt) {
//    gpio->MODER = (gpio->MODER & ~(3 << (2 * pin))) | (mode << (2 * pin));
//    // OTYPER is left as default push-pull
//    // OSPEEDR is left as default low speed
//    gpio->PUPDR = (gpio->PUPDR & ~(3 << (2 * pin))) | (pull << (2 * pin));
//    gpio->AFR[pin >> 3] = (gpio->AFR[pin >> 3] & ~(15 << (4 * (pin & 7)))) | (alt << (4 * (pin & 7)));
}
#define gpio_get(gpio, pin) ((gpio->DATA >> (pin)) & 1)
#define gpio_set(gpio, pin, value) do { gpio->DATA = (gpio->DATA & ~(1 << (pin))) | (value << pin); } while (0)
#define gpio_low(gpio, pin) do { gpio->DATA = (gpio->DATA & ~(1 << (pin))); } while (0)
#define gpio_high(gpio, pin) do { gpio->DATA = (gpio->DATA | (1 << (pin))); } while (0)

void stm32_init(void) {
    //volatile unsigned int dummy;
    // basic MCU config
    //RCC->CR |= (uint32_t)0x00000001; // set HSION
    //RCC->CFGR = 0x00000000; // reset all
    //RCC->CR &= (uint32_t)0xfef6ffff; // reset HSEON, CSSON, PLLON
    //RCC->PLLCFGR = 0x24003010; // reset PLLCFGR
    //RCC->CR &= (uint32_t)0xfffbffff; // reset HSEBYP
    //RCC->CIR = 0x00000000; // disable IRQs
    //


    asm volatile ("cpsid i");

    //// leave the clock as-is (internal 16MHz)

    //// enable GPIO clocks
    //RCC->AHB1ENR |= 0x00000003; // GPIOAEN, GPIOBEN
    *RCGC2 |= 0x3F; 
    // Enable clock for UART0
    *RCGC1 |= 1;

    asm volatile ("nop");
    asm volatile ("nop");
    asm volatile ("nop");
    asm volatile ("nop");
    asm volatile ("nop");
    asm volatile ("nop");
    //dummy = ~1;
    //// turn on an LED! (on pyboard it's the red one)
    //gpio_init(GPIOA, 13, GPIO_MODE_OUT, GPIO_PULL_NONE, 0);
    //gpio_high(GPIOA, 13);
    


    // enable UART1 at 9600 baud (TX=B6, RX=B7)
    UART0->CTL &= ~1;
    //UART0->IBRD = 104;    // IBRD = int(16,000,000 / (16 * 9600)) = int(104.166666)
    //UART0->FBRD = 11;     // FBRD = round(0.166666 * 64) = 11

    UART0->IBRD = 43;                    // IBRD = int(80,000,000 / (16 * 115200)) = int(43.402778)
    UART0->FBRD = 26;                    // FBRD = round(0.402778 * 64) = 26  

    UART0->LCRH = (0x3 << 5) + (1 << 4);  // Set to 8 bits (WLEN) and enable FIFO (FEN)

    UART0->CTL |= 1; 

    GPIOA->AFSEL |= 0x03;
    GPIOA->DEN |= 0x03;
    GPIOA->PCTL = (GPIOA->PCTL & ~0xFF) + 0x11;
    GPIOA->AMSEL &= ~0x3; 

    //gpio_init(GPIOB, 6, GPIO_MODE_ALT, GPIO_PULL_NONE, 7);
    //gpio_init(GPIOB, 7, GPIO_MODE_ALT, GPIO_PULL_NONE, 7);
    //RCC->APB2ENR |= 0x00000010; // USART1EN
    //USART1->BRR = (104 << 4) | 3; // 16MHz/(16*104.1875) = 9598 baud
    //USART1->CR1 = 0x0000200c; // USART enable, tx enable, rx enable

    clock_init();

    // turn on an LED! (on pyboard it's the red one)
    GPIOF->DIR = 0x0E;
    GPIOF->DEN = 0x0E;
    GPIOF->DATA = 0x02;
}


#define SYSCTL_RCC_R            (*((volatile unsigned long *)0x400FE060))
#define SYSCTL_RCC_XTAL_16MHZ   0x00000540  // 16 MHz
#define SYSCTL_RCC_BYPASS       0x00000800  // PLL Bypass
#define SYSCTL_RCC_USESYSDIV    0x00400000  // Enable System Clock Divider
#define SYSCTL_RCC_OSCSRC_INT   0x00000010  // IOSC
#define SYSCTL_RCC2_R           (*((volatile unsigned long *)0x400FE070))
#define SYSCTL_RCC2_USERCC2     0x80000000  // Use RCC2
#define SYSCTL_RCC2_DIV400      0x40000000  // Divide PLL as 400 MHz vs. 200
#define SYSCTL_RCC2_OSCSRC2_MO  0x00000000  // MOSC
#define SYSCTL_RIS_R            (*((volatile unsigned long *)0x400FE050))
#define SYSCTL_RIS_PLLLRIS      0x00000040  // PLL Lock Raw Interrupt Status
#define SYSCTL_RCC2_BYPASS2     0x00000800  // PLL Bypass 2


#define SYSCTL_SYSDIV_2_5  (0x04 << 22)
void clock_init(void)
{
  // Configure PLL and enable Main OSC but keep running on PIOSC
  SYSCTL_RCC_R = SYSCTL_RCC_XTAL_16MHZ | SYSCTL_RCC_BYPASS | SYSCTL_RCC_USESYSDIV | SYSCTL_RCC_OSCSRC_INT;  
  // Use Register RCC2, Enable Div400, config SysDiv2 = 2 and SysDiv2LSB = 0, Switch to main Osc and disable Bypass
  //SYSCTL_RCC2_R = SYSCTL_RCC2_USERCC2 | SYSCTL_RCC2_DIV400 | SYSCTL_SYSDIV_2_5 | SYSCTL_RCC2_OSCSRC2_MO;
  SYSCTL_RCC2_R = SYSCTL_RCC2_USERCC2 | SYSCTL_RCC2_DIV400 | SYSCTL_SYSDIV_2_5 | SYSCTL_RCC2_OSCSRC2_MO | SYSCTL_RCC2_BYPASS2;
    
  //asm volatile ("nop");
  //asm volatile ("nop");
  //asm volatile ("nop");
  //asm volatile ("nop");
  //asm volatile ("nop");
  //asm volatile ("nop");

  while ( !(SYSCTL_RIS_R & SYSCTL_RIS_PLLLRIS) );
  SYSCTL_RCC2_R &= ~SYSCTL_RCC2_BYPASS2;
}

#endif
