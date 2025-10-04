#include "serial_stdio.h"
#include <sys/unistd.h>  // _write
#include <stdio.h>

static inline void uart1_putc(uint8_t c) {
    while ((USART1->SR & USART_SR_TXE) == 0u) {}
    USART1->DR = c;
}
static inline int uart1_txc_done(void) {
    return (USART1->SR & USART_SR_TC) != 0;
}

void serial_stdio_init(uint32_t baud) {
    // Clocks
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;

    // PA9 (TX), PA10 (RX) em AF7
    GPIOA->MODER   &= ~((3u << (9*2)) | (3u << (10*2)));
    GPIOA->MODER   |=  ((2u << (9*2)) | (2u << (10*2)));
    GPIOA->PUPDR   &= ~((3u << (9*2)) | (3u << (10*2)));
    GPIOA->OSPEEDR |=  (2u << (9*2)); // TX um pouco mais rápido
    GPIOA->AFR[1]  &= ~((0xFu << ((9-8)*4)) | (0xFu << ((10-8)*4)));
    GPIOA->AFR[1]  |=  ((7u  << ((9-8)*4)) | (7u  << ((10-8)*4)));

    // USART1 8N1 oversampling 16
    USART1->CR1 = 0u;
    USART1->CR2 = 0u;
    USART1->CR3 = 0u;

    // BRR (OVER8=0): mantissa.frac[3:0]
    // Em F411: USART1 clock = PCLK2 (= APB2), aqui 100 MHz (clock.c)
    uint32_t pclk = SystemCoreClock;  // APB2 prescaler = 1 → 100 MHz
    uint32_t div16 = (pclk + (baud/2u)) / baud; // arredonda
    uint32_t mant  = div16 / 16u;
    uint32_t frac  = div16 % 16u;
    USART1->BRR = (mant << 4) | (frac & 0xFu);

    // Habilita TX/RX e USART
    USART1->CR1 |= USART_CR1_TE | USART_CR1_RE;
    USART1->CR1 |= USART_CR1_UE;

    // stdout sem buffer para imprimir na hora
    setvbuf(stdout, NULL, _IONBF, 0);
}

// ---- Retarget do printf: syscall _write ----
int _write(int fd, const void *buf, size_t count) {
    (void)fd; // stdout/stderr
    const uint8_t *p = (const uint8_t*)buf;
    for (size_t i = 0; i < count; i++) {
        uint8_t c = p[i];
        if (c == '\n') uart1_putc('\r'); // CRLF
        uart1_putc(c);
    }
    while (!uart1_txc_done()) {}
    return (int)count;
}

// Opcional: stubs mínimos (evitam link-errors caso use scanf/malloc em newlib nano)
__attribute__((weak)) int _read(int fd, void *buf, size_t count) {
    (void)fd; (void)buf; (void)count; return 0;
}
__attribute__((weak)) caddr_t _sbrk(int incr) {
    extern uint8_t _end;     // fornecido pelo linker
    static uint8_t *heap_end;
    uint8_t *prev;
    if (heap_end == 0) heap_end = &_end;
    prev = heap_end;
    heap_end += incr;
    return (caddr_t)prev;
}

// Exposição opcional:
void serial_putc(uint8_t c) { uart1_putc(c); }
int  serial_tx_done(void)   { return uart1_txc_done(); }
