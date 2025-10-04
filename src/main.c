#include "stm32f4xx.h"
#include <stdio.h>
#include "serial_stdio.h"
#include "mpu6050.h"


static void delay_ms(uint32_t ms) {
    SysTick->LOAD = (SystemCoreClock/1000u) - 1u;
    SysTick->VAL  = 0;
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;
    while (ms--) {
        while ((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) == 0);
    }
    SysTick->CTRL = 0;
}


static uint32_t guess_apb1_hz(void) {
    return 42000000u;
}

int main(void) {           // ← Adicione esta linha
    SystemCoreClockUpdate();


    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
    GPIOC->MODER &= ~(3u<<(13*2));
    GPIOC->MODER |=  (1u<<(13*2));

    serial_stdio_init(115200);

    uint32_t apb1 = guess_apb1_hz();
    printf("Initializing I2C1 with APB1=%lu Hz\n", apb1);
    i2c1_init_100k(apb1);
    printf("I2C1 initialized successfully\n");
    
    // Verificar se PB8/PB9 estão configurados
    printf("GPIOB->MODER = 0x%08lX\n", GPIOB->MODER);
    printf("GPIOB->AFR[1] = 0x%08lX\n", GPIOB->AFR[1]);

    uint8_t who=0x00;
    printf("Scanning I2C addresses...\n");
    
    // // Teste básico: scan de endereços I2C
    // for(uint8_t addr = 0x08; addr < 0x78; addr++) {
    //     uint8_t dummy;
    //     if(i2c1_read_reg(addr, 0x00, &dummy) == 0) {
    //         printf("Device found at address 0x%02X\n", addr);
    //     }
    // }
    
    printf("Testing MPU6050 at 0x68...\n");
    if (i2c1_read_reg(MPU6050_ADDR, MPU6050_REG_WHOAMI, &who) < 0) {
        printf("I2C communication failed with MPU6050 (addr 0x%02X)\n", MPU6050_ADDR);
        printf("Check connections: SCL->PB8, SDA->PB9, VCC->3.3V, GND->GND\n");
        printf("LED will blink fast to indicate I2C error\n");
        for (;;) { GPIOC->ODR ^= (1u<<13); delay_ms(150); }
    } else if (who != 0x68) {
        printf("Wrong WHO_AM_I: got 0x%02X, expected 0x68\n", who);
        printf("Device responds but wrong ID - check if it's actually MPU6050\n");
        for (;;) { GPIOC->ODR ^= (1u<<13); delay_ms(150); }
    } else {
        printf("MPU6050 found successfully! WHO_AM_I = 0x%02X\n", who);
    }
    if (mpu6050_init() < 0) {
        for (;;) { GPIOC->ODR ^= (1u<<13); delay_ms(150); }
        printf("MPU6050 init failed\n");
    }

    printf("ax,ay,az,gx,gy,gz,temp\n");

    for (;;) {
        mpu6050_raw_t r;
        if (mpu6050_read_all(&r) == 0) {
            float ax = mpu6050_accel_g(r.ax);
            float ay = mpu6050_accel_g(r.ay);
            float az = mpu6050_accel_g(r.az);
            float gx = mpu6050_gyro_dps(r.gx);
            float gy = mpu6050_gyro_dps(r.gy);
            float gz = mpu6050_gyro_dps(r.gz);
            float tc = mpu6050_temp_c(r.temp_raw);

            printf("%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n", ax, ay, az, gx, gy, gz, tc);
        }
        // Alive blink ~50 Hz
        GPIOC->ODR ^= (1u<<13);
        delay_ms(200);
    }
}