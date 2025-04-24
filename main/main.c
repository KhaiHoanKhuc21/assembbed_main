#include "stm32f4xx.h"
#include <math.h>

#define PI 3.14159265359
#define MPU6050_ADDR 0xD0

float RatePitch;
float KalmanAnglePitch = 0;
float KalmanUncertaintyAnglePitch = 2*2;
float Kalman1DOutput[] = {0, 0};
float ax, ay, az;
float AnglePitch;
int mode;

void TIM2_Manual_Init(void) {
    RCC->APB1ENR |= (1 << 0);

    TIM2->PSC = 100 - 1;
    TIM2->ARR = 0xFFFFFFFF;
    TIM2->CR1 &= ~(1 << 4);
    TIM2->CR1 &= ~(3 << 5);
    TIM2->EGR |= (1 << 0);

    TIM2->CR1 |= (1 << 0);
}

float get_dt(void) {
    static uint32_t last_time = 0;
    uint32_t current_time = TIM2->CNT;
    uint32_t delta_ticks;
    if (current_time >= last_time) {
        delta_ticks = current_time - last_time;
    } else {
        delta_ticks = (0xFFFFFFFF - last_time) + current_time + 1; // Xử lý tràn
    }
    last_time = current_time;
    float dt = delta_ticks / 1000000.0f; // Giây
    if (dt < 0.0001f) dt = 0.0001f; // Giới hạn dt tối thiểu (100µs)
    return dt;
}

// Sửa hàm computePID
double computePID(double input, double setpoint, double KP, double KI, double KD) {
    static double prevError = 0;
    static double integral = 0;
    static double motorOutput = 0;

    float dt = get_dt(); // Lấy dt từ get_dt

    double error = setpoint - input;
    double proportional = KP * error;
    integral += KI * error * dt;
    if (integral > 255.0) integral = 255.0;
    else if (integral < -255.0) integral = -255.0;
    double derivative = (dt > 0.0f) ? (KD * (error - prevError) / dt) : 0.0;
    motorOutput = proportional + integral + derivative;
    if (motorOutput > 255.0) motorOutput = 255.0;
    else if (motorOutput < -255.0) motorOutput = -255.0;
    prevError = error;
    return motorOutput;
}

void kalman_1d(float KalmanState, float KalmanUncertainty, float KalmanInput, float KalmanMeasurement) {
    const float dt = 0.0025f;
    const float process_noise_variance = 1.0f;   // giảm nhẹ so với 4.0 để tin tưởng mô hình hơn
    const float measurement_noise_variance = 16.0f; // tăng nhẹ (ví dụ 4² = 16), để giảm ảnh hưởng của nhiễu cảm biến

    KalmanState += dt * KalmanInput;
    KalmanUncertainty += dt * dt * process_noise_variance;

    float KalmanGain = KalmanUncertainty / (KalmanUncertainty + measurement_noise_variance);

    KalmanState += KalmanGain * (KalmanMeasurement - KalmanState);
    KalmanUncertainty *= (1.0f - KalmanGain);

    Kalman1DOutput[0] = KalmanState;
    Kalman1DOutput[1] = KalmanUncertainty;
}


void SystemClock_Config(void) {
    RCC->CR |= (1 << 0);
    while (!(RCC->CR & (1 << 1)));
    RCC->CFGR = 0;
}

void I2C_GPIO_Config(void) {
    RCC->AHB1ENR |= (1 << 1);
    GPIOB->MODER |= (2 << 12) | (2 << 14);
    GPIOB->OTYPER |= (1 << 6) | (1 << 7);
    GPIOB->OSPEEDR |= (3 << 12) | (3 << 14);
    GPIOB->PUPDR |= (1 << 12) | (1 << 14);
    GPIOB->AFR[0] |= (4 << 24) | (4 << 28);
}

void I2C_Config(void) {
    RCC->APB1ENR |= (1 << 21);
    RCC->APB1RSTR |= (1 << 21);
    RCC->APB1RSTR &= ~(1 << 21);
    I2C1->CR2 = 16;
    I2C1->CCR = 80;
    I2C1->TRISE = 17;
    I2C1->CR1 |= (1 << 0);
}

int I2C_Start(void) {
    I2C1->CR1 |= (1 << 8);
    uint32_t timeout = 10000;
    while (!(I2C1->SR1 & (1 << 0)) && timeout--);
    return timeout > 0 ? 0 : -1;
}

int I2C_Write(uint8_t dev_addr, uint8_t reg_addr, uint8_t* data, uint16_t len) {
    uint32_t timeout = 10000;
    while (I2C1->SR2 & (1 << 1));
    if (I2C_Start() < 0) return -1;
    I2C1->DR = dev_addr & 0xFE;
    while (!(I2C1->SR1 & (1 << 1)) && timeout--);
    if (timeout == 0) return -1;
    (void)I2C1->SR2;
    timeout = 10000;
    while (!(I2C1->SR1 & (1 << 7)) && timeout--);
    if (timeout == 0) return -1;
    I2C1->DR = reg_addr;
    for (uint16_t i = 0; i < len; i++) {
        timeout = 10000;
        while (!(I2C1->SR1 & (1 << 7)) && timeout--);
        if (timeout == 0) return -1;
        I2C1->DR = data[i];
    }
    while (!(I2C1->SR1 & (1 << 2)));
    I2C1->CR1 |= (1 << 9);
    return 0;
}

int MPU6050_Read(uint8_t reg_addr, uint8_t* data, uint16_t len) {
    uint32_t timeout = 10000;
    while (I2C1->SR2 & (1 << 1));
    if (I2C_Start() < 0) return -1;
    I2C1->DR = MPU6050_ADDR & 0xFE;
    while (!(I2C1->SR1 & (1 << 1)) && timeout--);
    if (timeout == 0) return -1;
    (void)I2C1->SR2;
    timeout = 10000;
    while (!(I2C1->SR1 & (1 << 7)) && timeout--);
    if (timeout == 0) return -1;
    I2C1->DR = reg_addr;
    timeout = 10000;
    while (!(I2C1->SR1 & (1 << 7)) && timeout--);
    if (timeout == 0) return -1;
    if (I2C_Start() < 0) return -1;
    I2C1->DR = MPU6050_ADDR | 0x01;
    timeout = 10000;
    while (!(I2C1->SR1 & (1 << 1)) && timeout--);
    if (timeout == 0) return -1;
    (void)I2C1->SR2;
    for (uint16_t i = 0; i < len; i++) {
        if (i == len - 1) {
            I2C1->CR1 &= ~(1 << 10);
            I2C1->CR1 |= (1 << 9);
        }
        timeout = 10000;
        while (!(I2C1->SR1 & (1 << 6)) && timeout--);
        if (timeout == 0) return -1;
        data[i] = I2C1->DR;
    }
    I2C1->CR1 |= (1 << 10);
    return 0;
}

int MPU6050_Init(void) {
    uint8_t data = 0x00;
    if (I2C_Write(MPU6050_ADDR, 0x6B, &data, 1) < 0) return -1; // Tắt chế độ sleep
    if (I2C_Write(MPU6050_ADDR, 0x1A, &data, 1) < 0) return -1; // Cấu hình DLPF
    if (I2C_Write(MPU6050_ADDR, 0x1C, &data, 1) < 0) return -1; // Cấu hình accelerometer ±2g
    if (I2C_Write(MPU6050_ADDR, 0x1B, &data, 1) < 0) return -1; // Cấu hình gyroscope ±250°/s
    return 0;
}

int MPU6050_GetData(int16_t* accelData, int16_t* gyroData) {
    uint8_t buffer[14];
    if (MPU6050_Read(0x3B, buffer, 14) < 0) return -1;
    accelData[0] = (int16_t)((buffer[0] << 8) | buffer[1]);
    accelData[1] = (int16_t)((buffer[2] << 8) | buffer[3]);
    accelData[2] = (int16_t)((buffer[4] << 8) | buffer[5]);
    gyroData[0] = (int16_t)((buffer[8] << 8) | buffer[9]);
    gyroData[1] = (int16_t)((buffer[10] << 8) | buffer[11]);
    gyroData[2] = (int16_t)((buffer[12] << 8) | buffer[13]);
    return 0;
}

void USART2_Init(void) {
    RCC->AHB1ENR |= (1 << 0);
    RCC->APB1ENR |= (1 << 17);
    GPIOA->MODER &= ~(3 << 4);
    GPIOA->MODER |= (2 << 4);
    GPIOA->AFR[0] |= (7 << 8);
    USART2->CR1 = 0;
    USART2->BRR = 0x8B;
    USART2->CR1 |= (1 << 3) | (1 << 13);
}

void USART2_SendChar(char ch) {
    while (!(USART2->SR & (1 << 7)));
    USART2->DR = ch;
}

// Cấu hình GPIO: PA0 (IN2), PA1 (IN1), PA5 (IN4), PA6 (IN3)
void GPIO_Init(void) {
    // Bật clock cho GPIOA
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

    // PA0, PA1, PA5, PA6: Output mode (01)
    GPIOA->MODER &= ~(GPIO_MODER_MODE0 | GPIO_MODER_MODE1 | GPIO_MODER_MODE5 | GPIO_MODER_MODE6);
    GPIOA->MODER |= (GPIO_MODER_MODE0_0 | GPIO_MODER_MODE1_0 | GPIO_MODER_MODE5_0 | GPIO_MODER_MODE6_0);

    // PA0, PA1, PA5, PA6: Push-pull
    GPIOA->OTYPER &= ~(GPIO_OTYPER_OT0 | GPIO_OTYPER_OT1 | GPIO_OTYPER_OT5 | GPIO_OTYPER_OT6);

    // PA0, PA1, PA5, PA6: No pull-up/pull-down
    GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPD0 | GPIO_PUPDR_PUPD1 | GPIO_PUPDR_PUPD5 | GPIO_PUPDR_PUPD6);

    // Khởi tạo = 0 (động cơ dừng)
    GPIOA->BSRR = (GPIO_BSRR_BR0 | GPIO_BSRR_BR1 | GPIO_BSRR_BR5 | GPIO_BSRR_BR6);
}

// Cấu hình PWM: PA9 (ENA, TIM1_CH2), PA8 (ENB, TIM1_CH1)
void PWM_Init(void) {
    // Bật clock cho TIM1 và GPIOA
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

    // PA8, PA9: Alternate function mode (10)
    GPIOA->MODER &= ~(GPIO_MODER_MODE8 | GPIO_MODER_MODE9);
    GPIOA->MODER |= (GPIO_MODER_MODE8_1 | GPIO_MODER_MODE9_1);

    // PA8: AF1 (TIM1_CH1), PA9: AF1 (TIM1_CH2)
    GPIOA->AFR[1] &= ~(GPIO_AFRH_AFRH0 | GPIO_AFRH_AFRH1);
    GPIOA->AFR[1] |= (0x1 << 0) | (0x1 << 4);

    // Cấu hình TIM1
    TIM1->CR1 = 0;
    TIM1->ARR = 999;            // PWM ~1 kHz (168 MHz / (168 * 1000))
    TIM1->PSC = 167;            // Prescaler = 168
    TIM1->CCMR1 &= ~(TIM_CCMR1_OC1M | TIM_CCMR1_OC2M);
    TIM1->CCMR1 |= (0x6 << 4) | (0x6 << 12); // PWM mode 1 cho CH1, CH2
    TIM1->CCER |= TIM_CCER_CC1E | TIM_CCER_CC2E; // Bật kênh 1, 2
    TIM1->BDTR |= TIM_BDTR_MOE;  // Bật main output
    TIM1->CCR1 = 0;             // ENB (PA8) duty = 0%
    TIM1->CCR2 = 0;             // ENA (PA9) duty = 0%
    TIM1->CR1 |= TIM_CR1_CEN;   // Bật Timer
}
void motorA(int16_t speed, int mode) {
    uint16_t abs_speed = (speed < 0) ? -speed : speed;
    if (abs_speed > 255) abs_speed = 255;
    uint16_t pulse = (abs_speed * 999) / 255;

    // Điều khiển hướng dựa trên mode
    if (mode == 1) { // Tiến
        GPIOA->BSRR = GPIO_BSRR_BS1 | GPIO_BSRR_BR0;
    } else { // Lùi
        GPIOA->BSRR = GPIO_BSRR_BR1 | GPIO_BSRR_BS0;
    }
    TIM1->CCR2 = pulse;
}

void motorB(int16_t speed, int mode) {
    uint16_t abs_speed = (speed < 0) ? -speed : speed;
    if (abs_speed > 255) abs_speed = 255;
    uint16_t pulse = (abs_speed * 999) / 255;

    if (mode == 1) { // Tiến
        GPIOA->BSRR = GPIO_BSRR_BS6 | GPIO_BSRR_BR5;
    } else { // Lùi
        GPIOA->BSRR = GPIO_BSRR_BR6 | GPIO_BSRR_BS5;
    }
    TIM1->CCR1 = pulse;
}

void print(const char *str) {
    while (*str) USART2_SendChar(*str++);
}

void print_int(int16_t num) {
    if (num < 0) {
        USART2_SendChar('-');
        num = -num;
    }
    char buf[6];
    int i = 0;
    do {
        buf[i++] = (num % 10) + '0';
    } while (num /= 10);
    while (i--) USART2_SendChar(buf[i]);
}

void print_float(float val) {
    if (val < 0) {
        USART2_SendChar('-');
        val = -val;
    }
    print_int((int)val);
    USART2_SendChar('.');
    int dec = (int)(val * 100) % 100;
    if (dec < 10) USART2_SendChar('0');
    print_int(dec);
}
void print_double(double val) {
    if (val < 0) {
        USART2_SendChar('-');
        val = -val;
    }
    print_int((int)val);
    USART2_SendChar('.');
    int dec = (int)(val * 100) % 100;
    if (dec < 10) USART2_SendChar('0');
    print_int(dec);
}
int main(void) {
	SystemInit();
    SystemClock_Config();
    USART2_Init();
    I2C_GPIO_Config();
    I2C_Config();
    GPIO_Init();
    PWM_Init();
    if (MPU6050_Init() < 0) {
        print("MPU6050 Init Failed\r\n");
        while (1);
    }
    print("MPU6050 Init OK\r\n");

    int16_t accelData[3], gyroData[3];

    while (1) {
        if (MPU6050_GetData(accelData, gyroData) < 0) {
            print("Read Failed\r\n");
        } else {
            ax = accelData[0] / 16384.0f;
            ay = accelData[1] / 16384.0f;
            az = accelData[2] / 16384.0f;

            double motorSpeed ;
            AnglePitch = atan2(-ax, sqrt(ay * ay + az * az));
            RatePitch = gyroData[1] / 131.0f;

            double setpoint = -0.02;
            double Kp=3000;
            double Ki=0;
            double Kd=12.5;

            kalman_1d(KalmanAnglePitch, KalmanUncertaintyAnglePitch, RatePitch * PI / 180.0f, AnglePitch);
            KalmanAnglePitch = Kalman1DOutput[0] ;
            KalmanUncertaintyAnglePitch = Kalman1DOutput[1];
            motorSpeed = computePID(KalmanAnglePitch,setpoint,Kp,Ki,Kd);

            if (KalmanAnglePitch < 1 && KalmanAnglePitch > -1) {
                if (motorSpeed <= 0) {
                    mode = 1; // Chế độ tiến
                } else {
                    mode = 0; // Chế độ lùi
                    motorSpeed = -motorSpeed;
                }

                if (motorSpeed > 255) motorSpeed = 255;
            }
            else {
                motorSpeed = 0;
            }
            motorA((int16_t)motorSpeed, mode);
            motorB((int16_t)motorSpeed, mode);

            print_float(KalmanAnglePitch);
            print("\r\n");
            print_double(motorSpeed);
        }
    }
}
