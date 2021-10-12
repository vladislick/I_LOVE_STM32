#include <stm32f1xx.h>
#include <stm32f103xb.h>
#include <stdio.h>

// Ситсемная частота (SYSCLK)
#define F 72000000UL
// Максимальная длина строки
#define STR_MAX 256

// Системный счётчик (уменьшается каждую миллисекунду)
volatile uint32_t SysCounter = 0;

// Реализация задержки в мс (по тактам)
inline void delay_ms_(uint32_t time) {
    for (uint32_t i = 0; i < F / 14400 * time; i++) __NOP();
}

// Реализация задержки в мс
void delay_ms(uint32_t time) {
    MODIFY_REG(SysTick->VAL, SysTick_VAL_CURRENT_Msk, F / 1000 - 1);
    SysCounter = time;
    while(SysCounter);
}

// Инициализация системного таймера
void SysTick_Init() {
    // Настраиваем SysTick на 1 мс 
    MODIFY_REG(SysTick->LOAD, SysTick_LOAD_RELOAD_Msk, F / 1000 - 1);
    // Очищаем текущее значение таймера
    CLEAR_BIT(SysTick->VAL, SysTick_VAL_CURRENT_Msk);
    // Хз, в мануале не нашёл
    SET_BIT(SysTick->CTRL, SysTick_CTRL_CLKSOURCE_Msk);
    SET_BIT(SysTick->CTRL, SysTick_CTRL_ENABLE_Msk);
    SET_BIT(SysTick->CTRL, SysTick_CTRL_TICKINT_Msk);
}

// Обработчик прерывания системного таймера
void SysTick_Handler() {
    if (SysCounter != 0) SysCounter--;
}

// Инициализация тактирования микроконтроллера (портов, таймеров и т.д.)       
void RCC_Init() {
    // Включаем внешнее тактирование и ждём его запуска
    SET_BIT(RCC->CR, RCC_CR_HSEON); while(READ_BIT(~RCC->CR, RCC_CR_HSERDY));
    // Устанавливаем умножитель PLL на 9 (чтобы получить 72МГц на SYSCLK)
    MODIFY_REG(RCC->CFGR, RCC_CFGR_PLLMULL_Msk, RCC_CFGR_PLLMULL9);
    // Устанавливаем внешний источник тактирования PLL
    SET_BIT(RCC->CFGR, RCC_CFGR_PLLSRC);
    // Включаем PLL и ждём его запуска
    SET_BIT(RCC->CR, RCC_CR_PLLON); while(READ_BIT(~RCC->CR, RCC_CR_PLLRDY));
    // Настраиваем интервал задержки для Flash-памяти 
    MODIFY_REG(FLASH->ACR, FLASH_ACR_LATENCY_Msk, FLASH_ACR_LATENCY_1);
    // Устанавливаем предделитель HPRE на 1
    MODIFY_REG(RCC->CFGR, RCC_CFGR_HPRE_Msk, RCC_CFGR_HPRE_DIV1);
    // Устанавливаем предделитель PPRE1 на 2
    MODIFY_REG(RCC->CFGR, RCC_CFGR_PPRE1_Msk, RCC_CFGR_PPRE2_DIV2);
    // Устанавливаем предделитель PPRE2 на 1
    MODIFY_REG(RCC->CFGR, RCC_CFGR_PPRE2_Msk, RCC_CFGR_PPRE2_DIV1);
    // Выключаем предделитель LSB
    CLEAR_BIT(RCC->CFGR, RCC_CFGR_PLLXTPRE);
    // Устанавливаем источник PLL для SYSCLK и ждём его запуска
    MODIFY_REG(RCC->CFGR, RCC_CFGR_SW_Msk, RCC_CFGR_SW_PLL); 
    while(READ_BIT(~RCC->CFGR, RCC_CFGR_SWS_PLL));
}

// Инициализация режимов работы портов микроконтроллера 
void GPIO_Init() {
    // Включаем тактирование порта C
    SET_BIT(RCC->APB2ENR, RCC_APB2ENR_IOPCEN);
    // Настраиваем пин 13 порта C на выход
    SET_BIT(GPIOC->CRH, GPIO_CRH_MODE13_0);
}

// Инициализация работы USART1
void USART1_Init() {
    // Включаем тактирование порта A, альтернативных функицй и USART1
    SET_BIT(RCC->APB2ENR, RCC_APB2ENR_IOPAEN|RCC_APB2ENR_AFIOEN|RCC_APB2ENR_USART1EN);
    // Настраиваем порты для USART1
    CLEAR_BIT(GPIOA->CRH, GPIO_CRH_MODE9|GPIO_CRH_MODE10|GPIO_CRH_CNF9|GPIO_CRH_CNF10);
    SET_BIT(GPIOA->CRH, GPIO_CRH_MODE9_0|GPIO_CRH_CNF9_1|GPIO_CRH_CNF10_0);
    // Задаём скорость работы
    USART1->BRR = F / 9600;
    USART1->CR1 = USART_CR1_UE|USART_CR1_TE|USART_CR1_RE;
}

// Передать символ ch по USART1
void USART1_Tx(char ch) {
    while(READ_BIT(~USART1->SR, USART_SR_TXE)) {}
    USART1->DR = ch;
}

// Передать строку по USART1
void USART1_TxStr(char* str) {
    // Отправляем символы, пока не встретим символ конца строки
    // или счётчик не переполнится
    for (uint16_t i = 0; i < STR_MAX; i++) {
        if (str[i] == '\0') break;
        USART1_Tx(str[i]);
    }
}

int main() {
    // Инициализируем RCC
    RCC_Init();
    // Инициализируем GPIO
    GPIO_Init();
    // Инициализируем SysTick
    SysTick_Init();
    // Инициализируем USART1
    USART1_Init();

    uint16_t count = 0;
    char str[STR_MAX];
    double a = 0.01;

    while (1) {
        GPIOC->BSRR = GPIO_BSRR_BS13;
        sprintf(str, "Count is %d.\n", (int)a);
        USART1_TxStr(str);
        delay_ms(500);
        GPIOC->BSRR = GPIO_BSRR_BR13;
        delay_ms(500);
        a += 0.51;
    }
}