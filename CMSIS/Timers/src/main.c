#include <stm32f1xx.h>

// Системная частота (SYSCLK)
#define F 72000000UL

/* ========================================================= */
/* ======== Немного макросов для упрощения жизни :) ======== */
/* ========================================================= */

// Включить прерывание по переполнению таймера TIMx 
#define TIM_EnableIT_UPDATE(TIMx) SET_BIT(TIMx->DIER, TIM_DIER_UIE)
// Запустить счётчик таймера TIMx
#define TIM_EnableCounter(TIMx) SET_BIT(TIMx->CR1, TIM_CR1_CEN)
// Остановить счётчик таймера TIMx
#define TIM_DisableCounter(TIMx) CLEAR_BIT(TIMx->CR1, TIM_CR1_CEN)
// Прочитать флаг переполнения таймера
#define TIM_UpdateFlag(TIMx) READ_BIT(TIMx->SR, TIM_SR_UIF)
// Сбросить флаг переполнения таймера
#define TIM_ClearUpdateFlag(TIMx) CLEAR_BIT(TIMx->SR, TIM_SR_UIF)

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
    // Настраиваем пин 14 порта C на выход
    SET_BIT(GPIOC->CRH, GPIO_CRH_MODE14_0);
}

// Инициализация таймера 2
void TIM2_Init() {
    // Включаем тактирование таймера
    SET_BIT(RCC->APB1ENR, RCC_APB1ENR_TIM2EN);
    // Включаем глобальное прерывание таймера
    NVIC_EnableIRQ(TIM2_IRQn);
    // Задаём значение делителя счётчика (на 1 меньше желаемого значения)
    WRITE_REG(TIM2->PSC, 3599);
    // Задаём значение таймера, до которого он будет считать
    WRITE_REG(TIM2->ARR, 20000);
    // Разрешаем использование прерывания по переполнению
    TIM_EnableIT_UPDATE(TIM2);
    // Запускаем счётчик
    TIM_EnableCounter(TIM2);
}

// Инициализация таймера 3
void TIM3_Init() {
    // Включаем тактирование таймера
    SET_BIT(RCC->APB1ENR, RCC_APB1ENR_TIM3EN);
    // Включаем глобальное прерывание таймера
    NVIC_EnableIRQ(TIM3_IRQn);
    // Задаём значение делителя счётчика (на 1 меньше желаемого значения)
    WRITE_REG(TIM3->PSC, 3599);
    // Задаём значение таймера, до которого он будет считать
    WRITE_REG(TIM3->ARR, 5000);
    // Разрешаем использование прерывания по переполнению
    TIM_EnableIT_UPDATE(TIM3);
    // Запускаем счётчик
    TIM_EnableCounter(TIM3);
}

// Прерывание таймера 2
void TIM2_IRQHandler() {
    // Проверяем, вызвано ли прерывание переполнением счётчика
    if (!TIM_UpdateFlag(TIM2)) return;
    // Сбрасываем флаг переполнения счётчика
    TIM_ClearUpdateFlag(TIM2);
    // Меняем значение пина порта
    if (READ_BIT(GPIOC->ODR, GPIO_ODR_ODR13))
        SET_BIT(GPIOC->BSRR, GPIO_BSRR_BR13);
    else
        SET_BIT(GPIOC->BSRR, GPIO_BSRR_BS13);
}

// Прерывание таймера 3
void TIM3_IRQHandler() {
    // Проверяем, вызвано ли прерывание переполнением счётчика
    if (!TIM_UpdateFlag(TIM3)) return;
    // Сбрасываем флаг переполнения счётчика
    TIM_ClearUpdateFlag(TIM3);
    // Меняем значение пина порта
    if (READ_BIT(GPIOC->ODR, GPIO_ODR_ODR14))
        SET_BIT(GPIOC->BSRR, GPIO_BSRR_BR14);
    else
        SET_BIT(GPIOC->BSRR, GPIO_BSRR_BS14);
}

int main() {
    // Инициализируем RCC
    RCC_Init();
    // Инициализируем GPIO
    GPIO_Init();
    // Инициализируем таймер 2
    TIM2_Init();
    // Инициализируем таймер 3
    TIM3_Init();

    while (1) {}
}