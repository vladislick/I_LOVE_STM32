#include <stm32f1xx.h>
#include <stdio.h>

// Системная частота (SYSCLK)
#define F 72000000UL
// Максимальная длина строки
#define STR_MAX 256
// Режим отладки (отправлять в USART информацию)
#define DEBUG_MODE 1
// Точка удержания шарика
#define MAIN_POINT 190

// Количество замеров для усреднённого значения
#define SMOOTH_VALUE 200

// Крайнее правое положение желоба
#define SERVO_RIGHT 250
// Центральное положение желоба
#define SERVO_CENTER 340
// Крайнее левое положение желоба
#define SERVO_LEFT 450

/* ========================================================= */
/* ======== Объявляем порты ввода-вывода устройств  ======== */
/* ========================================================= */

// Пин лапки Trig ультразвукового датчика (порт B)
#define TRIG_PIN 3
// Пин лапки Echo ультразвукового датчика (порт B)
#define ECHO_PIN 11
// Пин USART1 для приёма данных (порт A)
#define USART1_RX_PIN 10
// Пин USART1 для прередачи данных (порт A)
#define USART1_TX_PIN 9
// Пин сервопривода (порт С)
#define SERVO_PIN 15

// Системный счётчик (уменьшается каждую миллисекунду)
volatile uint32_t SysCounter = 0;
// Счётчик импульсов сервопривода 
volatile uint16_t servoCounter = 0;
// Текущее положение сервопривода (тиков таймера)
volatile int16_t servoPosition = SERVO_CENTER;
// Сглаживающий значение АЦП массив
volatile uint32_t smoothValue = 0;
// Текущий индекс сглаживающего массива
volatile uint16_t smoothIndex = 0;
// Сглаженное значение АЦП
volatile uint16_t smoothValueReady = 0;

/* ========================================================= */
/* ======== Немного макросов для упрощения жизни :) ======== */
/* ========================================================= */

// Включить прерывание по переполнению таймера TIMx 
#define TIM_EnableIT_UPDATE(TIMx) SET_BIT(TIMx->DIER, TIM_DIER_UIE)
// Запустить счётчик таймера TIMx
#define TIM_EnableCounter(TIMx) SET_BIT(TIMx->CR1, TIM_CR1_CEN)
// Очистить значение счётчика таймера TIMx
#define TIM_ClearCounter(TIMx) TIMx->CNT = 1;
// Остановить счётчик таймера TIMx
#define TIM_DisableCounter(TIMx) CLEAR_BIT(TIMx->CR1, TIM_CR1_CEN)
// Прочитать флаг переполнения таймера
#define TIM_UpdateFlag(TIMx) READ_BIT(TIMx->SR, TIM_SR_UIF)
// Сбросить флаг переполнения таймера
#define TIM_ClearUpdateFlag(TIMx) CLEAR_BIT(TIMx->SR, TIM_SR_UIF)

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
    // Включаем тактирование порта B
    SET_BIT(RCC->APB2ENR, RCC_APB2ENR_IOPBEN);
    // Настраиваем пин 13 порта C на выход
    SET_BIT(GPIOC->CRH, GPIO_CRH_MODE13_0);
    SET_BIT(GPIOC->BSRR, GPIO_BSRR_BR13);
    // Настраиваем пин 3 порта B на выход (Trig)
    MODIFY_REG(GPIOB->CRL, GPIO_CRL_CNF3|GPIO_CRL_MODE3, GPIO_CRL_MODE3_0);
    // Настраиваем пин 11 порта B на вход (Echo)
    MODIFY_REG(GPIOB->CRH, GPIO_CRH_CNF11|GPIO_CRH_MODE11, GPIO_CRH_CNF11_0);
    // Настраиваем пин 15 порта С на выход (Servo)
    SET_BIT(GPIOC->CRH, GPIO_CRH_MODE15_0|GPIO_CRH_CNF15_0);
    MODIFY_REG(GPIOA->CRL, GPIO_CRL_MODE0|GPIO_CRL_CNF0, 0);
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
    // Запускаем модуль USART1
    USART1->CR1 = USART_CR1_UE|USART_CR1_TE|USART_CR1_RE;
}

// Передать символ ch по USART1
void USART1_Tx(char ch) {
    // Ожидаем готовности модуля передачи USART1
    while(READ_BIT(~USART1->SR, USART_SR_TXE)) {}
    // Передаём символ модулю передачи USART1
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

// Инициализация таймера 2
void TIM2_Init() {
    // Выключаем таймер 2
    TIM_DisableCounter(TIM2);
    // Включаем тактирование таймера
    SET_BIT(RCC->APB1ENR, RCC_APB1ENR_TIM2EN);
    // Включаем глобальное прерывание таймера
    //NVIC_EnableIRQ(TIM2_IRQn);
    // Задаём значение делителя счётчика (на 1 меньше желаемого значения)
    WRITE_REG(TIM2->PSC, 72-1);
    // Задаём значение таймера, до которого он будет считать
    WRITE_REG(TIM2->ARR, 65000);
    // Разрешаем использование прерывания по переполнению
    //TIM_EnableIT_UPDATE(TIM2);
    // Включаем таймер 2 в режим обратного отсчёта
    SET_BIT(TIM2->CR1, TIM_CR1_DIR);
}

// Инициализация таймера 3
void TIM3_Init() {
    // Включаем тактирование таймера
    SET_BIT(RCC->APB1ENR, RCC_APB1ENR_TIM3EN);
    // Включаем глобальное прерывание таймера
    NVIC_EnableIRQ(TIM3_IRQn);
    // Задаём значение делителя счётчика (на 1 меньше желаемого значения)
    WRITE_REG(TIM3->PSC, 36-1);
    // Задаём значение таймера, до которого он будет считать
    WRITE_REG(TIM3->ARR, 10);
    // Разрешаем использование прерывания по переполнению
    TIM_EnableIT_UPDATE(TIM3);
    // Запускаем счётчик
    TIM_EnableCounter(TIM3);
}

void ADC_Init() {
    // Запрещаем использование АЦП
    CLEAR_BIT(ADC1->CR2, ADC_CR2_ADON);
    // Включаем тактирование АЦП
    SET_BIT(RCC->APB2ENR, RCC_APB2ENR_ADC1EN);
    // Включаем тактирование порта А
    SET_BIT(RCC->APB2ENR, RCC_APB2ENR_IOPAEN);
    // Устанавливаем предделитель АЦП на 6-ть (нельзя превышать 14 МГц)
    MODIFY_REG(RCC->CFGR, RCC_CFGR_ADCPRE, RCC_CFGR_ADCPRE_DIV6);
    // Очищаем регистр CR1 для будущей настройки
    CLEAR_REG(ADC1->CR1);
    // Очищаем регистр CR2 для будущей настройки
    CLEAR_REG(ADC1->CR2);
    // Устанавливаем время выборки (количество тактов для преобразования)
    // 239.5 циклов
    MODIFY_REG(ADC1->SMPR2, ADC_SMPR2_SMP0, ADC_SMPR2_SMP0_0|ADC_SMPR2_SMP0_1|ADC_SMPR2_SMP0_2);
    // Выставляем нулевой канал для замера
    CLEAR_REG(ADC1->SQR3);
    CLEAR_REG(ADC1->SQR2);
    CLEAR_REG(ADC1->SQR1);
    // Выключаем инжектируемые каналы
    CLEAR_REG(ADC1->JSQR);
    // Программный источник запуска
    SET_BIT(ADC1->CR2, ADC_CR2_EXTSEL);
    // Разрешаем внешний запуск
    SET_BIT(ADC1->CR2, ADC_CR2_EXTTRIG);
    // Выключаем режим непрерывного преобразования
    CLEAR_BIT(ADC1->CR2, ADC_CR2_CONT);
    // Выключаем режим сканирования каналов
    CLEAR_BIT(ADC1->CR1, ADC_CR1_SCAN);
    // Включаем глобальные прерывания для АЦП
    NVIC_EnableIRQ(ADC1_2_IRQn);
    // Включаем прерывание по завершению преобразования
    SET_BIT(ADC1->CR1, ADC_CR1_EOCIE);
    // Разрешаем использование АЦП
    SET_BIT(ADC1->CR2, ADC_CR2_ADON);
    // Запуск калибровки
    SET_BIT(ADC1->CR2, ADC_CR2_CAL);
    // Ждём окончания калибровки 
    while(READ_BIT(ADC1->CR2, ADC_CR2_CAL));
    // Запускаем преобразование АЦП
    SET_BIT(ADC1->CR2, ADC_CR2_SWSTART);
}

// Прерывание таймера 2
void TIM2_IRQHandler() {
    // Проверяем, вызвано ли прерывание переполнением счётчика
    if (!TIM_UpdateFlag(TIM2)) return;
    // Сбрасываем флаг переполнения счётчика
    TIM_ClearUpdateFlag(TIM2);
}

// Прерывание таймера 3
void TIM3_IRQHandler() {
    // Проверяем, вызвано ли прерывание переполнением счётчика
    if (!TIM_UpdateFlag(TIM3)) return;
    // Сбрасываем флаг переполнения счётчика
    TIM_ClearUpdateFlag(TIM3);
    if (servoCounter < 4000) {
        // Выключаем импульс на сервопривод
        if (servoCounter > servoPosition) GPIOC->BSRR = GPIO_BSRR_BR15;
        servoCounter++;
    } else {
        servoCounter = 0;
        // Подаём импульс на сервопривод
        GPIOC->BSRR = GPIO_BSRR_BS15;
    }
}

// Прерывание АЦП
void ADC1_2_IRQHandler() {
    if (smoothIndex < SMOOTH_VALUE) {
        smoothValue += ADC1->DR;
        smoothIndex++;
    } else {
        smoothValueReady = smoothValue / SMOOTH_VALUE;
        smoothValue = 0;
        smoothIndex = 0;
    }
    // Запускаем преобразование АЦП
    SET_BIT(ADC1->CR2, ADC_CR2_SWSTART);
}

int16_t my_abs(int16_t value) {
    if (value < 0) return 0-value;
    else return value;
}

// Отправка сообщений для дебага
void debug(uint16_t value, uint16_t distance) {
    // Строка для готового сообщения
    char str[STR_MAX];
    // Напряжение на входе АЦП
    int16_t voltage = value * 33 / 41;
    // Формируем строку для передачи данных
    sprintf(str, "Voltage is %d.\nDistance is %d.\n", voltage, distance);
    // Отправляем строку на ПК
    USART1_TxStr(str);
}

// Повернуть сервопривод на угол, -200 <= angle <= 200
void servoSet(int16_t angle) {
    // Проверка на дурака
    if (angle > 100 || angle < -100) return;
    /*
    int16_t step = 10;
    int16_t diff = servoPosition - (SERVO_CENTER + angle);
    if (my_abs(diff) >= step) {
        if (diff > 0) servoPosition -= step;
        else if (diff < 0) servoPosition += step;
    } else if (diff != 0) {
        if (diff > 0) servoPosition -= diff;
        else if (diff < 0) servoPosition += diff;
    } */
    servoPosition = (SERVO_CENTER + angle);
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
    // Инициализируем SysTick
    SysTick_Init();
    // Инициализируем USART1
    USART1_Init();
    // Инициализируем АЦП
    ADC_Init();

    // Время приходящего с датчика импульса в мкс
    volatile uint16_t value, voltage, time;
    double period;
    uint8_t count = 0;

    // Входное воздействие (сигнал с датчика расстояния)
    double P = 0, I = 0, D = 0, lastP = 0;
    // Коэффициенты регулятора
    double kp, ki, kd, km;
    // Расстояние до шарика в мм
    uint16_t distance;



    // Топ коэффициентов:
    // P * I * D
    // 1) 3 * 0.2 * 0.1
    // 2) 2 * 0.4 * 0.1

    // Коэффициент пропорциональности
    kp = 2;
    // Коэффициент интегрирования
    ki = 0.01;
    // Коэффициент дифференцирования
    kd = 0.5; 
    km = 0.05;
    // Выходной сигнал (сигнал управления сервоприводом)
    // От -100 до 100
    double y = 0; 

    while (1) {
        value = smoothValueReady;
        
        if (value < 700) {
            delay_ms(5);
            continue;
        }

        voltage = value * 33 / 41;
        distance = (uint32_t)328154/voltage - 48;
        if (count > 50) {
            // Формируем сообщение и отправляем на ПК
            if (DEBUG_MODE) debug(value, distance);
            count = 0;
        } else count++;
        
        
        // Находим период замера показаний датчика
        period = 0.100;
        // Вычисляем коэффициент пропорциональности
        P = (MAIN_POINT - distance)*0.1;
        // Вычисляем коэффициент интегрирования
        I = I + P * period;
        // Вычисляем коэффициент дифференцирования
        D = (P - lastP) / period * (km * I + 1);
        // Находим выходной сигнал регулятора
        y = P*kp + I*ki + D*kd;

        // Выставляем нужный угол сервопривода
        servoSet(-y);
        // Запоминаем предыдущее показние датчика
        lastP = P;
        // Делаем задержку перед следующим замером
        delay_ms(100);
    }
}

/*
// Инициализируем АЦП
void ADC_Init() {
    // Запрещаем использование АЦП
    CLEAR_BIT(ADC1->CR2, ADC_CR2_ADON);
    // Включаем тактирование АЦП
    SET_BIT(RCC->APB2ENR, RCC_APB2ENR_ADC1EN);
    // Включаем тактирование порта А
    SET_BIT(RCC->APB2ENR, RCC_APB2ENR_IOPAEN);
    // Устанавливаем предделитель АЦП на 6-ть (нельзя превышать 14 МГц)
    MODIFY_REG(RCC->CFGR, RCC_CFGR_ADCPRE, RCC_CFGR_ADCPRE_DIV6);
    // Очищаем регистр CR1 для будущей настройки
    CLEAR_REG(ADC1->CR1);
    // Очищаем регистр CR2 для будущей настройки
    CLEAR_REG(ADC1->CR2);
    // Устанавливаем время выборки (количество тактов для преобразования)
    // 28.5 циклов
    MODIFY_REG(ADC1->SMPR2, ADC_SMPR2_SMP0, ADC_SMPR2_SMP0_0|ADC_SMPR2_SMP0_1);
    // Выставляем нулевой канал для замера
    CLEAR_REG(ADC1->SQR3);
    CLEAR_REG(ADC1->SQR2);
    CLEAR_REG(ADC1->SQR1);
    // Выключаем инжектируемые каналы
    CLEAR_REG(ADC1->JSQR);
    // Запуск калибровки
    SET_BIT(ADC1->CR2, ADC_CR2_CAL);
    // Ждём окончания калибровки 
    while(READ_BIT(ADC1->CR2, ADC_CR2_CAL));
    // Программный источник запуска
    SET_BIT(ADC1->CR2, ADC_CR2_EXTSEL);
    // Разрешаем внешний запуск
    SET_BIT(ADC1->CR2, ADC_CR2_EXTTRIG);
    // Включаем режим непрерывного преобразования
    SET_BIT(ADC1->CR2, ADC_CR2_CONT);
    // Выключаем режим сканирования каналов
    CLEAR_BIT(ADC1->CR1, ADC_CR1_SCAN);
    // Разрешаем использование АЦП
    SET_BIT(ADC1->CR2, ADC_CR2_ADON);
    // Запускаем преобразование АЦП
    SET_BIT(ADC1->CR2, ADC_CR2_SWSTART);
}
*/