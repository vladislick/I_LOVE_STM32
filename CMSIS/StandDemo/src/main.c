#include <stm32f1xx.h>
#include <stdio.h>
#include <string.h>

// Системная частота (SYSCLK)
#define F 72000000UL
// Максимальная длина строки
#define STR_MAX 256


/* ========================================================= */
/* ======== Объявляем порты ввода-вывода устройств  ======== */
/* ========================================================= */

// Пин лапки Trig ультразвукового датчика (порт B)
#define HCSR04_TRIG_PORT GPIOB
// Пин лапки Trig ультразвукового датчика (порт B)
#define HCSR04_TRIG_PIN 7
// Пин лапки Trig ультразвукового датчика (порт B)
#define HCSR04_ECHO_PORT GPIOB
// Пин лапки Trig ультразвукового датчика (порт B)
#define HCSR04_ECHO_PIN 6

// Пин сервопривода (порт С)
#define SERVO_PORT GPIOB
// Пин сервопривода (порт С)
#define SERVO_PIN 13

// Порт USART1
#define USART1_PORT GPIOA
// Пин USART1 для приёма данных (порт A)
#define USART1_PIN_RX 10
// Пин USART1 для прередачи данных (порт A)
#define USART1_PIN_TX 9

// Системный счётчик (уменьшается каждую миллисекунду)
volatile uint32_t SysCounter = 0;

// Текущее положение сервопривода (тиков таймера)
volatile int16_t servoPosition = 500;

struct user_input {
    char field[16][32];
    uint8_t field_count;
};

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
    if (SysCounter > 0) SysCounter--;
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
    // Настраиваем сервопривод
    MODIFY_REG(SERVO_PORT->CRH, GPIO_CRH_MODE13|GPIO_CRH_CNF13, GPIO_CRH_MODE13_0|GPIO_CRH_CNF13_0);
    SERVO_PORT->BSRR = GPIO_BSRR_BR13;
    // Настраиваем Trig
    MODIFY_REG(HCSR04_TRIG_PORT->CRL, GPIO_CRL_MODE7|GPIO_CRL_CNF7, GPIO_CRL_MODE7_0|GPIO_CRL_CNF7_0); 
    HCSR04_TRIG_PORT->BSRR = GPIO_BSRR_BS7;
    // Настраиваем Echo
    MODIFY_REG(HCSR04_ECHO_PORT->CRL, GPIO_CRL_MODE6|GPIO_CRL_CNF6, GPIO_CRL_CNF6_1);
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
void USART1_TxStr(const char* str) {
    for (uint16_t i = 0; str[i] != '\0'; i++) USART1_Tx(str[i]);
}

// Обработчик USART1
char* USART1_Tick() {
    // Буффер для хранения введенных команд пользователем
    static char tmp[128];
    // Текущее положение курсора на экране
    static uint8_t tmp_index = 0;
    // Считываемый байт данных
    static char tmp_char;
    // Если что-то пришло
    if (USART1->SR & USART_SR_RXNE) {
        tmp_char = USART1->DR;
        // Если пришёл символ стирания (Backspace)
        if (tmp_char == 8) {
            tmp_index--;
            USART1_Tx(tmp_char);
            USART1_Tx(' ');
            USART1_Tx(tmp_char);
            return NULL;
        } else if (tmp_char == '\n') {
            tmp[tmp_index] = '\0';
            tmp_index = 0;
            return tmp;
        } else if (tmp_char == '\r') return NULL;
        tmp[tmp_index++] = tmp_char;
        USART1_Tx(tmp_char);
    }
    return NULL;
}

// Инициализация таймера 2 (работа с HC-SR04)
void TIM2_Init() {
    // Выключаем таймер 2
    //TIM_DisableCounter(TIM2);
    // Включаем тактирование таймера
    SET_BIT(RCC->APB1ENR, RCC_APB1ENR_TIM2EN);
    // Включаем глобальное прерывание таймера
    NVIC_EnableIRQ(TIM2_IRQn);
    // Задаём значение делителя счётчика (на 1 меньше желаемого значения)
    WRITE_REG(TIM2->PSC, 36-1);
    // Задаём значение таймера, до которого он будет считать
    WRITE_REG(TIM2->ARR, 4-1);
    // Разрешаем использование прерывания по переполнению
    TIM_EnableIT_UPDATE(TIM2);
    // Запускаем счётчик
    TIM_EnableCounter(TIM2);
}

// Инициализация таймера 3 (работа с сервоприводом)
void TIM3_Init() {
    // Выключаем таймер 3
    //TIM_DisableCounter(TIM3);
    // Включаем тактирование таймера
    SET_BIT(RCC->APB1ENR, RCC_APB1ENR_TIM3EN);
    // Включаем глобальное прерывание таймера
    NVIC_EnableIRQ(TIM3_IRQn);
    // Задаём значение делителя счётчика (на 1 меньше желаемого значения)
    WRITE_REG(TIM3->PSC, 36-1);
    // Задаём значение таймера, до которого он будет считать
    WRITE_REG(TIM3->ARR, 10-1);
    // Разрешаем использование прерывания по переполнению
    TIM_EnableIT_UPDATE(TIM3);
    // Запускаем счётчик
    TIM_EnableCounter(TIM3);
}

// Прерывание таймера 2 (работа с HC-SR04)
void TIM2_IRQHandler() {
    // Счётчик для датчика 
    static volatile uint16_t sensorTrig = 10000;
    // Проверяем, вызвано ли прерывание переполнением счётчика
    if (!TIM_UpdateFlag(TIM2)) return;
    // Сбрасываем флаг переполнения счётчика
    TIM_ClearUpdateFlag(TIM2);

    if (sensorTrig > 0) sensorTrig--;
    else {
        if (HCSR04_TRIG_PORT->ODR & GPIO_ODR_ODR7) {
            HCSR04_TRIG_PORT->BSRR = GPIO_BSRR_BR7;
            sensorTrig = 19996;
            return;
        } else {
            HCSR04_TRIG_PORT->BSRR = GPIO_BSRR_BS7;
            sensorTrig = 4;
            return;
        }
    }
}

// Прерывание таймера 3 (работа с сервоприводом)
void TIM3_IRQHandler() {
    // Счётчик импульсов сервопривода 
    static volatile uint16_t servoCounter;
    // Проверяем, вызвано ли прерывание переполнением счётчика
    if (!TIM_UpdateFlag(TIM3)) return;
    // Сбрасываем флаг переполнения счётчика
    TIM_ClearUpdateFlag(TIM3);

    // Проверка значения счетчика
    if (servoCounter < 4000) {
        servoCounter++;
        // Выключаем импульс на сервопривод
        if (servoCounter >= servoPosition) SERVO_PORT->BSRR = GPIO_BSRR_BR13;
    } else {
        servoCounter = 0;
        // Подаём импульс на сервопривод
        SERVO_PORT->BSRR = GPIO_BSRR_BS13;
    }
}

int16_t my_abs(int16_t value) {
    if (value < 0) return 0-value;
    else return value;
}

// Является ли str дробным числом
uint8_t isFloat(const char* str) {
    for (uint16_t i = 0; str[i] != '\0'; i++)
        if ((str[i] < 48 || str[i] > 57) && str[i] != '-' && str[i] != '.') return 0;
    return 1;
}

// Возращает количество элементов, которые можно парсить
uint16_t parsingCount(const char* str, char separator) {
    // Флаг
    uint8_t flag = 0;
    // Счетчик
    uint16_t counter = 0;
    // Обход по строке
    for (uint16_t i = 0; str[i] != '\0' && i < STR_MAX; i++) {
        if (str[i] != separator) {
            if (flag == 0) flag = ++counter;
        } else flag = 0;
    }
    // Вернуть количество распознанных элементов
    return counter;
}

// Возращает количество элементов, которые можно парсить
void parsing(char* destStr, const char* str, char separator, uint16_t index) {
    // Очистка строки ответа
    memset(destStr, '\0', STR_MAX);
    // Флаг
    uint8_t flag = 0;
    // Счетчик
    uint16_t count = 0;
    // Счетчик символов строки
    uint16_t strCount = 0;
    // Обход по строке
    for (uint16_t i = 0; str[i] != '\0' && i < STR_MAX; i++) {
        if (str[i] != separator) {
            if (flag == 0) flag = ++count;
        } else flag = 0;
        // Копируем строку, если совпадает
        if (flag != 0 && index == (count - 1)) {
            destStr[strCount++] = str[i];
        }
    }
}

// Обработчик введенного пользователем текста
struct user_input* textProcessing(const char* str) {
    static struct user_input tmp;
    // Флаг
    uint8_t flag = 0;
    // Счетчик
    tmp.field_count = 0;
    // Счетчик символов строки
    uint16_t strCount = 0;
    // Обход по строке
    for (uint16_t i = 0; str[i] != '\0'; i++) {
        if (str[i] != ' ') {
            if (flag == 0) flag = ++tmp.field_count;
        } else {
            tmp.field[flag - 1][strCount] = '\0';
            flag = strCount = 0;
        }
        // Копируем строку, если совпадает
        if (flag != 0) {
            tmp.field[flag - 1][strCount++] = str[i];
        }
    }
    tmp.field[flag - 1][strCount] = '\0';
    return &tmp;
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

    char* str;
    char instr[16];
    struct user_input* user_str;

    while (1) {
        // Запускаем таймер на 50 мс
        SysCounter = 50;
        // Выводим надпись
        USART1_TxStr("~> ");


        // Пока ожидаем таймер, обрабатываем USART1
        while(SysCounter) {
            if ((str = USART1_Tick()) != NULL) {
                USART1_Tx('\n');

                user_str = textProcessing(str);

                if (user_str->field_count == 0) {
                    USART1_TxStr("ERROR: You didn't enter anything\n");
                }

                if (strcmp(user_str->field[0], "set")) {
                    
                }
                for (uint8_t i = 0; i < user_str->field_count; i++) {
                   
                }
            }
        }
    }
}