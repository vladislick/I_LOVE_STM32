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
volatile int16_t servoPosition = 0;
volatile int16_t servoCenterPosition = 300;
volatile uint16_t sensorValue = 0;
volatile float sensorDistanceCM = 0;

volatile float pid_p = 0, pid_i = 0, pid_d = 0;
volatile float T = 0.2, d = 0.8;
volatile float setPoint = 25.0;

volatile float d2y, dy, y;

volatile float P, I, D;

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
    MODIFY_REG(HCSR04_ECHO_PORT->CRL, GPIO_CRL_MODE6|GPIO_CRL_CNF6, GPIO_CRL_CNF6_0);
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
            sensorTrig = 9996;
            return;
        } else {
            d2y = sensorDistanceCM/(T*T) - 2*d*dy/T - y/(T*T);
            dy = dy + d2y*0.02;
            y = y + dy*0.02;
            HCSR04_TRIG_PORT->BSRR = GPIO_BSRR_BS7;
            sensorTrig = 4;
            return;
        }
    }

    // Если пришёл сигнал
    if (HCSR04_ECHO_PORT->IDR & GPIO_IDR_IDR6) sensorValue++;
    // Если сигнал пропал
    else if (sensorValue != 0) {
        sensorDistanceCM = (sensorValue * (float)2.0) / (float)58.0;
        if (sensorDistanceCM > 50.0) sensorDistanceCM = 50;
        sensorValue = 0;
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
    if (servoCounter < 8000) {
        servoCounter++;
        // Выключаем импульс на сервопривод
        if (servoCounter >= (servoCenterPosition - servoPosition)) SERVO_PORT->BSRR = GPIO_BSRR_BR13;
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

// Находит совпадающую строку
uint8_t find(const char* str, const char** str_array, uint8_t size) {
    for (uint8_t i = 0; i < size; i++)
        if (!strcmp(str, str_array[i])) return i;
    return -1;
}

/* There are also built-in functions for these */
uint8_t isdigit(char c) {
   return c >= '0' && c <= '9';
}

uint8_t isspace(char c) {
   return c == ' ';
}


int itoa_s(int value, char *buf) {
        int index = 0;
        int i = value % 10;
        if (value >= 10) {
                index += itoa_s(value / 10, buf);
        }
        buf[index] = i+0x30;
        index++;
        return index;
}

/* Definition of atof */
float atof(char s[]) {
    float val, power;
    int i, sign;

    // Skip initial whitespace
    for (i = 0; isspace(s[i]); i++);

    // Look for sign prefix
    sign = (s[i] == '-') ? -1 : 1;
    if (s[i] == '+' || s[i] == '-') ++i;

    // Compute the value for all digits before the point
    for (val = 0.0; isdigit(s[i]); ++i) {
        val = 10.0 * val + (s[i] - '0');
    }

    // Compute the fractional power for all digits after the point
    if (s[i] == '.') {
        ++i;
    }

    for (power = 1.0; isdigit(s[i]); ++i) {
        val = 10.0 * val + (s[i] - '0');
        power *= 10.0;
    }

    return (sign * val / (power));
}


void ftoa(float value, int decimals, char* buf) {
        int index = 0;
        // Handle negative values
        if (value < 0) {
                buf[index++] = '-';
                value = -value;
        }
        
        // Rounding
        float rounding = 0.5;
        for (int d = 0; d < decimals; rounding /= 10.0, d++);
        value += rounding;

        // Integer part
        index += itoa_s((int)(value), buf+index);
        buf[index++] = '.';

        // Remove everything except the decimals
        value = value - (int)(value);

        // Convert decmial part to integer
        int ival = 1;
        for (int d = 0; d < decimals; ival *= 10, d++);
        ival *= value;

        // Add decimal part to string
        index += itoa_s(ival, buf+index);
        buf[index] = '\0';
}

// Список инструкций
const char* instructions[] = { "set", "get", "watch", "save" };
// Список аргументов для команды set
const char* argSet[] = { "pid-p", "pid-i", "pid-d", "T", "d", "servo", "servo_default", "pid" };
// Список инструкций
const char* argGet[] = { "all", "pid-p", "pid-i", "pid-d", "T", "d", "servo", "servo_default", "sensor", "pid" };
// Список инструкций
const char* argWatch[] = { "sensor", "y", "dy"};
// Количество элементов в массиве строк
#define str_array_size(array) sizeof(array) / sizeof(char*)

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
    char tmp[32];
    struct user_input* user_str;
    uint8_t instr_index = 255;
    uint8_t arg_index = 255;
    uint8_t welcome_flag = 1;
    uint8_t tmp_for_all = 1;
    uint8_t watch_index[10];
    memset(watch_index, 255, 10);
    uint8_t watch_flag = 0;
    uint8_t regulator = 1;

    while (1) {
        // Очищаем текущее значение таймера
        CLEAR_BIT(SysTick->VAL, SysTick_VAL_CURRENT_Msk);
        // Запускаем таймер на 50 мс
        SysCounter = 25;

        if (welcome_flag) {
            // Выводим надпись
            USART1_TxStr("~> ");
            welcome_flag = 0;
        }

        // Обработка команды watch
        watch_flag = 0;
        for (uint8_t i = 0; watch_index[i] != 255; i++) {
            if (watch_flag) USART1_TxStr(",");
            if (watch_index[i] == 0) {
                ftoa(sensorDistanceCM*10, 2, tmp);
                USART1_TxStr(tmp);
            } else if (watch_index[i] == 1) {
                ftoa(y*10, 2, tmp);
                USART1_TxStr(tmp);
            } else if (watch_index[i] == 2) {
                ftoa(dy*10, 2, tmp);
                USART1_TxStr(tmp);
            }
            watch_flag = 1;
        }
        if (watch_flag) USART1_TxStr("\n\r");

        
        P = setPoint - y;
        I = I + P * 0.025;
        D = dy;

        // Если регулятор включен
        if (regulator) servoPosition = pid_p * P + pid_i * I + pid_d * D;
        if (servoPosition > 100) servoPosition = 100;
        else if (servoPosition < -100) servoPosition = -100;

        // Пока ожидаем таймер, обрабатываем USART1
        while(SysCounter) {
            if ((str = USART1_Tick()) != NULL) {
                welcome_flag = 1;
                USART1_TxStr("\n\r");
                
                // Если пришёл символ отмены
                if (!strcmp(str, "c")) {
                    memset(watch_index, 255, 10);
                    continue;
                }

                // Разделяем строку на отдельные слова
                user_str = textProcessing(str);

                // Если ничего не введено
                if (user_str->field_count == 0) {
                    USART1_TxStr("ERROR: You didn't enter anything\n\r");
                    continue;
                }

                // Определяем индекс инструкции
                instr_index = find(user_str->field[0], instructions, str_array_size(instructions));
                if (instr_index == 255) {
                    USART1_TxStr("ERROR: Unknown command \"");
                    USART1_TxStr(user_str->field[0]);
                    USART1_TxStr("\"\n\r");
                    continue;
                }

                // Проходим по всем аргументам
                for (uint8_t i = 1; i < user_str->field_count; i++) {
                    // Если это команда set
                    if (instr_index == 0) {
                        // Если это аргумент
                        if ((i % 2) == 1) {
                            arg_index = find(user_str->field[i], argSet, str_array_size(argSet));
                            if (arg_index == 255) {
                                USART1_TxStr("ERROR: Unknown argument \"");
                                USART1_TxStr(user_str->field[i]);
                                USART1_TxStr("\" for command \"");
                                USART1_TxStr(instructions[instr_index]);
                                USART1_TxStr("\"\n\r");
                                continue;
                            }
                        } 
                        // Если это значение аргумента
                        else {
                            if (!isFloat(user_str->field[i])) {
                                USART1_TxStr("ERROR: Incorrect argument value \"");
                                USART1_TxStr(user_str->field[i]);
                                USART1_TxStr("\"\n\r");
                                continue;
                            }
                            if (arg_index == 0) {
                                pid_p = atof(user_str->field[i]);
                                USART1_TxStr("PID-P is set to ");
                                ftoa(pid_p, 3, tmp);
                                USART1_TxStr(tmp);
                                USART1_TxStr("\n\r");
                            } else if (arg_index == 1) {
                                pid_i = atof(user_str->field[i]);
                                USART1_TxStr("PID-I is set to ");
                                ftoa(pid_i, 3, tmp);
                                USART1_TxStr(tmp);
                                USART1_TxStr("\n\r");
                            } else if (arg_index == 2) {
                                pid_d = atof(user_str->field[i]);
                                USART1_TxStr("PID-D is set to ");
                                ftoa(pid_d, 3, tmp);
                                USART1_TxStr(tmp);
                                USART1_TxStr("\n\r");
                            } else if (arg_index == 3) {
                                T = atof(user_str->field[i]);
                                USART1_TxStr("Filter-T is set to ");
                                ftoa(T, 3, tmp);
                                USART1_TxStr(tmp);
                                USART1_TxStr("\n\r");
                            } else if (arg_index == 4) {
                                d = atof(user_str->field[i]);
                                USART1_TxStr("Filter-d is set to ");
                                ftoa(d, 3, tmp);
                                USART1_TxStr(tmp);
                                USART1_TxStr("\n\r");
                            } else if (arg_index == 5) {
                                servoPosition = (int16_t)atof(user_str->field[i]);
                                USART1_TxStr("Servo is set to ");
                                ftoa(servoPosition, 3, tmp);
                                USART1_TxStr(tmp);
                                USART1_TxStr("\n\r");
                            } else if (arg_index == 6) {
                                servoCenterPosition = (uint16_t)atof(user_str->field[i]);
                                USART1_TxStr("Servo default is set to ");
                                ftoa(servoCenterPosition, 3, tmp);
                                USART1_TxStr(tmp);
                                USART1_TxStr("\n\r");
                            } else if (arg_index == 7) {
                                regulator = (uint16_t)atof(user_str->field[i]);
                                USART1_TxStr("Regulator state is set to ");
                                ftoa(regulator, 3, tmp);
                                USART1_TxStr(tmp);
                                USART1_TxStr("\n\r");
                            }
                        }
                    } 
                    // Если это команда get
                    else if (instr_index == 1) {
                        arg_index = find(user_str->field[i], argGet, str_array_size(argGet));
                        if (arg_index == 255) {
                            USART1_TxStr("ERROR: Unknown argument \"");
                            USART1_TxStr(user_str->field[i]);
                            USART1_TxStr("\" for command \"");
                            USART1_TxStr(instructions[instr_index]);
                            USART1_TxStr("\"\n\r");
                            continue;
                        }
                        tmp_for_all = 1;
                        for (uint8_t i = 0; i < tmp_for_all; i++) {
                            if (tmp_for_all != 1) arg_index = i;
                            if (arg_index == 0) {
                                tmp_for_all = str_array_size(argGet);
                            } else if (arg_index == 1) {
                                USART1_TxStr("PID-P is ");
                                ftoa(pid_p, 3, tmp);
                                USART1_TxStr(tmp);
                                USART1_TxStr("\n\r");
                            } else if (arg_index == 2) {
                                USART1_TxStr("PID-I is ");
                                ftoa(pid_i, 3, tmp);
                                USART1_TxStr(tmp);
                                USART1_TxStr("\n\r");
                            } else if (arg_index == 3) {
                                USART1_TxStr("PID-D is ");
                                ftoa(pid_d, 3, tmp);
                                USART1_TxStr(tmp);
                                USART1_TxStr("\n\r");
                            } else if (arg_index == 4) {
                                USART1_TxStr("Filter-T is ");
                                ftoa(T, 3, tmp);
                                USART1_TxStr(tmp);
                                USART1_TxStr("\n\r");
                            } else if (arg_index == 5) {
                                USART1_TxStr("Filter-d is ");
                                ftoa(d, 3, tmp);
                                USART1_TxStr(tmp);
                                USART1_TxStr("\n\r");
                            } else if (arg_index == 6) {
                                USART1_TxStr("Servo is ");
                                ftoa(servoPosition, 0, tmp);
                                USART1_TxStr(tmp);
                                USART1_TxStr("\n\r");
                            } else if (arg_index == 7) {
                                USART1_TxStr("Servo default is ");
                                ftoa(servoCenterPosition, 0, tmp);
                                USART1_TxStr(tmp);
                                USART1_TxStr("\n\r");
                            } else if (arg_index == 8) {
                                USART1_TxStr("Sensor distance in cm is ");
                                ftoa(sensorDistanceCM, 2, tmp);
                                USART1_TxStr(tmp);
                                USART1_TxStr("\n\r");
                            } else if (arg_index == 9) {
                                USART1_TxStr("Regulator state is ");
                                ftoa(regulator, 2, tmp);
                                USART1_TxStr(tmp);
                                USART1_TxStr("\n\r");
                            }
                        }
                    } 
                    // Если это команда watch
                    else if (instr_index == 2) {
                        arg_index = find(user_str->field[i], argWatch, str_array_size(argWatch));
                        if (arg_index == 255) {
                            USART1_TxStr("ERROR: Unknown argument \"");
                            USART1_TxStr(user_str->field[i]);
                            USART1_TxStr("\" for command \"");
                            USART1_TxStr(instructions[instr_index]);
                            USART1_TxStr("\"\n\r");
                            continue;
                        }
                        // Находим не занятый индекс
                        uint8_t j;
                        for (j = 0; watch_index[j] != 255; j++);
                        watch_index[j] = arg_index;
                    } 
                    // Если это команда save
                    else if (instr_index == 3) {

                    }
                }
            }
        }
    }
}