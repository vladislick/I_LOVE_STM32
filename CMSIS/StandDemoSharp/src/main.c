#include <stm32f1xx.h>
#include <string.h>
#include <stdio.h>
#include <math.h>

#define PROJECT_NAME "StandDemoSharp2Y0A21"
#define SOFTWARE_VERSION "0.3"

// Системная частота (SYSCLK)
#define F 72000000UL
// Максимальная длина строки
#define STR_MAX 256
// Режим отладки (0 - выключено, 1 - числовая информация о расстоянии, 2 - график значений:
// расстояние в мм, положение сервопривода, значение П, значение И, значение Д составляющих
#define DEFAULT_DEBUG_MODE 0
// Размер окна для вычисления интеграла (количество значений)
#define DEFAULT_INTEGRAL_BUFFER_SIZE 256
// Время между замерами расстояния в мс (для аналогового датчика)
#define DEFAULT_SENSOR_UPDATE_TIME 20
// Количество замеров для усреднённого значения
#define DEFAULT_SENSOR_BUFFER_SIZE 20

// Крайнее правое положение желоба
#define DEFAULT_SERVO_RIGHT 250
// Центральное положение желоба
#define DEFAULT_SERVO_CENTER 200
// Крайнее левое положение желоба
#define DEFAULT_SERVO_LEFT 450

#define DEFAULT_PID_P 1.2
#define DEFAULT_PID_I 0.4
#define DEFAULT_PID_D 1.0
#define DEFAULT_PID_m 2.0

/* ========================================================= */
/* ======== Объявляем порты ввода-вывода устройств  ======== */
/* ========================================================= */

// Ножка TRIG ультразвукового датчика HC-SR04
#define HCSR04_TRIG_PIN 3
#define HCSR04_TRIG_PORT PORTB
// Ножка ECHO ультразвукового датчика HC-SR04
#define HCSR04_ECHO_PIN 11
#define HCSR04_ECHO_PORT PORTB
// Ножка USART для приёма данных
#define USART_RX_PIN 10
#define USART_RX_PORT PORTA
// Ножка USART для прередачи данных
#define USART_TX_PIN 9
#define USART_TX_PORT PORTA
// Ножка управелния сервопривода
#define SERVO_PIN 15
#define SERVO_PORT PORTC


// Системный счётчик (уменьшается каждую миллисекунду)
volatile uint32_t SysCounter;
// Счётчик импульсов сервопривода 
volatile uint16_t servoCounter;
// Текущее положение сервопривода (тиков таймера)
volatile int16_t servoPosition;
// Сглаживающий значение АЦП массив
volatile uint32_t smoothValue;
// Текущий индекс сглаживающего массива
volatile uint16_t smoothIndex;
// Сглаженное значение АЦП
volatile uint16_t smoothValueReady;

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

////////////////////////////////////////////////////////////////////////////////
////////////  Объявление структур для инструкций и их аргументов  //////////////
////////////////////////////////////////////////////////////////////////////////

// Структура аргумента для инструкции
struct argument
{
    // Имя аргумента (например, "--default")
    const char* name;
    // Описание аргумента (за что отвечает)
    const char* description;
    // Флаговый регистр, описывает тип значений аргумента (подробнее ниже)
    uint8_t flag;
    // Минимальное разрешённое значение
    float min;
    // Максимальное разрешённое значение
    float max;
    // Строка ссылок значений (ссылается на существующее значение)
    const char** links;
    // Строка описания ссылок
    const char** links_description;
};

// Структура инструкции
struct instruction
{
    // Имя инструкции (например, "set")
    const char* name;
    // Описание инструкции
    const char* description;
    // Количество аргументов иструкции 
    uint8_t args_count;
    // Указатель на массив с аргументами для данной инструкции
    const struct argument* args;
};

////////////////////////////////////////////////////////////////////////////////
////////////////          Описание самих инструкций             ////////////////
////////////////////////////////////////////////////////////////////////////////

// Список доступных команд
const char* instructions_list[] = { "set", "get", "controller" };
// Описание каждой команды
const char* instructions_description[] = {  
    // Команда "set" (индекс 0) 
    "to change any parameter", 
    // Команда "get" (индекс 1)
    "to get the current value of some parameter",  
    // Команда "controller" (индекс 2)
    "to change or show state of the PID controller"
};

////////////////////////////////////////////////////////////////////////////////
////////////////       Описание аргументов инструкций           ////////////////
////////////////////////////////////////////////////////////////////////////////

// Количество аргументов у каждой команды
const uint8_t instructions_args_count[] = {
    // Кол-во аргуметов у команды "set" (индекс 0)
    3,
    // Кол-во аргуметов у команды "get" (индекс 1)
    3,
    // Кол-во аргуметов у команды "controller" (индекс 2)
    3
};

// Названия аргументов
const char* instructions_args[] = {
    // Команда "set" (индекс 0)
    "--pid-p",
    "--pid-i",
    "--pid-d",
    // Команда "get" (индекс 1)
    "--pid-p",
    "--pid-i",
    "--pid-d",
    // Команда "controller" (индекс 2)
    "off",
    "on",
    "status"
};

// Описания аргументов
const char* instructions_args_description[] = {
    // Команда "set" (индекс 0)
    "access to parameter P of the PID controller", // --pid-p
    "access to parameter I of the PID controller", // --pid-i
    "access to parameter D of the PID controller", // --pid-d
    // Команда "get" (индекс 1)
    "access to parameter P of the PID controller", // --pid-p
    "access to parameter I of the PID controller", // --pid-i
    "access to parameter D of the PID controller",  // --pid-d
    // Команда "controller" (индекс 2)
    "power off the PID controller, manual mode is available", // off
    "power on the PID controller, manual mode is unavailable", // on
    "show information of the current status of PID controller"  // status
};



////////////////////////////////////////////////////////////////////////////////
////////////////    Описание значений аргументов инструкций     ////////////////
////////////////////////////////////////////////////////////////////////////////

// Описание всех флаговых регистров для аргументов
// Биты 0-4 - сколько константных значений имеется (текстовых)
// Бит 5 - является ли беззнаковым
// Бит 6 - требуется ли действительно дробное значение
// Бит 7 - принимает ли вообще числовые значения
// Если все биты оставить в нуле, это означает, что аргументу 
// не требуется значение
// Биты расколожены так: 0b76543210
const uint8_t args_flags[] = {
    // Команда "set" (индекс 0)
    0b11000001, // --pid-p
    0b11000001, // --pid-i
    0b11000001, // --pid-d
    // Команда "get" (индекс 1)
    0b00000000, // --pid-p
    0b00000000, // --pid-i
    0b00000000,  // --pid-d
    // Команда "controller" (индекс 2)
    0b00000000, // off
    0b00000000, // on
    0b00000000  // status
};

// Описание всех минимальных значений для аргументов
// Стандартным значением (ограничение максимального значения не требуется)
// является -1000000000.0 (минус один миллиард)
const float args_min[] = {
    // Команда "set" (индекс 0)
    -1000000000, // --pid-p
    -1000000000, // --pid-i
    -1000000000, // --pid-d
    // Команда "get" (индекс 1)
    -1000000000, // --pid-p
    -1000000000, // --pid-i
    -1000000000,  // --pid-d
    // Команда "controller" (индекс 2)
    -1000000000, // off
    -1000000000, // on
    -1000000000  // status
};

// Описание всех максимальных значений для аргументов
// Стандартным значением (ограничение максимального значения не требуется)
// является 1000000000.0 (один миллиард)
const float args_max[] = {
    // Команда "set" (индекс 0)
    1000000000, // --pid-p
    1000000000, // --pid-i
    1000000000, // --pid-d
    // Команда "get" (индекс 1)
    1000000000, // --pid-p
    1000000000, // --pid-i
    1000000000,  // --pid-d
    // Команда "controller" (индекс 2)
    1000000000, // off
    1000000000, // on
    1000000000  // status
};

// Пример ссылок для аргумента инструкции
//const char* args_links_for_something[] = { "default", "servo-max" };

const char* args_links_default[] = { "default" };

// Названия всех ссылок у значений аргументов (пустая строка = ссылок нет)
const char** args_links[] = {
    // Команда "set" (индекс 0)
    args_links_default, // --pid-p
    args_links_default, // --pid-i
    args_links_default, // --pid-d
    // Команда "get" (индекс 1)
    NULL, // --pid-p
    NULL, // --pid-i
    NULL,  // --pid-d
    // Команда "controller" (индекс 2)
    NULL, // off
    NULL, // on
    NULL  // status
    //args_links_for_something
};

// Пример описания ссылок для аргумента инструкции
//const char* args_links_for_something_description[] = { "default", "servo-max" };

const char* args_links_default_description[] = { "use the default value for the argument" };

// Описания всех ссылок у значений аргументов (пустая строка = описания нет)
const char** args_links_description[] = {
    // Команда "set" (индекс 0)
    args_links_default_description, // --pid-p
    args_links_default_description, // --pid-i
    args_links_default_description, // --pid-d
    // Команда "get" (индекс 1)
    NULL, // --pid-p
    NULL, // --pid-i
    NULL,  // --pid-d
    // Команда "controller" (индекс 2)
    NULL, // off
    NULL, // on
    NULL  // status
    //args_links_for_something_description
};

////////////////////////////////////////////////////////////////////////////////
////////////////        Создание необходимых структур           ////////////////
////////////////////////////////////////////////////////////////////////////////

// Общее количество инструкций
#define INSTR_COUNT sizeof(instructions_list) / sizeof(const char*)
// Массив с описанием всех доступных инструкций
struct instruction instr[INSTR_COUNT];

// Общее количество аргументов
#define INSTR_ARG_COUNT sizeof(instructions_args) / sizeof(const char*)
// Массив с описанием всех доступных аргументов инструкций
struct argument arg[INSTR_ARG_COUNT];

// Временная строка 1
char tmp[STR_MAX];
// Временная строка 2
char tmp2[STR_MAX];

////////////////////////////////////////////////////////////////////////////////
//////////     Заполнение данных об инструкциях и их аргументах      ///////////
////////////////////////////////////////////////////////////////////////////////

// Инициализация списков инструкций и их аргументов
void INTRUCTIONS_Init() {
    // Счетчик аргументов
    uint8_t instr_arg_counter = 0;

    // Заполнение структур аргументов
    for (uint8_t i = 0; i < INSTR_ARG_COUNT; i++) {
        arg[i].name = instructions_args[i];
        arg[i].description = instructions_args_description[i];
        arg[i].flag = args_flags[i];
        arg[i].min = args_min[i];
        arg[i].max = args_max[i];
        arg[i].links = args_links[i];
        arg[i].links_description = args_links_description[i];
    }

    // Заполнение структур инструкций
    for (uint8_t i = 0; i < INSTR_COUNT; i++) {
        instr[i].name = instructions_list[i];
        instr[i].description = instructions_description[i];
        instr[i].args_count = instructions_args_count[i];
        instr[i].args = arg + instr_arg_counter; // Смещение
        instr_arg_counter += instr[i].args_count;
    }
}

////////////////////////////////////////////////////////////////////////////////
//////////                Аппаратные настройки STM32                 ///////////
////////////////////////////////////////////////////////////////////////////////

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
    // Настраиваем пин 13 порта B на выход (Servo)
    MODIFY_REG(GPIOB->CRH, GPIO_CRH_CNF13|GPIO_CRH_MODE13, GPIO_CRH_MODE13_0);
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
    while(READ_BIT(~USART1->SR, USART_SR_TXE));
    // Передаём символ модулю передачи USART1
    USART1->DR = ch;
}

// Передать строку по USART1
void USART1_TxStr(const char* str) {
    // Отправляем символы, пока не встретим символ конца строки
    // или счётчик не переполнится
    for (uint16_t i = 0; str[i] != '\0' && i < STR_MAX; i++) {
        // Ожидаем готовности модуля передачи USART1
        while(READ_BIT(~USART1->SR, USART_SR_TXE));
        // Передаём символ модулю передачи USART1
        USART1->DR = str[i];
    }
}

// Получить строку по USART1
uint16_t USART1_RxStr(char* str) {
    // Переменная для хранения количества считанных символов
    uint16_t i = 0;
    // Очистка строки
    memset(str, '\0', STR_MAX);
    // Пока есть куда сохранять
    while (i < STR_MAX) {
        // Запускаем обратный отсчёт в мс
        SysCounter = 30;
        // Ожидать приема символа (если линия не освободилась)
        while(READ_BIT(~USART1->SR, USART_SR_RXNE))
            if (i != 0 && SysCounter == 0) return i;
        // Считываем символ
        str[i++] = USART1->DR;
    } 
    return i;
}


// Инициализация таймера 2
void TIM2_Init() {
    // Выключаем таймер 2
    TIM_DisableCounter(TIM2);
    // Включаем тактирование таймера
    SET_BIT(RCC->APB1ENR, RCC_APB1ENR_TIM2EN);
    // Включаем глобальное прерывание таймера
    //NVIC_EnableIRQ(TIM2_IRQn);main_testing.c:(.text.SysTick_Init+0x0): multiple definition of `SysTick_Init'
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
        if (servoCounter > servoPosition) GPIOB->BSRR = GPIO_BSRR_BR13;
        servoCounter++;
    } else {
        servoCounter = 0;
        // Подаём импульс на сервопривод
        GPIOB->BSRR = GPIO_BSRR_BS13;
    }
}

// Прерывание АЦП
void ADC1_2_IRQHandler() {
    
    // Запускаем преобразование АЦП
    SET_BIT(ADC1->CR2, ADC_CR2_SWSTART);
}

// Реализация задержки в мс
void my_delay_ms(uint32_t time) {
    MODIFY_REG(SysTick->VAL, SysTick_VAL_CURRENT_Msk, F / 1000 - 1);
    SysCounter = time;
    while(SysCounter);
}

// Перевести в нижний регистр
char tolowercase(char ch) {
    if (ch > 64 && ch < 91) return ch+32;
    else return ch;
}

int16_t my_abs(int16_t value) {
    if (value < 0) return 0-value;
    else return value;
}

// Является ли str строкой
uint8_t isString(const char* str) {
    for (uint16_t i = 0; str[i] != '\0'; i++)
        if ((str[i] > 122 || str[i] < 97) && str[i] != '_' && str[i] != ' ') return 0;
    return 1;
}

// Является ли str целым числом
uint8_t isInteger(const char* str) {
    for (uint16_t i = 0; str[i] != '\0'; i++)
        if ((str[i] < 48 || str[i] > 57) && str[i] != '-') return 0;
    return 1;
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

// Повернуть сервопривод на угол, -200 <= angle <= 200
void servoSet(int16_t angle) {
    // Проверка на дурака
    if (angle > 100 || angle < -100) return;
    
    int16_t step = 2;
    int16_t diff = servoPosition - (DEFAULT_SERVO_CENTER + angle);
    if (my_abs(diff) >= step) {
        if (diff > 0) servoPosition -= step;
        else if (diff < 0) servoPosition += step;
    } else if (diff != 0) {
        if (diff > 0) servoPosition -= diff;
        else if (diff < 0) servoPosition += diff;
    } 
    //servoPosition = (SERVO_CENTER + angle);
}



// Вывод приветствующего сообщения
void UI_Init() {
    // Небольшая пауза
    my_delay_ms(500);
    // Вывод сообщения 
    USART1_TxStr("Welcome to the" PROJECT_NAME "v" SOFTWARE_VERSION "!\n"
                 "Type commands below. Hints are displayed automatically if the"
                 " input is incorrect.\nFor more help just type help.\n");
}

// Ввод данных от пользователя
void UI_Enter(char* str) {
    // Счетчик введенных символов
    uint16_t tmpCount = 0;
    // Строка
    char tmp[STR_MAX];
    // Очистка строки
    memset(str, '\0', STR_MAX);
    // Вывод знака ввода пользователю
    USART1_TxStr("~> ");
    // Чтение посимвольно или целиком
    while(tmpCount < STR_MAX) {
        if (USART1_RxStr(tmp) > 1 && tmp[0] != '\r') {
            strcpy(str, tmp);
            USART1_TxStr(tmp);
            break;
        } else {
            if (tmp[0] == '\r') break;
            str[tmpCount++] = tmp[0];
            USART1_Tx(tmp[0]);
        }
    }
    // Перевод в нижний регистр
    for (uint16_t i = 0; str[i] != '\0'; i++)
                str[i] = tolowercase(str[i]);
    // Завершаем строку ввода данных
    USART1_Tx('\n');
}

// Проверяет, существует ли такая инструкция
// Выводит список возможных, если указанной инструкции не существует
// Возвращает индекс инструкции, если найдена, -1 если такой, но есть возможные,
// -2 если не совпало и нет похожих по названию
int16_t isInstructionValid(char* dest_list, const char* instr_str) {
    // Очистка переменной
    memset(dest_list, '\0', STR_MAX);
    // Поиск данной инструкции
    for (uint8_t i = 0; i < INSTR_COUNT; i++) {
        // Если текст полностью совпадает с названием инструкции
        if (strcmp(instr_str, instr[i].name) == 0) return i;
        // Если попалась похожая по названию инструкция
        else if (strstr(instr[i].name, instr_str) != NULL) {
            strcat(dest_list, instr[i].name);
            strcat(dest_list, " (");
            strcat(dest_list, instr[i].description);
            strcat(dest_list, ")\n");
        }
    }
    // Возвращаем получившийся результат
    if (dest_list[0] != '\0') return -1;
    else return -2;
}

// Проверяет, существует ли такой аргумент у инструкции с индексом instr_index
// Выводит список возможных, если указанного аргумента не существует
// Возвращает индекс аргумента, если найден, -1 если не найден, но есть возможные,
// -2 если не совпало и нет похожих по названию
int16_t isArgumentValid(char* dest_list, const char* arg_str, uint8_t instr_index) {
    // Очистка переменной
    memset(dest_list, '\0', STR_MAX);
    // Поиск полного совпадения названия
    for (uint8_t i = 0; i < instr[instr_index].args_count; i++)
        // Если совпадение найдено
        if (strcmp(arg_str, instr[instr_index].args[i].name) == 0) return i;
        // Если найден похожий по названию аргумент
        else if (strstr(instr[instr_index].args[i].name, arg_str) != NULL) {
            strcat(dest_list, instr[instr_index].args[i].name);
            strcat(dest_list, " (");
            strcat(dest_list, instr[instr_index].args[i].description);
            strcat(dest_list, ")\n");
        }
    // Возвращаем получившийся результат
    if (dest_list[0] != '\0') return -1;
    else return -2;
}

// Проверяет, допустим ли аргумент 
int16_t isArgumentValueValid(char* dest_list, const char* arg_value_str, uint8_t instr_index, uint8_t arg_index) {
    // Достаем флаговый регистр из аргумента
    uint8_t arg_flag = instr[instr_index].args[arg_index].flag;
    // Очистка переменной
    memset(dest_list, '\0', STR_MAX);

    // Если это строка
    if (isString(arg_value_str)) {
        // Если у аргумента есть текстовые значения
        if (arg_flag & (0b11111)) {
            // Поиск полного совпадения названия
            for (uint8_t i = 0; i < (arg_flag & 0b11111); i++)
                // Если совпадение найдено
                if (strcmp(instr[instr_index].args[arg_index].links[i], arg_value_str) == 0) return 1;
                // Если попалась похожая по названию
                else if (strstr(instr[instr_index].args[arg_index].links[i], arg_value_str) != NULL) {
                    strcat(dest_list, instr[instr_index].args[arg_index].links[i]);
                    strcat(dest_list, " (");
                    strcat(dest_list, instr[instr_index].args[arg_index].links_description[i]);
                    strcat(dest_list, ")\n");
                }
            // Добавляем в конец точку, если были совпадения
            // Возвращаем получившийся результат
            if (dest_list[0] != '\0') return 2;
        }
        return 0;
    } 
    // Если аргумент принимает числовые значения
    else if (arg_flag & (1 << 7)) {
        // Если необходимо дробное значение
        if ((arg_flag & (1 << 6)) && isFloat(arg_value_str)) {
            // Достаем значение
            float value; sscanf(arg_value_str, "%f", &value);
            // Проверяем диапазон
            if (value >= instr[instr_index].args[arg_index].min &&
                value <= instr[instr_index].args[arg_index].max) return 1;
            else return 3;
        } 
        // Если необходимо целочисленное значение
        else if (((arg_flag & (1 << 6)) == 0) && isInteger(arg_value_str)) {
            // Достаем значение
            int32_t value; sscanf(arg_value_str, "%li", &value);
            // Проверяем диапазон
            if (value >= roundf(instr[instr_index].args[arg_index].min) &&
                value <= roundf(instr[instr_index].args[arg_index].max)) return 1;
            else return 3;
        }
    }
    return 0;
}

// Печатает в dest_list допустимые значения аргумента с индексом arg_index
void sprintArgValuesInfo(char* dest_list, const struct instruction* instr_list, uint8_t instr_index, uint8_t arg_index) {
    // Очистка переменной
    memset(dest_list, '\0', STR_MAX);
    // Достаем флаговый регистр из аргумента
    uint8_t arg_flag = instr_list[instr_index].args[arg_index].flag;
    
    strcat(dest_list, "Argument \"");
    strcat(dest_list, instr_list[instr_index].args[arg_index].name);
    
    // Принимает ли данный аргумент какие-либо значения
    if (arg_flag == 0) {
        strcat(dest_list, "\" does not require any values.");
    } else {
        strcat(dest_list, "\" accepts");
        // Принимает ли числа
        if (arg_flag & (1 << 7)) {
            // Принимает ли только положительные значения
            if (arg_flag & (1 << 5))
                strcat(dest_list, " unsigned");
            else 
                strcat(dest_list, " signed");
            if (arg_flag & (1 << 6))
                strcat(dest_list, " float");
            else 
                strcat(dest_list, " integer");
            if (roundf(instr_list[instr_index].args[arg_index].min) != -1000000000.0 && !(arg_flag & (1 << 5)))
            {
                strcat(dest_list, " from ");
                char tmp[32];
                sprintf(tmp, "%.3f", instr_list[instr_index].args[arg_index].min);
            }
            if (roundf(instr_list[instr_index].args[arg_index].max) != 1000000000.0)
            {
                strcat(dest_list, " up to ");
                char tmp[32];
                sprintf(tmp, "%.3f", instr_list[instr_index].args[arg_index].max);
            }
        }
        // Если аргумент принимает текстовые значения
        if (arg_flag & (0b11111)) {
            // Если до этого указывались числовые значения
            if (arg_flag & (1 << 7)) strcat(dest_list, " or");
            strcat(dest_list, " text value:\n");
            // Проходим по всем текстовым значениям
            for (uint8_t i = 0; i < (arg_flag & 0b11111); i++) {
                strcat(dest_list, instr_list[instr_index].args[arg_index].links[i]);
                strcat(dest_list, " (");
                strcat(dest_list, instr_list[instr_index].args[arg_index].links_description[i]);
                strcat(dest_list, ")\n");
            }
        }
    }
}

// Проверка введенной пользователем команды
// Возвращает -1, если такой команды нет
// Возвращает -2, если команды нет, но есть похожие
// Возвращает индекс команды, если таковая есть
int8_t UI_CheckCommand(const char* text) {
    // Достаем только команду
    parsing(tmp, text, ' ', 0);
    // Пробуем узнать индекс команды
    int16_t instr_index = isInstructionValid(tmp2, tmp);
    // Если ничего не введено
    if (tmp[0] == '\0') {
        USART1_TxStr("ERROR: You didn't enter anything. You must use instructions below:\n");
        USART1_TxStr(tmp2);
    }
    // Если такой команды не существует, но есть похожие по названию
    else if (instr_index == -1) {
        USART1_TxStr("ERROR: \"");
        USART1_TxStr(tmp);
        USART1_TxStr("\" instruction does not exist. But maybe you meant:\n");
        USART1_TxStr(tmp2);
    } 
    // Если такой команды вообще не существует
    else if (instr_index == -2) {
        USART1_TxStr("ERROR: \"");
        USART1_TxStr(tmp);
        USART1_TxStr("\" instruction does not exist. You must use instructions below:\n");
        isInstructionValid(tmp2, "");
        USART1_TxStr(tmp2);
    }
    // Возвратить индекс команды
    return instr_index;
}

// Проверка всех аргументов введенной команды 
uint8_t UI_CheckArgs(const char* text, uint8_t instr_index) {
    // Необходимо ли значение аргумента
    uint8_t valueNeeded = 0;
    // Индекс аргумента
    int16_t arg_index;
    // Если у рассматриваемой команды должны быть аргументы
    if (instr[instr_index].args_count != 0) {
        // Если к инструкции не приведены аргументы, а должны
        if (parsingCount(text, ' ') < 2) {
            isArgumentValid(tmp2, "", instr_index);
            USART1_TxStr("ERROR: There is no argument for the instruction. Available arguments for \"");
            USART1_TxStr(instr[instr_index].name);
            USART1_TxStr("\" is:\n");
            USART1_TxStr(tmp2);
            return 0;
        }
        // Проходим по всем имеющимся аргументам (или значениям аргументов)
        for (uint8_t i = 1; i < parsingCount(text, ' '); i++) {
            // Получаем следующее слово из строки, введенной пользователем
            parsing(tmp, text, ' ', i);
            
            // Если текущее считанное слово должно быть значением аргумента
            if (valueNeeded) {
                int16_t arg_value_code = isArgumentValueValid(tmp2, tmp, instr_index, arg_index);
                // Если это текстовое значение аргумента, которому есть похожая ссылка
                if (arg_value_code == 2) {
                    USART1_TxStr("ERROR: Argument value \"");
                    USART1_TxStr(tmp);
                    USART1_TxStr("\" is non valid, but maybe you meant:\n");
                    USART1_TxStr(tmp2);
                    return 0;
                } 
                // Если это числовое значение аргумента, но не подходящее по диапазону
                else if (arg_value_code == 3) {
                    USART1_TxStr("ERROR: Argument value \"");
                    USART1_TxStr(tmp);
                    sprintArgValuesInfo(tmp2, instr, instr_index, arg_index);
                    USART1_TxStr("\" is out of available range.\n");
                    USART1_TxStr(tmp2);
                    return 0;
                } else if (arg_value_code == 0) {
                    if (isArgumentValid(tmp2, tmp, instr_index) >= 0) {
                        USART1_TxStr("ERROR: There is no value for argument \"");
                        USART1_TxStr(instr[instr_index].args[arg_index].name);
                        USART1_TxStr("\".\n");
                        sprintArgValuesInfo(tmp2, instr, instr_index, arg_index);
                        USART1_TxStr(tmp2);
                    } else {
                        USART1_TxStr("ERROR: Argument value \"");
                        USART1_TxStr(tmp);
                        USART1_TxStr("\" is non valid.\n");
                    }
                    return 0;
                } else {
                    valueNeeded = 0;
                    continue;
                }
            }
            
            // Пробуем узнать индекс аргумента
            arg_index = isArgumentValid(tmp2, tmp, instr_index);
            // Если такого аргумента не существует
            if (arg_index == -1) {
                USART1_TxStr("ERROR: Argument \"");
                USART1_TxStr(tmp);
                USART1_TxStr("\" is non valid, but maybe you meant:\n");
                USART1_TxStr(tmp2);
                return 0;
            } else if (arg_index == -2) {
                USART1_TxStr("ERROR: Argument \"");
                USART1_TxStr(tmp);
                USART1_TxStr("\" is non valid.\n");
                return 0;
            }

            // Если аргумент существует и ему требуется значение
            if (instr[instr_index].args[arg_index].flag > 0) valueNeeded = 1;
        }

        if (valueNeeded) {
            USART1_TxStr("ERROR: There is no value for argument \"");
            USART1_TxStr(instr[instr_index].args[arg_index].name);
            USART1_TxStr("\".\n");
            sprintArgValuesInfo(tmp2, instr, instr_index, arg_index);
            USART1_TxStr(tmp2);
            return 0;
        }
    }
    return 1;
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
    // Приветствие
    UI_Init();
    // Инициализируем список инструкций и их аргументов
    INTRUCTIONS_Init();
    
    // Временная строка
    char user[STR_MAX];

    /*// Временная строка
    char tmp[STR_MAX];
    // Временная строка
    char tmp2[STR_MAX];
    // Фактическая строка, присланная пользователем
    char userAnswer[STR_MAX];
    // Строка для отправки ответа пользователю
    char myAnswer[STR_MAX];*/

    while(1) { 
        // Ввод данных пользователем
        UI_Enter(user);

        // Проверка существования такой команды
        int8_t instr_index = UI_CheckCommand(user);
        if (instr_index < 0) continue;

        // Проверка аргументов этой команды
        if (!UI_CheckArgs(user, instr_index)) continue;

        USART1_TxStr("Done.\n");
    }
}