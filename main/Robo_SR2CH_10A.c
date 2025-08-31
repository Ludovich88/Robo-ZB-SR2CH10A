// Подключаем стандартные библиотеки C для работы с строками, памятью и математикой
#include <stdio.h>          // Для функций printf, sprintf, scanf и т.д.
#include <string.h>         // Для функций работы со строками (strcpy, strlen, strcmp)
#include <stdlib.h>         // Для функций malloc, free, atoi и т.д.

// Подключаем FreeRTOS библиотеки для многозадачности и управления памятью
#include <freertos/FreeRTOS.h>      // Основные определения FreeRTOS (типы, константы)
#include <freertos/task.h>          // Функции для создания и управления задачами
#include <freertos/queue.h>         // Функции для работы с очередями сообщений
#include <freertos/timers.h>        // Функции для работы с таймерами

// Подключаем драйверы ESP-IDF для работы с железом
#include <driver/gpio.h>            // Драйвер для работы с GPIO пинами
#include <driver/uart.h>            // Драйвер для работы с UART интерфейсом

// Подключаем системные библиотеки ESP-IDF
#include <esp_log.h>                // Система логирования ESP-IDF
#include <esp_system.h>             // Системные функции (перезагрузка, информация о системе)
#include <esp_timer.h>              // Высокоточные таймеры для измерения времени

// Подключаем библиотеки для работы с памятью и Zigbee
#include <nvs_flash.h>              // Non-Volatile Storage - постоянная память
#include <esp_zigbee_core.h>        // Основная библиотека Zigbee для ESP32

// Определяем тег для логирования - будет отображаться в начале каждой строки лога
static const char *TAG = "SMART_RELAY";

// ============================================================================
// ОПРЕДЕЛЕНИЯ GPIO ПИНОВ - каждый пин имеет конкретное назначение
// ============================================================================

// IO0 - кнопка для запуска процесса pairing (подключения к Zigbee сети)
#define BUTTON_PAIRING_GPIO     GPIO_NUM_0    

// IO1 - светодиод индикации состояния связи (включен/выключен)
#define LED_STATUS_GPIO         GPIO_NUM_1    

// IO2 - сигнал zero cross от датчика пересечения нуля AC напряжения
#define ZERO_CROSS_GPIO         GPIO_NUM_2    

// IO4 - прием данных от микросхемы энергомониторинга BL0940
#define UART_RX_GPIO            GPIO_NUM_4    

// IO5 - передача команд к микросхеме энергомониторинга BL0940
#define UART_TX_GPIO            GPIO_NUM_5    

// IO14 - внешний переключатель для управления реле 1
#define SWITCH_1_GPIO           GPIO_NUM_14   

// IO15 - внешний переключатель для управления реле 2
#define SWITCH_2_GPIO           GPIO_NUM_15   

// IO18 - управление реле номер 2 (включение/выключение нагрузки)
#define RELAY_2_GPIO            GPIO_NUM_18   

// IO19 - управление реле номер 1 (включение/выключение нагрузки)
#define RELAY_1_GPIO            GPIO_NUM_19   

// ============================================================================
// КОНФИГУРАЦИЯ UART ДЛЯ BL0940
// ============================================================================

// Используем UART1 для связи с BL0940
#define UART_NUM                UART_NUM_1

// Скорость передачи данных - BL0940 работает на 4800 бод
#define UART_BAUD_RATE          4800

// Размер буфера для приема данных от BL0940
#define UART_BUF_SIZE           1024

// ============================================================================
// КОНФИГУРАЦИЯ КНОПКИ - временные интервалы для различных типов нажатий
// ============================================================================

// Время длинного нажатия в миллисекундах - для входа в режим pairing
#define BUTTON_LONG_PRESS_TIME  5000          

// Максимальное время нажатия в миллисекундах - для выхода из режима pairing
#define BUTTON_MAX_PRESS_TIME   10000         

// Время между кликами для определения двойного нажатия в миллисекундах
#define BUTTON_DOUBLE_CLICK_TIME 300          

// ============================================================================
// КОНФИГУРАЦИЯ LED - простое управление через GPIO (включен/выключен)
// ============================================================================

// Уровень для включения светодиода (HIGH)
#define LED_ON                  1

// Уровень для выключения светодиода (LOW)
#define LED_OFF                 0

// ============================================================================
// ГЛОБАЛЬНЫЕ ПЕРЕМЕННЫЕ - состояние устройства и данные
// ============================================================================

// Состояние реле 1: false = выключено, true = включено
static bool relay_1_state = false;

// Состояние реле 2: false = выключено, true = включено
static bool relay_2_state = false;

// Режим pairing: false = обычный режим, true = поиск Zigbee сети
static bool pairing_mode = false;

// Накопленная энергия в милливатт-часах (mWh)
static uint32_t energy_consumption = 0;

// Текущая потребляемая мощность в милливаттах (mW)
static uint32_t power_consumption = 0;

// Действующее напряжение в милливольтах (mV)
static uint32_t voltage_rms = 0;

// Действующий ток в миллиамперах (mA)
static uint32_t current_rms = 0;

// ============================================================================
// ПЕРЕМЕННЫЕ ДЛЯ ОБРАБОТКИ КНОПКИ - отслеживание состояния нажатий
// ============================================================================

// Время начала нажатия кнопки в миллисекундах
static uint32_t button_press_start_time = 0;

// Флаг нажатия кнопки: false = не нажата, true = нажата
static bool button_pressed = false;

// Время последнего отпускания кнопки в миллисекундах
static uint32_t last_button_release_time = 0;

// Счетчик кликов кнопки: 0, 1 или 2
static uint8_t button_click_count = 0;

// Указатель на таймер для обработки длинных нажатий
static TimerHandle_t button_timer = NULL;

// ============================================================================
// ОЧЕРЕДИ И ТАЙМЕРЫ - для передачи сообщений между задачами
// ============================================================================

// Очередь для передачи событий GPIO между прерыванием и задачей
static QueueHandle_t gpio_evt_queue;

// ============================================================================
// ПЕРЕМЕННЫЕ ZIGBEE - состояние подключения к сети
// ============================================================================

// Флаг подключения к Zigbee сети: false = не подключен, true = подключен
static bool zigbee_joined = false;

// Флаг готовности Zigbee стека: false = не готов, true = готов к работе
static bool zigbee_ready = false;

// ============================================================================
// ПРОТОТИПЫ ФУНКЦИЙ - объявления функций перед их реализацией
// ============================================================================

// Функции инициализации оборудования
static void gpio_init(void);        // Инициализация GPIO пинов
static void uart_init(void);        // Инициализация UART для BL0940
static void ledc_init(void);        // Инициализация LED (простое GPIO управление)

// Функции обработки прерываний
static void gpio_isr_handler(void *arg);        // Обработчик прерываний GPIO
static void zero_cross_isr_handler(void *arg);  // Обработчик прерываний zero cross

// Задачи FreeRTOS
static void status_led_task(void *pvParameter);     // Задача управления LED
static void energy_monitoring_task(void *pvParameter); // Задача энергомониторинга
static void button_task(void *pvParameter);         // Задача обработки кнопки

// Функции таймеров
static void button_timer_callback(TimerHandle_t xTimer); // Обработчик таймера кнопки

// Вспомогательные функции
static void toggle_relay(uint8_t relay_num, bool state); // Переключение реле
static void handle_button_press(void);                   // Обработка нажатия кнопки
static void handle_button_release(void);                 // Обработка отпускания кнопки

// Функции Zigbee
static void zigbee_init(void);                    // Инициализация Zigbee стека
static void zigbee_start(void);                   // Запуск Zigbee стека
static void zigbee_task(void *pvParameter);       // Задача управления Zigbee
static void zigbee_commissioning_start(void);     // Запуск процесса pairing
static void zigbee_commissioning_stop(void);      // Остановка процесса pairing

// ============================================================================
// ОБРАБОТЧИК ПРЕРЫВАНИЙ GPIO - вызывается при изменении состояния пинов
// ============================================================================

// IRAM_ATTR означает, что функция должна быть размещена в RAM, а не в Flash
// Это необходимо для прерываний, так как Flash может быть недоступен
static void IRAM_ATTR gpio_isr_handler(void *arg)
{
    // Получаем номер GPIO пина, который вызвал прерывание
    uint32_t gpio_num = (uint32_t) arg;
    
    // Отправляем номер пина в очередь для обработки в основной задаче
    // xQueueSendFromISR используется для отправки из прерывания
    // NULL означает, что мы не хотим получать уведомление о результате
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

// ============================================================================
// ОБРАБОТЧИК ПРЕРЫВАНИЙ ZERO CROSS - для синхронизации с AC напряжением
// ============================================================================

static void IRAM_ATTR zero_cross_isr_handler(void *arg)
{
    // Zero cross обнаружен - AC напряжение пересекло нулевую точку
    // Здесь можно добавить логику для фазового управления нагрузкой
    // Например, включение реле в определенный момент синусоиды
    // Пока что функция пустая, но готова для расширения
}

// ============================================================================
// ИНИЦИАЛИЗАЦИЯ GPIO ПИНОВ - настройка всех пинов для работы
// ============================================================================

static void gpio_init(void)
{
    // Конфигурация кнопки pairing - вход с подтяжкой к питанию
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_ANYEDGE,    // Прерывание при любом изменении (нажатие/отпускание)
        .mode = GPIO_MODE_INPUT,           // Режим входа
        .pin_bit_mask = (1ULL << BUTTON_PAIRING_GPIO), // Маска для IO0
        .pull_down_en = 0,                // Отключаем подтяжку к земле
        .pull_up_en = 1,                  // Включаем подтяжку к питанию (активный низкий)
    };
    gpio_config(&io_conf);                // Применяем конфигурацию

    // Конфигурация LED статуса - выход без прерываний
    gpio_config_t led_conf = {
        .intr_type = GPIO_INTR_DISABLE,   // Отключаем прерывания для LED
        .mode = GPIO_MODE_OUTPUT,         // Режим выхода
        .pin_bit_mask = (1ULL << LED_STATUS_GPIO), // Маска для IO1
        .pull_down_en = 0,                // Отключаем подтяжку к земле
        .pull_up_en = 0,                  // Отключаем подтяжку к питанию
    };
    gpio_config(&led_conf);               // Применяем конфигурацию

    // Конфигурация реле - выходы без прерываний
    gpio_config_t relay_conf = {
        .intr_type = GPIO_INTR_DISABLE,   // Отключаем прерывания для реле
        .mode = GPIO_MODE_OUTPUT,         // Режим выхода
        .pin_bit_mask = (1ULL << RELAY_1_GPIO) | (1ULL << RELAY_2_GPIO), // IO18 и IO19
        .pull_down_en = 0,                // Отключаем подтяжку к земле
        .pull_up_en = 0,                  // Отключаем подтяжку к питанию
    };
    gpio_config(&relay_conf);             // Применяем конфигурацию

    // Конфигурация внешних переключателей - входы с прерываниями
    gpio_config_t switch_conf = {
        .intr_type = GPIO_INTR_NEGEDGE,   // Прерывание при переходе с HIGH на LOW
        .mode = GPIO_MODE_INPUT,          // Режим входа
        .pin_bit_mask = (1ULL << SWITCH_1_GPIO) | (1ULL << SWITCH_2_GPIO), // IO14 и IO15
        .pull_down_en = 0,                // Отключаем подтяжку к земле
        .pull_up_en = 1,                  // Включаем подтяжку к питанию
    };
    gpio_config(&switch_conf);            // Применяем конфигурацию

    // Конфигурация zero cross - вход с прерываниями
    gpio_config_t zero_cross_conf = {
        .intr_type = GPIO_INTR_POSEDGE,   // Прерывание при переходе с LOW на HIGH
        .mode = GPIO_MODE_INPUT,          // Режим входа
        .pin_bit_mask = (1ULL << ZERO_CROSS_GPIO), // Маска для IO2
        .pull_down_en = 0,                // Отключаем подтяжку к земле
        .pull_up_en = 1,                  // Включаем подтяжку к питанию
    };
    gpio_config(&zero_cross_conf);        // Применяем конфигурацию

    // Устанавливаем сервис обработки прерываний GPIO
    // 0 означает использование стандартного приоритета
    gpio_install_isr_service(0);
    
    // Добавляем обработчики прерываний для каждого пина
    // Каждый пин передает свой номер как аргумент
    gpio_isr_handler_add(BUTTON_PAIRING_GPIO, gpio_isr_handler, (void*) BUTTON_PAIRING_GPIO);
    gpio_isr_handler_add(SWITCH_1_GPIO, gpio_isr_handler, (void*) SWITCH_1_GPIO);
    gpio_isr_handler_add(SWITCH_2_GPIO, gpio_isr_handler, (void*) SWITCH_2_GPIO);
    gpio_isr_handler_add(ZERO_CROSS_GPIO, zero_cross_isr_handler, (void*) ZERO_CROSS_GPIO);

    // Создаем очередь для передачи событий GPIO между прерыванием и задачей
    // 10 - максимальное количество сообщений в очереди
    // sizeof(uint32_t) - размер каждого сообщения (номер GPIO пина)
    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));

    // Создаем таймер для обработки длинных нажатий кнопки
    // "button_timer" - имя таймера для отладки
    // pdMS_TO_TICKS(100) - период таймера (100мс)
    // pdFALSE - таймер не автоматический (запускается вручную)
    // NULL - без параметров
    // button_timer_callback - функция, вызываемая по таймеру
    button_timer = xTimerCreate("button_timer", pdMS_TO_TICKS(100), pdFALSE, NULL, button_timer_callback);

    // Логируем успешную инициализацию GPIO
    ESP_LOGI(TAG, "GPIO initialized successfully");
}

// ============================================================================
// ИНИЦИАЛИЗАЦИЯ LED - простое управление через GPIO
// ============================================================================

static void ledc_init(void)
{
    // LED уже настроен в gpio_init() как выход
    // Просто выключаем его изначально
    gpio_set_level(LED_STATUS_GPIO, LED_OFF);
    
    // Логируем успешную инициализацию LED
    ESP_LOGI(TAG, "LED GPIO initialized successfully");
}

// ============================================================================
// ИНИЦИАЛИЗАЦИЯ UART ДЛЯ СВЯЗИ С BL0940
// ============================================================================

static void uart_init(void)
{
    // Конфигурация UART для BL0940
    uart_config_t uart_config = {
        .baud_rate = UART_BAUD_RATE,      // Скорость 4800 бод
        .data_bits = UART_DATA_8_BITS,    // 8 бит данных
        .parity = UART_PARITY_DISABLE,    // Без контроля четности
        .stop_bits = UART_STOP_BITS_1,    // 1 стоп-бит
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE, // Без управления потоком
        .source_clk = UART_SCLK_DEFAULT,  // Используем системный тактовый сигнал
    };

    // Применяем конфигурацию к UART1
    uart_param_config(UART_NUM, &uart_config);
    
    // Настраиваем пины UART (TX, RX, RTS, CTS)
    // UART_PIN_NO_CHANGE означает, что пин не используется
    uart_set_pin(UART_NUM, UART_TX_GPIO, UART_RX_GPIO, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    
    // Устанавливаем драйвер UART
    // UART_BUF_SIZE * 2 - размер буфера приема (удваиваем для надежности)
    // 0 - размер буфера передачи (не используется)
    // 0 - размер буфера событий (не используется)
    // NULL - без обработчика событий
    // 0 - без приоритета
    uart_driver_install(UART_NUM, UART_BUF_SIZE * 2, 0, 0, NULL, 0);

    // Логируем успешную инициализацию UART
    ESP_LOGI(TAG, "UART initialized successfully");
}

// ============================================================================
// ФУНКЦИЯ ПЕРЕКЛЮЧЕНИЯ РЕЛЕ - управление нагрузкой
// ============================================================================

static void toggle_relay(uint8_t relay_num, bool state)
{
    if (relay_num == 1) {
        // Управляем реле 1
        relay_1_state = state;                    // Сохраняем новое состояние
        gpio_set_level(RELAY_1_GPIO, state ? 1 : 0); // Устанавливаем уровень пина
        ESP_LOGI(TAG, "Relay 1: %s", state ? "ON" : "OFF"); // Логируем действие
    } else if (relay_num == 2) {
        // Управляем реле 2
        relay_2_state = state;                    // Сохраняем новое состояние
        gpio_set_level(RELAY_2_GPIO, state ? 1 : 0); // Устанавливаем уровень пина
        ESP_LOGI(TAG, "Relay 2: %s", state ? "ON" : "OFF"); // Логируем действие
    }
}

// ============================================================================
// ОБРАБОТЧИК ТАЙМЕРА КНОПКИ - для определения длинных нажатий
// ============================================================================

static void button_timer_callback(TimerHandle_t xTimer)
{
    // Этот таймер используется для обнаружения длинных нажатий
    // Фактическая логика обрабатывается в handle_button_release
    // Таймер запускается при нажатии и останавливается при отпускании
}

// ============================================================================
// ОБРАБОТКА НАЖАТИЯ КНОПКИ - начало отсчета времени
// ============================================================================

static void handle_button_press(void)
{
    if (!button_pressed) {  // Если кнопка еще не была нажата
        button_pressed = true;  // Устанавливаем флаг нажатия
        
        // Запоминаем время начала нажатия в миллисекундах
        // esp_timer_get_time() возвращает время в микросекундах, делим на 1000
        button_press_start_time = esp_timer_get_time() / 1000;
        
        // Запускаем таймер для обработки длинных нажатий
        // 0 означает немедленный запуск
        xTimerStart(button_timer, 0);
    }
}

// ============================================================================
// ОБРАБОТКА ОТПУСКАНИЯ КНОПКИ - анализ типа нажатия
// ============================================================================

static void handle_button_release(void)
{
    if (button_pressed) {  // Если кнопка была нажата
        // Вычисляем длительность нажатия в миллисекундах
        uint32_t current_time = esp_timer_get_time() / 1000;
        uint32_t press_duration = current_time - button_press_start_time;
        
        button_pressed = false;  // Сбрасываем флаг нажатия
        xTimerStop(button_timer, 0);  // Останавливаем таймер
        
        if (press_duration < BUTTON_LONG_PRESS_TIME) {
            // Короткое нажатие - обрабатываем клик
            // Вычисляем время с последнего отпускания
            uint32_t time_since_last_release = current_time - last_button_release_time;
            
            if (time_since_last_release < BUTTON_DOUBLE_CLICK_TIME) {
                // Двойной клик обнаружен
                button_click_count = 2;
                ESP_LOGI(TAG, "Double click detected - toggling relay 2");
                toggle_relay(2, !relay_2_state);  // Инвертируем состояние реле 2
            } else {
                // Одиночный клик обнаружен
                button_click_count = 1;
                ESP_LOGI(TAG, "Single click detected - toggling relay 1");
                toggle_relay(1, !relay_1_state);  // Инвертируем состояние реле 1
            }
            
            // Запоминаем время отпускания для следующего клика
            last_button_release_time = current_time;
            
        } else if (press_duration >= BUTTON_LONG_PRESS_TIME && press_duration < BUTTON_MAX_PRESS_TIME) {
            // Длинное нажатие (5-10 секунд) - запуск/остановка pairing
            ESP_LOGI(TAG, "Long press detected - starting pairing mode");
            if (!pairing_mode) {
                zigbee_commissioning_start();  // Запускаем процесс pairing
            } else {
                zigbee_commissioning_stop();   // Останавливаем процесс pairing
            }
        }
        // Если нажатие больше 10 секунд - игнорируем (защита от случайных нажатий)
    }
}

// ============================================================================
// ЗАДАЧА УПРАВЛЕНИЯ LED - индикация состояния устройства
// ============================================================================

static void status_led_task(void *pvParameter)
{
    ESP_LOGI(TAG, "Status LED task started");
    
    // Бесконечный цикл задачи
    while (1) {
        if (pairing_mode) {
            // Быстрое мигание во время pairing (поиск сети)
            gpio_set_level(LED_STATUS_GPIO, LED_ON);   // Включаем LED
            vTaskDelay(pdMS_TO_TICKS(100));            // Ждем 100мс
            gpio_set_level(LED_STATUS_GPIO, LED_OFF);  // Выключаем LED
            vTaskDelay(pdMS_TO_TICKS(100));            // Ждем 100мс
        } else if (zigbee_joined) {
            // Постоянный свет при подключении к Zigbee сети
            gpio_set_level(LED_STATUS_GPIO, LED_ON);   // Включаем LED
            vTaskDelay(pdMS_TO_TICKS(1000));           // Ждем 1 секунду
        } else if (zigbee_ready) {
            // Среднее мигание когда Zigbee готов, но не подключен
            gpio_set_level(LED_STATUS_GPIO, LED_ON);   // Включаем LED
            vTaskDelay(pdMS_TO_TICKS(500));            // Ждем 500мс
            gpio_set_level(LED_STATUS_GPIO, LED_OFF);  // Выключаем LED
            vTaskDelay(pdMS_TO_TICKS(500));            // Ждем 500мс
        } else {
            // Медленное мигание когда не подключен
            gpio_set_level(LED_STATUS_GPIO, LED_ON);   // Включаем LED
            vTaskDelay(pdMS_TO_TICKS(1000));           // Ждем 1 секунду
            gpio_set_level(LED_STATUS_GPIO, LED_OFF);  // Выключаем LED
            vTaskDelay(pdMS_TO_TICKS(1000));           // Ждем 1 секунду
        }
    }
}

// ============================================================================
// ЗАДАЧА ЭНЕРГОМОНИТОРИНГА - чтение данных от BL0940
// ============================================================================

static void energy_monitoring_task(void *pvParameter)
{
    // Буфер для приема данных от BL0940
    uint8_t data[UART_BUF_SIZE];
    uint8_t len;  // Количество полученных байт
    
    ESP_LOGI(TAG, "Energy monitoring task started");
    
    // Бесконечный цикл задачи
    while (1) {
        // Читаем данные от BL0940 с таймаутом 100мс
        // Если данных нет, функция вернет 0
        len = uart_read_bytes(UART_NUM, data, UART_BUF_SIZE, pdMS_TO_TICKS(100));
        
        if (len > 0) {
            // Получили данные от BL0940
            ESP_LOGI(TAG, "Received %d bytes from BL0940", len);
            
            // Парсим данные BL0940
            // BL0940 отправляет данные в определенном формате
            if (len >= 8 && data[0] == 0x58) { // 0x58 - стартовый байт
                // Парсим напряжение, ток, мощность, энергию
                voltage_rms = (data[1] << 8) | data[2];      // Объединяем 2 байта
                current_rms = (data[3] << 8) | data[4];      // Объединяем 2 байта
                power_consumption = (data[5] << 8) | data[6]; // Объединяем 2 байта
                
                // Накопляем энергию (конвертируем в Вт*ч)
                // power_consumption в мВт, делим на 3600 для перевода в часы
                energy_consumption += power_consumption / 3600;
                
                // Логируем полученные данные
                ESP_LOGI(TAG, "BL0940 Data - V: %lu mV, I: %lu mA, P: %lu mW, E: %lu mWh", 
                         voltage_rms, current_rms, power_consumption, energy_consumption);
            } else {
                // Логируем сырые данные для отладки
                ESP_LOGW(TAG, "Invalid BL0940 data format, len=%d, start=0x%02x", len, data[0]);
                for (int i = 0; i < len && i < 16; i++) {
                    ESP_LOGW(TAG, "Data[%d]: 0x%02x", i, data[i]);
                }
            }
        }
        
        // Ждем 1 секунду перед следующим чтением
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// ============================================================================
// ЗАДАЧА ОБРАБОТКИ КНОПКИ - обработка всех GPIO событий
// ============================================================================

static void button_task(void *pvParameter)
{
    uint32_t gpio_num;  // Номер GPIO пина, вызвавшего событие
    
    ESP_LOGI(TAG, "Button task started");
    
    // Бесконечный цикл задачи
    while (1) {
        // Ждем события GPIO из очереди
        // portMAX_DELAY означает бесконечное ожидание
        if (xQueueReceive(gpio_evt_queue, &gpio_num, portMAX_DELAY)) {
            
            if (gpio_num == BUTTON_PAIRING_GPIO) {
                // Событие кнопки pairing
                bool button_level = gpio_get_level(BUTTON_PAIRING_GPIO);
                if (button_level == 0) { // Кнопка нажата (активный низкий)
                    ESP_LOGI(TAG, "Pairing button pressed");
                    handle_button_press();  // Обрабатываем нажатие
                } else { // Кнопка отпущена
                    ESP_LOGI(TAG, "Pairing button released");
                    handle_button_release(); // Обрабатываем отпускание
                }
                
            } else if (gpio_num == SWITCH_1_GPIO) {
                // Переключен внешний переключатель 1
                bool switch_state = gpio_get_level(SWITCH_1_GPIO);
                ESP_LOGI(TAG, "External switch 1 toggled: %s", switch_state ? "ON" : "OFF");
                toggle_relay(1, switch_state);  // Управляем реле 1
                
            } else if (gpio_num == SWITCH_2_GPIO) {
                // Переключен внешний переключатель 2
                bool switch_state = gpio_get_level(SWITCH_2_GPIO);
                ESP_LOGI(TAG, "External switch 2 toggled: %s", switch_state ? "ON" : "OFF");
                toggle_relay(2, switch_state);  // Управляем реле 2
                
            } else if (gpio_num == ZERO_CROSS_GPIO) {
                // Обнаружен zero cross
                ESP_LOGD(TAG, "Zero cross detected");
                // Пока что только логируем, можно добавить логику управления
            }
        }
    }
}

// ============================================================================
// ГЛАВНАЯ ФУНКЦИЯ ПРИЛОЖЕНИЯ - точка входа
// ============================================================================

void app_main(void)
{
    ESP_LOGI(TAG, "Starting Smart Relay Application");
    
    // Инициализируем NVS (Non-Volatile Storage) - постоянная память
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGI(TAG, "NVS needs to be erased, erasing...");
        ESP_ERROR_CHECK(nvs_flash_erase());  // Стираем NVS если нужно
        ret = nvs_flash_init();              // Повторно инициализируем
    }
    ESP_ERROR_CHECK(ret);  // Проверяем результат инициализации
    ESP_LOGI(TAG, "NVS initialized successfully");
    
    // Создаем очередь для событий GPIO
    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    ESP_LOGI(TAG, "GPIO event queue created");
    
    // Инициализируем оборудование
    ESP_LOGI(TAG, "Initializing GPIO...");
    gpio_init();  // Настраиваем все GPIO пины
    
    ESP_LOGI(TAG, "Initializing LED...");
    ledc_init();  // Настраиваем LED
    
    ESP_LOGI(TAG, "Initializing UART...");
    uart_init();  // Настраиваем UART для BL0940
    
    // Инициализируем Zigbee
    ESP_LOGI(TAG, "Initializing Zigbee...");
    zigbee_init();  // Инициализируем Zigbee стек
    
    // Создаем задачи FreeRTOS
    ESP_LOGI(TAG, "Creating tasks...");
    
    // Задача управления LED - приоритет 5, стек 2048 байт
    xTaskCreate(status_led_task, "status_led", 2048, NULL, 5, NULL);
    
    // Задача энергомониторинга - приоритет 4, стек 4096 байт
    xTaskCreate(energy_monitoring_task, "energy_mon", 4096, NULL, 4, NULL);
    
    // Задача обработки кнопки - приоритет 3, стек 2048 байт
    xTaskCreate(button_task, "button", 2048, NULL, 3, NULL);
    
    // Задача управления Zigbee - приоритет 2, стек 8192 байт
    xTaskCreate(zigbee_task, "zigbee", 8192, NULL, 2, NULL);
    
    // Запускаем Zigbee стек
    ESP_LOGI(TAG, "Starting Zigbee stack...");
    zigbee_start();  // Запускаем Zigbee стек
    
    // Логируем успешный запуск приложения
    ESP_LOGI(TAG, "Smart Relay Application started successfully");
    ESP_LOGI(TAG, "Features:");
    ESP_LOGI(TAG, "- Zigbee Router mode");
    ESP_LOGI(TAG, "- Energy monitoring via BL0940");
    ESP_LOGI(TAG, "- External switch control");
    ESP_LOGI(TAG, "- Button pairing (long press 5-10s)");
    ESP_LOGI(TAG, "- Single click: Relay 1, Double click: Relay 2");
    ESP_LOGI(TAG, "You can now connect via UART to see logs");
}

// ============================================================================
// ИНИЦИАЛИЗАЦИЯ ZIGBEE - настройка Zigbee стека
// ============================================================================

static void zigbee_init(void)
{
    ESP_LOGI(TAG, "Initializing Zigbee...");
    
    // Инициализируем Zigbee стек с конфигурацией по умолчанию
    // Устройство начнет как End Device и может стать Router при подключении к сети
    esp_zb_cfg_t zb_cfg = {
        .esp_zb_role = ESP_ZB_DEVICE_TYPE_ED,  // Начинаем как End Device
        .install_code_policy = false,           // Отключаем политику install code
        .nwk_cfg = {
            .zczr_cfg = {
                .max_children = 20,            // Может стать Router с до 20 дочерних устройств
            },
        },
    };
    
    // Инициализируем Zigbee стек с нашей конфигурацией
    esp_zb_init(&zb_cfg);
    ESP_LOGI(TAG, "Zigbee initialized successfully as End Device (can become Router)");
}

// ============================================================================
// ЗАПУСК ZIGBEE - запуск Zigbee стека
// ============================================================================

static void zigbee_start(void)
{
    ESP_LOGI(TAG, "Starting Zigbee...");
    
    // Запускаем Zigbee стек
    // false означает, что устройство не будет автоматически подключаться к сети
    esp_err_t ret = esp_zb_start(false);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Zigbee started successfully");
    } else {
        ESP_LOGE(TAG, "Failed to start Zigbee: %s", esp_err_to_name(ret));
    }
    
    // Ждем немного для стабилизации Zigbee
    vTaskDelay(pdMS_TO_TICKS(1000));
}

// ============================================================================
// ЗАПУСК ZIGBEE COMMISSIONING (PAIRING) - начало процесса подключения
// ============================================================================

static void zigbee_commissioning_start(void)
{
    ESP_LOGI(TAG, "Starting Zigbee commissioning...");
    
    // Запускаем процесс commissioning
    // false означает, что устройство не будет автоматически подключаться
    esp_err_t ret = esp_zb_start(false);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Zigbee commissioning started successfully");
        pairing_mode = true;  // Включаем режим pairing
    } else {
        ESP_LOGE(TAG, "Failed to start Zigbee commissioning: %s", esp_err_to_name(ret));
        pairing_mode = false; // Отключаем режим pairing при ошибке
    }
}

// ============================================================================
// ОСТАНОВКА ZIGBEE COMMISSIONING - выход из режима pairing
// ============================================================================

static void zigbee_commissioning_stop(void)
{
    ESP_LOGI(TAG, "Stopping Zigbee commissioning...");
    pairing_mode = false;  // Отключаем режим pairing
}

// ============================================================================
// ЗАДАЧА ZIGBEE - управление commissioning и сетевыми операциями
// ============================================================================

static void zigbee_task(void *pvParameter)
{
    ESP_LOGI(TAG, "Zigbee task started");
    
    // Бесконечный цикл задачи
    while (1) {
        if (pairing_mode && !zigbee_joined) {
            // Если в режиме pairing и не подключен, пытаемся подключиться к сети
            ESP_LOGI(TAG, "Attempting to join Zigbee network...");
            
            // Запускаем Zigbee стек для начала commissioning
            if (!esp_zb_is_started()) {
                esp_zb_start(false);  // Запускаем стек если он не запущен
            }
            
            // Ждем 5 секунд перед следующей попыткой
            vTaskDelay(pdMS_TO_TICKS(5000));
        }
        
        // Основная задержка задачи - 1 секунда
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// ============================================================================
// ОБРАБОТЧИК СИГНАЛОВ ZIGBEE - эта функция должна быть определена пользователем
// ============================================================================

void esp_zb_app_signal_handler(esp_zb_app_signal_t *signal)
{
    // Получаем указатель на сигнал и статус
    uint32_t *p_sg_p = signal->p_app_signal;  // Указатель на тип сигнала
    esp_err_t status = signal->esp_err_status; // Статус выполнения операции
    
    // Логируем полученный сигнал
    ESP_LOGI(TAG, "Zigbee signal received: %lu, status: %d", *p_sg_p, status);
    
    // Обрабатываем различные типы сигналов Zigbee
    switch (*p_sg_p) {
        case ESP_ZB_ZDO_SIGNAL_DEFAULT_START:
            // Сигнал о запуске Zigbee устройства
            if (status == ESP_OK) {
                ESP_LOGI(TAG, "Zigbee device started and joined network");
                zigbee_joined = true;    // Устанавливаем флаг подключения
                zigbee_ready = true;     // Устанавливаем флаг готовности
                pairing_mode = false;    // Выходим из режима pairing при подключении
                ESP_LOGI(TAG, "Device successfully joined Zigbee network - exiting pairing mode");
            } else {
                ESP_LOGE(TAG, "Zigbee device startup failed with status: %s", esp_err_to_name(status));
                zigbee_ready = false;    // Сбрасываем флаг готовности при ошибке
                ESP_LOGI(TAG, "Device failed to join network - will retry if in pairing mode");
            }
            break;
            
        case ESP_ZB_ZDO_SIGNAL_DEVICE_ANNCE:
            // Сигнал о том, что устройство объявило о себе в сети
            ESP_LOGI(TAG, "Zigbee device announced to network");
            break;
            
        case ESP_ZB_ZDO_SIGNAL_LEAVE:
            // Сигнал о том, что устройство покинуло сеть
            ESP_LOGI(TAG, "Zigbee device left network");
            zigbee_joined = false;  // Сбрасываем флаг подключения
            break;
            
        case ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP:
            // Сигнал о том, что Zigbee стек готов к работе
            ESP_LOGI(TAG, "Zigbee stack framework started - ready for commissioning");
            zigbee_ready = true;  // Устанавливаем флаг готовности
            break;
            
        case ESP_ZB_ZDO_SIGNAL_ERROR:
            // Сигнал об ошибке Zigbee
            ESP_LOGE(TAG, "Zigbee error occurred: %s", esp_err_to_name(status));
            break;
            
        default:
            // Неизвестный сигнал - логируем для отладки
            ESP_LOGI(TAG, "Unknown Zigbee signal: %lu, status: %d", *p_sg_p, status);
            break;
    }
}