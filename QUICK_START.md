# Быстрый старт - Умное реле ESP32-C6

## 🚀 Быстрая установка (5 минут)

### 1. Установка ESP-IDF

#### Windows (рекомендуемый)
1. Скачайте ESP-IDF Windows Installer: https://dl.espressif.com/dl/esp-idf/
2. Запустите установщик и следуйте инструкциям
3. ESP-IDF будет установлен в `C:\Espressif\frameworks\esp-idf-v5.3.2\`

#### Windows (ручная установка)
```cmd
git clone --recursive https://github.com/espressif/esp-idf.git
cd esp-idf
install.bat
export.bat
```

#### Linux/Mac
```bash
git clone --recursive https://github.com/espressif/esp-idf.git
cd esp-idf
./install.sh
source export.sh
```

### 2. Клонирование проекта
```bash
git clone <your-repo-url>
cd Robo-ZB-SR2CH10A
```

### 3. Быстрая сборка и прошивка

#### Windows
```cmd
build_and_flash.bat
```

#### Linux/Mac
```bash
./build_and_flash.sh
```

## 🔧 Подключение компонентов

### Минимальная схема для тестирования
```
ESP32-C6:
├── GPIO0  → Кнопка → GND (пейринг)
├── GPIO1  → LED → GND (статус)
├── GPIO4  → RX BL0940
├── GPIO5  → TX BL0940
├── GPIO14 → Переключатель 1 → GND
├── GPIO15 → Переключатель 2 → GND
├── GPIO18 → Реле 2
├── GPIO19 → Реле 1
└── GPIO2  → Zero-cross детектор
```

## 📱 Тестирование

### 1. Проверка прошивки
```bash
idf.py monitor
```
Ожидаемый вывод:
```
I (1234) SMART_RELAY: Starting Smart Relay Application
I (1234) SMART_RELAY: GPIO initialized
I (1234) SMART_RELAY: UART initialized for BL0940
I (1234) SMART_RELAY: Zigbee initialized
I (1234) SMART_RELAY: Smart Relay Application started successfully
```

### 2. Тест кнопок
- Нажмите кнопку на GPIO0 → LED должен быстро мигать
- Переключите переключатель на GPIO14 → Реле 1 должно сработать
- Переключите переключатель на GPIO15 → Реле 2 должно сработать

### 3. Тест Zigbee
1. Откройте Home Assistant
2. Добавьте интеграцию ZHA
3. Нажмите кнопку пейринга на GPIO0
4. Устройство должно появиться в сети

## 🐛 Частые проблемы

### Ошибка компиляции
```bash
# Очистите проект
idf.py fullclean
idf.py build
```

### Ошибка NVS шифрования
```bash
# Используйте обновленный sdkconfig.defaults
# Очистите и пересоберите проект
idf.py fullclean
idf.py build
```

### Ошибка прошивки
```bash
# Проверьте COM порт
idf.py -p COM3 flash

# Принудительная загрузка
# Удерживайте BOOT кнопку при прошивке
```

### Zigbee не подключается
1. Проверьте LED индикатор
2. Убедитесь в работе координатора
3. Перезапустите пейринг

## 📚 Дополнительная информация

- **Полная документация**: [README.md](README.md)
- **Подключение BL0940**: [BL0940_Connection.md](BL0940_Connection.md)
- **Настройка Zigbee**: [Zigbee_Setup.md](Zigbee_Setup.md)

## 🆘 Поддержка

При проблемах:
1. Проверьте логи через `idf.py monitor`
2. Убедитесь в правильности подключения
3. Проверьте версии ESP-IDF
4. Создайте issue в репозитории

## ⚡ Готово!

Ваше умное реле готово к работе! 🎉
