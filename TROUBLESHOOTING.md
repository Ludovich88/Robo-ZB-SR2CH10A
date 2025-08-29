# Устранение неполадок

## Ошибки компиляции

### 1. Ошибка NVS шифрования

**Симптомы:**
```
error: #error "NVS Encryption (HMAC): Configured eFuse block (CONFIG_NVS_SEC_HMAC_EFUSE_KEY_ID) out of range!"
```

**Причина:**
Проблема с конфигурацией NVS шифрования для ESP32-C6.

**Решение:**
1. Убедитесь, что используется обновленный `sdkconfig.defaults`
2. Очистите проект:
   ```bash
   idf.py fullclean
   ```
3. Пересоберите проект:
   ```bash
   idf.py build
   ```

**Альтернативное решение:**
Если проблема сохраняется, откройте `sdkconfig` и найдите строки:
```
CONFIG_NVS_ENCRYPTION=y
CONFIG_NVS_SECURITY_ENABLED=y
```
Измените их на:
```
CONFIG_NVS_ENCRYPTION=n
CONFIG_NVS_SECURITY_ENABLED=n
```

### 2. Ошибка Zigbee библиотек

**Симптомы:**
```
fatal error: esp_zigbee_core.h: No such file or directory
```

**Причина:**
Не установлены или не найдены Zigbee компоненты.

**Решение:**
1. Проверьте `idf_component.yml`:
   ```yaml
   dependencies:
     espressif/esp-zboss-lib: "~1.6.0"
     espressif/esp-zigbee-lib: "~1.6.0"
   ```
2. Обновите компоненты:
   ```bash
   idf.py reconfigure
   ```

### 3. Ошибка версии ESP-IDF

**Симптомы:**
```
ESP-IDF version 4.x.x is not supported
```

**Причина:**
Используется старая версия ESP-IDF.

**Решение:**
1. Обновите ESP-IDF до версии 5.0.0 или выше
2. Переустановите ESP-IDF:
   ```bash
   # Windows
   C:\Espressif\frameworks\esp-idf-v5.3.2\install.bat
   
   # Linux/Mac
   ./install.sh
   ```

## Ошибки прошивки

### 1. Устройство не найдено

**Симптомы:**
```
No serial ports found
```

**Решение:**
1. Проверьте USB подключение
2. Установите драйверы USB-to-Serial
3. Перезагрузите компьютер
4. Попробуйте другой USB кабель

### 2. Ошибка доступа к порту

**Симптомы:**
```
Permission denied: COM3
```

**Решение:**
1. Закройте все программы, использующие COM порт
2. Запустите командную строку от имени администратора
3. Проверьте, не используется ли порт другими приложениями

### 3. Ошибка загрузки

**Симптомы:**
```
Failed to connect to ESP32-C6
```

**Решение:**
1. Удерживайте кнопку BOOT при прошивке
2. Нажмите кнопку RESET после начала прошивки
3. Проверьте питание ESP32-C6

## Ошибки Zigbee

### 1. Устройство не подключается

**Симптомы:**
LED не мигает, устройство не появляется в сети.

**Решение:**
1. Проверьте логи через `idf.py monitor`
2. Убедитесь в работе Zigbee координатора
3. Проверьте расстояние до координатора
4. Перезапустите пейринг

### 2. Нестабильная связь

**Симптомы:**
Устройство периодически отключается.

**Решение:**
1. Улучшите размещение координатора
2. Добавьте Zigbee повторители
3. Проверьте помехи от WiFi
4. Обновите прошивку координатора

## Ошибки энергомониторинга

### 1. BL0940 не отвечает

**Симптомы:**
Нет данных от BL0940 в логах.

**Решение:**
1. Проверьте подключение питания BL0940
2. Убедитесь в правильности UART подключения
3. Проверьте скорость UART (4800 baud)
4. Проверьте делитель напряжения

### 2. Неверные показания

**Симптомы:**
Показания сильно отличаются от реальных.

**Решение:**
1. Проверьте делитель напряжения
2. Убедитесь в правильности токового трансформатора
3. Проверьте калибровку
4. Добавьте фильтрующие конденсаторы

## Общие решения

### 1. Очистка проекта
```bash
idf.py fullclean
idf.py build
```

### 2. Сброс конфигурации
```bash
idf.py reconfigure
```

### 3. Обновление компонентов
```bash
idf.py update-dependencies
```

### 4. Проверка версий
```bash
idf.py --version
idf.py list-targets
```

## Получение помощи

### 1. Логи
Всегда начинайте с проверки логов:
```bash
idf.py monitor
```

### 2. Документация
- [ESP-IDF Programming Guide](https://docs.espressif.com/projects/esp-idf/en/latest/)
- [ESP32-C6 Technical Reference](https://www.espressif.com/sites/default/files/documentation/esp32-c6_technical_reference_manual_en.pdf)

### 3. Сообщество
- [ESP32 Forum](https://esp32.com/)
- [GitHub Issues](https://github.com/espressif/esp-idf/issues)

### 4. Поддержка Espressif
- [Technical Support](https://www.espressif.com/en/support)
- [Documentation](https://docs.espressif.com/)
