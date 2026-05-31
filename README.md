# Drone_Vzglad
Прошивка полётного контроллера для квадрокоптера на базе YD-ESP32-S3. Поддерживает только стабилизированный режим полёта (Angle/Self-Level). Разработана на ESP-IDF v5.x с использованием FreeRTOS.

## Комплектующие
- МК - YD-ESP32-S3
- IMU - MPU6050 (I2C)
- Приёмник - TBS Crossfire Nano RX
- Регуляторы - 4× ESC (BLHeli/AM32)
- Питание 5S + BMS

## Принципиальная схема подключения

<img width="1185" height="537" alt="схема на esp32" src="https://github.com/user-attachments/assets/064d7b7b-6223-47ef-b632-fe0f72ce66db" />


## Особенности
- Цикл управления: 250 Гц (задача FreeRTOS на ядре 1)
- Фильтрация: Комплементарный фильтр (Roll/Pitch), прямой интегратор Yaw
- Стабилизация: PID-регулятор, Angle mode (удержание горизонта)
- Приёмник: Парсинг CRSF, failsafe при потере сигнала >300 мс
- Батарея: Мониторинг 5S, аварийное отключение при <16.0V
- Арминг: Газ <5%, стики по центру, рысканье влево (Disarm: вправо)
- Выход: LEDC PWM 200 Гц, 16-бит разрешение, диапазон 0–100%

## Сборка
- 1 Инициализация ESP-IDF (требуется v5.2+)
- 2 Выбор целевого чипа
```
idf.py set-target esp32s3
```
- 3 Сборка, прошивка и мониторинг
```
idf.py build flash monitor
```



Медиаматериалы по проекту "Взгляд"


![photo_2024-07-27_13-58-01](https://github.com/user-attachments/assets/23eeaeb7-7cb1-4ad1-b37e-5b02d97d3a3d)
![photo_2024-05-26_00-27-09](https://github.com/user-attachments/assets/9940e1da-72e0-4e2b-884f-dfc7bb954327)
![photo_2024-05-16_21-58-53](https://github.com/user-attachments/assets/c4452819-15cf-493a-8f8b-1ddc903964f0)
![photo_2024-05-16_14-54-20](https://github.com/user-attachments/assets/e21a6d2c-40b6-4075-a955-da716854bdd8)
![photo_2024-05-13_21-13-46](https://github.com/user-attachments/assets/9a42ec63-5347-4e77-b8fd-98acaf2bc49c)
![3chJCTjFALaI9Ziow908ZMlHtPZBdtQDPWgbjmVMZPJj3HTig_5myfVMsM0j7pfehil7I_ybW-2TkOPmWTjqkAcs](https://github.com/user-attachments/assets/d8f47c8a-0a2f-4db5-8fc8-a4cc7473c508)


