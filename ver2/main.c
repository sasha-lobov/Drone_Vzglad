/* USER CODE BEGIN Header */

/* USER CODE END Header */


#include "main.h"
#include <math.h>
#include <string.h>

#include "mpu6050_driver.h"
#include "bmp280_driver.h"
#include "uart_parser.h"


typedef struct {
    float kp, ki, kd;
    float integrator;
    float prev_error;
    float out_min;
    float out_max;
} PID_t;

typedef struct {
    float roll, pitch, yaw;       // Углы (rad)
    float roll_sp, pitch_sp, yaw_sp, alt_sp; // Setpoints (deg/m)
    float throttle;               // Базовый газ (0..1)
    float altitude;               // Оценка высоты (m)
    float bat_voltage;            // Напряжение батареи (V)
    bool armed;
    bool uart_link_ok;
    uint32_t last_uart_tick;
} FlightState_t;


#define CONTROL_FREQ_HZ     200
#define CONTROL_DTS         (1.0f / CONTROL_FREQ_HZ)
#define PWM_MIN_US          1050
#define PWM_MAX_US          2000
#define UART_TIMEOUT_MS     200
#define BAT_CRIT_V          16.5f  // ~3.3V/cell для 5S
#define BARO_CALIB_SAMPLES  300
#define MADGWICK_BETA       0.1f


extern TIM_HandleTypeDef htim1;  // PWM
extern TIM_HandleTypeDef htim6;  // 200Hz control tick
extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c2;
extern UART_HandleTypeDef huart1;
extern ADC_HandleTypeDef hadc1;

FlightState_t fs = {0};
PID_t pid_roll, pid_pitch, pid_yaw, pid_alt;


static float q0=1.0f, q1=0.0f, q2=0.0f, q3=0.0f;
static float baro_offset = 0.0f;
static volatile bool control_tick = false; // Set in TIM6 ISR


void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM6_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_ADC1_Init(void);

static void PID_Init(PID_t *pid, float kp, float ki, float kd, float min, float max);
static float PID_Update(PID_t *pid, float sp, float pv, float dt);
static void MadgwickUpdate(float gx, float gy, float gz, float ax, float ay, float az, float dt);
static void UpdateAltitudeComplementary(float baro_alt, float acc_z);
static void RunControlLoop(void);
static void MixAndApplyPWM(float throttle, float roll, float pitch, float yaw);
static float ReadBatteryVoltage(void);

/* USER CODE BEGIN PFP */

void ControlTickCallback(void) {
    control_tick = true;
}
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */


  HAL_Init();
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */


  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM1_Init();
  MX_TIM6_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();

  /* USER CODE BEGIN 2 */
  // Драйверы
  MPU6050_Init(&hi2c1);
  BMP280_Init(&hi2c2);
  UART_InitParser(&huart1);

  // Калибровка барометра
  HAL_Delay(100);
  for(int i=0; i<BARO_CALIB_SAMPLES; i++) {
      float alt = 0.0f;
      BMP280_StartAltReadDMA();
      HAL_Delay(4);
      BMP280_GetAltitude(&alt);
      baro_offset += alt;
  }
  baro_offset /= BARO_CALIB_SAMPLES;

  PID_Init(&pid_roll,  2.0f, 0.0f, 0.08f, -0.8f,  0.8f);
  PID_Init(&pid_pitch, 2.0f, 0.0f, 0.08f, -0.8f,  0.8f);
  PID_Init(&pid_yaw,   1.5f, 0.0f, 0.05f, -0.8f,  0.8f);
  PID_Init(&pid_alt,   0.6f, 0.01f, 0.4f,  0.0f,  1.0f);

  // контрольный таймер 200Hz
  HAL_TIM_Base_Start_IT(&htim6);
  fs.armed = false;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      // Батарея (неблокирующе)
      static uint32_t bat_tick = 0;
      if(HAL_GetTick() - bat_tick > 100) {
          fs.bat_voltage = ReadBatteryVoltage();
          bat_tick = HAL_GetTick();
      }

      // фейл по напряжению
      if(fs.bat_voltage > 0.1f && fs.bat_voltage < BAT_CRIT_V) {
          fs.armed = false;
          fs.throttle = 0.0f;
      }

      // Основной контур 200Hz
      if(control_tick) {
          control_tick = false;
          RunControlLoop();
      }

      // хертбит 20Hz
      static uint32_t hb_tick = 0;
      if(HAL_GetTick() - hb_tick > 50) {
          float bat_mv = fs.bat_voltage * 1000.0f;
          bool link = (HAL_GetTick() - fs.last_uart_tick) < UART_TIMEOUT_MS;
          UART_SendHeartbeat(bat_mv, link);
          hb_tick = HAL_GetTick();
      }


  }
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
  /* USER CODE END 3 */
}

/* USER CODE BEGIN 4 */
// ПИД
static void PID_Init(PID_t *pid, float kp, float ki, float kd, float min, float max) {
    pid->kp = kp; pid->ki = ki; pid->kd = kd;
    pid->out_min = min; pid->out_max = max;
    pid->integrator = 0.0f; pid->prev_error = 0.0f;
}

static float PID_Update(PID_t *pid, float sp, float pv, float dt) {
    float err = sp - pv;
    pid->integrator += err * dt;
    // анти-виндап
    if(pid->integrator > pid->out_max) pid->integrator = pid->out_max;
    if(pid->integrator < pid->out_min) pid->integrator = pid->out_min;

    float out = pid->kp * err + pid->ki * pid->integrator + pid->kd * (err - pid->prev_error) / dt;
    pid->prev_error = err;
    if(out > pid->out_max) out = pid->out_max;
    if(out < pid->out_min) out = pid->out_min;
    return out;
}

// AHRS
static void MadgwickUpdate(float gx, float gy, float gz, float ax, float ay, float az, float dt) {
    float recipNorm, s0, s1, s2, s3, qDot1, qDot2, qDot3, qDot4;
    float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2, _8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

    // нормализация acc
    recipNorm = 1.0f / sqrtf(ax*ax + ay*ay + az*az);
    ax *= recipNorm; ay *= recipNorm; az *= recipNorm;

    _2q0 = 2.0f*q0; _2q1 = 2.0f*q1; _2q2 = 2.0f*q2; _2q3 = 2.0f*q3;
    _4q0 = 4.0f*q0; _4q1 = 4.0f*q1; _4q2 = 4.0f*q2; _8q1 = 8.0f*q1; _8q2 = 8.0f*q2;
    q0q0 = q0*q0; q1q1 = q1*q1; q2q2 = q2*q2; q3q3 = q3*q3;

    s0 = _4q0*q2q2 + _2q2*ax + _4q0*q1q1 - _2q1*ay;
    s1 = _4q1*q3q3 - _2q3*ax + 4.0f*q0q0*q1 - _2q0*ay - _4q1 + _8q1*q1q1 + _8q1*q2q2 + _4q1*az;
    s2 = 4.0f*q0q0*q2 + _2q0*ax + _4q2*q3q3 - _2q3*ay - _4q2 + _8q2*q1q1 + _8q2*q2q2 + _4q2*az;
    s3 = 4.0f*q1q1*q3 - _2q1*ax + 4.0f*q2q2*q3 - _2q2*ay;

    recipNorm = 1.0f / sqrtf(s0*s0 + s1*s1 + s2*s2 + s3*s3);
    s0 *= recipNorm; s1 *= recipNorm; s2 *= recipNorm; s3 *= recipNorm;

    qDot1 = 0.5f*(-q1*gx - q2*gy - q3*gz) - MADGWICK_BETA*s0;
    qDot2 = 0.5f*( q0*gx + q2*gz - q3*gy) - MADGWICK_BETA*s1;
    qDot3 = 0.5f*( q0*gy - q1*gz + q3*gx) - MADGWICK_BETA*s2;
    qDot4 = 0.5f*( q0*gz + q1*gy - q2*gx) - MADGWICK_BETA*s3;

    q0 += qDot1 * dt; q1 += qDot2 * dt;
    q2 += qDot3 * dt; q3 += qDot4 * dt;

    recipNorm = 1.0f / sqrtf(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    q0 *= recipNorm; q1 *= recipNorm; q2 *= recipNorm; q3 *= recipNorm;

    // ейлер радианы
    fs.roll  = atan2f(2.0f*(q0*q1 + q2*q3), 1.0f - 2.0f*(q1*q1 + q2*q2));
    fs.pitch = asinf(2.0f*(q0*q2 - q3*q1));
    fs.yaw   = atan2f(2.0f*(q0*q3 + q1*q2), 1.0f - 2.0f*(q2*q2 + q3*q3));
}

// компл. фильтр
static void UpdateAltitudeComplementary(float baro_alt, float acc_z) {
    static float vel = 0.0f;
    static float alt_est = 0.0f;
    const float alpha = 0.97f;

    // acc_z в g, переводим в m/s^2 и вычитаем g
    float acc_mps2 = acc_z * 9.81f;
    vel += (acc_mps2 - 9.81f) * CONTROL_DTS;
    alt_est = alpha * (alt_est + vel * CONTROL_DTS) + (1.0f - alpha) * baro_alt;
    fs.altitude = alt_est;
}

// цикл контроля
static void RunControlLoop(void) {
    // сенсоры
    MPU6050_Data_t imu;
    if(!MPU6050_GetData(&imu)) return; // скипаю дма если нет данных

    // конвертация
    float ax = imu.ax / 16384.0f;
    float ay = imu.ay / 16384.0f;
    float az = imu.az / 16384.0f;
    float gx = imu.gx * 0.000061035f; // 2000/32768 * DEG2RAD
    float gy = imu.gy * 0.000061035f;
    float gz = imu.gz * 0.000061035f;

    // AHRS и Alt
    MadgwickUpdate(gx, gy, gz, ax, ay, az, CONTROL_DTS);

    float baro_alt = 0.0f;
    static uint32_t baro_tick = 0;
    if(HAL_GetTick() - baro_tick > 15) { // ~66Hz
        BMP280_StartAltReadDMA();
        baro_tick = HAL_GetTick();
    }
    if(BMP280_GetAltitude(&baro_alt)) {
        UpdateAltitudeComplementary(baro_alt, az);
    }

    // UART сетпоинты и фейлсейв
    VisionCmd_t vc;
    if(UART_GetCommand(&vc)) {
        fs.roll_sp  = vc.roll_sp  * 0.0174533f; // deg2rad
        fs.pitch_sp = vc.pitch_sp * 0.0174533f;
        fs.yaw_sp   = vc.yaw_sp   * 0.0174533f;
        fs.alt_sp   = vc.alt_sp;
        fs.last_uart_tick = vc.last_tick;
        fs.uart_link_ok = true;
    }

    if(HAL_GetTick() - fs.last_uart_tick > UART_TIMEOUT_MS) {
        fs.uart_link_ok = false;
        fs.roll_sp = fs.pitch_sp = fs.yaw_sp = 0.0f;
        if(fs.throttle > 0.2f) fs.throttle -= 0.002f;
        else fs.throttle = 0.0f;
    }

    // пид контроль
    float r_out = PID_Update(&pid_roll,  fs.roll_sp,  fs.roll,  CONTROL_DTS);
    float p_out = PID_Update(&pid_pitch, fs.pitch_sp, fs.pitch, CONTROL_DTS);
    float y_out = PID_Update(&pid_yaw,   fs.yaw_sp,   fs.yaw,   CONTROL_DTS);

    // Alt пид
    float a_out = PID_Update(&pid_alt, fs.alt_sp, fs.altitude, CONTROL_DTS);

    // миксер пида и выход
    float thr_cmd = fs.throttle + a_out;
    if(thr_cmd < 0.0f) thr_cmd = 0.0f;
    if(thr_cmd > 1.0f) thr_cmd = 1.0f;

    if(fs.armed) {
        MixAndApplyPWM(thr_cmd, r_out, p_out, y_out);
    } else {
        MixAndApplyPWM(0.0f, 0.0f, 0.0f, 0.0f);
    }
}

static void MixAndApplyPWM(float throttle, float roll, float pitch, float yaw) {
    // микшер на моторы
    float m1 = throttle - pitch + roll - yaw;
    float m2 = throttle - pitch - roll + yaw;
    float m3 = throttle + pitch + roll + yaw;
    float m4 = throttle + pitch - roll - yaw;

    float mix[4] = {m1, m2, m3, m4};
    for(int i=0; i<4; i++) {
        if(mix[i] < 0.0f) mix[i] = 0.0f;
        if(mix[i] > 1.0f) mix[i] = 1.0f;
    }

    // конвертация в шим
    uint32_t pwm[4];
    for(int i=0; i<4; i++) {
        pwm[i] = (uint32_t)(PWM_MIN_US + mix[i] * (PWM_MAX_US - PWM_MIN_US));
    }

    // применение шима
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pwm[0]);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, pwm[1]);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, pwm[2]);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, pwm[3]);
}

static float ReadBatteryVoltage(void) {
    const float divider = 5.0f;
    HAL_ADC_Start(&hadc1);
    if(HAL_ADC_PollForConversion(&hadc1, 5) == HAL_OK) {
        float adc = HAL_ADC_GetValue(&hadc1);
        return (adc * 3.3f / 4095.0f) * divider;
    }
    return 0.0f;
}
/* USER CODE END 4 */

/**

  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
    Error_Handler();
  }
}

/**

  */
void Error_Handler(void)
{
  __disable_irq();
  while (1) {}
}
