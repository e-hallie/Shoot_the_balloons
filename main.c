/* ==========================================================================
 * VIRTUELE BALLONNENSCHIETER
 * Schietspel voor Visueel Gehandicapte Deelnemers
 * 
 * STM32F401RET6
 * Versie 3.3 - GYRO VISUALISATIE MODUS
 * Ontwerper: Raymond Hallie
 * Datum: 19 januari 2026
 * ==========================================================================
 * 
 * AANGEPAST VOOR PYTHON VISUALISATIE:
 * - Alle debug UART output verwijderd
 * - Toegevoegd: UART output in formaat voor gyro_visualizer.py
 *   Output: Acc(g): X=+0.000 Y=+0.000 Z=+0.000 | Gyro(dps): X=+0.00 Y=+0.00 Z=+0.00
 * 
 * WIJZIGINGEN in v3.1:
 * - Kalman filter toegevoegd voor de fusie Gyro en Acc.
 * - VPA_MARGIN verkleind van 10? naar 5? voor beter ballon placement
 * - Ballonnen spawnen nu tussen 5?-20? elevatie (was 10?-35?)
 * - Azimuth range aangepast naar -55? tot +55? (was -50? tot +50?)
 * 
 * WIJZIGINGEN in v3.0:
 * - Geen schot gelost (shotsFired == 0): alleen e_niet.wav
 * - Precies 1 ballon (score == 1): aantal schoten + e_1_bal.wav  
 * - Meerdere ballonnen (score >= 2): volledige score met aantallen
 * 
 * SENSOR ORIENTATIE:
 * - X-as: Recht vooruit (schietrichting)
 * - Y-as: Links/rechts (azimuth)
 * - Z-as: Omhoog/omlaag (elevatie)
 * 
 * VPA CONFIGURATIE:
 * - Horizontaal: ?60? (totaal 120?)
 * - Verticaal: 0? tot +45? (alleen boven horizon)
 * 
 * Hardware:
 * - LSM6DSL sensor via I2C (TCA9548A multiplexer poort 3)
 * - MAX98357A audio via I2S2 (PB12/13/15)
 * - Piezo buzzer via PWM TIM2_CH1 (PA0)
 * - SD card via SPI3 (PC10/11/12, CS=PB0)
 * - Trigger button (PA1), Start/Stop button (PA2)
 * - Debug UART (PB6/PB7, 115200 baud)
 * 
 * ========================================================================== */

#include "stm32f4xx_hal.h"
#include "main.h"
#include "fatfs.h"
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdlib.h>
#include <math.h>

/* ==========================================================================
 * GAME CONFIGURATION - Aangepast voor kalibratie
 * ========================================================================== */

#define GAME_DURATION       120      /* Speeltijd in seconden */
#define HIT_RADIUS          15.0f //8.0f    /* Trefradius in graden 15 = makkelijk */
#define SAMPLE_RATE         200     /* Sensor sample rate Hz (verhoogd voor snelle bewegingen) */

/* Piezo toon configuratie */
#define MIN_TONE_FREQ       300     /* Minimale toonfrequentie (veraf) */
#define MAX_TONE_FREQ       2500    /* Maximale toonfrequentie (dichtbij/raak) */

/* VPA BEREIK - Aangepast naar jouw specificaties */
#define VPA_AZIMUTH_RANGE   60.0f   /* Horizontaal bereik ? graden (totaal 120?) */
#define VPA_ELEVATION_MIN   0.0f    /* Minimale elevatie (horizon) */
#define VPA_ELEVATION_MAX   25.0f   /* Maximale elevatie (omhoog) */

/* Filter en timing */
#define FILTER_ALPHA        0.995f  /* Complementary filter: hoger = meer gyro, stabieler */
#define DEBOUNCE_MS         50      /* Knop debounce tijd */
#define LONG_PRESS_MS       2000    /* Lange druk threshold */

/* Vibration motor pin */
#define VIBRATION_PIN       GPIO_PIN_5
#define VIBRATION_PORT      GPIOB

/* I2C Addresses */
#define TCA9548A_ADDR       0x70    /* I2C Multiplexer */
#define LSM6DSL_ADDR        0x6A    /* Motion sensor */
#define LSM6DSL_MUX_PORT    3       /* Multiplexer poort voor sensor */

/* LSM6DSL Registers */
#define LSM6DSL_WHO_AM_I    0x0F
#define LSM6DSL_CTRL1_XL    0x10
#define LSM6DSL_CTRL2_G     0x11
#define LSM6DSL_OUTX_L_G    0x22
#define LSM6DSL_OUTX_L_XL   0x28

/* Sensitivity constants */
#define ACC_SENSITIVITY     0.061f   /* mg/LSB for ?2g */
#define GYRO_SENSITIVITY    17.50f   /* mdps/LSB for ?500dps */

/* Audio buffer */
#define AUDIO_BUFFER_SIZE   4100

/* ==========================================================================
 * CONSTANTEN
 * ========================================================================== */

#define M_PI 3.14159265358979f

/* ==========================================================================
 * HANDLES
 * ========================================================================== */

I2C_HandleTypeDef hi2c1;
SPI_HandleTypeDef hspi3;
I2S_HandleTypeDef hi2s2;
TIM_HandleTypeDef htim2;
UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_i2s2_tx;

DMA_HandleTypeDef hdma_spi2_tx;
DMA_HandleTypeDef hdma_spi3_rx;

/* ==========================================================================
 * GAME STATE MACHINE
 * ========================================================================== */

typedef enum {
    STATE_IDLE,         /* Wacht op start */
    STATE_GAME_START,   /* Initialisatie */
    STATE_ACTIVE,       /* Actief spelen */
    STATE_SHOOT,        /* Schot verwerken */
    STATE_END_GAME      /* Spel be?indigd */
} GameState_t;

/* ==========================================================================
 * GLOBAL VARIABLES
 * ========================================================================== */

/* Game state */
GameState_t currentState = STATE_IDLE;
uint32_t gameStartTime = 0;
uint32_t score = 0;
uint32_t shotsFired = 0;        /* Aantal afgevuurde schoten */
bool gameRunning = false;

/* Sensor ruwe data */
float acc_x, acc_y, acc_z;      /* Accelerometer in g */
float gyro_x, gyro_y, gyro_z;   /* Gyroscope in dps */

/* Huidige ori?ntatie (relatief t.o.v. VPA centrum) */
float azimuth = 0.0f;           /* Horizontale hoek: + = rechts, - = links */
float elevation = 0.0f;         /* Verticale hoek: + = omhoog, - = omlaag */

/* VPA centrum (wordt vastgelegd bij game start) */
float vpa_center_az = 0.0f;
float vpa_center_el = 0.0f;

/* Kalibratie offsets (worden ingesteld bij startup) */
float cal_azimuth_offset = 0.0f;
float cal_elevation_offset = 0.0f;

/* Gyroscope drift compensatie */
float gyro_offset_x = 0.0f;
float gyro_offset_y = 0.0f;
float gyro_offset_z = 0.0f;

/* Balloon position (relatief t.o.v. VPA centrum) */
float balloon_az = 0.0f;        /* -60 tot +60 graden */
float balloon_el = 0.0f;        /* -45 tot +45 graden */

/* Button state */
uint32_t triggerPressTime = 0;
uint32_t startPressTime = 0;
bool triggerPressed = false;
bool startPressed = false;
bool triggerHandled = false;
bool startHandled = false;

/* Audio buffers */
FATFS FatFs;
FIL AudioFile;
UINT bytesRead;
uint8_t audioBuffer1[AUDIO_BUFFER_SIZE];
uint8_t audioBuffer2[AUDIO_BUFFER_SIZE];
uint8_t *currentAudioBuffer;
volatile uint8_t bufferReady = 0;
volatile uint8_t sdReadComplete = 0;
volatile uint8_t activeBuffer = 0;

/* Timing */
uint32_t lastSensorRead = 0;
uint32_t lastDebugPrint = 0;

/* Eigen random generator seed */
static uint32_t g_random_seed = 12345;

/* ==========================================================================
 * FUNCTION PROTOTYPES
 * ========================================================================== */

/* System init */
void SystemClock_Config(void);
void MX_GPIO_Init(void);
void MX_DMA_Init(void);
void MX_I2C1_Init(void);
void MX_SPI3_Init(void);
void MX_I2S2_Init(void);
void MX_TIM2_Init(void);
void MX_USART1_UART_Init(void);
void Error_Handler(void);

/* Sensor functions */
void TCA9548A_SelectChannel(uint8_t channel);
bool LSM6DSL_Init(void);
void LSM6DSL_Read(void);
void CalibrateRestPosition(void);

/* Motion filter */
void UpdateOrientation(float dt);

/* Audio functions */
void Sound_Handler(const char* filename);
void SpeakNumber(uint32_t number);
void SpeakScore(uint32_t shots, uint32_t hits);
void Piezo_SetFrequency(uint16_t freq);
void Piezo_Start(void);
void Piezo_Stop(void);

/* Game functions */
void SpawnBalloon(void);
float CalculateDistance(void);
void UpdateAimTone(void);
bool CheckHit(void);
void HandleTriggerPress(void);
void HandleStartButton(void);
void GameStateMachine(void);

/* Utility */
void UART_Print(const char* fmt, ...);
void UART_SendGyroData(void);  /* NIEUW: Stuur sensor data naar Python visualisatie */

/* Vibration motor */
void Vibration_On(void);
void Vibration_Off(void);

/* ==========================================================================
 * UART PRINT
 * ========================================================================== */

void UART_Print(const char* fmt, ...) {
    char buffer[256];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buffer, sizeof(buffer), fmt, args);
    va_end(args);
    HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
}

/* ==========================================================================
 * UART SEND GYRO DATA - Voor Python visualisatie
 * 
 * Output formaat (uitgebreid met game data):
 * GAME:Az=+12.3,El=+5.6,Baz=+30.0,Bel=+15.0,Score=3,Active=1,Hit=0,Pop=0,Paz=+0.0,Pel=+0.0
 * 
 * Velden:
 * - Az/El: Huidige richt-positie (azimuth/elevatie in graden)
 * - Baz/Bel: Ballon positie (azimuth/elevatie in graden)
 * - Score: Huidige score
 * - Active: 1 = game actief, 0 = idle
 * - Hit: 1 = binnen trefzone, 0 = buiten trefzone
 * - Pop: 1 = ballon zojuist geraakt (voor plof animatie), 0 = normaal
 * - Paz/Pel: Positie waar ballon geplofd is (voor animatie)
 * ========================================================================== */

/* Globale variabelen voor balloon pop event */
static volatile uint8_t balloonPopped = 0;
static float popped_balloon_az = 0.0f;  /* Positie waar ballon geplofd is */
static float popped_balloon_el = 0.0f;

void UART_SendGyroData(void) {
    char buffer[150];
    
    /* Bereken of we binnen de hit zone zijn */
    float distance = CalculateDistance();
    uint8_t inHitZone = (distance <= HIT_RADIUS) ? 1 : 0;
    
    /* Stuur uitgebreide game data inclusief pop event en pop positie */
    snprintf(buffer, sizeof(buffer), 
             "GAME:Az=%+.1f,El=%+.1f,Baz=%+.1f,Bel=%+.1f,Score=%u,Active=%d,Hit=%d,Pop=%d,Paz=%+.1f,Pel=%+.1f\r\n",
             azimuth, elevation,
             balloon_az, balloon_el,
             score,
             gameRunning ? 1 : 0,
             inHitZone,
             balloonPopped,
             popped_balloon_az, popped_balloon_el);
    HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
    
    /* Reset pop flag na verzenden (wordt maar 1x verstuurd) */
    if (balloonPopped) {
        balloonPopped = 0;
    }
}

/* ==========================================================================
 * DMA CALLBACKS
 * ========================================================================== */

void HAL_I2S_TxCpltCallback(I2S_HandleTypeDef *hi2s) {
    bufferReady = 1;
}

/* ==========================================================================
 * I2C MULTIPLEXER
 * ========================================================================== */

void TCA9548A_SelectChannel(uint8_t channel) {
    uint8_t data = (1 << channel);
    HAL_I2C_Master_Transmit(&hi2c1, TCA9548A_ADDR << 1, &data, 1, 100);
}

/* ==========================================================================
 * LSM6DSL SENSOR
 * ========================================================================== */

bool LSM6DSL_Init(void) {
    uint8_t data;
    
    /* Select multiplexer channel */
    TCA9548A_SelectChannel(LSM6DSL_MUX_PORT);
    HAL_Delay(10);
    
    /* Check WHO_AM_I */
    HAL_I2C_Mem_Read(&hi2c1, LSM6DSL_ADDR << 1, LSM6DSL_WHO_AM_I, 1, &data, 1, 100);
    if (data != 0x6A) {
        return false;
    }
    
    /* Software reset */
    data = 0x01;  /* SW_RESET bit */
    HAL_I2C_Mem_Write(&hi2c1, LSM6DSL_ADDR << 1, 0x12, 1, &data, 1, 100);  /* CTRL3_C */
    HAL_Delay(50);
    
    /* Configure accelerometer: 416Hz, ?2g
     * CTRL1_XL (0x10):
     * - ODR_XL[7:4] = 0110 (416 Hz)
     * - FS_XL[3:2] = 00 (?2g)
     */
    data = 0x60;  /* 416Hz, ?2g */
    HAL_I2C_Mem_Write(&hi2c1, LSM6DSL_ADDR << 1, LSM6DSL_CTRL1_XL, 1, &data, 1, 100);
    
    /* Configure gyroscope: 416Hz, ?500dps
     * CTRL2_G (0x11):
     * - ODR_G[7:4] = 0110 (416 Hz)
     * - FS_G[3:2] = 01 (?500dps)
     */
    data = 0x64;  /* 416Hz, ?500dps */
    HAL_I2C_Mem_Write(&hi2c1, LSM6DSL_ADDR << 1, LSM6DSL_CTRL2_G, 1, &data, 1, 100);
    
    /* Enable Block Data Update (BDU) for consistent readings
     * CTRL3_C (0x12):
     * - BDU = 1, IF_INC = 1
     */
    data = 0x44;
    HAL_I2C_Mem_Write(&hi2c1, LSM6DSL_ADDR << 1, 0x12, 1, &data, 1, 100);
    
    /* Configure gyroscope high-pass filter
     * CTRL7_G (0x16):
     * - HP_EN_G = 1 (enable high-pass filter)
     * - HPM_G[5:4] = 00 (16 mHz cutoff)
     */
    data = 0x40;
    HAL_I2C_Mem_Write(&hi2c1, LSM6DSL_ADDR << 1, 0x16, 1, &data, 1, 100);
    
    return true;
}

void LSM6DSL_Read(void) {
    uint8_t buffer[12];
    int16_t raw[6];
    
    TCA9548A_SelectChannel(LSM6DSL_MUX_PORT);
    
    /* Read gyroscope (6 bytes) */
    HAL_I2C_Mem_Read(&hi2c1, LSM6DSL_ADDR << 1, LSM6DSL_OUTX_L_G, 1, buffer, 6, 100);
    
    /* Read accelerometer (6 bytes) */
    HAL_I2C_Mem_Read(&hi2c1, LSM6DSL_ADDR << 1, LSM6DSL_OUTX_L_XL, 1, &buffer[6], 6, 100);
    
    /* Convert to signed 16-bit */
    for (int i = 0; i < 6; i++) {
        raw[i] = (int16_t)(buffer[i*2] | (buffer[i*2+1] << 8));
    }
    
    /* Apply sensitivity */
    gyro_x = raw[0] * GYRO_SENSITIVITY / 1000.0f;  /* dps */
    gyro_y = raw[1] * GYRO_SENSITIVITY / 1000.0f;
    gyro_z = raw[2] * GYRO_SENSITIVITY / 1000.0f;
    
    acc_x = raw[3] * ACC_SENSITIVITY / 1000.0f;    /* g */
    acc_y = raw[4] * ACC_SENSITIVITY / 1000.0f;
    acc_z = raw[5] * ACC_SENSITIVITY / 1000.0f;
}

/* ==========================================================================
 * KALIBRATIE - Bepaal rust positie
 * ========================================================================== */

void CalibrateRestPosition(void) {
    /* Neem gemiddelde van meerdere metingen */
    float sum_acc_x = 0.0f;
    float sum_gz = 0.0f;
    const int samples = 25;
    
    for (int i = 0; i < samples; i++) {
        LSM6DSL_Read();
        
        /* Accelerometer X voor elevatie offset */
        sum_acc_x += acc_x;
        
        /* Gyroscope Z voor azimuth offset */
        sum_gz += gyro_z;
        
        HAL_Delay(10);
    }
    
    /* Gyroscope Z offset */
    gyro_offset_z = sum_gz / (float)samples;
    
    /* Elevatie offset: bereken hoek uit gemiddelde acc_x */
    float avg_acc_x = sum_acc_x / (float)samples;
    if (avg_acc_x > 1.0f) avg_acc_x = 1.0f;
    if (avg_acc_x < -1.0f) avg_acc_x = -1.0f;
    cal_elevation_offset = -asinf(avg_acc_x) * 180.0f / M_PI;
    
    /* Reset hoeken naar 0 */
    azimuth = 0.0f;
    elevation = 0.0f;
}

/* ==========================================================================
 * MOTION FILTER
 * 
 * SENSOR CONFIGURATIE (bepaald door testen):
 * - AZIMUTH (links/rechts): gyro_z - geen zwaartekracht referentie mogelijk
 * - ELEVATIE (omhoog/omlaag): accelerometer X-as - stabiel door zwaartekracht!
 *   - Omhoog = Acc_x negatiever
 *   - Omlaag = Acc_x positiever
 *   - Horizontaal (0) = Acc_x ? 0
 * ========================================================================== */

/* Drempel voor "stil staan" detectie */
#define GYRO_STILL_THRESHOLD  2.0f   /* dps - onder deze waarde = waarschijnlijk stil */
#define DRIFT_CORRECTION_RATE 0.001f /* Hoe snel de offset wordt aangepast */

void UpdateOrientation(float dt) {
    /* =====================================================
     * AZIMUTH: gyro_z (met drift correctie)
     * ===================================================== */
    float gyro_z_comp = gyro_z - gyro_offset_z;
    
    /* Dynamische drift correctie voor azimuth */
    if (gyro_z_comp > -GYRO_STILL_THRESHOLD && gyro_z_comp < GYRO_STILL_THRESHOLD) {
        gyro_offset_z += gyro_z_comp * DRIFT_CORRECTION_RATE;
    }
    
    /* Herbereken na correctie */
    gyro_z_comp = gyro_z - gyro_offset_z;
    
    /* Teken correctie: rechts = positief */
    gyro_z_comp = -gyro_z_comp;
    
    /* Dead zone */
    if (gyro_z_comp > -0.3f && gyro_z_comp < 0.3f) gyro_z_comp = 0.0f;
    
    /* Update azimuth */
    azimuth = azimuth + gyro_z_comp * dt;
    
    /* Normalize naar ?180? */
    while (azimuth > 180.0f) azimuth -= 360.0f;
    while (azimuth < -180.0f) azimuth += 360.0f;
    
    /* =====================================================
     * ELEVATIE: Accelerometer X-as (stabiel, geen drift!)
     * 
     * acc_x meet de zwaartekracht component langs de X-as:
     * - Horizontaal: acc_x ? 0
     * - Omhoog gericht: acc_x < 0 (zwaartekracht trekt "achteruit")
     * - Omlaag gericht: acc_x > 0 (zwaartekracht trekt "vooruit")
     * 
     * We gebruiken asin() omdat acc_x = -sin(elevation) * 1g
     * ===================================================== */
    
    /* Clamp acc_x naar -1.0 tot +1.0 voor asin() */
    float acc_x_clamped = acc_x;
    if (acc_x_clamped > 1.0f) acc_x_clamped = 1.0f;
    if (acc_x_clamped < -1.0f) acc_x_clamped = -1.0f;
    
    /* Bereken elevatie: negatief teken omdat omhoog = acc_x negatiever */
    elevation = -asinf(acc_x_clamped) * 180.0f / M_PI;
    
    /* Pas kalibratie offset toe */
    elevation = elevation - cal_elevation_offset;
    
    /* Clamp naar ?90? */
    if (elevation > 90.0f) elevation = 90.0f;
    if (elevation < -90.0f) elevation = -90.0f;
}

/* ==========================================================================
 * AUDIO - Sound Handler (werkende code)
 * ========================================================================== */

void Sound_Handler(const char* filename) {
    if (f_mount(&FatFs, "", 1) != FR_OK) {
        return;
    }
    
    if (f_open(&AudioFile, filename, FA_READ) != FR_OK) {
        f_mount(NULL, "", 0);
        return;
    }
    
    /* Fill first buffer */
    f_read(&AudioFile, audioBuffer1, AUDIO_BUFFER_SIZE, &bytesRead);
    
    if (bytesRead == 0) {
        f_close(&AudioFile);
        f_mount(NULL, "", 0);
        return;
    }
    
    sdReadComplete = 1;
    bufferReady = 0;
    activeBuffer = 0;
    
    /* Start I2S DMA */
    HAL_I2S_Transmit_DMA(&hi2s2, (uint16_t*)audioBuffer1, AUDIO_BUFFER_SIZE / 2);
    currentAudioBuffer = audioBuffer2;
    
    /* Main playback loop */
    while (bytesRead > 0) {
        if (bufferReady == 1 && sdReadComplete == 1) {
            bufferReady = 0;
            sdReadComplete = 0;
            
            if (activeBuffer == 0) {
                currentAudioBuffer = audioBuffer2;
                HAL_I2S_Transmit_DMA(&hi2s2, (uint16_t*)currentAudioBuffer, AUDIO_BUFFER_SIZE / 2);
                activeBuffer = 1;
            } else {
                currentAudioBuffer = audioBuffer1;
                HAL_I2S_Transmit_DMA(&hi2s2, (uint16_t*)currentAudioBuffer, AUDIO_BUFFER_SIZE / 2);
                activeBuffer = 0;
            }
            
            f_read(&AudioFile, currentAudioBuffer, AUDIO_BUFFER_SIZE, &bytesRead);
            sdReadComplete = 1;
        }
    }
    
    HAL_I2S_DMAStop(&hi2s2);
    f_close(&AudioFile);
    f_mount(NULL, "", 0);
}

/**
 * SpeakNumber - Spreek een getal uit (0-50)
 * 
 * Vereiste bestanden op SD kaart:
 * - n_0.wav tot n_50.wav
 * 
 * Getallen boven 50 worden afgekapt naar 50
 */
void SpeakNumber(uint32_t number) {
    char filename[16];
    
    /* Beperk tot 0-50 */
    if (number > 50) {
        number = 50;
    }
    
    /* Bouw bestandsnaam: n_0.wav, n_1.wav, etc. */
    snprintf(filename, sizeof(filename), "n_%u.wav", number);
    
    Sound_Handler(filename);
}

/**
 * SpeakScore - Spreek de eindscore uit
 * 
 * Speelt: "Je hebt [X] keer geschoten en [Y] ballonnen geraakt"
 * 
 * Vereiste bestanden op SD kaart:
 * - sc_jh.wav              ("Je hebt")
 * - n_0.wav tot n_50.wav   (getallen)
 * - sc_ks.wav              ("keer geschoten en")
 * - sc_bg.wav              ("ballonnen geraakt")
 */
void SpeakScore(uint32_t shots, uint32_t hits) {
    /* "Je hebt" */
    Sound_Handler("sc_jh.wav");
    
    /* [aantal schoten] */
    SpeakNumber(shots);
    
    /* "keer geschoten en" */
    Sound_Handler("sc_ks.wav");
    
    /* [aantal raak] */
    SpeakNumber(hits);
    
    /* "ballonnen geraakt" */
    Sound_Handler("sc_bg.wav");
}

/* ==========================================================================
 * PIEZO PWM - Richt-toon Feedback
 * 
 * Toon frequentie gebaseerd op afstand tot ballon:
 * - Veraf (max afstand): MIN_TONE_FREQ (300 Hz) - lage toon
 * - Dichtbij (< HIT_RADIUS): MAX_TONE_FREQ (2500 Hz) - hoge toon
 * ========================================================================== */

void Piezo_SetFrequency(uint16_t freq) {
    if (freq < MIN_TONE_FREQ) freq = MIN_TONE_FREQ;
    if (freq > MAX_TONE_FREQ) freq = MAX_TONE_FREQ;
    
    /* Timer clock = 84MHz / (prescaler+1) = 84MHz / 84 = 1MHz */
    /* Period = 1MHz / freq */
    uint32_t period = 1000000 / freq;
    
    __HAL_TIM_SET_AUTORELOAD(&htim2, period - 1);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, period / 2);  /* 50% duty */
}

void Piezo_Start(void) {
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
}

void Piezo_Stop(void) {
    HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
}

/* ==========================================================================
 * VIBRATION MOTOR - Haptische feedback bij doel in schootsveld
 * ========================================================================== */

void Vibration_On(void) {
    HAL_GPIO_WritePin(VIBRATION_PORT, VIBRATION_PIN, GPIO_PIN_SET);
}

void Vibration_Off(void) {
    HAL_GPIO_WritePin(VIBRATION_PORT, VIBRATION_PIN, GPIO_PIN_RESET);
}

/* ==========================================================================
 * GAME LOGIC
 * ========================================================================== */

/**
 * SimpleRandom - Eigen Linear Congruential Generator
 * Retourneert een waarde tussen 0 en 999
 */
uint32_t SimpleRandom(void) {
    /* LCG parameters (dezelfde als glibc) */
    g_random_seed = g_random_seed * 1103515245 + 12345;
    return (g_random_seed >> 16) % 1000;
}


/**
 * SpawnBalloon - Plaats een nieuwe ballon binnen de VPA
 * 
 * VPA bereik:
 * - Azimuth: -60 tot +60 graden (relatief t.o.v. VPA centrum)
 * - Elevatie: 0 tot +25 graden (relatief t.o.v. VPA centrum)
 * 
 * Ballonnen worden NIET te dicht bij de rand geplaatst (marge van 5 graden)
 */
#define VPA_MARGIN  5.0f  /* Minimale afstand van VPA rand in graden */

void SpawnBalloon(void) {
    /* Gebruik eigen random generator */
    uint32_t rand1 = SimpleRandom();  /* 0-999 */
    uint32_t rand2 = SimpleRandom();  /* 0-999 */
    
    /* Bereken bruikbaar bereik (VPA minus marge aan beide kanten) */
    float az_range = VPA_AZIMUTH_RANGE - VPA_MARGIN;  /* 60 - 5 = 55 graden */
    float el_min = VPA_ELEVATION_MIN + VPA_MARGIN;    /* 0 + 5 = 5 graden */
    float el_max = VPA_ELEVATION_MAX - VPA_MARGIN;    /* 25 - 5 = 20 graden */
    
    /* Azimuth: -55 tot +55 graden (met marge) */
    float rand_az = ((float)rand1 / 500.0f) - 1.0f;  /* -1.0 tot +1.0 */
    balloon_az = rand_az * az_range;
    
    /* Elevatie: 5 tot 20 graden (met marge, alleen BOVEN horizon) */
    float rand_el = (float)rand2 / 1000.0f;  /* 0.0 tot 1.0 */
    balloon_el = el_min + rand_el * (el_max - el_min);
    
    /* Zorg dat ballon niet te dicht bij huidige positie spawnt (minimaal 15 graden) */
    float dist = sqrtf((azimuth - balloon_az) * (azimuth - balloon_az) + 
                       (elevation - balloon_el) * (elevation - balloon_el));
    
    if (dist < 15.0f) {
        /* Te dichtbij, verplaats ballon verder weg */
        if (balloon_az >= 0) {
            balloon_az = fminf(balloon_az + 25.0f, az_range);
        } else {
            balloon_az = fmaxf(balloon_az - 25.0f, -az_range);
        }
    }
}

/**
 * CalculateDistance - Bereken hoekafstand tussen pistool en ballon
 * 
 * Retourneert afstand in graden
 */
float CalculateDistance(void) {
    /* Verschil in azimuth en elevatie */
    float daz = azimuth - balloon_az;
    float del = elevation - balloon_el;
    
    /* Euclidische afstand in graden */
    return sqrtf(daz * daz + del * del);
}


/**
 * UpdateAimTone - Update piezo toon gebaseerd op afstand tot ballon
 * 
 * Feedback:
 * - Buiten VPA: Korte piepjes (aan/uit) als waarschuwing
 * - Binnen VPA: Continue toon, frequentie afhankelijk van afstand
 * - Binnen trefzone (< HIT_RADIUS): Hoge toon + vibration motor
 */
static uint16_t last_freq = 0;
static uint32_t lastBeepTime = 0;
static bool beepState = false;

/* Beep timing voor buiten VPA */
#define BEEP_ON_TIME    80    /* ms piep aan */
#define BEEP_OFF_TIME   200   /* ms stilte */
#define BEEP_FREQ       800   /* Hz voor waarschuwingspiep */

void UpdateAimTone(void) {
    if (!gameRunning) return;
    
    /* Check of pistool binnen VPA is */
    bool in_vpa = (azimuth >= -VPA_AZIMUTH_RANGE && azimuth <= VPA_AZIMUTH_RANGE &&
                   elevation >= VPA_ELEVATION_MIN && elevation <= VPA_ELEVATION_MAX);
    
    uint16_t freq;
    bool in_hit_zone = false;  /* Track of we binnen schootsveld zijn */
    
    if (!in_vpa) {
        /* Buiten VPA - korte piepjes als waarschuwing */
        uint32_t now = HAL_GetTick();
        
        if (beepState) {
            /* Piep is aan, check of tijd om uit te zetten */
            if (now - lastBeepTime >= BEEP_ON_TIME) {
                Piezo_Stop();
                beepState = false;
                lastBeepTime = now;
            }
        } else {
            /* Piep is uit, check of tijd voor nieuwe piep */
            if (now - lastBeepTime >= BEEP_OFF_TIME) {
                Piezo_SetFrequency(BEEP_FREQ);
                Piezo_Start();
                beepState = true;
                lastBeepTime = now;
            }
        }
        
        /* Vibration motor UIT buiten VPA */
        Vibration_Off();
        return;  /* Geen verdere verwerking nodig */
        
    } else {
        /* Binnen VPA - zorg dat piezo aan staat (continue toon) */
        if (!beepState) {
            Piezo_Start();
        }
        beepState = true;  /* Markeer dat we in continue modus zijn */
        
        /* Bereken afstand tot ballon */
        float distance = CalculateDistance();
        
        /* Maximale afstand in VPA (diagonaal) */
        float max_distance = sqrtf(VPA_AZIMUTH_RANGE * VPA_AZIMUTH_RANGE + 
                                   VPA_ELEVATION_MAX * VPA_ELEVATION_MAX);
        
        if (distance <= HIT_RADIUS) {
            /* Binnen trefzone - constante hoge toon */
            freq = MAX_TONE_FREQ;
            in_hit_zone = true;  /* Binnen schootsveld! */
        } else if (distance >= max_distance) {
            /* Maximale afstand - lage toon */
            freq = MIN_TONE_FREQ;
        } else {
            /* Lineair interpoleren: dichterbij = hogere toon */
            float normalized = (distance - HIT_RADIUS) / (max_distance - HIT_RADIUS);
            freq = MAX_TONE_FREQ - (uint16_t)(normalized * (MAX_TONE_FREQ - MIN_TONE_FREQ));
        }
    }
    
    /* Vibration motor: AAN als binnen schootsveld, anders UIT */
    if (in_hit_zone) {
        Vibration_On();
    } else {
        Vibration_Off();
    }
    
    /* Stabilisatie: alleen updaten als verschil groot genoeg is (>50 Hz) */
    int16_t diff = (int16_t)freq - (int16_t)last_freq;
    if (diff < 0) diff = -diff;  /* abs() */
    
    if (diff > 50 || last_freq == 0) {
        Piezo_SetFrequency(freq);
        last_freq = freq;
    }
}

/**
 * CheckHit - Controleer of schot de ballon raakt
 */
bool CheckHit(void) {
    float distance = CalculateDistance();
    return (distance <= HIT_RADIUS);
}

/* ==========================================================================
 * BUTTON HANDLING (met debouncing)
 * ========================================================================== */

void HandleTriggerPress(void) {
    bool btnPressed = (HAL_GPIO_ReadPin(GPIO_PA1_GPIO_Port, GPIO_PA1_Pin) == GPIO_PIN_RESET);
    
    if (btnPressed && !triggerPressed) {
        /* Button just pressed */
        triggerPressTime = HAL_GetTick();
        triggerPressed = true;
        triggerHandled = false;
    }
    else if (!btnPressed && triggerPressed) {
        /* Button released */
        if (!triggerHandled && (HAL_GetTick() - triggerPressTime) > DEBOUNCE_MS) {
            /* Valid short press - SHOOT! */
            if (gameRunning) {
                currentState = STATE_SHOOT;
            }
        }
        triggerPressed = false;
    }
}

void HandleStartButton(void) {
    bool btnPressed = (HAL_GPIO_ReadPin(GPIO_PA2_GPIO_Port, GPIO_PA2_Pin) == GPIO_PIN_RESET);
    
    if (btnPressed && !startPressed) {
        startPressTime = HAL_GetTick();
        startPressed = true;
        startHandled = false;
    }
    else if (btnPressed && startPressed && !startHandled) {
        /* Check for long press */
        if ((HAL_GetTick() - startPressTime) > LONG_PRESS_MS) {
            /* Long press - stop game */
            if (gameRunning) {
                currentState = STATE_END_GAME;
            }
            startHandled = true;
        }
    }
    else if (!btnPressed && startPressed) {
        /* Button released */
        if (!startHandled && (HAL_GetTick() - startPressTime) > DEBOUNCE_MS) {
            /* Short press - start game */
            if (!gameRunning) {
                currentState = STATE_GAME_START;
            }
        }
        startPressed = false;
    }
}

/* ==========================================================================
 * GAME STATE MACHINE
 * ========================================================================== */

void GameStateMachine(void) {
    uint32_t elapsedTime;
    
    switch (currentState) {
        
        case STATE_IDLE:
            /* Wacht op start knop - wordt afgehandeld in HandleStartButton() */
            break;
            
        case STATE_GAME_START:
            /* Speel instructie geluid EERST */
            Sound_Handler("e_2_min.wav");

            /* KALIBRATIE na audio */
            HAL_Delay(1500);  /* Geef speler tijd om stil te houden */
            
            /* Kalibreer sensor */
            CalibrateRestPosition();				         
          
            /* BELANGRIJK: Reset hoeken naar 0 NA kalibratie */
            azimuth = 0.0f;
            elevation = 0.0f;
        
            /* Daadwerkelijke start van het spel aangeven */
            Sound_Handler("e_start1.wav");                       

            /* Reset score, schoten en timer */
            score = 0;
            shotsFired = 0;
            gameStartTime = HAL_GetTick();
            gameRunning = true;
            
            /* Spawn eerste ballon */
            SpawnBalloon();
            
            /* Start piezo feedback */
            Piezo_Start();
            
            /* Speel "start" geluid */
            Sound_Handler("e_go.wav");
            
            currentState = STATE_ACTIVE;
            break;
            
        case STATE_ACTIVE:
            /* Check speeltijd */
            elapsedTime = (HAL_GetTick() - gameStartTime) / 1000;
            if (elapsedTime >= GAME_DURATION) {
                currentState = STATE_END_GAME;
                break;
            }
            
            /* Tijdswaarschuwingen */
            {
                static bool warned1Min = false;
                static bool warned30Sec = false;
                uint32_t remainingTime = GAME_DURATION - elapsedTime;
                
                /* Nog 1 minuut waarschuwing */
                if (!warned1Min && remainingTime <= 60 && remainingTime > 30) {
                    warned1Min = true;
                    Piezo_Stop();
                    Sound_Handler("ng_1min.wav");
                    Piezo_Start();
                }
                
                /* Nog 30 seconden waarschuwing */
                if (!warned30Sec && remainingTime <= 30) {
                    warned30Sec = true;
                    Piezo_Stop();
                    Sound_Handler("ng_30sec.wav");
                    Piezo_Start();
                }
                
                /* Reset flags bij nieuwe game (wanneer elapsedTime weer laag is) */
                if (elapsedTime < 10) {
                    warned1Min = false;
                    warned30Sec = false;
                }
            }
            
            /* Update aim feedback - maar niet te vaak (max 20Hz) */
            {
                static uint32_t lastToneUpdate = 0;
                if (HAL_GetTick() - lastToneUpdate >= 50) {  /* 50ms = 20Hz */
                    lastToneUpdate = HAL_GetTick();
                    UpdateAimTone();
                }
            }
            break;
            
        case STATE_SHOOT:
            /* Tel schot */
            shotsFired++;
            
            /* Stop piezo en vibration tijdens geluidseffecten */
            Piezo_Stop();
            Vibration_Off();
            
            /* Speel schot geluid */
            //Sound_Handler("laser_x2.wav");
						Sound_Handler("laser_x1.wav");
            
            /* Check hit */
            if (CheckHit()) {
                score++;
                
                /* BELANGRIJK: Sla OUDE ballon positie op VOORDAT nieuwe wordt gespawned */
                popped_balloon_az = balloon_az;
                popped_balloon_el = balloon_el;
                
                /* Zet pop flag voor Python visualisatie */
                balloonPopped = 1;
								UART_SendGyroData();
								balloon_az = -10.0f; 
								balloon_el = -10.0f;
                UART_SendGyroData();
														
                Sound_Handler("knal_1.wav");
								//Sound_Handler("b_pop.wav");
                Sound_Handler("e_raak.wav");
                
                /* Spawn nieuwe ballon op andere positie */
                SpawnBalloon();
            } else {
                Sound_Handler("e_mis.wav");
            }
            
            /* Hervat piezo (als tijd over) */
            elapsedTime = (HAL_GetTick() - gameStartTime) / 1000;
            if (elapsedTime < GAME_DURATION) {
                Piezo_Start();
                currentState = STATE_ACTIVE;
            } else {
                currentState = STATE_END_GAME;
            }
            break;
            
        case STATE_END_GAME:
            /* Voorkom dat end game meerdere keren wordt uitgevoerd */
            {
                static bool endGameHandled = false;
                
                if (endGameHandled) {
                    /* Al afgehandeld, wacht tot alles klaar is */
                    break;
                }
                endGameHandled = true;
                
                /* Stop piezo en vibration */
                Piezo_Stop();
                Vibration_Off();
                gameRunning = false;
                
                /* Speel eind geluid */
                Sound_Handler("e_over.wav");
                
                /* Verschillende eindmeldingen afhankelijk van score */
                if (shotsFired == 0) {
                    /* Geen schot gelost - alleen e_niet.wav */
                    Sound_Handler("e_niet.wav");
                    
                } else if (score == 1) {
                    /* Precies 1 ballon geraakt - aantal schoten + e_1_bal.wav */
                    
                    /* "Je hebt" */
                    Sound_Handler("sc_jh.wav");
                    
                    /* [aantal schoten] */
                    SpeakNumber(shotsFired);
                    
                    /* "keer geschoten en" */
                    Sound_Handler("sc_ks.wav");
                    
                    /* Speciale melding voor 1 ballon */
                    Sound_Handler("e_1_bal.wav");
                    
                } else {
                    /* Meerdere ballonnen geraakt - volledige score */
                    SpeakScore(shotsFired, score);
                }
                
                /* Reset voor volgende game */
                endGameHandled = false;
                
                currentState = STATE_IDLE;
            }
            break;
    }
}

/* ==========================================================================
 * MAIN
 * ========================================================================== */

int main(void) {
    /* Initialize HAL */
    HAL_Init();
    SystemClock_Config();
    
    /* Initialize peripherals - DMA MOET VOOR I2S! */
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_USART1_UART_Init();
    MX_I2C1_Init();
    MX_SPI3_Init();
    MX_I2S2_Init();
    MX_TIM2_Init();
    MX_FATFS_Init();
    
    /* Initialize sensor */
    if (!LSM6DSL_Init()) {
        /* Sensor init failed - blijf in error state */
        while(1) {
            HAL_Delay(1000);
        }
    }
    
    /* Seed random generator met huidige tick */
    g_random_seed = HAL_GetTick() ^ 0xDAADEFEF;
    
		Sound_Handler("e_s_aan.wav");  /* Indicatie dat spelpistool aan staat */
    
    /* Main loop */
    while (1) {
        uint32_t now = HAL_GetTick();
        
        /* Sensor reading at SAMPLE_RATE Hz */
        if (now - lastSensorRead >= (1000 / SAMPLE_RATE)) {
            float dt = (now - lastSensorRead) / 1000.0f;
            lastSensorRead = now;
            
            LSM6DSL_Read();
            UpdateOrientation(dt);
        }
        
        /* Button handling */
        HandleTriggerPress();
        HandleStartButton();
        
        /* Game state machine */
        GameStateMachine();
        
        /* GYRO DATA OUTPUT voor Python visualisatie - elke 100ms (10 Hz) */
        if (now - lastDebugPrint >= 100) {
            lastDebugPrint = now;
            UART_SendGyroData();
        }
    }
}

/* ==========================================================================
 * PERIPHERAL INITIALIZATION
 * ========================================================================== */

void MX_GPIO_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    
    /* Enable GPIO clocks */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    
    /* SD Card CS pin - SD_CS_PIN_Pin (PB0) */
    HAL_GPIO_WritePin(SD_CS_PIN_GPIO_Port, SD_CS_PIN_Pin, GPIO_PIN_SET);
    GPIO_InitStruct.Pin = SD_CS_PIN_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(SD_CS_PIN_GPIO_Port, &GPIO_InitStruct);
    
    /* Trigger button - GPIO_PA1_Pin (PA1) - Active LOW with pull-up */
    GPIO_InitStruct.Pin = GPIO_PA1_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIO_PA1_GPIO_Port, &GPIO_InitStruct);
    
    /* Start/Stop button - GPIO_PA2_Pin (PA2) - Active LOW with pull-up */
    GPIO_InitStruct.Pin = GPIO_PA2_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIO_PA2_GPIO_Port, &GPIO_InitStruct);
    
    /* Green LED - Green_LED_PA5_Pin (PA5) */
    HAL_GPIO_WritePin(Green_LED_PA5_GPIO_Port, Green_LED_PA5_Pin, GPIO_PIN_SET);
    GPIO_InitStruct.Pin = Green_LED_PA5_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(Green_LED_PA5_GPIO_Port, &GPIO_InitStruct);
    
    /* Vibration motor - PB5 - Start UIT */
    HAL_GPIO_WritePin(VIBRATION_PORT, VIBRATION_PIN, GPIO_PIN_RESET);
    GPIO_InitStruct.Pin = VIBRATION_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(VIBRATION_PORT, &GPIO_InitStruct);
    
    /* I2C Multiplexer Reset - I2C_MUX_RST_Pin (PC0) */
    HAL_GPIO_WritePin(I2C_MUX_RST_GPIO_Port, I2C_MUX_RST_Pin, GPIO_PIN_SET);
    GPIO_InitStruct.Pin = I2C_MUX_RST_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(I2C_MUX_RST_GPIO_Port, &GPIO_InitStruct);
}

void MX_DMA_Init(void) {
    __HAL_RCC_DMA1_CLK_ENABLE();
    
    hdma_i2s2_tx.Instance = DMA1_Stream4;
    hdma_i2s2_tx.Init.Channel = DMA_CHANNEL_0;
    hdma_i2s2_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_i2s2_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_i2s2_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_i2s2_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_i2s2_tx.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    hdma_i2s2_tx.Init.Mode = DMA_NORMAL;
    hdma_i2s2_tx.Init.Priority = DMA_PRIORITY_HIGH;
    hdma_i2s2_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    
    HAL_DMA_Init(&hdma_i2s2_tx);
    __HAL_LINKDMA(&hi2s2, hdmatx, hdma_i2s2_tx);
    
    HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);
}

void MX_I2C1_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    
    __HAL_RCC_I2C1_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    
    /* I2C1: PB8=SCL, PB9=SDA */
    GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    
    hi2c1.Instance = I2C1;
    hi2c1.Init.ClockSpeed = 400000;
    hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
    hi2c1.Init.OwnAddress1 = 0;
    hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    
    HAL_I2C_Init(&hi2c1);
}

void MX_SPI3_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    
    __HAL_RCC_SPI3_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    
    GPIO_InitStruct.Pin = GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
    
    hspi3.Instance = SPI3;
    hspi3.Init.Mode = SPI_MODE_MASTER;
    hspi3.Init.Direction = SPI_DIRECTION_2LINES;
    hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
    hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
    hspi3.Init.NSS = SPI_NSS_SOFT;
    hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
    hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    
    HAL_SPI_Init(&hspi3);
}

void MX_I2S2_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    
    __HAL_RCC_SPI2_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    
    GPIO_InitStruct.Pin = GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_15;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    
    hi2s2.Instance = SPI2;
    hi2s2.Init.Mode = I2S_MODE_MASTER_TX;
    hi2s2.Init.Standard = I2S_STANDARD_PHILIPS;
    hi2s2.Init.DataFormat = I2S_DATAFORMAT_16B;
    hi2s2.Init.MCLKOutput = I2S_MCLKOUTPUT_DISABLE;
    hi2s2.Init.AudioFreq = I2S_AUDIOFREQ_26K;
    hi2s2.Init.CPOL = I2S_CPOL_LOW;
    hi2s2.Init.ClockSource = I2S_CLOCK_PLL;
    hi2s2.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
    
    HAL_I2S_Init(&hi2s2);
}

void MX_TIM2_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    TIM_OC_InitTypeDef sConfigOC = {0};
    
    __HAL_RCC_TIM2_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    
    /* PA0 = TIM2_CH1 */
    GPIO_InitStruct.Pin = GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    
    /* Timer: 84MHz / 84 = 1MHz tick */
    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 83;
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = 999;  /* Default 1kHz */
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    
    HAL_TIM_PWM_Init(&htim2);
    
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 500;  /* 50% duty */
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    
    HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1);
}

void MX_USART1_UART_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    
    __HAL_RCC_USART1_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    
    GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    
    huart1.Instance = USART1;
    huart1.Init.BaudRate = 115200;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    
    HAL_UART_Init(&huart1);
}

void SystemClock_Config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
    
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);
    
    /* HSE met jouw werkende PLL settings */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 12;
    RCC_OscInitStruct.PLL.PLLN = 84;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 4;
    
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }
    
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
        Error_Handler();
    }
}

void Error_Handler(void) {
    __disable_irq();
    while (1);
}


