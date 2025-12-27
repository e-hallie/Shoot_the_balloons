# Virtuele Ballonnenschieter 🎯🎈

Een audiogame voor visueel gehandicapte spelers, ontwikkeld op de STM32F401RET6 microcontroller.

## Beschrijving

De Virtuele Ballonnenschieter is een schietspel waarbij spelers met behulp van audio-feedback virtuele ballonnen moeten raken. Het spel is speciaal ontworpen voor mensen met een visuele beperking en maakt gebruik van:

- **3D audio-feedback** via een piezo buzzer (toonhoogte geeft afstand tot ballon aan)
- **Tactiele feedback** via een vibration motor (trillen wanneer je op de ballon richt)
- **Gesproken instructies** via WAV-bestanden op SD-kaart

## Hardware

| Component | Beschrijving | Aansluiting |
|-----------|--------------|-------------|
| STM32F401RET6 | Microcontroller | - |
| LSM6DSL | 6-axis IMU (gyroscoop + accelerometer) | I2C via TCA9548A poort 3 |
| TCA9548A | I2C Multiplexer | I2C1 (PB8/PB9) |
| MAX98357A | I2S Audio DAC | I2S2 (PB12/13/15) |
| Piezo buzzer | Richt-feedback | PWM TIM2_CH1 (PA0) |
| Vibration motor | Trefzone-feedback | GPIO (PB5) |
| SD-kaart module | Audio opslag | SPI3 (PC10/11/12), CS=PB0 |
| Trigger knop | Schieten | PA1 (actief laag) |
| Start/Stop knop | Game besturing | PA2 (actief laag) |

### Aansluitschema

```
STM32F401RET6
├── I2C1 (PB8=SCL, PB9=SDA)
│   └── TCA9548A Multiplexer (0x70)
│       └── Poort 3: LSM6DSL (0x6A)
├── I2S2 (PB12=WS, PB13=CK, PB15=SD)
│   └── MAX98357A → Speaker
├── SPI3 (PC10=SCK, PC11=MISO, PC12=MOSI)
│   └── SD-kaart module (CS=PB0)
├── TIM2_CH1 (PA0)
│   └── Piezo buzzer
├── GPIO (PB5)
│   └── Vibration motor
└── GPIO Inputs
    ├── PA1: Trigger (PULLUP)
    └── PA2: Start/Stop (PULLUP)
```

## Gameplay

### Besturing
- **Korte druk Start-knop (PA2)**: Start nieuw spel
- **Lange druk Start-knop (2 sec)**: Stop spel voortijdig
- **Trigger-knop (PA1)**: Schiet op ballon

### Spelverloop
1. Druk op Start → Kalibratie instructie + 1.5 sec stilhouden
2. Sensor kalibreert de rustpositie als VPA-centrum
3. "Start" geluid → Spel begint (120 seconden)
4. Zoek ballonnen met audio-feedback:
   - **Lage toon**: Ballon is ver weg
   - **Hoge toon**: Ballon is dichtbij
   - **Hoogste toon + vibratie**: Binnen trefzone!
5. Schiet met trigger-knop
6. Tijdswaarschuwingen: "Nog 1 minuut" en "Nog 30 seconden"
7. Na 120 sec: Score wordt voorgelezen

### Audio Feedback
| Situatie | Feedback |
|----------|----------|
| Buiten speelveld (VPA) | Korte piepjes (waarschuwing) |
| Binnen VPA, ver van ballon | Lage continue toon (~300 Hz) |
| Binnen VPA, dichtbij ballon | Hoge continue toon (~2500 Hz) |
| Binnen trefzone | Hoogste toon + vibration motor |

## Configuratie

Belangrijke parameters in `main.c`:

```c
#define GAME_DURATION       120      /* Speeltijd in seconden */
#define HIT_RADIUS          15.0f    /* Trefradius in graden */
#define VPA_AZIMUTH_RANGE   60.0f    /* Horizontaal bereik ±60° */
#define VPA_ELEVATION_MIN   0.0f     /* Minimale elevatie (horizon) */
#define VPA_ELEVATION_MAX   25.0f    /* Maximale elevatie */
#define MIN_TONE_FREQ       300      /* Piezo frequentie (ver) */
#define MAX_TONE_FREQ       2500     /* Piezo frequentie (dichtbij) */
```

## SD-kaart Bestanden

Plaats de volgende WAV-bestanden op de SD-kaart (root directory):

| Bestand | Beschrijving |
|---------|--------------|
| `e_2_min.wav` | "Je hebt 2 minuten" (instructie) |
| `e_start1.wav` | Start instructie |
| `e_go.wav` | "Go!" |
| `e_raak.wav` | "Raak!" |
| `e_mis.wav` | "Mis!" |
| `e_over.wav` | "Game over" |
| `e_niet.wav` | "Je hebt niet geschoten" |
| `e_1_bal.wav` | "1 ballon geraakt" |
| `ng_1min.wav` | "Nog 1 minuut" |
| `ng_30sec.wav` | "Nog 30 seconden" |
| `laser_x1.wav` | Schot geluidseffect |
| `knal_1.wav` | Ballon knal effect |
| `sc_jh.wav` | "Je hebt" |
| `sc_ks.wav` | "keer geschoten en" |
| `0.wav` - `20.wav` | Getallen 0-20 |
| `30.wav` - `90.wav` | Tientallen |
| `100.wav` | "honderd" |
| `sc_bg.wav` | "ballonnen geraakt" |

**WAV formaat**: 22kHz of 26kHz, 16-bit, mono

## Bouwen

### Vereisten
- STM32CubeIDE of arm-none-eabi-gcc
- STM32F4 HAL drivers
- FatFS middleware

### Compileren
```bash
# Met STM32CubeIDE: importeer project en build
# Of met Makefile:
make all
```

## Sensor Oriëntatie

De LSM6DSL moet als volgt georiënteerd zijn:
- **X-as**: Recht vooruit (schietrichting)
- **Y-as**: Links/rechts
- **Z-as**: Omhoog/omlaag

De code gebruikt:
- **Azimuth (links/rechts)**: Gyroscoop Z-as met drift correctie
- **Elevatie (op/neer)**: Accelerometer X-as (geen drift!)

## Versiegeschiedenis

### v3.2 (27 december 2025)
- Tijdswaarschuwingen toegevoegd ("nog 1 minuut" en "nog 30 seconden")

### v3.1 (17 december 2025)
- VPA margins aangepast voor betere ballon placement
- Ballon spawn range: 5°-20° elevatie, ±55° azimuth

### v3.0
- Verbeterde score rapportage met verschillende scenario's
- Python visualisatie ondersteuning via UART

### v2.0
- Stabiele azimuth tracking met dynamische drift correctie
- Elevatie via accelerometer (drift-vrij)

## Licentie

Dit project is ontwikkeld door Raymond Hallie.

## Bijdragen

Suggesties en verbeteringen zijn welkom! Open een issue of pull request.







# Virtual Balloon Shooter 🎯🎈

An audio game for visually impaired players, developed on the STM32F401RET6 microcontroller.

## Description

The Virtual Balloon Shooter is a shooting game where players must hit virtual balloons using audio feedback. The game is specifically designed for people with visual impairments and uses:

- **3D audio feedback** via a piezo buzzer (pitch indicates distance to balloon)
- **Tactile feedback** via a vibration motor (vibrates when aiming at the balloon)
- **Voice instructions** via WAV files stored on SD card

## Hardware

| Component | Description | Connection |
|-----------|-------------|------------|
| STM32F401RET6 | Microcontroller | - |
| LSM6DSL | 6-axis IMU (gyroscope + accelerometer) | I2C via TCA9548A port 3 |
| TCA9548A | I2C Multiplexer | I2C1 (PB8/PB9) |
| MAX98357A | I2S Audio DAC | I2S2 (PB12/13/15) |
| Piezo buzzer | Aiming feedback | PWM TIM2_CH1 (PA0) |
| Vibration motor | Hit zone feedback | GPIO (PB5) |
| SD card module | Audio storage | SPI3 (PC10/11/12), CS=PB0 |
| Trigger button | Shoot | PA1 (active low) |
| Start/Stop button | Game control | PA2 (active low) |

### Wiring Diagram

```
STM32F401RET6
├── I2C1 (PB8=SCL, PB9=SDA)
│   └── TCA9548A Multiplexer (0x70)
│       └── Port 3: LSM6DSL (0x6A)
├── I2S2 (PB12=WS, PB13=CK, PB15=SD)
│   └── MAX98357A → Speaker
├── SPI3 (PC10=SCK, PC11=MISO, PC12=MOSI)
│   └── SD card module (CS=PB0)
├── TIM2_CH1 (PA0)
│   └── Piezo buzzer
├── GPIO (PB5)
│   └── Vibration motor
└── GPIO Inputs
    ├── PA1: Trigger (PULLUP)
    └── PA2: Start/Stop (PULLUP)
```

## Gameplay

### Controls
- **Short press Start button (PA2)**: Start new game
- **Long press Start button (2 sec)**: Stop game early
- **Trigger button (PA1)**: Shoot at balloon

### Game Flow
1. Press Start → Calibration instruction + hold still for 1.5 sec
2. Sensor calibrates rest position as VPA center
3. "Start" sound → Game begins (120 seconds)
4. Find balloons using audio feedback:
   - **Low pitch**: Balloon is far away
   - **High pitch**: Balloon is nearby
   - **Highest pitch + vibration**: Within hit zone!
5. Shoot with trigger button
6. Time warnings: "One minute remaining" and "30 seconds remaining"
7. After 120 sec: Score is announced

### Audio Feedback
| Situation | Feedback |
|-----------|----------|
| Outside play area (VPA) | Short beeps (warning) |
| Inside VPA, far from balloon | Low continuous tone (~300 Hz) |
| Inside VPA, close to balloon | High continuous tone (~2500 Hz) |
| Within hit zone | Highest tone + vibration motor |

## Configuration

Key parameters in `main.c`:

```c
#define GAME_DURATION       120      /* Play time in seconds */
#define HIT_RADIUS          15.0f    /* Hit radius in degrees */
#define VPA_AZIMUTH_RANGE   60.0f    /* Horizontal range ±60° */
#define VPA_ELEVATION_MIN   0.0f     /* Minimum elevation (horizon) */
#define VPA_ELEVATION_MAX   25.0f    /* Maximum elevation */
#define MIN_TONE_FREQ       300      /* Piezo frequency (far) */
#define MAX_TONE_FREQ       2500     /* Piezo frequency (close) */
```

## SD Card Files

Place the following WAV files on the SD card (root directory):

| File | Description |
|------|-------------|
| `e_2_min.wav` | "You have 2 minutes" (instruction) |
| `e_start1.wav` | Start instruction |
| `e_go.wav` | "Go!" |
| `e_raak.wav` | "Hit!" |
| `e_mis.wav` | "Miss!" |
| `e_over.wav` | "Game over" |
| `e_niet.wav` | "You didn't shoot" |
| `e_1_bal.wav` | "1 balloon hit" |
| `ng_1min.wav` | "One minute remaining" |
| `ng_30sec.wav` | "30 seconds remaining" |
| `laser_x1.wav` | Shot sound effect |
| `knal_1.wav` | Balloon pop effect |
| `sc_jh.wav` | "You have" |
| `sc_ks.wav` | "shots fired and" |
| `0.wav` - `20.wav` | Numbers 0-20 |
| `30.wav` - `90.wav` | Tens |
| `100.wav` | "hundred" |
| `sc_bg.wav` | "balloons hit" |

**WAV format**: 22kHz or 26kHz, 16-bit, mono

## Building

### Requirements
- STM32CubeIDE or arm-none-eabi-gcc
- STM32F4 HAL drivers
- FatFS middleware

### Compiling
```bash
# With STM32CubeIDE: import project and build
# Or with Makefile:
make all
```

## Sensor Orientation

The LSM6DSL must be oriented as follows:
- **X-axis**: Straight ahead (shooting direction)
- **Y-axis**: Left/right
- **Z-axis**: Up/down

The code uses:
- **Azimuth (left/right)**: Gyroscope Z-axis with drift correction
- **Elevation (up/down)**: Accelerometer X-axis (no drift!)

## Version History

### v3.2 (December 27, 2025)
- Added time warnings ("one minute remaining" and "30 seconds remaining")

### v3.1 (December 17, 2025)
- Adjusted VPA margins for better balloon placement
- Balloon spawn range: 5°-20° elevation, ±55° azimuth

### v3.0
- Improved score reporting with different scenarios
- Python visualization support via UART

### v2.0
- Stable azimuth tracking with dynamic drift correction
- Elevation via accelerometer (drift-free)

## License

This project was developed by Raymond Hallie.

## Contributing

Suggestions and improvements are welcome! Open an issue or pull request.


