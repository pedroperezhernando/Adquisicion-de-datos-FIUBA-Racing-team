//====================================================
//                    LIBRERÍAS
//====================================================
#include <SPI.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

#include <Wire.h>

#include <libmaple/iwdg.h>

#include <libmaple/spi.h>
#include <libmaple/gpio.h>
#include <libmaple/rcc.h>
#include <libmaple/nvic.h>

//====================================================
//           ENVÍO DE DATOS (SPI SLAVE)
//====================================================
// Trama de 62 bytes:
//  [0]      0xAA
//  [1..2]   contador de muestra (uint16)
//  [3..54]  13 floats
//  [55..58] cambio (int32)
//  [59..60] contador de muestra repetido
//  [61]     checksum XOR de los bytes 1..60
#define MAX_DATOS 62
#define SPI_SYNC  0xAA

// Por si la versión de libmaple no los define
#ifndef SPI_CR2_TXEIE
#define SPI_CR2_TXEIE (1U << 7)
#endif
#ifndef SPI_SR_TXE
#define SPI_SR_TXE    (1U << 1)
#endif

// Transmisión:
//  - RX (RXNE): recibe los bytes de comando de la Raspi y cuenta bytes.
//  - TX (TXE):  cuando el byte N empieza a salir se carga el N+1, así hay un byte
//               entero de tiempo para tenerlo listo en DR.
//  - El 0xAA se precarga en DR cuando sube CS (es igual en los dos buffers).
volatile uint8_t bufferActivo = 0;
volatile uint8_t bufferEscritura = 1;
volatile bool datoNuevoListo = false;
uint8_t misDatos[2][MAX_DATOS];

volatile int i_dato = 0;          // bytes recibidos en la transacción en curso
volatile int tx_idx = 1;          // próximo byte a cargar en DR
volatile uint32_t cntBytes = 0;   // total de bytes recibidos (para el timeout del loop)

//====================================================
//     COMANDOS RASPI -> STM32 (MOSI, bytes 0..6)
//====================================================
// 0x55, código, parámetro int32, checksum XOR (código + parámetro).
// Si la Raspi manda ceros, no pasa nada.
#define CMD_LEN 7
volatile uint8_t rxCmd[CMD_LEN];
volatile bool    comandoNuevo = false;
volatile uint8_t cmdCodigo    = 0;
volatile int32_t cmdParam     = 0;

//====================================================
//                      CAMBIOS
//====================================================
#define TOLERANCIA 40
#define NUM_CONFIRMACIONES 2

// Valor de ADC de cada marcha
int t1      = 3420;
int t2      = 2980;
int t3      = 1976;
int t4      = 3959;
int t5      = 3860;
int t6      = 3700;
int neutro  = 4095;
int pin_cambios = PA4;
int cambio = 0;

int estado_actual = -1;         // último cambio confirmado
int candidato = -2;             // valor en evaluación
int contador_confirmacion = 0;  // veces seguidas que se repitió el candidato


//====================================================
//                EXTENSÓMETROS
//====================================================
const float DMIN    =   0.0f;
float       DMAX    =  75.0f;   
const float DFIJO   =   0.0f;
const float K_ELAST =  35.0f;

const int PIN_EXT1 = PA6;
const int PIN_EXT2 = PA5;
const int PIN_EXT3 = PB1;


//====================================================
//                      IMU
//====================================================
Adafruit_MPU6050 mpu;

bool     imu_online        = false;
uint32_t imu_ultimo_intento = 0;
const uint32_t IMU_REINTENTO_MS = 2000; // reintento de conexión si está offline

const float OFFSET_LATERAL = 0.06f;

// acelerómetro en m/s² y giróscopo en rad/s.
const float OFFSET_AX = 0.43f;
const float OFFSET_AY = 0.18f;
const float OFFSET_AZ = 9.05f - 9.81f;
//en rad/s
const float OFFSET_GX = 0.32f;
const float OFFSET_GY = -0.01f;
const float OFFSET_GZ = 0.0f;
float OFFSET_LONGITUDINAL  = 0;     

float acel_lateral_g = 0.0f;
float acel_long_g    = 0.0f;
float magnitud_g     = 0.0f;
float angulo_vector  = 0.0f;

float ax = 0.0f, ay = 0.0f, az = 0.0f;
float gx = 0.0f, gy = 0.0f, gz = 0.0f;

//====================================================
//          ENCODER VOLANTE
//====================================================
const int PIN_ENCODER_A = PB12;
const int PIN_ENCODER_B = PB13;

volatile long pulsos         = 0;
         long pulsos_anterior = 0;

float angulo = 0.0f;
const float FACTOR_CONVERSION = 0.6f;   // grados por pulso

//====================================================
//                      PRESIONES
//====================================================
int pin_presion1 = PA3;
int pin_presion2 = PA2;

//====================================================
//                      SETUP
//====================================================
void setup() {
  Serial.begin(115200);

  // Avisa si el último reinicio fue por watchdog
  if (RCC_BASE->CSR & RCC_CSR_IWDGRSTF) {
    Serial.println("[ALERTA] El STM32 se reinicio por WATCHDOG (IWDG)!");
    RCC_BASE->CSR |= RCC_CSR_RMVF; // limpia las banderas de reset
  }

  pinMode(PIN_ENCODER_A, INPUT_PULLUP);
  pinMode(PIN_ENCODER_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_A), encoderISR, FALLING);

  Wire.begin();
  intentarConectarIMU();

  configurarSPISlave();
  Serial.println("[OK] SPI Slave configurado.");

  memset(misDatos, 0, sizeof(misDatos));

  // Watchdog de ~2 s
  iwdg_init(IWDG_PRE_64, 1250);
  IWDG_BASE->KR = 0xCCCC;
  Serial.println("[OK] Setup completo. Iniciando adquisicion...");
}

//====================================================
//                      LOOP
//====================================================
const uint32_t PERIODO_MUESTREO_US = 20000; // 50 Hz
uint32_t tiempo_previo_us = 0;

void loop() {

  //--- Trama cortada sin flanco de CS: si pasan 2 ms sin bytes nuevos, resincroniza ---
  static uint32_t cntPrev  = 0;
  static uint32_t tCntPrev = 0;
  uint32_t cntNow  = cntBytes;
  uint32_t ahoraUs = micros();
  if (cntNow != cntPrev) {
    cntPrev  = cntNow;
    tCntPrev = ahoraUs;
  } else if (i_dato != 0 && (ahoraUs - tCntPrev) > 2000) {
    noInterrupts();
    if (i_dato != 0 && cntBytes == cntPrev) resincronizarSPI();
    interrupts();
  }

  //--- Comando de la Raspi (se ejecuta fuera de la ISR) ---
  if (comandoNuevo) {
    noInterrupts();
    uint8_t c = cmdCodigo;
    int32_t p = cmdParam;
    comandoNuevo = false;
    interrupts();

    switch (c) {
      case 1: OFFSET_LONGITUDINAL = p / 1000.0f; break;                // milésimas de g
      case 2: pulsos = 0; pulsos_anterior = 0; angulo = 0; break;      // volante en cero
      case 3: DMAX = p / 10.0f; break;                                 // décimas de mm
      default: break;
    }
  }

  uint32_t tiempo_actual = micros();

  if (tiempo_actual - tiempo_previo_us >= PERIODO_MUESTREO_US) {
    tiempo_previo_us += PERIODO_MUESTREO_US;

    iwdg_feed();

    //--- Reconexión de la IMU ---
    if (!imu_online) {
      uint32_t ahora = millis();
      if (ahora - imu_ultimo_intento >= IMU_REINTENTO_MS) {
        imu_ultimo_intento = ahora;
        intentarConectarIMU();
      }
    }

    //--- Extensómetros ---
    float dx1 = leerExtensometro(PIN_EXT1);
    float dx2 = leerExtensometro(PIN_EXT2);
    float dx3 = leerExtensometro(PIN_EXT3);

    //--- Cambios ---
    uint16_t cambio_adc = analogRead(pin_cambios);
    int cambio_confirmado = leer_cambio_confirmado(cambio_adc);

    //--- Presión ---
    float presion1 = leer_presion(pin_presion1);
    float presion2 = leer_presion(pin_presion2);

    //--- IMU ---
    if (imu_online) {
      if (imuResponde()) {
        sensors_event_t a, g, temp;
        mpu.getEvent(&a, &g, &temp);

        calcularFuerzasG(a.acceleration.x, a.acceleration.y, a.acceleration.z, acel_lateral_g, acel_long_g);

        magnitud_g = sqrtf(acel_lateral_g * acel_lateral_g + acel_long_g * acel_long_g);

        ax = a.acceleration.x - OFFSET_AX;
        ay = a.acceleration.y - OFFSET_AY;
        az = a.acceleration.z - OFFSET_AZ;
        gx = g.gyro.x - OFFSET_GX;
        gy = g.gyro.y - OFFSET_GY;
        gz = g.gyro.z - OFFSET_GZ;
      } else {
        imu_online = false;
        Serial.println("[IMU] Desconexion detectada. Reintentando en 2 s...");
      }
    }

    //--- Encoder ---
    long copia_pulsos = pulsos;
    if (copia_pulsos != pulsos_anterior) {
      angulo          = copia_pulsos * FACTOR_CONVERSION;
      pulsos_anterior = copia_pulsos;
    }

    //--- Armar paquete en el buffer inactivo ---
    armarPaquete(angulo, acel_lateral_g, ax, ay, az, gx, gy, gz,
                 dx1, dx2, dx3, cambio_confirmado, presion1, presion2);

    //--- Intercambio de buffers ---
    noInterrupts();
    if (i_dato == 0 && digitalRead(PA15) == HIGH) {
      // SPI en reposo: se intercambia ya (el 0xAA en DR sirve para los dos buffers)
      bufferActivo    = bufferEscritura;
      bufferEscritura = 1 - bufferActivo;
      datoNuevoListo  = false;
    } else {
      // SPI transmitiendo: el intercambio lo hace la ISR de CS
      datoNuevoListo = true;
    }
    interrupts();
  }
}

//====================================================
//         CONFIGURACIÓN SPI SLAVE
//====================================================
void configurarSPISlave() {
  rcc_clk_enable(RCC_SPI1);
  rcc_clk_enable(RCC_GPIOA);
  rcc_clk_enable(RCC_GPIOB);
  rcc_clk_enable(RCC_AFIO);

  afio_cfg_debug_ports(AFIO_DEBUG_SW_ONLY);
  afio_remap(AFIO_REMAP_SPI1);

  pinMode(PA15, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PA15), nss_rising_isr, RISING);

  gpio_set_mode(GPIOA, 15, GPIO_INPUT_FLOATING);
  gpio_set_mode(GPIOB, 3, GPIO_INPUT_FLOATING);
  gpio_set_mode(GPIOB, 5, GPIO_INPUT_FLOATING);
  gpio_set_mode(GPIOB, 4, GPIO_AF_OUTPUT_PP);

  SPI1->regs->CR1  = 0;
  SPI1->regs->CR2 |= SPI_CR2_RXNEIE;
  SPI1->regs->CR1 |= SPI_CR1_SPE;

  precargarTx();
  nvic_irq_enable(NVIC_SPI1);
}

// Prepara la próxima transacción: 0xAA en DR y TXE habilitado.
// Con DR cargado TXE no dispara hasta que empiece a salir el primer byte.
void precargarTx() {
  tx_idx = 1;
  SPI1->regs->DR = SPI_SYNC;
  SPI1->regs->CR2 |= SPI_CR2_TXEIE;
}
//====================================================
//         ISR — CHIP SELECT (PA15 / NSS)
//====================================================
// Valida el comando de los primeros 7 bytes, si hubo uno
void procesarRx() {
  if (i_dato < CMD_LEN) return;          // trama demasiado corta
  if (rxCmd[0] != 0x55) return;          // sin comando

  uint8_t chk = 0;
  for (int j = 1; j <= 5; j++) chk ^= rxCmd[j];
  if (chk != rxCmd[6]) return;           // checksum malo

  cmdCodigo = rxCmd[1];
  int32_t p;
  memcpy(&p, (const void*)&rxCmd[2], 4);
  cmdParam = p;
  comandoNuevo = true;
  rxCmd[0] = 0;                          // evita ejecutarlo dos veces
}

// Resincronización de emergencia (se llama con interrupciones deshabilitadas)
void resincronizarSPI() {
  procesarRx();
  i_dato = 0;
  if (datoNuevoListo) {
    bufferActivo    = bufferEscritura;
    bufferEscritura = 1 - bufferActivo;
    datoNuevoListo  = false;
  }
  precargarTx();
}

// CS sube: terminó la transferencia
void nss_rising_isr() {
  procesarRx();
  i_dato = 0;

  // Si hay datos nuevos, se intercambia el buffer
  if (datoNuevoListo) {
    bufferActivo    = bufferEscritura;
    bufferEscritura = 1 - bufferActivo;
    datoNuevoListo  = false;
  }

  precargarTx();
}
//====================================================
//         ISR — SPI1
//====================================================
extern "C" void __irq_spi1() {
  uint32_t sr = SPI1->regs->SR;

  // Overrun: se limpia leyendo DR y SR, y se releen los flags
  if (sr & SPI_SR_OVR) {
    (void)SPI1->regs->DR;
    (void)SPI1->regs->SR;
    sr = SPI1->regs->SR;
  }

  // RX: byte de la Raspi (comando o ceros)
  if (sr & SPI_SR_RXNE) {
    uint8_t rx = SPI1->regs->DR;
    if (i_dato < CMD_LEN) rxCmd[i_dato] = rx;
    cntBytes++;
    i_dato++;
  }

  // TX: el byte anterior pasó al shift register, se carga el siguiente
  if ((sr & SPI_SR_TXE) && (SPI1->regs->CR2 & SPI_CR2_TXEIE)) {
    if (tx_idx < MAX_DATOS) {
      SPI1->regs->DR = misDatos[bufferActivo][tx_idx];
      tx_idx++;
    }
    if (tx_idx >= MAX_DATOS) {
      // Último byte cargado: se apaga TXE hasta que suba CS
      SPI1->regs->CR2 &= ~SPI_CR2_TXEIE;
    }
  }
}

//====================================================
//              FUNCIONES AUXILIARES
//====================================================
void armarPaquete(float ang, float acel,
                  float ax, float ay, float az,
                  float gx, float gy, float gz,
                  float dx1, float dx2, float dx3, int cambio,
                  float presion1, float presion2) {
  uint8_t* buf = misDatos[bufferEscritura];
  int pos = 0;

  static uint16_t nMuestra = 0;
  uint16_t cnt = ++nMuestra;

  buf[pos++] = 0xAA;
  memcpy(&buf[pos], &cnt,  2); pos += 2;   // contador (inicio)
  memcpy(&buf[pos], &ang,  4); pos += 4;
  memcpy(&buf[pos], &acel, 4); pos += 4;
  memcpy(&buf[pos], &ax,   4); pos += 4;
  memcpy(&buf[pos], &ay,   4); pos += 4;
  memcpy(&buf[pos], &az,   4); pos += 4;
  memcpy(&buf[pos], &gx,   4); pos += 4;
  memcpy(&buf[pos], &gy,   4); pos += 4;
  memcpy(&buf[pos], &gz,   4); pos += 4;
  memcpy(&buf[pos], &dx1,  4); pos += 4;
  memcpy(&buf[pos], &dx2,  4); pos += 4;
  memcpy(&buf[pos], &dx3,  4); pos += 4;
  memcpy(&buf[pos], &presion1,  4); pos += 4;
  memcpy(&buf[pos], &presion2,  4); pos += 4;
  memcpy(&buf[pos], &cambio,  4); pos += 4;
  memcpy(&buf[pos], &cnt,  2); pos += 2;   // contador (fin)

  uint8_t chk = 0;
  for (int j = 1; j < pos; j++) chk ^= buf[j];
  buf[pos] = chk;
}

//====================================================
//         RECUPERACIÓN DE BUS I2C
//====================================================
#define I2C_RECOVERY_SCL PB6
#define I2C_RECOVERY_SDA PB7

void recuperarBusI2C() {
  pinMode(I2C_RECOVERY_SDA, INPUT_PULLUP);
  pinMode(I2C_RECOVERY_SCL, OUTPUT);

  if (digitalRead(I2C_RECOVERY_SDA) == HIGH) {
    Wire.begin();
    return;
  }

  Serial.println("[I2C] Bus trabado (SDA en bajo). Recuperando...");

  for (int i = 0; i < 9 && digitalRead(I2C_RECOVERY_SDA) == LOW; i++) {
    digitalWrite(I2C_RECOVERY_SCL, LOW);
    delayMicroseconds(5);
    digitalWrite(I2C_RECOVERY_SCL, HIGH);
    delayMicroseconds(5);
  }

  pinMode(I2C_RECOVERY_SDA, OUTPUT);
  digitalWrite(I2C_RECOVERY_SDA, LOW);
  delayMicroseconds(5);
  digitalWrite(I2C_RECOVERY_SCL, HIGH);
  delayMicroseconds(5);
  digitalWrite(I2C_RECOVERY_SDA, HIGH);
  delayMicroseconds(5);

  if (digitalRead(I2C_RECOVERY_SDA) == LOW) {
    Serial.println("[I2C] No se pudo liberar SDA. Revisar cableado/pull-ups.");
  } else {
    Serial.println("[I2C] Bus liberado.");
  }

  Wire.begin();
}

void intentarConectarIMU() {
  imu_ultimo_intento = millis();
  recuperarBusI2C();
  if (mpu.begin()) {
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);
    imu_online = true;
    Serial.println("[IMU] Conectada.");
  } else {
    imu_online = false;
    Serial.println("[IMU] No responde. Reintento en 2 s...");
  }
}

bool imuResponde() {
  Wire.beginTransmission(0x68);
  Wire.write(0x75);
  if (Wire.endTransmission(false) != 0) return false;
  Wire.requestFrom(0x68, 1);
  if (!Wire.available()) return false;
  return (Wire.read() == 0x68);
}

void calcularFuerzasG(float Ax, float Ay, float Az, float &lat, float &lon) {
  float pitch = atan2f(-Ax, sqrtf(Ay * Ay + Az * Az));
  float roll  = atan2f(Ay, Az);

  float gy = 9.81f * sinf(roll) * cosf(pitch);
  float gx = 9.81f * sinf(pitch);

  float ay_lin = Ay - gy;
  float ax_lin = Ax - gx;

  lat = (ay_lin / 9.81f) - OFFSET_LATERAL;
  lon = (ax_lin / 9.81f) - OFFSET_LONGITUDINAL;
}

float leerExtensometro(int pin) {
    int raw = analogRead(pin);
    float dmed = DMIN + (DMAX - DMIN) * (4095.0f - raw) / 4095.0f;
    return dmed;
}


void encoderISR() {
  if (digitalRead(PIN_ENCODER_B) == HIGH) pulsos = pulsos + 1;
  else                                    pulsos = pulsos - 1;
}


int clasificar_lectura(int valor) {
  if (valor >= (t1 - TOLERANCIA) && valor <= (t1 + TOLERANCIA)) return 1;
  if (valor >= (t2 - TOLERANCIA) && valor <= (t2 + TOLERANCIA)) return 2;
  if (valor >= (t3 - TOLERANCIA) && valor <= (t3 + TOLERANCIA)) return 3;
  if (valor >= (t4 - TOLERANCIA) && valor <= (t4 + TOLERANCIA)) return 4;
  if (valor >= (t5 - TOLERANCIA) && valor <= (t5 + TOLERANCIA)) return 5;
  if (valor >= (t6 - TOLERANCIA) && valor <= (t6 + TOLERANCIA)) return 6;
  if (valor >= (neutro - TOLERANCIA) && valor <= (neutro + TOLERANCIA)) return 0;
  return -1;
}

int leer_cambio_confirmado(int valor_adc) {
  int lectura = clasificar_lectura(valor_adc);

  if (lectura == -1) {
    contador_confirmacion = 0;
    candidato = -2;
    return estado_actual;
  }

  if (lectura == candidato) {
    contador_confirmacion++;
  } else {
    candidato = lectura;
    contador_confirmacion = 1;
  }

  if (contador_confirmacion >= NUM_CONFIRMACIONES) {
    estado_actual = candidato;
  }

  return estado_actual;
}

float leer_presion(int pin_presion) {
    int adc_raw = analogRead(pin_presion);

    // Tensión en el pin (0 a 3,3 V)
    float v_adc = (adc_raw * 3.3f) / 4095.0f;

    // Tensión del sensor antes del divisor: (1,8k + 3,3k) / 3,3k
    float v_sensor = v_adc * (5.1f / 3.3f);

    // P [PSI] = 400 * (V_sensor - 0,5)
    float presion = 400.0f * (v_sensor - 0.5f);

    // Sin negativos por ruido en reposo
    if (presion < 0.0f) {
        presion = 0.0f;
    }

    return presion;
}
