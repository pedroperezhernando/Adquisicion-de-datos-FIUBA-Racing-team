# FIUBA Racing — Sistema de adquisición de datos

Sistema de adquisición de datos para el monoplaza de combustión interna de **FIUBA Racing** (Formula SAE, Universidad de Buenos Aires). Registra variables del motor y de la dinámica del vehículo durante la conducción en pista para analizar después, de forma objetiva, cómo se comportó el auto.

Trabajo práctico profesional de Ingeniería Electrónica (FIUBA).

## Contenido del repositorio

| Archivo | Corre en | Función |
|---|---|---|
| `daq_stm.ino` | STM32F103 | Lee los sensores a 50 Hz y entrega cada muestra por SPI |
| `daq_raspi.py` | Raspberry Pi 3 | Integra STM32, ECU, GPS y neumáticos; manda datos a la pantalla y graba el CSV |

## Qué mide

| Fuente | Variables |
|---|---|
| **ECU** Racetec R1000 (serie sobre USB) | RPM, posición del acelerador (TPS), relación aire-combustible (AFR), temperatura del motor, temperatura de aire, tensión de batería |
| **STM32** (SPI) | Ángulo del volante, aceleraciones y velocidades angulares en 3 ejes, aceleración lateral, recorrido de suspensión (3 extensómetros), presión de freno delantera y trasera, marcha engranada |
| **GPS** de la pantalla (CAN) | Latitud, longitud y velocidad |
| **Sensores infrarrojos KRE** (CAN) | Temperatura superficial de los 4 neumáticos |

## Arquitectura

```
 Sensores ──► STM32 ──SPI──► ┌──────────────┐ ──CAN──► Pantalla Spilba (piloto)
                             │ Raspberry Pi │ ◄─CAN─── GPS y neumáticos
 ECU R1000 ──USB/serie─────► └──────────────┘ ──────► CSV en la tarjeta SD
```

El sistema sigue un esquema maestro-esclavo: el STM32 (esclavo) muestrea los sensores y la Raspberry Pi (maestro) lo lee por SPI, junta todas las fuentes, envía los parámetros del motor y la marcha a la pantalla del piloto por CAN y guarda todo en un CSV a 25 Hz.

## Firmware del STM32 (`daq_stm.ino`)

Lazo principal sin RTOS que adquiere los sensores cada 20 ms (50 Hz). El encoder y el SPI se atienden por interrupciones.

### Sensores y pines

| Sensor | Pines | Procesamiento |
|---|---|---|
| Encoder del volante (A, B) | PB12, PB13 | Interrupción en flanco descendente de A; B da el sentido. 0,6° por pulso |
| IMU MPU6050 (I²C) | PB6 (SCL), PB7 (SDA) | Rango ±8 g y ±500 °/s, filtro de 44 Hz. Calcula la aceleración lateral en g restando la gravedad según la inclinación |
| Extensómetros 1, 2 y 3 | PA6, PA5, PB1 | Conversión lineal del ADC de 12 bits a 0–DMAX mm (DMAX = 110 mm por defecto) |
| Presión de freno 1 y 2 | PA3, PA2 | Divisor 1,8 kΩ / 3,3 kΩ; P [PSI] = 400 · (V<sub>sensor</sub> − 0,5) |
| Sensor de marcha | PA4 | Cada marcha tiene un valor de ADC (±40). Una marcha se confirma tras 2 lecturas iguales y solo si la transición desde la anterior es posible |

> Las aceleraciones `ax`, `ay`, `az` salen de la biblioteca de Adafruit en m/s² y las velocidades angulares en rad/s. La aceleración lateral sale en g.

### Trama SPI (62 bytes, little-endian)

| Byte | Campo |
|---|---|
| 0 | Sincronismo `0xAA` |
| 1–2 | Contador de muestra (uint16) |
| 3–54 | 13 floats: ángulo, aceleración lateral, ax, ay, az, gx, gy, gz, dx1, dx2, dx3, presión 1, presión 2 |
| 55–58 | Marcha (int32) |
| 59–60 | Contador de muestra repetido |
| 61 | Checksum XOR de los bytes 1 a 60 |

El STM32 es esclavo en SPI1 remapeado: **PA15 (CS), PB3 (SCK), PB4 (MISO), PB5 (MOSI)**.

- **Doble búfer:** la muestra nueva se arma en el búfer inactivo y se intercambia solo cuando no hay una lectura en curso; si la Raspberry está leyendo, el cambio se hace al subir CS. Así nunca se mezclan datos de dos muestras.
- **Carga byte a byte:** cada byte se carga en la interrupción TXE, apenas empieza a salir el anterior. El `0xAA` se precarga al terminar la transferencia anterior.
- **Red de seguridad:** si queda una trama a medias y pasan más de 2 ms sin bytes nuevos (por ejemplo, un flanco de CS perdido por ruido), se reinicia la recepción.

### Comandos desde la Raspberry

En los primeros 7 bytes que manda por MOSI, la Raspberry puede enviar un comando: `0x55`, código, parámetro int32 y checksum XOR. Si manda ceros, no pasa nada.

| Código | Acción | Parámetro |
|---|---|---|
| 1 | Offset de aceleración longitudinal | Milésimas de g |
| 2 | Pone el ángulo del volante en cero | — |
| 3 | Cambia DMAX de los extensómetros | Décimas de mm |

### Supervisión

- **Watchdog independiente (IWDG)** de ~2 s, alimentado en cada ciclo. Al arrancar se informa por serie si el reinicio fue por watchdog.
- **IMU:** si deja de responder, se marca como desconectada y se reintenta cada 2 s, liberando antes el bus I²C si quedó trabado.

## Software de la Raspberry Pi (`daq_raspi.py`)

Programa en Python con una arquitectura multihilo. Cada fuente guarda su último valor válido en un estado compartido protegido por lock (*estado sticky*):

| Hilo | Función | Período |
|---|---|---|
| Principal | Lectura y validación de la trama SPI, armado de la fila CSV | 40 ms (25 Hz) |
| `hilo_ecu` | Consulta a la ECU y decodificación de la respuesta | 40 ms (25 Hz) |
| `hilo_can_rx` | Recepción de GPS y temperatura de neumáticos | Por evento |
| `hilo_can_tx` | Envío de datos a la pantalla del cockpit | 100 ms (10 Hz) |
| `hilo_csv` | Escritura del CSV desde una cola, con `fsync` cada 0,5 s | Por evento |
| `hilo_consola` | Panel de depuración local (opcional) | 100 ms (10 Hz) |

- **Lectura del STM32:** SPI1 a 100 kHz, modo 0. En cada lectura se busca una trama con sincronismo, checksum y contadores correctos. Si falla, se reintenta una vez en el mismo ciclo, y tras ~1 s de fallas seguidas se reabre el dispositivo SPI.
- **Validación por canal:** cada variable del STM32 tiene un rango admisible y un salto máximo. Un valor inválido se descarta y se conserva el anterior; tras 25 rechazos seguidos (~1 s) el canal queda vacío.
- **ECU:** se abre `/dev/ttyUSB0` a 115200 baudios, con reintentos y resincronización si se pierde la comunicación. La temperatura del motor pasa por un filtro exponencial de τ = 3 s.
- **Diagnóstico:** las fallas de SPI se cuentan por causa y las tramas fallidas se guardan en hexadecimal en un archivo `_spi_fallos.log` (máximo 500 por sesión).

Un servicio de **systemd** configura la interfaz `can0` a 1 Mbit/s, lanza el programa al encender el auto y lo reinicia si se cae.

## Autores

- Lautaro Tadeo Dominguez
- Pedro Perez Hernando

Tutor: Pablo Gomez
