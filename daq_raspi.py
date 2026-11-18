#!/usr/bin/env python3
"""
TELEMETRÍA UNIFICADA v3 — Raspberry Pi

Entradas:
  STM32      → SPI     (marcha, ángulo, aceleraciones, giros, dx1..dx3, presiones)
  ECU        → Serial  (RPM, TPS, AFR, temperaturas, batería)
  GPS        → CAN RX  (latitud, longitud, velocidad)
  Neumáticos → CAN RX  (temperatura de las 4 ruedas)
Salidas:
  CAN TX  → pantalla del cockpit, 10 Hz
  CSV     → registro de la sesión
  Consola → debug local
"""

import can, spidev, serial, struct, csv, time, os, threading, queue, math
import signal
from collections import deque
from datetime import datetime

# =============================================================
# FLAGS DE PRUEBA
# =============================================================

# Muestra el panel de debug en consola. En False ahorra CPU.
MOSTRAR_CONSOLA = True

# Constante de tiempo (s) del filtro de temp. de refrigerante. 0 = sin filtro.
TEMP_REFRIG_TAU_S = 3.0

# =============================================================
# CONFIGURACIÓN
# =============================================================

CICLO_S = 0.04        # 25 Hz, loop principal
DISPLAY_HZ = 10
DISPLAY_PERIODO_S = 1.0 / DISPLAY_HZ

CAN_TX_PERIODO_S = 0.1  # 10 Hz

# --- SPI (STM32) ---
SPI_BUS, SPI_DEVICE = 1, 0
SPI_VELOCIDAD = 100_000
SPI_MODO = 0b00
STM32_SYNC = 0xAA

STM32_TAM_PKT = 62        # sync + contador + 13 floats + int32 marcha + contador + checksum
STM32_FORMATO = '<H13fiH'

STM32_TAM_LECTURA = STM32_TAM_PKT * 2   # se leen 2 tramas para encontrar una completa

# Reabrir el SPI tras fallas seguidas. Apagado: cerrar y abrir el SPI puede meter
# pulsos espurios en SCK/CS y no arregla un problema del lado del STM.
SPI_REABRIR_HABILITADO = False
SPI_FALLOS_PARA_REABRIR = 25            # fallas seguidas (~1 s) antes de reabrir el SPI
SPI_RECUPERAR_BITSLIP = True            # si no hay trama válida, probar la ventana corrida 1 a 7 bits
SPI_REINTENTO_PAUSA_S = 0.008           # pausa antes de reintentar

# Comandos del firmware del STM32
CMD_STM_OFFSET_LONG  = 0  # offset longitudinal, en milésimas de g
CMD_STM_CERO_VOLANTE = 0  # pone el volante en cero
CMD_STM_DMAX         = 0  # DMAX de extensómetros, en décimas de mm

# --- Reset del STM32 por hardware (GPIO → NRST) ---
# En reposo el GPIO queda como entrada flotante; para resetear se pone en bajo unos ms.
RESET_STM_HABILITADO = True
STM_NRST_GPIO = 4                  # numeración BCM (pin físico 7)
STM_NRST_PULSO_S = 0.05            # tiempo con NRST a masa

# Se resetea tras RESET_MALOS_SEGUIDOS ciclos malos seguidos (lectura fallida o contador
# congelado). Un ciclo sano reinicia la cuenta.
RESET_MALOS_SEGUIDOS = 40          # ~1,6 s
RESET_GRACIA_INICIAL_S = 10.0      # sin resets al arrancar el script
STM_BOOT_PAUSA_S = 1.5             # sin acceso SPI mientras el STM arranca
RESET_ESPERA_ARRANQUE_S = 2.0      # tiempo extra sin evaluar al reanudar el SPI
RESET_COOLDOWN_S = 15.0            # mínimo entre resets; se duplica si no se recupera
RESET_COOLDOWN_MAX_S = 120.0
RESET_SANOS_PARA_RECUPERADO = 50   # ciclos sanos (~2 s) para darlo por recuperado

# Configuración a reenviar al STM tras un reset (None / False = no se reenvía)
STM_REENVIAR_OFFSET_LONG = None    # milésimas de g
STM_REENVIAR_DMAX = None           # décimas de mm
STM_REENVIAR_CERO_VOLANTE = False  # solo si el GPS marca menos de STM_CERO_VOLANTE_VEL_MAX_KMH
STM_CERO_VOLANTE_VEL_MAX_KMH = 3.0

# --- Diagnóstico SPI ---
# Entre lecturas el contador del STM avanza ~2; más de esto cuenta como salto.
STM_DELTA_CNT_MAX = 4
SPI_DUMP_FALLOS = True          # guarda en archivo las tramas fallidas
SPI_DUMP_MAX = 500              # máximo de tramas guardadas por sesión
SPI_DUMP_BYTES_CONSOLA = 8      # bytes de la última falla que se muestran

# --- ECU (Serial) ---
ECU_PUERTO    = "/dev/ttyUSB0"
ECU_BAUDRATE  = 115200
ECU_HEADER    = b'\xFF\x7F'
ECU_FOOTER    = b'\x7F\xFF'

ECU_PERIODO_S        = 0.04   # período de pedidos a la ECU
ECU_RETRY_ESPERA      = 3.0   # espera entre intentos de abrir el puerto
ECU_BUFFER_MAX        = 2048  # bytes máximos en buffer sin sincronizar
ECU_RESYNC_TIMEOUT_S  = 2.0   # sin frames válidos en este tiempo → se purga el buffer
ECU_OK_TIMEOUT_S      = 1.0   # sin frames en este tiempo → "SIN DATOS"

# --- CAN ---
CAN_CANAL   = 'can0'
CAN_BITRATE = 1_000_000

# Un bus, un hilo RX y un hilo TX, sin recuperación automática.

CAN_ID_MOTOR       = 0x640
CAN_ID_PRES_ACEITE = 0x644
CAN_ID_TEMPS       = 0x649
CAN_ID_MARCHA      = 0x64D
CAN_ID_LAMBDA      = 0x460

CAN_ID_GPS_LATLON  = 0x680
CAN_ID_GPS_TIEMPO  = 0x681
CAN_ID_GPS_FECHA   = 0x682

CAN_ID_NEUMATICO_DI = 0x3F0  # delantera izquierda
CAN_ID_NEUMATICO_DD = 0x3F4  # delantera derecha
CAN_ID_NEUMATICO_TI = 0x3F8  # trasera izquierda
CAN_ID_NEUMATICO_TD = 0x3FC  # trasera derecha

#Variables para decodificar las tramas de la ecu
SIG_RPM         = {'off': 0, 'len': 2, 'mask': 0xFFFF, 'mult': 1,    'div': 1,    'add': 0}
SIG_TPS         = {'off': 6, 'len': 2, 'mask': 0xFFFF, 'mult': 1,    'div': 10,   'add': 0}
SIG_PRES_ACEITE = {'off': 6, 'len': 2, 'mask': 0xFFFF, 'mult': 1000, 'div': 1,    'add': 0}
SIG_TEMP_REFRIG = {'off': 0, 'len': 1, 'mask': 0xFF,   'mult': 1,    'div': 1,    'add': 40}
SIG_TEMP_AIRE   = {'off': 1, 'len': 1, 'mask': 0xFF,   'mult': 1,    'div': 1,    'add': 40}
SIG_MARCHA      = {'off': 6, 'len': 1, 'mask': 0x0F,   'mult': 1,    'div': 1,    'add': 0}
SIG_BATERIA     = {'off': 5, 'len': 1, 'mask': 0xFF,   'mult': 1,    'div': 10,   'add': 0}
SIG_LAMBDA      = {'off': 0, 'len': 2, 'mask': 0xFFFF, 'mult': 1000, 'div': 1,    'add': 0}

# --- CSV ---
CSV_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "sesiones")
os.makedirs(CSV_DIR, exist_ok=True)

def _nombre_csv_unico():
    base = datetime.now().strftime('%Y%m%d_%H-%M-%S')
    candidato = os.path.join(CSV_DIR, f"sesion_{base}.csv")
    n = 1
    while os.path.exists(candidato):
        candidato = os.path.join(CSV_DIR, f"sesion_{base}_{n}.csv")
        n += 1
    return candidato

CSV_NOMBRE = _nombre_csv_unico()
SPI_DUMP_NOMBRE = CSV_NOMBRE.replace(".csv", "_spi_fallos.log")
FSYNC_CADA_S = 0.5

CSV_COLUMNAS = [
    "tiempo_pc_ms",
    "marcha", "angulo", "acel_lateral",
    "acel_x", "acel_y", "acel_z",
    "giro_x", "giro_y", "giro_z",
    "dx1", "dx2", "dx3",
    "stm_presion1", "stm_presion2",
    "rpm", "tps", "afr",
    "temp_refrigerante", "temp_aire",
    "presion_aceite", "bateria_v",
    "gps_lat", "gps_lon", "gps_vel_kmh",
    "temp_neumatico_di", "temp_neumatico_dd", "temp_neumatico_ti", "temp_neumatico_td",
]

STM_CAMPOS = ["marcha", "angulo", "acel_lateral", "acel_x", "acel_y", "acel_z",
              "giro_x", "giro_y", "giro_z", "dx1", "dx2", "dx3",
              "stm_presion1", "stm_presion2"]
ECU_CAMPOS = ["rpm", "tps", "afr", "temp_refrigerante", "temp_aire",
              "presion_aceite", "bateria_v"]
GPS_CAMPOS = ["gps_lat", "gps_lon", "gps_vel_kmh"]
NEUMATICO_CAMPOS = ["temp_neumatico_di", "temp_neumatico_dd", "temp_neumatico_ti", "temp_neumatico_td"]

NOMBRES_MARCHA = {0: "N", 1: "1ª", 2: "2ª", 3: "3ª", 4: "4ª", 5: "5ª", 6: "6ª"}

# --- Colores ANSI para la consola ---
RESET    = "\033[0m"
BOLD     = "\033[1m"
CYAN     = "\033[36m"
AZUL     = "\033[34m"
VERDE    = "\033[32m"
AMARILLO = "\033[33m"
ROJO     = "\033[31m"
GRIS     = "\033[90m"
CLEAR    = "\033[2J\033[H"

def _fmt(val, decimales=2):
    """Formatea un número, o '---' si no hay dato."""
    if val is None:
        return f"{GRIS}---{RESET}"
    return f"{val:.{decimales}f}"

def _fmt_neumatico(temp, decimales=1):
    """Temperatura de neumático coloreada según rango."""
    if temp is None:
        return f"{GRIS}---{RESET}"
    if temp < 30:
        color = AZUL
    elif temp < 75:
        color = VERDE
    elif temp < 90:
        color = AMARILLO
    else:
        color = ROJO
    return f"{color}{temp:.{decimales}f}{RESET}"

# =============================================================
# ESTADO COMPARTIDO
# =============================================================

_lock_gps = threading.Lock()
_estado_gps = {"gps_lat": None, "gps_lon": None, "gps_vel_kmh": None}

def guardar_gps(datos):
    with _lock_gps:
        _estado_gps.update(datos)

def leer_gps():
    with _lock_gps:
        return dict(_estado_gps)

_lock_neumaticos = threading.Lock()
_estado_neumaticos = {k: 0.0 for k in NEUMATICO_CAMPOS}   # 0 hasta que llegue dato

def guardar_neumaticos(datos):
    with _lock_neumaticos:
        _estado_neumaticos.update(datos)

def leer_neumaticos():
    with _lock_neumaticos:
        return dict(_estado_neumaticos)

_lock_ecu = threading.Lock()
_estado_ecu = {
    "rpm": None, "tps": None, "afr": None,
    "temp_refrigerante": None, "temp_aire": None,
    "presion_aceite": None, "bateria_v": None,
}
_ultimo_ecu_ts = 0.0

# --- Filtro exponencial de temp_refrigerante (usa el dt real entre frames) ---
_temp_refrig_filtrada = None
_temp_refrig_ts = 0.0

def _filtrar_temp_refrig(valor, ahora):
    global _temp_refrig_filtrada, _temp_refrig_ts
    if valor is None:
        return _temp_refrig_filtrada
    if TEMP_REFRIG_TAU_S <= 0 or _temp_refrig_filtrada is None:
        _temp_refrig_filtrada = valor
    else:
        dt = max(ahora - _temp_refrig_ts, 0.0)
        alpha = 1.0 - math.exp(-dt / TEMP_REFRIG_TAU_S)
        _temp_refrig_filtrada += alpha * (valor - _temp_refrig_filtrada)
    _temp_refrig_ts = ahora
    return _temp_refrig_filtrada

def guardar_ecu(datos):
    global _ultimo_ecu_ts
    ahora = time.perf_counter()
    with _lock_ecu:
        if "temp_refrigerante" in datos:
            datos = dict(datos)
            datos["temp_refrigerante"] = _filtrar_temp_refrig(datos["temp_refrigerante"], ahora)
        _estado_ecu.update(datos)
        _ultimo_ecu_ts = ahora

def leer_ecu_sticky():
    with _lock_ecu:
        return dict(_estado_ecu)

def ecu_esta_ok(timeout=ECU_OK_TIMEOUT_S):
    with _lock_ecu:
        ts = _ultimo_ecu_ts
    return ts > 0 and (time.perf_counter() - ts) < timeout

_lock_marcha = threading.Lock()
_marcha = 0

def guardar_marcha(valor):
    global _marcha
    with _lock_marcha:
        _marcha = int(valor)

def leer_marcha():
    with _lock_marcha:
        return _marcha

# --- Estado del STM32 (cada canal se valida por separado) ---
_lock_stm = threading.Lock()
_estado_stm_sticky = {k: None for k in STM_CAMPOS}
_fallos_consecutivos_stm = {k: 0 for k in STM_CAMPOS}
MAX_FALLOS_STICKY = 25  # ~1 s de fallas antes de dar el sensor por muerto

STM_LIMITES = {
    "angulo":       {"min": -360, "max": 360,   "delta_max": 20},     # °
    "acel_lateral": {"min": -10,   "max": 10,     "delta_max": 5},       # g
    "acel_x":       {"min": -40,   "max": 40,     "delta_max": 8},      # g
    "acel_y":       {"min": -40,   "max": 40,     "delta_max": 8},      # g
    "acel_z":       {"min": -40,   "max": 40,     "delta_max": 8},      # g
    "giro_x":       {"min": -2000, "max": 2000,   "delta_max": 1000},    # °/s
    "giro_y":       {"min": -2000, "max": 2000,   "delta_max": 1000},    # °/s
    "giro_z":       {"min": -2000, "max": 2000,   "delta_max": 1000},    # °/s
    "dx1":          {"min": 0,     "max": 75,    "delta_max": 30},      # mm
    "dx2":          {"min": 0,     "max": 75,    "delta_max": 30},      # mm
    "dx3":          {"min": 0,     "max": 75,    "delta_max": 30},      # mm
    "marcha":       {"min": 0,     "max": 6,      "delta_max": 2},       # 0 = neutro
    "stm_presion1": {"min": 0.0,   "max": 2000.0, "delta_max": 1200.0},  # PSI
    "stm_presion2": {"min": 0.0,   "max": 2000.0, "delta_max": 1200.0},  # PSI
}

def _sensor_valido(campo, val, prev):
    """Chequea rango y salto máximo respecto del valor anterior."""
    lim = STM_LIMITES.get(campo)
    if lim is None:
        return True

    if val is None:
        return False

    if not (lim["min"] <= val <= lim["max"]):
        return False

    if prev is not None and abs(val - prev) > lim["delta_max"]:
        return False

    return True

def guardar_stm(datos):
    """Actualiza cada canal válido; los inválidos mantienen el último valor."""
    global _fallos_consecutivos_stm
    with _lock_stm:
        for campo, val in datos.items():
            if campo not in _estado_stm_sticky:
                continue
            prev = _estado_stm_sticky.get(campo)
            if _sensor_valido(campo, val, prev):
                _estado_stm_sticky[campo] = val
                _fallos_consecutivos_stm[campo] = 0
            else:
                _fallos_consecutivos_stm[campo] += 1
                if _fallos_consecutivos_stm[campo] > MAX_FALLOS_STICKY:
                    _estado_stm_sticky[campo] = None

def leer_stm_sticky():
    with _lock_stm:
        return dict(_estado_stm_sticky)

# --- Estadísticas para la consola ---
_lock_stats = threading.Lock()
_stats = {
    "n_muestras": 0,
    "n_stm_fail": 0,
    "n_csv_perdidas": 0,     # filas descartadas por cola llena
    # Causas de falla SPI
    "fail_sync": 0,          # no hay 0xAA en la ventana
    "fail_sync_corrido": 0,  # sin uso
    "fail_chk": 0,           # checksum incorrecto
    "fail_cnt": 0,           # contador inicial != final
    "fail_unpack": 0,
    "desfase_recuperado": 0, # trama corrida en bytes que se pudo recuperar
    "bitslip_recuperado": 0, # trama corrida en bits que se pudo recuperar
    "bitslip_s1": 0, "bitslip_s2": 0, "bitslip_s3": 0, "bitslip_s4": 0,   # bits de corrimiento
    "bitslip_s5": 0, "bitslip_s6": 0, "bitslip_s7": 0,
    "slip_cambios": 0,       # veces que cambió el corrimiento entre lecturas válidas
    "spi_reabiertos": 0,
    "stm_resets": 0,         # resets del STM32 por NRST
    # CAN
    "can_tx_ok": 0,
    "can_tx_err": 0,
    "can_rx_err": 0,
    # Continuidad del contador (solo tramas válidas)
    "cnt_repetido": 0,       # mismo contador que la anterior
    "cnt_salto": 0,          # se perdieron muestras
}

def incrementar_stat(campo, delta=1):
    with _lock_stats:
        _stats[campo] += delta

def leer_stats():
    with _lock_stats:
        return dict(_stats)

# Última trama SPI fallida: (tipo, primeros bytes en hex)
_lock_ultimo_fallo = threading.Lock()
_ultimo_fallo_spi = None

def registrar_fallo_spi(tipo, raw):
    """Cuenta el fallo, lo guarda para la consola y lo vuelca a archivo."""
    global _ultimo_fallo_spi
    incrementar_stat("n_stm_fail")
    incrementar_stat(tipo)
    cabecera = raw[:SPI_DUMP_BYTES_CONSOLA].hex(' ')
    with _lock_ultimo_fallo:
        _ultimo_fallo_spi = (tipo, cabecera)
    if SPI_DUMP_FALLOS:
        _volcar_fallo_spi(tipo, raw)

def leer_ultimo_fallo_spi():
    with _lock_ultimo_fallo:
        return _ultimo_fallo_spi

_n_dumps_spi = 0

def _volcar_fallo_spi(tipo, raw):
    """Agrega la trama completa en hex al archivo de fallos (hasta SPI_DUMP_MAX)."""
    global _n_dumps_spi
    if _n_dumps_spi >= SPI_DUMP_MAX:
        return
    _n_dumps_spi += 1
    try:
        with open(SPI_DUMP_NOMBRE, 'a') as f:
            f.write(f"{time.time():.3f} {tipo} {raw.hex(' ')}\n")
    except OSError:
        pass

# --- Estado para la consola ---
_lock_display = threading.Lock()
_ultimo_estado_display = None

def actualizar_display(estado):
    global _ultimo_estado_display
    with _lock_display:
        _ultimo_estado_display = estado

def leer_estado_display():
    with _lock_display:
        return _ultimo_estado_display

# =============================================================
# ECU — protocolo serial
# =============================================================

def _checksum_ecu(trama):
    return struct.pack('<I', sum(trama) & 0xFFFFFFFF)

def _armar_pedido_ecu(cmd):
    n = 10
    base = ECU_HEADER + bytes([cmd[0], cmd[1], cmd[2], n]) + ECU_FOOTER
    chk = _checksum_ecu(base)
    return bytearray(ECU_HEADER + bytes([cmd[0], cmd[1]]) + chk + bytes([cmd[2], n]) + ECU_FOOTER)

def _decodificar_ecu(payload):
    try:
        rpm        = struct.unpack_from('>h', payload, 8)[0]
        afr        = struct.unpack_from('B',  payload, 10)[0] / 10.0
        tps        = struct.unpack_from('>h', payload, 14)[0] / 10.0
        temp_motor = struct.unpack_from('>h', payload, 18)[0] / 10.0
        temp_aire  = struct.unpack_from('>h', payload, 20)[0] / 10.0
        vbat       = struct.unpack_from('>h', payload, 22)[0] / 100.0
    except struct.error:
        return None
    return {
        "rpm": rpm, "tps": tps, "afr": afr,
        "temp_refrigerante": temp_motor, "temp_aire": temp_aire,
        "presion_aceite": 0,
        "bateria_v": vbat,
    }

def hilo_ecu(stop):
    ser = None
    intentos = 0
    while ser is None and not stop.is_set():
        try:
            ser = serial.Serial(ECU_PUERTO, ECU_BAUDRATE, timeout=0)
            ser.reset_input_buffer()
            ser.reset_output_buffer()
        except Exception as e:
            intentos += 1
            if intentos == 1 or intentos % 10 == 0:
                print(f"[ECU] No se pudo abrir {ECU_PUERTO} (intento {intentos}): {e} — reintentando...")
            time.sleep(ECU_RETRY_ESPERA)

    if stop.is_set():
        return

    print(f"[ECU] Conectado en {ECU_PUERTO} @ {ECU_BAUDRATE} baud")

    try:
        ser.write(_armar_pedido_ecu([0, 0, 0]))
        time.sleep(0.05)
        ser.reset_input_buffer()
        print("[ECU] Handshake OK")
    except Exception as e:
        print(f"[ECU] Error en handshake: {e}")

    pedido = _armar_pedido_ecu([6, 0, 0])
    buffer = bytearray()
    ultimo = 0.0
    ultimo_frame_ok = time.perf_counter()

    while not stop.is_set():
        try:
            ahora = time.perf_counter()

            if ahora - ultimo >= ECU_PERIODO_S:
                ser.write(pedido)
                ultimo = ahora

            n = ser.in_waiting
            if n:
                buffer.extend(ser.read(n))

            hubo_frame = False
            while ECU_HEADER in buffer and ECU_FOOTER in buffer:
                ini = buffer.find(ECU_HEADER)
                fin = buffer.find(ECU_FOOTER, ini)
                if ini == -1 or fin == -1:
                    break
                trama  = buffer[ini: fin + 2]
                buffer = buffer[fin + 2:]

                if len(trama) > 12 and trama[2] == 180:
                    datos = _decodificar_ecu(trama[10:-2])
                    if datos:
                        guardar_ecu(datos)
                        hubo_frame = True

            if hubo_frame:
                ultimo_frame_ok = ahora

            if len(buffer) > ECU_BUFFER_MAX:
                print(f"[ECU] Buffer > {ECU_BUFFER_MAX} bytes sin sync, purgando...")
                buffer.clear()
                ultimo_frame_ok = ahora
            elif (ahora - ultimo_frame_ok) > ECU_RESYNC_TIMEOUT_S and len(buffer) > 0:
                print(f"[ECU] Sin frames válidos hace {ECU_RESYNC_TIMEOUT_S:.0f}s, purgando...")
                buffer.clear()
                ultimo_frame_ok = ahora

            time.sleep(0.001)

        except serial.SerialException as e:
            print(f"[ECU] Puerto perdido: {e} — reintentando...")
            buffer.clear()
            time.sleep(3.0)
            try:
                ser.close()
                ser.open()
                ser.reset_input_buffer()
                print("[ECU] Reconectado OK")
            except Exception as e2:
                print(f"[ECU] Reconexión fallida: {e2}")

        except Exception as e:
            print(f"[ECU] Error inesperado: {e}")
            buffer.clear()
            time.sleep(0.1)

    if ser and ser.is_open:
        ser.close()
    print("[ECU] Puerto cerrado.")

# =============================================================
# STM32 — SPI
# =============================================================

_lock_cmd = threading.Lock()
_cmds_pendientes = deque()   # [cmd, param, repeticiones_restantes]

def enviar_comando_stm(cmd, param=0, repeticiones=3):
    """Encola un comando para el STM32. Se manda en las próximas lecturas,
    repetido por si alguna trama se corrompe."""
    with _lock_cmd:
        _cmds_pendientes.append([cmd, param, repeticiones])

def _tomar_comando():
    with _lock_cmd:
        if not _cmds_pendientes:
            return None
        item = _cmds_pendientes[0]
        item[2] -= 1
        if item[2] <= 0:
            _cmds_pendientes.popleft()
        return item[0], item[1]

def _armar_tx(cmd=None, param=0):
    """Sin comando: todo ceros. Con comando: 0x55, código, int32, checksum."""
    tx = [0x00] * STM32_TAM_LECTURA
    if cmd is not None:
        raw = bytes([cmd]) + struct.pack('<i', param)
        chk = 0
        for b in raw:
            chk ^= b
        tx[0:7] = [0x55] + list(raw) + [chk]
    return tx

_cnt_stm_prev = None         # contador de la última trama válida
_cnt_repetido_seguidos = 0   # lecturas válidas seguidas con el mismo contador

def _buscar_trama_bytes(raw):
    """Busca en la ventana una trama con sync, checksum y contadores correctos."""
    motivo = "fail_sync"
    ultimo = len(raw) - STM32_TAM_PKT
    i = raw.find(STM32_SYNC)
    while i != -1 and i <= ultimo:
        t = raw[i:i + STM32_TAM_PKT]
        chk = 0
        for b in t[1:-1]:
            chk ^= b
        if chk == t[-1]:
            if t[1:3] == t[59:61]:          # contador inicial == final
                return t, i, None
            motivo = "fail_cnt"
        elif motivo == "fail_sync":
            motivo = "fail_chk"
        i = raw.find(STM32_SYNC, i + 1)
    return None, -1, motivo

_slip_prev = None   # corrimiento en bits de la última lectura válida (0 = alineada)

def _registrar_slip(s):
    """Cuenta el corrimiento y cuántas veces cambia."""
    global _slip_prev
    if s:
        incrementar_stat(f"bitslip_s{s}")
    if _slip_prev is not None and s != _slip_prev:
        incrementar_stat("slip_cambios")
    _slip_prev = s

def _buscar_trama(raw):
    """Como _buscar_trama_bytes, pero si falla prueba la ventana corrida 1 a 7 bits
    (un pulso de reloj espurio deja al STM desfasado en bits)."""
    trama, offset, motivo = _buscar_trama_bytes(raw)
    if trama is not None:
        _registrar_slip(0)
        return trama, offset, motivo
    if not SPI_RECUPERAR_BITSLIP:
        return trama, offset, motivo
    n = len(raw)
    valor = int.from_bytes(raw, 'big')
    mascara = (1 << (8 * n)) - 1
    for s in range(1, 8):
        corrido = ((valor << s) & mascara).to_bytes(n, 'big')
        t, off, _ = _buscar_trama_bytes(corrido)
        if t is not None:
            incrementar_stat("bitslip_recuperado")
            _registrar_slip(s)
            return t, off, None
    return None, -1, motivo

def leer_stm32(spi):
    global _cnt_stm_prev, _cnt_repetido_seguidos
    c = _tomar_comando()
    try:
        raw = bytes(spi.xfer2(_armar_tx(*c) if c else _armar_tx()))
    except OSError:
        registrar_fallo_spi("fail_sync", bytes(8))   # error del driver SPI
        return None

    # 1. Buscar una trama válida en la ventana
    trama, offset, motivo = _buscar_trama(raw)
    if trama is None:
        registrar_fallo_spi(motivo, raw)
        return None
    if offset > 0:
        incrementar_stat("desfase_recuperado")

    # 2. Desempaquetado
    try:
        (n_ini,
         angulo, acel_lateral,
         ax, ay, az, gx, gy, gz,
         dx1, dx2, dx3,
         presion1, presion2,
         cambio,
         n_fin) = struct.unpack_from(STM32_FORMATO, trama, 1)
    except struct.error:
        registrar_fallo_spi("fail_unpack", raw)
        return None

    # 3. Continuidad del contador (uint16 con wrap)
    if _cnt_stm_prev is not None:
        delta = (n_ini - _cnt_stm_prev) & 0xFFFF
        if delta == 0:
            incrementar_stat("cnt_repetido")
            _cnt_repetido_seguidos += 1
        else:
            _cnt_repetido_seguidos = 0
            if delta > STM_DELTA_CNT_MAX:
                incrementar_stat("cnt_salto")
    _cnt_stm_prev = n_ini

    return {
        "marcha": cambio, "angulo": angulo, "acel_lateral": acel_lateral,
        "acel_x": ax, "acel_y": ay, "acel_z": az,
        "giro_x": gx, "giro_y": gy, "giro_z": gz,
        "dx1": dx1, "dx2": dx2, "dx3": dx3,
        "stm_presion1": presion1, "stm_presion2": presion2,
    }

# =============================================================
# STM32 — reset por hardware (GPIO → NRST)
# =============================================================

_gpio_modo = None        # 'rpigpio' | 'gpiozero' | None
_gpio_zero_dev = None

def iniciar_gpio_nrst():
    """Deja el GPIO de NRST como entrada flotante. Usa RPi.GPIO o, si no anda, gpiozero.
    Devuelve True si quedó utilizable."""
    global _gpio_modo, _gpio_zero_dev
    if not RESET_STM_HABILITADO:
        print("[STM] Reset por hardware deshabilitado (RESET_STM_HABILITADO = False).")
        return False
    try:
        import RPi.GPIO as GPIO
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(STM_NRST_GPIO, GPIO.IN, pull_up_down=GPIO.PUD_OFF)
        _gpio_modo = 'rpigpio'
        print(f"[STM] NRST listo en GPIO{STM_NRST_GPIO} (RPi.GPIO).")
        return True
    except Exception:
        pass
    try:
        from gpiozero import InputDevice
        _gpio_zero_dev = InputDevice(STM_NRST_GPIO, pull_up=None, active_state=True)
        _gpio_modo = 'gpiozero'
        print(f"[STM] NRST listo en GPIO{STM_NRST_GPIO} (gpiozero).")
        return True
    except Exception as e:
        print(f"[STM] No se pudo preparar GPIO{STM_NRST_GPIO} para NRST: {e} — reset por hardware DESHABILITADO.")
        return False

def _pulso_nrst():
    """Pone NRST a masa STM_NRST_PULSO_S segundos y lo vuelve a soltar."""
    global _gpio_zero_dev
    if _gpio_modo == 'rpigpio':
        import RPi.GPIO as GPIO
        try:
            GPIO.setup(STM_NRST_GPIO, GPIO.OUT, initial=GPIO.LOW)
            time.sleep(STM_NRST_PULSO_S)
        finally:
            GPIO.setup(STM_NRST_GPIO, GPIO.IN, pull_up_down=GPIO.PUD_OFF)
    elif _gpio_modo == 'gpiozero':
        from gpiozero import InputDevice, OutputDevice
        _gpio_zero_dev.close()
        salida = None
        try:
            salida = OutputDevice(STM_NRST_GPIO, active_high=True, initial_value=False)
            time.sleep(STM_NRST_PULSO_S)
        finally:
            if salida is not None:
                salida.close()
            _gpio_zero_dev = InputDevice(STM_NRST_GPIO, pull_up=None, active_state=True)
    else:
        raise RuntimeError("GPIO de NRST no inicializado")

def _log_evento(texto):
    """Agrega un evento al archivo de fallos SPI, para cruzarlo con las tramas."""
    try:
        with open(SPI_DUMP_NOMBRE, 'a') as f:
            f.write(f"{time.time():.3f} EVENTO {texto}\n")
    except OSError:
        pass

def resetear_stm32(motivo):
    """Da el pulso de reset. Devuelve True si se pudo."""
    global _cnt_stm_prev, _cnt_repetido_seguidos, _slip_prev
    _slip_prev = None               # el corrimiento puede cambiar tras el reset
    print(f"[STM] RESET por hardware (GPIO{STM_NRST_GPIO} → NRST). Motivo: {motivo}")
    _log_evento(f"reset_stm motivo={motivo}")
    try:
        _pulso_nrst()
    except Exception as e:
        print(f"[STM] Falló el pulso de reset: {e}")
        return False
    incrementar_stat("stm_resets")
    _cnt_stm_prev = None            # el contador del STM vuelve a cero
    _cnt_repetido_seguidos = 0
    with _lock_cmd:
        _cmds_pendientes.clear()    # los comandos encolados ya no sirven
    return True

class VigilanteSTM:
    """Decide cuándo resetear el STM32:
      1. No actúa en los primeros RESET_GRACIA_INICIAL_S segundos.
      2. Hacen falta RESET_MALOS_SEGUIDOS ciclos malos seguidos.
      3. Tras un reset no toca el SPI STM_BOOT_PAUSA_S y espera RESET_ESPERA_ARRANQUE_S más.
      4. El cooldown entre resets se duplica mientras no se recupere."""

    def __init__(self):
        self.malos = 0
        self.sanos = 0
        self.resets_sin_recuperar = 0
        self.ultimo_reset = None
        self.no_evaluar_hasta = time.perf_counter() + RESET_GRACIA_INICIAL_S
        self.pausa_spi_hasta = 0.0
        self.reenviar_pendiente = False

    def spi_en_pausa(self, ahora):
        """True mientras el STM arranca tras un reset."""
        return ahora < self.pausa_spi_hasta

    def _cooldown(self):
        n = max(self.resets_sin_recuperar, 1)
        return min(RESET_COOLDOWN_S * (2 ** (n - 1)), RESET_COOLDOWN_MAX_S)

    def evaluar(self, ahora, lectura_ok, cnt_repetido_seguidos):
        """Se llama una vez por ciclo. Devuelve el motivo si hay que resetear, o None."""
        if ahora < self.no_evaluar_hasta:
            self.malos = 0
            self.sanos = 0
            return None

        if lectura_ok and cnt_repetido_seguidos == 0:
            self.malos = 0
            self.sanos += 1
            if self.sanos >= RESET_SANOS_PARA_RECUPERADO:
                self.resets_sin_recuperar = 0
            return None

        self.sanos = 0
        self.malos += 1
        if self.malos < RESET_MALOS_SEGUIDOS:
            return None
        if self.ultimo_reset is not None and (ahora - self.ultimo_reset) < self._cooldown():
            return None
        if not lectura_ok:
            return f"{self.malos} ciclos seguidos sin trama SPI válida"
        return f"{self.malos} ciclos seguidos con el contador del STM congelado"

    def registrar_reset(self, ahora):
        self.ultimo_reset = ahora
        self.resets_sin_recuperar += 1
        self.malos = 0
        self.sanos = 0
        self.pausa_spi_hasta = ahora + STM_BOOT_PAUSA_S
        self.no_evaluar_hasta = self.pausa_spi_hasta + RESET_ESPERA_ARRANQUE_S
        self.reenviar_pendiente = True

    def reenviar_configuracion(self, ahora):
        """Terminada la espera de arranque, reenvía la configuración al STM."""
        if not self.reenviar_pendiente or ahora < self.no_evaluar_hasta:
            return
        self.reenviar_pendiente = False
        if STM_REENVIAR_OFFSET_LONG is not None:
            enviar_comando_stm(CMD_STM_OFFSET_LONG, int(STM_REENVIAR_OFFSET_LONG))
        if STM_REENVIAR_DMAX is not None:
            enviar_comando_stm(CMD_STM_DMAX, int(STM_REENVIAR_DMAX))
        if STM_REENVIAR_CERO_VOLANTE:
            vel = leer_gps().get("gps_vel_kmh")
            if vel is not None and vel < STM_CERO_VOLANTE_VEL_MAX_KMH:
                enviar_comando_stm(CMD_STM_CERO_VOLANTE)
            else:
                print("[STM] No se reenvía el cero del volante: velocidad GPS desconocida o en movimiento.")

# =============================================================
# CAN RX (GPS y neumáticos)
# =============================================================

def _decodificar_gps_latlon(data):
    lat = struct.unpack_from('>i', data, 0)[0] * 1e-7
    lon = struct.unpack_from('>i', data, 4)[0] * 1e-7
    return {"gps_lat": lat, "gps_lon": lon}

def _decodificar_gps_tiempo(data):
    # Velocidad: bytes 4-5, big-endian (aunque la tabla diga Intel), 0.1 km/h
    velocidad_raw = struct.unpack_from('>H', data, 4)[0]
    return {"gps_vel_kmh": velocidad_raw * 0.1}

def _decodificador_neumatico(campo):
    """Las 4 ruedas usan el mismo formato: 2 bytes big-endian, 0.1 °C."""
    def decode(data):
        raw = struct.unpack_from('>H', data, 0)[0]
        return {campo: raw / 10.0}
    return decode

# CAN ID → (decodificador, función de guardado)
CAN_RX_HANDLERS = {
    CAN_ID_GPS_LATLON:   (_decodificar_gps_latlon, guardar_gps),
    CAN_ID_GPS_TIEMPO:   (_decodificar_gps_tiempo, guardar_gps),
    CAN_ID_NEUMATICO_DI: (_decodificador_neumatico("temp_neumatico_di"), guardar_neumaticos),
    CAN_ID_NEUMATICO_DD: (_decodificador_neumatico("temp_neumatico_dd"), guardar_neumaticos),
    CAN_ID_NEUMATICO_TI: (_decodificador_neumatico("temp_neumatico_ti"), guardar_neumaticos),
    CAN_ID_NEUMATICO_TD: (_decodificador_neumatico("temp_neumatico_td"), guardar_neumaticos),
}

_ultimo_error_can = ""

def hilo_can_rx(bus_can, stop):
    global _ultimo_error_can
    print("[CAN RX] Escuchando GPS y neumáticos (4 ruedas)...")
    while not stop.is_set():
        try:
            msg = bus_can.recv(timeout=0.1)
        except Exception as e:
            incrementar_stat("can_rx_err")
            _ultimo_error_can = f"RX {type(e).__name__}: {e}"[:80]
            time.sleep(0.1)
            continue
        if msg is None:
            continue
        handler = CAN_RX_HANDLERS.get(msg.arbitration_id)
        if handler:
            decoder, guardar = handler
            try:
                guardar(decoder(bytes(msg.data)))
            except Exception:
                continue  # frame corto o dato inválido
    print("[CAN RX] Hilo detenido.")

# =============================================================
# CAN TX (pantalla del cockpit)
# =============================================================

def _empaquetar_senal(payload, valor, sig):
    val = valor if valor is not None else 0
    raw = int((val + sig['add']) * sig['div'] * sig['mult']) & sig['mask']
    if sig['len'] == 1:
        payload[sig['off']] = raw & 0xFF
    elif sig['len'] == 2:
        payload[sig['off']]     = (raw >> 8) & 0xFF
        payload[sig['off'] + 1] =  raw        & 0xFF
    return payload

def enviar_can(bus_can, ecu, marcha):
    def enviar(can_id, señales):
        payload = [0] * 8
        for valor, sig in señales:
            _empaquetar_senal(payload, valor, sig)
        bus_can.send(can.Message(arbitration_id=can_id, data=payload, is_extended_id=False))

    datos_ecu = {
        'rpm': 0, 'tps': 0, 'afr': 0, 'temp_refrigerante': 0,
        'temp_aire': 0, 'bateria_v': 0, 'presion_aceite': 0
    }
    if ecu is not None:
        datos_ecu.update(ecu)

    enviar(CAN_ID_MOTOR,       [(datos_ecu['rpm'], SIG_RPM), (datos_ecu['tps'], SIG_TPS)])
    enviar(CAN_ID_TEMPS,       [(datos_ecu['temp_refrigerante'], SIG_TEMP_REFRIG),
                                (datos_ecu['temp_aire'], SIG_TEMP_AIRE),
                                (datos_ecu['bateria_v'], SIG_BATERIA)])
    enviar(CAN_ID_PRES_ACEITE, [(datos_ecu['presion_aceite'], SIG_PRES_ACEITE)])
    enviar(CAN_ID_MARCHA,      [(marcha, SIG_MARCHA)])
    enviar(CAN_ID_LAMBDA,      [(datos_ecu['afr'], SIG_LAMBDA)])

def hilo_can_tx(bus_can, stop):
    global _ultimo_error_can
    print("[CAN TX] Iniciado a 10 Hz...")
    while not stop.is_set():
        inicio = time.perf_counter()
        ecu = leer_ecu_sticky()
        marcha = leer_marcha()
        try:
            enviar_can(bus_can, ecu, marcha)
            incrementar_stat("can_tx_ok")
        except Exception as e:
            # el error se cuenta y el hilo sigue
            incrementar_stat("can_tx_err")
            _ultimo_error_can = f"{type(e).__name__}: {e}"[:80]

        resto = CAN_TX_PERIODO_S - (time.perf_counter() - inicio)
        if resto > 0:
            time.sleep(resto)
    print("[CAN TX] Hilo detenido.")

def _sigterm(signum, frame):
    """SIGTERM de systemd → apagado limpio, igual que Ctrl+C."""
    raise KeyboardInterrupt

# =============================================================
# CSV Y CONSOLA
# =============================================================

def hilo_csv(cola_csv, stop):
    print(f"[CSV] Guardando sesión en: {CSV_NOMBRE}")
    with open(CSV_NOMBRE, mode='w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=CSV_COLUMNAS)
        writer.writeheader()
        ultimo_fsync = time.perf_counter()

        while not stop.is_set() or not cola_csv.empty():
            try:
                fila = cola_csv.get(timeout=0.1)
                writer.writerow(fila)
                ahora = time.perf_counter()
                if ahora - ultimo_fsync >= FSYNC_CADA_S:
                    f.flush()
                    os.fsync(f.fileno())
                    ultimo_fsync = ahora
            except queue.Empty:
                continue
        f.flush()
        os.fsync(f.fileno())
    print("[CSV] Archivo cerrado.")

def mostrar_consola(estado):
    stats    = leer_stats()
    n_muest  = stats['n_muestras']
    n_fail   = stats['n_stm_fail']
    n_perd   = stats['n_csv_perdidas']
    tasa_err = (n_fail / n_muest * 100) if n_muest else 0.0

    marcha = NOMBRES_MARCHA.get(estado.get('marcha'), str(estado.get('marcha')))
    ecu_ok = estado.get('ecu_ok', False)

    c_err = VERDE if tasa_err < 1 else (AMARILLO if tasa_err < 5 else ROJO)
    c_ecu = VERDE if ecu_ok else ROJO

    print(CLEAR, end="")
    print(f"{BOLD}{CYAN}{'='*56}{RESET}")
    print(f"{BOLD}{CYAN}   TELEMETRÍA — STM32 + ECU + GPS{RESET}")
    print(f"{CYAN}{'='*56}{RESET}")
    print(f"\n  {BOLD}Muestra #{n_muest:<6}{RESET}  t_PC={estado.get('tiempo_pc_ms', 0):.1f} ms\n")

    print(f"  {'Marcha':<22}  {marcha:>10}")
    print(f"  {'Ángulo volante':<22}  {_fmt(estado.get('angulo'), 1)}  °")
    print(f"  {'Acel. lateral':<22}  {_fmt(estado.get('acel_lateral'), 3)}  g")
    print(f"  {'Acel X/Y/Z':<22}  {_fmt(estado.get('acel_x'), 2)} {_fmt(estado.get('acel_y'), 2)} {_fmt(estado.get('acel_z'), 2)}")
    print(f"  {'Giro X/Y/Z':<22}  {_fmt(estado.get('giro_x'), 2)} {_fmt(estado.get('giro_y'), 2)} {_fmt(estado.get('giro_z'), 2)}")
    print(f"  {'dx1':<22}  {_fmt(estado.get('dx1'), 1)}  mm")
    print(f"  {'dx2':<22}  {_fmt(estado.get('dx2'), 1)}  mm")
    print(f"  {'dx3':<22}  {_fmt(estado.get('dx3'), 1)}  mm")
    print(f"  {'Presión 1 (STM)':<22}  {_fmt(estado.get('stm_presion1'), 1)}")
    print(f"  {'Presión 2 (STM)':<22}  {_fmt(estado.get('stm_presion2'), 1)}")

    print(f"  {'RPM':<22}  {_fmt(estado.get('rpm'), 0)}  rpm")
    print(f"  {'TPS':<22}  {_fmt(estado.get('tps'), 1)}  %")
    print(f"  {'AFR':<22}  {_fmt(estado.get('afr'), 1)}")
    print(f"  {'Temp. refrigerante':<22}  {_fmt(estado.get('temp_refrigerante'))}  °C")
    print(f"  {'Temp. aire':<22}  {_fmt(estado.get('temp_aire'))}  °C")
    print(f"  {'Presión aceite':<22}  {_fmt(estado.get('presion_aceite'), 0)}")
    print(f"  {'Batería':<22}  {_fmt(estado.get('bateria_v'), 2)}  V")
    print(f"  {c_ecu}{'ECU':<22}  {'OK' if ecu_ok else 'SIN DATOS'}{RESET}")

    print(f"  {'Latitud':<22}  {_fmt(estado.get('gps_lat'), 7)}  °")
    print(f"  {'Longitud':<22}  {_fmt(estado.get('gps_lon'), 7)}  °")
    print(f"  {'Velocidad':<22}  {_fmt(estado.get('gps_vel_kmh'))}  km/h")

    print(f"  {'Neumático DI':<22}  {_fmt_neumatico(estado.get('temp_neumatico_di'))}  °C")
    print(f"  {'Neumático DD':<22}  {_fmt_neumatico(estado.get('temp_neumatico_dd'))}  °C")
    print(f"  {'Neumático TI':<22}  {_fmt_neumatico(estado.get('temp_neumatico_ti'))}  °C")
    print(f"  {'Neumático TD':<22}  {_fmt_neumatico(estado.get('temp_neumatico_td'))}  °C")

    print(f"\n  Lecturas SPI fallidas: {c_err}{n_fail} ({tasa_err:.1f}%){RESET}")
    print(f"    sin 0xAA en ventana:   {stats['fail_sync']}")
    print(f"    checksum XOR:          {stats['fail_chk']}")
    print(f"    desfases recuperados:  {stats['desfase_recuperado']}   SPI reabierto: {stats['spi_reabiertos']}")
    print(f"    bit slip recuperado:   {stats['bitslip_recuperado']}   cambios de corrimiento: {stats['slip_cambios']}")
    print(f"    corrimiento 1..7 bits: " + " ".join(f"{k}:{stats[f'bitslip_s{k}']}" for k in range(1, 8)))
    print(f"    contador ini != fin:   {stats['fail_cnt']}")
    print(f"    unpack:                {stats['fail_unpack']}")
    c_rst = VERDE if stats['stm_resets'] == 0 else AMARILLO
    print(f"    resets STM32 (NRST):   {c_rst}{stats['stm_resets']}{RESET}")
    print(f"    contador repetido:     {stats['cnt_repetido']}   saltos > {STM_DELTA_CNT_MAX}: {stats['cnt_salto']}")
    ultimo = leer_ultimo_fallo_spi()
    if ultimo:
        print(f"    {GRIS}última falla: {ultimo[0]}  [{ultimo[1]} ...]{RESET}")
    print(f"\n  CAN {CAN_CANAL}:   TX ok={stats['can_tx_ok']} err={stats['can_tx_err']}   RX err={stats['can_rx_err']}")
    if _ultimo_error_can:
        print(f"    {GRIS}último error CAN: {_ultimo_error_can}{RESET}")
    print(f"  CSV perdidos (cola llena): {n_perd}")
    print(f"\n  {GRIS}CSV → {CSV_NOMBRE}{RESET}")
    print(f"  {CYAN}Ctrl+C para detener{RESET}\n")

def hilo_consola(stop):
    while not stop.is_set():
        st = leer_estado_display()
        if st:
            mostrar_consola(st)
        time.sleep(DISPLAY_PERIODO_S)

# =============================================================
# LOOP PRINCIPAL
# =============================================================

def abrir_spi():
    spi = spidev.SpiDev()
    spi.open(SPI_BUS, SPI_DEVICE)
    spi.max_speed_hz = SPI_VELOCIDAD
    spi.mode = SPI_MODO
    return spi

def reabrir_spi(spi):
    """Cierra y reabre el SPI."""
    try:
        spi.close()
    except Exception:
        pass
    try:
        nuevo = abrir_spi()
        incrementar_stat("spi_reabiertos")
        print("[SPI] Dispositivo reabierto tras fallos seguidos.")
        return nuevo
    except Exception as e:
        print(f"[SPI] No se pudo reabrir: {e}")
        return spi

def main():
    try:
        signal.signal(signal.SIGTERM, _sigterm)   # systemctl stop → apagado limpio
    except (ValueError, OSError):
        pass
    stop = threading.Event()
    cola_csv = queue.Queue(maxsize=1000)

    # SPI
    try:
        spi = abrir_spi()
    except Exception as e:
        print(f"[ERROR] SPI init falló: {e}")
        return

    # CAN (can0 lo levanta el servicio)
    try:
        bus_can = can.interface.Bus(channel=CAN_CANAL, bustype='socketcan', bitrate=CAN_BITRATE)
    except Exception as e:
        print(f"[ERROR] CAN init falló ({CAN_CANAL}): {e}")
        spi.close()
        return

    # Hilos
    t_ecu = threading.Thread(target=hilo_ecu, args=(stop,), daemon=True)
    t_can_rx = threading.Thread(target=hilo_can_rx, args=(bus_can, stop), daemon=True)
    t_can_tx = threading.Thread(target=hilo_can_tx, args=(bus_can, stop), daemon=True)
    t_csv = threading.Thread(target=hilo_csv, args=(cola_csv, stop), daemon=False)
    t_display = threading.Thread(target=hilo_consola, args=(stop,), daemon=True) if MOSTRAR_CONSOLA else None

    t_ecu.start()
    t_can_rx.start()
    t_can_tx.start()
    t_csv.start()
    if t_display is not None:
        t_display.start()

    gpio_nrst_ok = iniciar_gpio_nrst()
    vigilante = VigilanteSTM()

    print(f"[MAIN] Loop principal a {1.0/CICLO_S:.0f} Hz...")
    t_inicio_pc = time.perf_counter()
    fallos_seguidos = 0

    try:
        while True:
            t_ciclo = time.perf_counter()

            # Tras un reset del STM no se toca el SPI hasta que arranque
            en_pausa = vigilante.spi_en_pausa(t_ciclo)
            if en_pausa:
                stm_datos = None
            else:
                stm_datos = leer_stm32(spi)
                if stm_datos is None:
                    time.sleep(SPI_REINTENTO_PAUSA_S)
                    stm_datos = leer_stm32(spi)
            incrementar_stat("n_muestras")

            if stm_datos is not None:
                fallos_seguidos = 0
                guardar_stm(stm_datos)
                marcha_ok = leer_stm_sticky().get("marcha")
                if marcha_ok is not None:
                    guardar_marcha(marcha_ok)
            elif not en_pausa:
                fallos_seguidos += 1
                if SPI_REABRIR_HABILITADO and fallos_seguidos >= SPI_FALLOS_PARA_REABRIR:
                    spi = reabrir_spi(spi)
                    fallos_seguidos = 0

            # Reset por hardware si el STM lleva ~1,6 s sin datos buenos
            motivo_reset = vigilante.evaluar(t_ciclo, stm_datos is not None, _cnt_repetido_seguidos)
            if motivo_reset and gpio_nrst_ok:
                if resetear_stm32(motivo_reset):
                    vigilante.registrar_reset(time.perf_counter())
                    fallos_seguidos = 0
                    print(f"[STM] SPI en pausa {STM_BOOT_PAUSA_S:.1f} s mientras el STM arranca (sin reabrir el dispositivo).")
            vigilante.reenviar_configuracion(t_ciclo)

            stm_actual = leer_stm_sticky()
            ecu_actual = leer_ecu_sticky()
            gps_actual = leer_gps()
            neumaticos_actual = leer_neumaticos()

            tiempo_ms = int((time.perf_counter() - t_inicio_pc) * 1000)

            fila = {"tiempo_pc_ms": tiempo_ms}
            fila.update(stm_actual)
            fila.update(ecu_actual)
            fila.update(gps_actual)
            fila.update(neumaticos_actual)

            try:
                cola_csv.put_nowait(fila)
            except queue.Full:
                incrementar_stat("n_csv_perdidas")

            # La consola recibe la fila más el estado de la ECU (no va al CSV)
            if MOSTRAR_CONSOLA:
                estado_display = dict(fila)
                estado_display["ecu_ok"] = ecu_esta_ok()
                actualizar_display(estado_display)

            dt = time.perf_counter() - t_ciclo
            resto = CICLO_S - dt
            if resto > 0:
                time.sleep(resto)

    except KeyboardInterrupt:
        print("\n[MAIN] Deteniendo sistema...")
        stop.set()
        t_csv.join(timeout=2.0)
        spi.close()
        try:
            bus_can.shutdown()
        except Exception:
            pass
        print("[MAIN] Apagado limpio completado.")

if __name__ == "__main__":
    main()
