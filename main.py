
import atexit
import struct
import threading
import time
from flask import Flask, jsonify, render_template_string
from cpx_io.cpx_system.cpx_ap.cpx_ap import CpxAp

# ============================================================
# CONFIGURACION
# ============================================================

CPX_IP = "192.168.0.88"
HOST = "0.0.0.0"
PORT = 8085

LED_COUNT = 20
VALVE_COUNT = 8

LED_STEP_DELAY = 0.05
VALVE_ON_DELAY = 0.25
VALVE_OFF_DELAY = 0.25
DEMO_PAUSE = 0.5
SENSOR_INTERVAL = 0.1
DISTANCE_LIMIT_MM = 3.95

# SMS V2: lectura de datos de proceso
SMS_MODULE_INDEX = 11
SMS_CHANNEL = 0
SMS_INTERVAL = 0.2

# Hipotesis de interpretacion de la posicion:
# bytes 6-9, entero de 32 bits con signo, big-endian, escala 1000
SMS_POSITION_OFFSET = 6
SMS_POSITION_LENGTH = 4
SMS_POSITION_SCALE = 1000.0

app = Flask(__name__)

# ============================================================
# ESTADO COMPARTIDO
# ============================================================

state_lock = threading.RLock()
startup_lock = threading.Lock()
sms_io_lock = threading.Lock()

stop_event = threading.Event()
motor_startup_complete = threading.Event()
motor_startup_status = "Pendiente"

sensor_data = {
    "distance": 0.0,
    "ssc1": False,
    "ssc2": False,
    "dsc1": False,
    "dsc2": False,
    "dsc3": False,
}

sms_data = {
    "position": None,
    "raw_hex": "",
    "length": 0,
}

valve_states = [False] * VALVE_COUNT
led_states = [False] * LED_COUNT
do8_i_states = [False] * 8
do8_a_states = [False] * 8

connection_status = "Desconectado"
last_error = ""

cpx = None

modules = {
    "ep": None,
    "vtux_interface": None,
    "valves_1": None,
    "valves_2": None,
    "vacuum": None,
    "do8_i": None,
    "iol_i": None,
    "backplane": None,
    "iol_soia": None,
    "di12_do4": None,
    "do8_a": None,
    "iol_sms": None,
}


# ============================================================
# FUNCIONES AUXILIARES
# ============================================================

def set_all_led_states(value=False):
    with state_lock:
        led_states[:] = [value] * LED_COUNT


def set_all_valve_states(value=False):
    with state_lock:
        valve_states[:] = [value] * VALVE_COUNT


def initialize_digital_outputs():
    do8_a = modules["do8_a"]
    do8_i = modules["do8_i"]

    if do8_a is None or do8_i is None:
        raise RuntimeError("Modulos DO8 no disponibles")

    # DO8-A: DO0 y DO1 permanecen activas.
    do8_a.set_channel(0)
    do8_a.set_channel(1)

    with state_lock:
        do8_a_states[0] = True
        do8_a_states[1] = True


def initialize_motor_fixed_outputs():
    """Comprueba que el modulo de salidas del motor esta disponible."""
    do_module = modules["di12_do4"]

    if do_module is None:
        raise RuntimeError(
            "Modulo de salidas del motor no disponible"
        )


# ============================================================
# ARRANQUE DEL MOTOR MEDIANTE STW1 / STW2
# ============================================================

def send_stw1(sms, value):
    # Evita que la lectura de monitorizacion coincida con la escritura.
    with sms_io_lock:
        pdo = bytearray(sms.read_channel(SMS_CHANNEL))

        if len(pdo) < 4:
            raise RuntimeError(
                f"Datos SMS insuficientes para STW1: {len(pdo)} bytes"
            )

        pdo[0] = value & 0xFF
        pdo[1] = (value >> 8) & 0xFF

        sms.write_channel(SMS_CHANNEL, bytes(pdo))


def send_stw2(sms, value):
    with sms_io_lock:
        pdo = bytearray(sms.read_channel(SMS_CHANNEL))

        if len(pdo) < 4:
            raise RuntimeError(
                f"Datos SMS insuficientes para STW2: {len(pdo)} bytes"
            )

        pdo[2] = value & 0xFF
        pdo[3] = (value >> 8) & 0xFF

        sms.write_channel(SMS_CHANNEL, bytes(pdo))


def run_motor_startup_sequence():
    """Ejecuta una sola vez la secuencia de arranque del motor."""
    global motor_startup_status

    sms = modules["iol_sms"]

    if sms is None:
        raise RuntimeError("Modulo SMS del motor no disponible")

    steps = (0x0004, 0x0604, 0x0704, 0x0F04, 0x7F04)

    for value in steps:
        if stop_event.is_set():
            return

        send_stw1(sms, value)

        if stop_event.wait(1):
            return

    with state_lock:
        motor_startup_status = "Completado"


def motor_startup_worker():
    global motor_startup_status, last_error

    try:
        with state_lock:
            motor_startup_status = "En curso"

        run_motor_startup_sequence()

    except Exception as error:
        with state_lock:
            motor_startup_status = "Error"
            last_error = f"Arranque del motor: {error}"

        print(f"ERROR EN ARRANQUE DEL MOTOR: {error}")

    finally:
        motor_startup_complete.set()


# ============================================================
# LECTURA DE DATOS DE PROCESO DEL SMS V2
# ============================================================

def read_sms_position(data):
    """
    Interpretacion provisional:
    bytes 6-9 como entero de 32 bits con signo, big-endian,
    dividido entre 1000.
    """

    if data is None:
        return None

    if len(data) < SMS_POSITION_OFFSET + SMS_POSITION_LENGTH:
        return None

    raw_position = int.from_bytes(
        data[
            SMS_POSITION_OFFSET:
            SMS_POSITION_OFFSET + SMS_POSITION_LENGTH
        ],
        byteorder="big",
        signed=True,
    )

    return raw_position / SMS_POSITION_SCALE


def read_sms_process_data():
    global last_error

    while not stop_event.is_set():
        try:
            sms = modules["iol_sms"]

            if sms is None:
                stop_event.wait(SMS_INTERVAL)
                continue

            # Coordina la lectura con las escrituras de STW1/STW2.
            with sms_io_lock:
                data = sms.read_channel(SMS_CHANNEL)

            if data is not None:
                raw = bytes(data)
                position = read_sms_position(raw)

                with state_lock:
                    sms_data["position"] = position
                    sms_data["raw_hex"] = raw.hex(" ").upper()
                    sms_data["length"] = len(raw)

        except Exception as error:
            with state_lock:
                last_error = f"Lectura SMS V2: {error}"

            print(f"ERROR LEYENDO SMS V2: {error}")

        stop_event.wait(SMS_INTERVAL)


# ============================================================
# LEDS IO-LINK
# ============================================================

def write_iol_leds(value):
    iol_i = modules["iol_i"]

    if iol_i is None:
        raise RuntimeError("Modulo IO-Link AP-I no disponible")

    iol_i.write_channel(
        0,
        int(value).to_bytes(4, byteorder="little", signed=False),
    )


def reset_known_outputs():
    """Intenta desactivar las salidas utilizadas por la demo."""

    try:
        if modules["iol_i"] is not None:
            write_iol_leds(0)
    except Exception as error:
        print(f"ERROR APAGANDO IO-LINK: {error}")

    for module_name in ("valves_1", "valves_2"):
        module = modules[module_name]

        if module is None:
            continue

        for channel in range(8):
            try:
                module.reset_channel(channel)
            except Exception as error:
                print(
                    f"ERROR RESETEANDO {module_name}, "
                    f"canal {channel}: {error}"
                )

    # No se apagan las salidas DO8 que tienen funcion permanente.
    set_all_led_states(False)
    set_all_valve_states(False)


def run_led_sequence():
    # Ida: LED 0 -> LED 19
    for bit in range(LED_COUNT):
        if stop_event.is_set():
            return

        write_iol_leds(1 << bit)

        with state_lock:
            for index in range(LED_COUNT):
                led_states[index] = index == bit

        if stop_event.wait(LED_STEP_DELAY):
            return

    # Vuelta: LED 18 -> LED 1
    for bit in range(LED_COUNT - 2, 0, -1):
        if stop_event.is_set():
            return

        write_iol_leds(1 << bit)

        with state_lock:
            for index in range(LED_COUNT):
                led_states[index] = index == bit

        if stop_event.wait(LED_STEP_DELAY):
            return

    write_iol_leds(0)
    set_all_led_states(False)
    stop_event.wait(DEMO_PAUSE)


# ============================================================
# SECUENCIA DE VALVULAS
# ============================================================

def run_valve_sequence():
    valve_pairs = [
        ("valves_1", 0, 1, 0),
        ("valves_1", 2, 3, 1),
        ("valves_1", 4, 5, 2),
        ("valves_1", 6, 7, 3),
        ("valves_2", 0, 1, 4),
        ("valves_2", 2, 3, 5),
        ("valves_2", 4, 5, 6),
        ("valves_2", 6, 7, 7),
    ]

    for module_name, channel_a, channel_b, index in valve_pairs:
        if stop_event.is_set():
            return

        module = modules[module_name]

        if module is None:
            raise RuntimeError(
                f"Modulo de valvulas no disponible: V{index + 1}"
            )

        module.set_channel(channel_a)
        module.set_channel(channel_b)

        with state_lock:
            valve_states[index] = True

        if stop_event.wait(VALVE_ON_DELAY):
            return

        module.reset_channel(channel_a)
        module.reset_channel(channel_b)

        with state_lock:
            valve_states[index] = False

        if stop_event.wait(VALVE_OFF_DELAY):
            return


# ============================================================
# SECUENCIA DE SALIDAS DIGITALES
# ============================================================

def run_digital_output_sequence():
    do8_i = modules["do8_i"]
    do8_a = modules["do8_a"]

    if do8_i is None or do8_a is None:
        raise RuntimeError("Modulos DO8 no disponibles")

    # Mantener DO0 y DO1 de DO8-A activas.
    do8_a.set_channel(0)
    do8_a.set_channel(1)

    with state_lock:
        do8_a_states[0] = True
        do8_a_states[1] = True

    for _ in range(4):
        if stop_event.is_set():
            return

        # Encender DO2-DO7 en ambos modulos.
        for channel in range(2, 8):
            do8_i.set_channel(channel)
            do8_a.set_channel(channel)

            with state_lock:
                do8_i_states[channel] = True
                do8_a_states[channel] = True

        if stop_event.wait(0.3):
            return

        # Apagar DO2-DO7 en ambos modulos.
        for channel in range(2, 8):
            do8_i.reset_channel(channel)
            do8_a.reset_channel(channel)

            with state_lock:
                do8_i_states[channel] = False
                do8_a_states[channel] = False

        if stop_event.wait(0.3):
            return


# ============================================================
# DEMO PRINCIPAL
# ============================================================

def demo_sequence():
    # Espera a que termine el arranque del motor.
    while (
        not stop_event.is_set()
        and not motor_startup_complete.wait(0.1)
    ):
        pass

    with state_lock:
        if motor_startup_status != "Completado":
            return

    while not stop_event.is_set():
        try:
            if (
                modules["iol_i"] is None
                or modules["valves_1"] is None
                or modules["valves_2"] is None
            ):
                stop_event.wait(1)
                continue

            run_led_sequence()
            run_valve_sequence()
            run_digital_output_sequence()

        except Exception as error:
            global last_error

            with state_lock:
                last_error = str(error)

            print(f"ERROR EN DEMO: {error}")
            reset_known_outputs()
            stop_event.wait(1)


# ============================================================
# LECTURA DEL SENSOR SOIA
# ============================================================

def read_soia_sensor():
    global last_error

    while not stop_event.is_set():
        try:
            iol_sensor = modules["iol_soia"]

            if iol_sensor is None:
                stop_event.wait(SENSOR_INTERVAL)
                continue

            data = iol_sensor.read_channel(0)

            if data is not None and len(data) == 4:
                data = data[::-1]

                status = data[0]
                scale = struct.unpack("b", data[1:2])[0]
                raw = struct.unpack("<h", data[2:4])[0]

                distance_mm = raw * (10 ** scale) * 1000

                with state_lock:
                    sensor_data["distance"] = distance_mm
                    sensor_data["ssc1"] = bool(status & 0x01)
                    sensor_data["ssc2"] = bool(status & 0x02)
                    sensor_data["dsc1"] = bool(status & 0x10)
                    sensor_data["dsc2"] = bool(status & 0x20)
                    sensor_data["dsc3"] = bool(status & 0x40)

        except Exception as error:
            with state_lock:
                last_error = str(error)

            print(f"ERROR LEYENDO SENSOR SOIA: {error}")

        stop_event.wait(SENSOR_INTERVAL)


# ============================================================
# CONEXION CON EL CPX-AP
# ============================================================

def start_background_tasks():
    global cpx, connection_status, last_error

    with startup_lock:
        if cpx is not None:
            return

        with state_lock:
            connection_status = "Conectando..."
            last_error = ""

        try:
            print("CONECTANDO CON CPX-AP...")

            cpx = CpxAp(ip_address=CPX_IP)

            module_names = [
                "ep",
                "vtux_interface",
                "valves_1",
                "valves_2",
                "vacuum",
                "do8_i",
                "iol_i",
                "backplane",
                "iol_soia",
                "di12_do4",
                "do8_a",
                "iol_sms",
            ]

            if len(cpx.modules) < len(module_names):
                raise RuntimeError(
                    f"Se esperaban 12 modulos y se encontraron "
                    f"{len(cpx.modules)}"
                )

            for index, name in enumerate(module_names):
                modules[name] = cpx.modules[index]

            initialize_digital_outputs()
            initialize_motor_fixed_outputs()

            threading.Thread(
                target=read_soia_sensor,
                daemon=True,
                name="soia_sensor",
            ).start()

            threading.Thread(
                target=read_sms_process_data,
                daemon=True,
                name="sms_process_data",
            ).start()

            threading.Thread(
                target=motor_startup_worker,
                daemon=True,
                name="motor_startup",
            ).start()

            threading.Thread(
                target=demo_sequence,
                daemon=True,
                name="demo_sequence",
            ).start()

            with state_lock:
                connection_status = "Conectado"

        except Exception as error:
            with state_lock:
                last_error = str(error)
                connection_status = "Error de conexion"

            print(f"ERROR INICIALIZANDO CPX-AP: {error}")

            failed_cpx = cpx
            cpx = None

            for name in modules:
                modules[name] = None

            if failed_cpx is not None:
                close_method = getattr(failed_cpx, "close", None)
                if callable(close_method):
                    try:
                        close_method()
                    except Exception:
                        pass


# ============================================================
# API DE ESTADO PARA EL NAVEGADOR
# ============================================================

@app.get("/api/status")
def api_status():
    with state_lock:
        return jsonify({
            "connection_status": connection_status,
            "motor_startup_status": motor_startup_status,
            "last_error": last_error,
            "sensor": dict(sensor_data),
            "sms": dict(sms_data),
            "valves": list(valve_states),
            "do8_i": list(do8_i_states),
            "do8_a": list(do8_a_states),
            "leds": list(led_states),
            "distance_limit_mm": DISTANCE_LIMIT_MM,
        })


# ============================================================
# INTERFAZ WEB
# ============================================================

HTML = r"""
<!DOCTYPE html>
<html lang="es">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>IO Link by FESTO</title>

<style>
    :root {
        font-family: Arial, Helvetica, sans-serif;
        color: #172b3a;
        background: #f2f5f7;
        font-synthesis: none;
        text-rendering: optimizeLegibility;
    }

    * { box-sizing: border-box; }

    body {
        margin: 0 auto;
        padding: 0 26px 26px;
        max-width: 1800px;
        min-height: 100vh;
        background: linear-gradient(
            180deg, #fff 0, #fff 150px, #f2f5f7 150px
        );
    }

    header {
        display: flex;
        justify-content: space-between;
        align-items: center;
        gap: 24px;
        flex-wrap: wrap;
        margin-bottom: 22px;
        border-bottom: 1px solid #dbe3e8;
    }

    h1 {
        margin: 0 0 7px;
        font-size: clamp(25px, 3vw, 34px);
        color: #12344a;
    }

    h2 {
        font-size: 16px;
        line-height: 1.35;
        margin: 0 0 15px;
        color: #12344a;
    }

    .muted {
        color: #667b89;
        font-size: 13px;
    }

    .header-logos {
        display: flex;
        align-items: center;
        gap: 12px;
        min-width: 0;
        height: 100px;
    }

    .io-link-logo {
        width: 220px;
        max-width: 45vw;
        height: 60px;
        object-fit: contain;
        object-position: left center;
    }

    .festo-logo {
        width: 200px;
        max-width: 60vw;
        object-fit: contain;
    }

    .logo-by {
        font-size: 20px;
        font-weight: 600;
        white-space: nowrap;
    }

    .main-layout {
        display: grid;
        grid-template-columns: minmax(260px, 0.8fr) minmax(0, 1.5fr);
        align-items: start;
        gap: 22px;
    }

    .image-placeholder {
        position: sticky;
        top: 20px;
        min-height: 680px;
        height: calc(100vh - 210px);
        max-height: 1050px;
        border: 2px dashed #aebfc9;
        border-radius: 8px;
        background: #eaf0f3;
        display: flex;
        align-items: center;
        justify-content: center;
        padding: 24px;
        overflow: hidden;
    }

    .system-image {
        width: 100%;
        height: 100%;
        object-fit: contain;
    }

    .dashboard {
        display: flex;
        flex-direction: column;
        gap: 15px;
        min-width: 0;
    }

    .card {
        background: #fff;
        border: 1px solid #dce5ea;
        border-top: 3px solid #009fe3;
        border-radius: 6px;
        padding: 19px 21px;
        min-width: 0;
        box-shadow: 0 3px 12px rgba(20, 53, 72, .045);
    }

    .distance {
        font-size: clamp(32px, 4vw, 48px);
        font-weight: 750;
        margin: 3px 0 0;
        color: #009fe3;
        font-variant-numeric: tabular-nums;
    }

    .indicators {
        display: flex;
        flex-wrap: wrap;
        gap: 18px;
    }

    .indicator {
        min-width: 48px;
        display: flex;
        align-items: center;
        flex-direction: column;
        gap: 8px;
        font-size: 11px;
        font-weight: 700;
        color: #526978;
    }

    .lamp {
        width: 18px;
        height: 18px;
        border-radius: 50%;
        background: #d8e0e5;
        border: 2px solid #b7c5ce;
        box-shadow: inset 0 1px 2px rgba(14, 39, 53, .12);
    }

    .lamp.green {
        background: #20bd73;
        border-color: #a2e8c5;
    }

    .lamp.blue {
        background: #009fe3;
        border-color: #9cdef7;
    }

    .lamp.yellow {
        background: #f5bd25;
        border-color: #ffe6a0;
    }

    .lamp-grid {
        display: grid;
        grid-template-columns: repeat(10, minmax(25px, 1fr));
        gap: 14px 7px;
    }

    .lamp-grid .indicator { min-width: 0; }

    .value-placeholder-grid {
        display: grid;
        grid-template-columns: repeat(2, minmax(0, 1fr));
        gap: 12px;
    }

    .value-placeholder {
        min-height: 94px;
        border: 2px dashed #b7c8d2;
        border-radius: 5px;
        background: #f7fafb;
        padding: 15px;
        display: flex;
        flex-direction: column;
        justify-content: center;
        gap: 8px;
        min-width: 0;
    }

    /* SOIA y SMS en la misma fila */
    .position-cards {
        display: grid;
        grid-template-columns: repeat(2, minmax(0, 1fr));
        align-items: stretch;
        gap: 15px;
        margin-bottom: 15px;
    }

    .position-cards .card {
        height: 100%;
        margin: 0;
    }

    .soia-layout,
    .sms-layout {
        display: grid;
        grid-template-columns: minmax(0, 1fr) 160px;
        align-items: center;
        gap: 8px;
        min-height: 130px;
        width: 100%;
    }

    .soia-layout .distance {
        min-width: 0;
        font-size: clamp(24px, 2.5vw, 40px);
        white-space: nowrap;
    }

    /* SMS sin recuadro: tipografía y color a juego con el valor SOIA */
    .sms-layout .value-placeholder {
        width: auto;
        max-width: none;
        min-width: 0;
        min-height: 0;
        padding: 0;
        border: 0;
        border-radius: 0;
        background: transparent;
        box-shadow: none;
        display: flex;
        flex-direction: column;
        justify-content: center;
        gap: 5px;
        flex-shrink: 1;
    }

    .sms-layout .label {
        color: #526978;
        font-size: 14px;
        font-weight: 600;
    }

    .sms-layout .value {
        color: #009fe3;
        font-size: clamp(24px, 2.5vw, 40px);
        font-weight: 750;
        line-height: 1.15;
        font-variant-numeric: tabular-nums;
        overflow-wrap: anywhere;
    }

    .soia-image{
        display: block;
        position: static;
        width: 100px;
        max-width: 100%;
        height: 80px;
        object-fit: contain;
        transform: none;
    }

    .sms-image {
        display: block;
        position: static;
        width: 160px;
        max-width: 100%;
        height: 130px;
        object-fit: contain;
        transform: none;
    }

    .sms-raw {
        margin-top: 12px;
        overflow-wrap: anywhere;
        word-break: break-word;
    }

    .error {
        color: #b4232f;
        overflow-wrap: anywhere;
        font-size: 13px;
        margin-top: 15px;
    }

    footer {
        margin-top: 22px;
        padding-top: 14px;
        border-top: 1px solid #dbe3e8;
        color: #728591;
        font-size: 12px;
    }

    @media (max-width: 850px) {
        body {
            padding: 0 17px 17px;
            background: linear-gradient(
                180deg, #fff 0, #fff 190px, #f2f5f7 190px
            );
        }

        .main-layout { grid-template-columns: 1fr; }

        .image-placeholder {
            position: relative;
            top: auto;
            min-height: 260px;
            height: 42vh;
            max-height: 420px;
        }

        .header-logos {
            flex-wrap: wrap;
            gap: 14px;
        }

        .io-link-logo { width: 180px; }
        .festo-logo { width: 95px; }
        .lamp-grid { grid-template-columns: repeat(5, minmax(25px, 1fr)); }
    }
    @media (max-width: 850px) {
        .position-cards {
            grid-template-columns: repeat(2, minmax(0, 1fr));
        }

        .soia-layout {
            grid-template-columns: minmax(0, 1fr) 75px;
            gap: 6px;
        }

        .sms-layout {
            grid-template-columns: minmax(0, 1fr) 130px;
            gap: 6px;
        }

        .soia-image {
            width: 75px;
            height: 70px;
        }

        .sms-image {
            width: 130px;
            height: 100px;
        }
    }

    @media (max-width: 560px) {
        .position-cards {
            grid-template-columns: 1fr;
        }

        .soia-layout {
            grid-template-columns: minmax(0, 1fr) 100px;
        }

        .sms-layout {
            grid-template-columns: minmax(0, 1fr) 160px;
        }

        .soia-image {
            width: 100px;
            height: 80px;
        }

        .sms-image {
            width: 160px;
            height: 130px;
        }
    }

    @media (max-width: 480px) {
        .card { padding: 16px; }
        .value-placeholder-grid { grid-template-columns: 1fr; }
        .indicators { gap: 15px; }
    }
</style>
</head>

<body>
<header>
    <div class="header-logos">
        <img
            src="/proxy/8085/static/IOLink.svg"
            alt="IO-Link"
            class="io-link-logo"
        >
        <span class="logo-by">by</span>
        <img
            src="/proxy/8085/static/Festo.png"
            alt="FESTO"
            class="festo-logo"
        >
    </div>
</header>

<main class="main-layout">
    <aside class="image-placeholder">
        <img
            src="/proxy/8085/static/SystemConfig.png"
            alt="System Configuration"
            class="system-image"
        >
    </aside>

    <div>
        <div class="dashboard">
            <div class="position-cards">
                <section class="card">
                <h2>SOIA Position</h2>

                <div class="soia-layout">
                    <div class="distance" id="distance">--.-- mm</div>

                    <img
                        src="/proxy/8085/static/SOIA.jpg"
                        alt="Sensor SOIA"
                        class="soia-image"
                    >
                </div>
            </section>

            <section class="card">
                <h2>SMS V2 Position</h2>

                <div class="sms-layout">
                    <div class="value-placeholder">
                        <span class="value" id="sms-position">--</span>
                    </div>

                    <img
                        src="/proxy/8085/static/SMS2.png"
                        alt="SMS V2"
                        class="sms-image"
                    >
                </div>
                </section>
            </div>

            <section class="card">
                <h2>VTUX-A-S-API</h2>
                <div id="valves" class="indicators"></div>
            </section>

            <section class="card">
                <h2>CPX-AP-I-8DO</h2>
                <div id="do8-i" class="indicators"></div>
            </section>

            <section class="card">
                <h2>CPX-AP-A-8DO</h2>
                <div id="do8-a" class="indicators"></div>
            </section>

            <section class="card">
                <h2>VTUX-A-P-IOS</h2>
                <div id="leds" class="lamp-grid"></div>
            </section>
        </div>

        <div id="error" class="error"></div>
    </div>
</main>

<script>
    function createIndicators(containerId, labels) {
        const container = document.getElementById(containerId);
        container.replaceChildren();

        labels.forEach((label, index) => {
            const item = document.createElement("div");
            item.className = "indicator";

            const lamp = document.createElement("span");
            lamp.className = "lamp";
            lamp.id = containerId + "-" + index;

            const text = document.createElement("span");
            text.textContent = label;

            item.append(lamp, text);
            container.append(item);
        });
    }

    function setLamp(id, active, activeColor = "green") {
        const lamp = document.getElementById(id);
        if (!lamp) return;

        lamp.className = "lamp" + (active ? " " + activeColor : "");
    }

    function updateIndicators(containerId, values, color) {
        values.forEach((active, index) => {
            setLamp(containerId + "-" + index, active, color);
        });
    }

    createIndicators(
        "valves",
        Array.from({length: 8}, (_, i) => "V" + (i + 1))
    );
    createIndicators(
        "do8-i",
        Array.from({length: 8}, (_, i) => "DO" + i)
    );
    createIndicators(
        "do8-a",
        Array.from({length: 8}, (_, i) => "DO" + i)
    );
    createIndicators(
        "leds",
        Array.from({length: 20}, (_, i) => String(i))
    );

    async function refresh() {
        try {
            const pagePath = window.location.pathname.replace(/\/$/, "");
            const apiUrl = pagePath.endsWith("/proxy/8085")
                ? pagePath + "/api/status"
                : "/api/status";

            const response = await fetch(apiUrl, {
                cache: "no-store"
            });

            if (!response.ok) {
                throw new Error("HTTP " + response.status);
            }

            const data = await response.json();
            const sensor = data.sensor;

            // Sensor SOIA
            const distance = document.getElementById("distance");

            if (sensor.distance > data.distance_limit_mm) {
                distance.textContent = "Out of range";
                distance.style.color = "#cfd5da";
            } else {
                distance.textContent =
                    Number(sensor.distance).toFixed(2) + " mm";
                distance.style.color = "#009fe3";
            }

            // SMS V2
            const sms = data.sms || {};

            const smsPosition = document.getElementById("sms-position");
            const smsSpeed = document.getElementById("sms-speed");
            const smsRaw = document.getElementById("sms-raw");

            if (
                sms.position !== null &&
                sms.position !== undefined &&
                Number.isFinite(Number(sms.position))
            ) {
                smsPosition.textContent =
                    Number(sms.position).toFixed(3) + " mm";
            } else {
                smsPosition.textContent = "--";
            }

            // Indicadores
            updateIndicators("valves", data.valves, "blue");
            updateIndicators("do8-i", data.do8_i, "green");
            updateIndicators("do8-a", data.do8_a, "green");
            updateIndicators("leds", data.leds, "yellow");

            document.getElementById("error").textContent =
                data.last_error
                    ? "Último error: " + data.last_error
                    : "";

        } catch (error) {
            console.error("Error actualizando el dashboard:", error);

            document.getElementById("error").textContent =
                "No se puede consultar el servidor: " + error.message;
        }
    }

    refresh();
    setInterval(refresh, 100);
</script>
</body>
</html>
"""


@app.get("/")
def index():
    return render_template_string(HTML)


# ============================================================
# CIERRE
# ============================================================

def shutdown():
    stop_event.set()

    try:
        reset_known_outputs()
    except Exception as error:
        print(f"Error durante el cierre: {error}")

    current_cpx = cpx

    close_method = getattr(current_cpx, "close", None)

    if callable(close_method):
        try:
            close_method()
        except Exception as error:
            print(f"Error cerrando CPX-AP: {error}")


atexit.register(shutdown)


if __name__ == "__main__":
    start_background_tasks()

    app.run(
        host=HOST,
        port=PORT,
        debug=False,
        threaded=True,
        use_reloader=False,
    )
