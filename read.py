#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Lectura Modbus RTU RS485 para Circutor Computer SMART III / Smart Computer III.

Estructura recomendada del proyecto:

    MODBUS_PROJECT/
    ├── .venv/
    ├── config.ini
    ├── read.py
    ├── requirements.txt
    └── data/
        └── raw/

Instalación inicial en Windows PowerShell, desde la carpeta MODBUS_PROJECT:

    python -m venv .venv
    .\.venv\Scripts\Activate.ps1
    python -m pip install --upgrade pip
    pip install minimalmodbus pyserial
    pip freeze > requirements.txt

Ejecución normal:

    python read.py

El script lee por defecto el archivo:

    config.ini

Puedes sobrescribir valores del config.ini desde la terminal, por ejemplo:

    python read.py --port COM5 --slave 1 --once

Notas importantes:
- El mapa del manual usa direcciones HEXADECIMALES. Este script usa esas direcciones directamente.
- Para variables de medida se usa función Modbus 04.
- Por defecto, el equipo suele usar 19200 baudios, 8N1, slave/periférico 1.
- El manual limita las tramas a 80 bytes; por seguridad aquí se leen registros de forma individual.
"""

import argparse
import configparser
import csv
import json
import os
import sys
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Callable, Dict, List, Optional, Tuple

import minimalmodbus
import serial

now_utc = datetime.now(timezone.utc)
now_local = now_utc.astimezone()

# -----------------------------
# Rutas base del proyecto
# -----------------------------
# Como tu estructura es:
#   MODBUS_PROJECT/
#   ├── config.ini
#   └── read.py
# PROJECT_ROOT será automáticamente la carpeta MODBUS_PROJECT.
PROJECT_ROOT = Path(__file__).resolve().parent
DEFAULT_CONFIG_PATH = PROJECT_ROOT / "config.ini"


# -----------------------------
# Ajustes de escalamiento
# -----------------------------
# En el manual aparece tensión como V/100 en la tabla, pero el ejemplo Modbus indica:
# 0x0000084D = 2125 -> 212.5 V, por lo que el factor práctico es /10.
# Si en campo observas 21.25 V o 2125 V, cambia --voltage-scale a 0.01 o 1.
DEFAULT_VOLTAGE_SCALE = 0.1


class Reg:
    def __init__(
        self,
        name: str,
        address: int,
        kind: str = "u32",
        scale: float = 1.0,
        unit: str = "",
        description: str = "",
    ):
        self.name = name
        self.address = address
        self.kind = kind
        self.scale = scale
        self.unit = unit
        self.description = description


# Variables principales de medida: función 04
# Direcciones según mapa Modbus del Computer SMART III.
MEASUREMENTS: List[Reg] = [
    # Fase L1
    Reg("voltage_L1_V", 0x0000, "u32", DEFAULT_VOLTAGE_SCALE, "V", "Tensión fase L1"),
    Reg("current_L1_A", 0x0002, "u32", 0.001, "A", "Corriente L1"),
    Reg("active_power_L1_kW", 0x0004, "u32", 0.001, "kW", "Potencia activa L1"),
    Reg("reactive_inductive_L1_kvar", 0x0006, "u32", 0.001, "kvarL", "Potencia reactiva inductiva L1"),
    Reg("reactive_capacitive_L1_kvar", 0x0008, "u32", 0.001, "kvarC", "Potencia reactiva capacitiva L1"),
    Reg("reactive_power_L1_kvar", 0x000A, "u32", 0.001, "kvar", "Potencia reactiva L1"),
    Reg("apparent_power_L1_kVA", 0x000C, "u32", 0.001, "kVA", "Potencia aparente L1"),
    Reg("power_factor_L1", 0x0012, "u32", 0.01, "", "Factor de potencia L1"),
    Reg("cos_phi_L1", 0x0014, "u32", 0.01, "", "Cos phi L1"),
    Reg("sign_kw_L1", 0x0016, "s32", 1.0, "", "Signo de kW L1"),
    Reg("sign_kvar_L1", 0x0018, "s32", 1.0, "", "Signo de kvar L1"),

    # Fase L2
    Reg("voltage_L2_V", 0x001A, "u32", DEFAULT_VOLTAGE_SCALE, "V", "Tensión fase L2"),
    Reg("current_L2_A", 0x001C, "u32", 0.001, "A", "Corriente L2"),
    Reg("active_power_L2_kW", 0x001E, "u32", 0.001, "kW", "Potencia activa L2"),
    Reg("reactive_inductive_L2_kvar", 0x0020, "u32", 0.001, "kvarL", "Potencia reactiva inductiva L2"),
    Reg("reactive_capacitive_L2_kvar", 0x0022, "u32", 0.001, "kvarC", "Potencia reactiva capacitiva L2"),
    Reg("reactive_power_L2_kvar", 0x0024, "u32", 0.001, "kvar", "Potencia reactiva L2"),
    Reg("apparent_power_L2_kVA", 0x0026, "u32", 0.001, "kVA", "Potencia aparente L2"),
    Reg("power_factor_L2", 0x002C, "u32", 0.01, "", "Factor de potencia L2"),
    Reg("cos_phi_L2", 0x002E, "u32", 0.01, "", "Cos phi L2"),
    Reg("sign_kw_L2", 0x0030, "s32", 1.0, "", "Signo de kW L2"),
    Reg("sign_kvar_L2", 0x0032, "s32", 1.0, "", "Signo de kvar L2"),

    # Fase L3
    Reg("voltage_L3_V", 0x0034, "u32", DEFAULT_VOLTAGE_SCALE, "V", "Tensión fase L3"),
    Reg("current_L3_A", 0x0036, "u32", 0.001, "A", "Corriente L3"),
    Reg("active_power_L3_kW", 0x0038, "u32", 0.001, "kW", "Potencia activa L3"),
    Reg("reactive_inductive_L3_kvar", 0x003A, "u32", 0.001, "kvarL", "Potencia reactiva inductiva L3"),
    Reg("reactive_capacitive_L3_kvar", 0x003C, "u32", 0.001, "kvarC", "Potencia reactiva capacitiva L3"),
    Reg("reactive_power_L3_kvar", 0x003E, "u32", 0.001, "kvar", "Potencia reactiva L3"),
    Reg("apparent_power_L3_kVA", 0x0040, "u32", 0.001, "kVA", "Potencia aparente L3"),
    Reg("power_factor_L3", 0x0046, "u32", 0.01, "", "Factor de potencia L3"),
    Reg("cos_phi_L3", 0x0048, "u32", 0.01, "", "Cos phi L3"),
    Reg("sign_kw_L3", 0x004A, "s32", 1.0, "", "Signo de kW L3"),
    Reg("sign_kvar_L3", 0x004C, "s32", 1.0, "", "Signo de kvar L3"),

    # Trifásico
    Reg("voltage_3ph_V", 0x004E, "u32", DEFAULT_VOLTAGE_SCALE, "V", "Tensión fase trifásica"),
    Reg("current_3ph_A", 0x0050, "u32", 0.001, "A", "Corriente trifásica"),
    Reg("active_power_3ph_kW", 0x0052, "u32", 0.001, "kW", "Potencia activa trifásica"),
    Reg("reactive_inductive_3ph_kvar", 0x0054, "u32", 0.001, "kvarL", "Potencia inductiva trifásica"),
    Reg("reactive_capacitive_3ph_kvar", 0x0056, "u32", 0.001, "kvarC", "Potencia capacitiva trifásica"),
    Reg("reactive_power_3ph_kvar", 0x0058, "u32", 0.001, "kvar", "Potencia reactiva trifásica"),
    Reg("apparent_power_3ph_kVA", 0x005A, "u32", 0.001, "kVA", "Potencia aparente trifásica"),
    Reg("power_factor_3ph", 0x0060, "u32", 0.01, "", "Factor de potencia trifásico"),
    Reg("cos_phi_3ph", 0x0062, "u32", 0.01, "", "Cos phi trifásico"),
    Reg("sign_kw_3ph", 0x0064, "s32", 1.0, "", "Signo de kW trifásico"),
    Reg("sign_kvar_3ph", 0x0066, "s32", 1.0, "", "Signo de kvar trifásico"),
    Reg("frequency_Hz", 0x0068, "u32", 0.1, "Hz", "Frecuencia"),
    Reg("voltage_L1_L2_V", 0x006A, "u32", DEFAULT_VOLTAGE_SCALE, "V", "Tensión L1-L2"),
    Reg("voltage_L2_L3_V", 0x006C, "u32", DEFAULT_VOLTAGE_SCALE, "V", "Tensión L2-L3"),
    Reg("voltage_L3_L1_V", 0x006E, "u32", DEFAULT_VOLTAGE_SCALE, "V", "Tensión L3-L1"),
    Reg("neutral_current_A", 0x0070, "u32", 0.001, "A", "Corriente de neutro"),
    Reg("leakage_current_mA", 0x0072, "u32", 1.0, "mA", "Corriente de fugas"),
    Reg("temperature_C", 0x0074, "u32", 0.1, "°C", "Temperatura"),

    # THD. El manual muestra % para THD; si el valor aparece x10, cambia el scale a 0.1.
    Reg("thd_voltage_L1_pct", 0x007C, "u32", 1.0, "%", "THD tensión L1"),
    Reg("thd_voltage_L2_pct", 0x007E, "u32", 1.0, "%", "THD tensión L2"),
    Reg("thd_voltage_L3_pct", 0x0080, "u32", 1.0, "%", "THD tensión L3"),
    Reg("thd_current_L1_pct", 0x0082, "u32", 1.0, "%", "THD corriente L1"),
    Reg("thd_current_L2_pct", 0x0084, "u32", 1.0, "%", "THD corriente L2"),
    Reg("thd_current_L3_pct", 0x0086, "u32", 1.0, "%", "THD corriente L3"),
]


# Energías: el manual separa kWh y Wh; aquí se integran en kWh/kvarh/kVAh.
ENERGY_PAIRS: List[Tuple[str, int, int, str]] = [
    ("active_energy_import_kWh", 0x0088, 0x008A, "Energía activa consumida"),
    ("reactive_inductive_import_kvarh", 0x008C, 0x008E, "Energía inductiva consumida"),
    ("reactive_capacitive_import_kvarh", 0x0090, 0x0092, "Energía capacitiva consumida"),
    ("apparent_energy_import_kVAh", 0x0094, 0x0096, "Energía aparente consumida"),
    ("active_energy_export_kWh", 0x0098, 0x009A, "Energía activa generada"),
    ("reactive_inductive_export_kvarh", 0x009C, 0x009E, "Energía inductiva generada"),
    ("reactive_capacitive_export_kvarh", 0x00A0, 0x00A2, "Energía capacitiva generada"),
    ("apparent_energy_export_kVAh", 0x00A4, 0x00A6, "Energía aparente generada"),
]



def resolve_project_path(path_value: str) -> str:
    """
    Convierte rutas relativas del config.ini a rutas absolutas dentro del proyecto.

    Ejemplo:
        data/raw/smart_computer_iii_data.csv

    se convierte en:
        C:/.../MODBUS_PROJECT/data/raw/smart_computer_iii_data.csv
    """
    path = Path(path_value)
    if path.is_absolute():
        return str(path)
    return str(PROJECT_ROOT / path)


def load_runtime_args(cli_args: argparse.Namespace) -> argparse.Namespace:
    """
    Carga la configuración desde config.ini y permite sobrescribir algunos valores desde consola.

    Prioridad:
        1. Argumentos escritos en terminal, por ejemplo: --port COM5
        2. Valores definidos en config.ini
        3. Valores por defecto dentro del código
    """
    config_path = Path(cli_args.config)
    if not config_path.is_absolute():
        config_path = PROJECT_ROOT / config_path

    if not config_path.exists():
        raise FileNotFoundError(f"No encontré el archivo de configuración: {config_path}")

    config = configparser.ConfigParser()
    config.read(config_path, encoding="utf-8")

    args = argparse.Namespace()

    # Comunicación RS485 / Modbus RTU
    args.port = cli_args.port or config.get("serial", "port", fallback="COM5")
    args.baud = cli_args.baud if cli_args.baud is not None else config.getint("serial", "baudrate", fallback=19200)
    args.parity = cli_args.parity or config.get("serial", "parity", fallback="none")
    args.bytesize = cli_args.bytesize if cli_args.bytesize is not None else config.getint("serial", "bytesize", fallback=8)
    args.stopbits = cli_args.stopbits if cli_args.stopbits is not None else config.getint("serial", "stopbits", fallback=1)
    args.timeout = cli_args.timeout if cli_args.timeout is not None else config.getfloat("serial", "timeout", fallback=1.0)

    # Equipo
    args.slave = cli_args.slave if cli_args.slave is not None else config.getint("device", "slave_id", fallback=1)
    args.max_relays = (
        cli_args.max_relays
        if cli_args.max_relays is not None
        else config.getint("device", "max_relays", fallback=14)
    )

    # Lectura
    config_once = config.getboolean("read", "once", fallback=True)
    args.once = True if cli_args.once else config_once
    args.interval = (
        cli_args.interval
        if cli_args.interval is not None
        else config.getfloat("read", "interval_seconds", fallback=0)
    )
    args.retries = cli_args.retries if cli_args.retries is not None else config.getint("read", "retries", fallback=1)
    args.status = True if cli_args.status else config.getboolean("read", "status", fallback=True)
    args.config_read = (
        True if cli_args.config_read else config.getboolean("read", "config_read", fallback=False)
    )

    # Salida / logging
    csv_from_config = config.get("logging", "csv_path", fallback="data/raw/smart_computer_iii_data.csv")
    args.csv = resolve_project_path(cli_args.csv or csv_from_config)
    args.json = True if cli_args.json else config.getboolean("logging", "print_json", fallback=False)
    args.debug = True if cli_args.debug else config.getboolean("logging", "debug", fallback=False)

    # Escalamiento
    args.voltage_scale = (
        cli_args.voltage_scale
        if cli_args.voltage_scale is not None
        else config.getfloat("scaling", "voltage_scale", fallback=DEFAULT_VOLTAGE_SCALE)
    )

    args.config_path = str(config_path)
    return args


def parse_parity(value: str) -> str:
    value = value.lower().strip()
    if value in ("n", "none", "sin", "no"):
        return serial.PARITY_NONE
    if value in ("e", "even", "par"):
        return serial.PARITY_EVEN
    if value in ("o", "odd", "impar"):
        return serial.PARITY_ODD
    raise ValueError("Paridad no válida. Usa: none, even u odd.")


def make_instrument(
    port: str,
    slave: int,
    baudrate: int,
    parity: str,
    bytesize: int,
    stopbits: int,
    timeout: float,
    debug: bool = False,
) -> minimalmodbus.Instrument:
    instrument = minimalmodbus.Instrument(port, slave)
    instrument.mode = minimalmodbus.MODE_RTU
    instrument.debug = debug

    instrument.serial.baudrate = baudrate
    instrument.serial.bytesize = bytesize
    instrument.serial.parity = parity
    instrument.serial.stopbits = stopbits
    instrument.serial.timeout = timeout

    instrument.clear_buffers_before_each_transaction = True
    instrument.close_port_after_each_call = False
    return instrument


def read_u16(inst: minimalmodbus.Instrument, address: int, functioncode: int = 4) -> int:
    return int(inst.read_register(address, number_of_decimals=0, functioncode=functioncode, signed=False))


def read_u32(inst: minimalmodbus.Instrument, address: int, functioncode: int = 4) -> int:
    return int(
        inst.read_long(
            address,
            functioncode=functioncode,
            signed=False,
            byteorder=minimalmodbus.BYTEORDER_BIG,
            number_of_registers=2,
        )
    )


def read_s32(inst: minimalmodbus.Instrument, address: int, functioncode: int = 4) -> int:
    return int(
        inst.read_long(
            address,
            functioncode=functioncode,
            signed=True,
            byteorder=minimalmodbus.BYTEORDER_BIG,
            number_of_registers=2,
        )
    )


def safe_read(
    read_func: Callable[[], Any],
    field_name: str,
    errors: Dict[str, str],
    retries: int = 1,
    retry_delay: float = 0.2,
) -> Optional[Any]:
    last_error = None
    for attempt in range(retries + 1):
        try:
            return read_func()
        except Exception as exc:  # minimalmodbus/serial exceptions
            last_error = exc
            if attempt < retries:
                time.sleep(retry_delay)
    errors[field_name] = str(last_error)
    return None


def scaled(value: Optional[int], factor: float) -> Optional[float]:
    if value is None:
        return None
    return value * factor


def apply_voltage_scale(measurements: List[Reg], voltage_scale: float) -> None:
    for reg in measurements:
        if reg.name.startswith("voltage_"):
            reg.scale = voltage_scale


def read_measurements(
    inst: minimalmodbus.Instrument,
    voltage_scale: float = DEFAULT_VOLTAGE_SCALE,
    retries: int = 1,
) -> Tuple[Dict[str, Any], Dict[str, str]]:
    errors: Dict[str, str] = {}
    data: Dict[str, Any] = {
    "timestamp_local": now_local.isoformat(timespec="seconds"),
    "timestamp_utc": now_utc.isoformat(timespec="seconds"),
    }

    apply_voltage_scale(MEASUREMENTS, voltage_scale)

    for reg in MEASUREMENTS:
        if reg.kind == "u32":
            raw = safe_read(lambda r=reg: read_u32(inst, r.address, 4), reg.name, errors, retries=retries)
        elif reg.kind == "s32":
            raw = safe_read(lambda r=reg: read_s32(inst, r.address, 4), reg.name, errors, retries=retries)
        elif reg.kind == "u16":
            raw = safe_read(lambda r=reg: read_u16(inst, r.address, 4), reg.name, errors, retries=retries)
        else:
            errors[reg.name] = f"Tipo no soportado: {reg.kind}"
            raw = None

        data[reg.name + "_raw"] = raw
        data[reg.name] = scaled(raw, reg.scale)

    # Valores firmados calculados con los registros de signo del equipo
    for phase in ("L1", "L2", "L3", "3ph"):
        p = data.get(f"active_power_{phase}_kW")
        sign_kw = data.get(f"sign_kw_{phase}")
        q = data.get(f"reactive_power_{phase}_kvar")
        sign_kvar = data.get(f"sign_kvar_{phase}")

        if isinstance(p, (int, float)) and sign_kw in (-1, 1):
            data[f"active_power_{phase}_kW_signed"] = p * sign_kw
        if isinstance(q, (int, float)) and sign_kvar in (-1, 1):
            data[f"reactive_power_{phase}_kvar_signed"] = q * sign_kvar

    # Energías integradas: valor_k + valor_unidad/1000
    for name, k_addr, unit_addr, _desc in ENERGY_PAIRS:
        k_val = safe_read(lambda a=k_addr: read_u32(inst, a, 4), name + "_k_raw", errors, retries=retries)
        unit_val = safe_read(lambda a=unit_addr: read_u32(inst, a, 4), name + "_unit_raw", errors, retries=retries)
        data[name + "_k_raw"] = k_val
        data[name + "_unit_raw"] = unit_val

        if k_val is not None and unit_val is not None:
            data[name] = k_val + (unit_val / 1000.0)
        else:
            data[name] = None

    return data, errors


def decode_relays(value: Optional[int], max_relays: int = 14) -> Dict[str, Optional[bool]]:
    out: Dict[str, Optional[bool]] = {}
    if value is None:
        for i in range(1, max_relays + 1):
            out[f"relay_{i}_on"] = None
        return out

    for i in range(1, max_relays + 1):
        out[f"relay_{i}_on"] = bool(value & (1 << (i - 1)))
    return out


def decode_alarms(value: Optional[int]) -> Dict[str, Optional[bool]]:
    out: Dict[str, Optional[bool]] = {}
    if value is None:
        for i in range(1, 18):
            out[f"alarm_E{i:02d}_on"] = None
        return out

    for i in range(1, 18):
        out[f"alarm_E{i:02d}_on"] = bool(value & (1 << (i - 1)))
    return out


def decode_outputs(value: Optional[int]) -> Dict[str, Optional[bool]]:
    if value is None:
        return {
            "fan_relay_on": None,
            "alarm_relay_on": None,
            "digital_output_1_on": None,
            "digital_output_2_on": None,
        }

    # Manual:
    # Bit 0: relé ventilador -> 1 ON / 0 OFF
    # Bit 1: relé alarma -> 1 ON / 0 OFF
    # Bit 2: salida digital 1 -> 1 OFF / 0 ON
    # Bit 3: salida digital 2 -> 1 OFF / 0 ON
    return {
        "fan_relay_on": bool(value & (1 << 0)),
        "alarm_relay_on": bool(value & (1 << 1)),
        "digital_output_1_on": not bool(value & (1 << 2)),
        "digital_output_2_on": not bool(value & (1 << 3)),
    }


def decode_inputs(value: Optional[int]) -> Dict[str, Optional[bool]]:
    if value is None:
        return {"digital_input_1_on": None, "digital_input_2_on": None}

    # Manual: bit 0 y bit 1 -> 1 ON / 0 OFF
    return {
        "digital_input_1_on": bool(value & (1 << 0)),
        "digital_input_2_on": bool(value & (1 << 1)),
    }


def read_status(
    inst: minimalmodbus.Instrument,
    retries: int = 1,
    max_relays: int = 14,
) -> Tuple[Dict[str, Any], Dict[str, str]]:
    errors: Dict[str, str] = {}
    data: Dict[str, Any] = {}

    relay_var = safe_read(lambda: read_u16(inst, 0x0600, 4), "relay_variable", errors, retries=retries)
    alarm_var = safe_read(lambda: read_u32(inst, 0x0605, 4), "alarm_variable", errors, retries=retries)
    outputs_var = safe_read(lambda: read_u16(inst, 0x0610, 4), "outputs_status", errors, retries=retries)
    inputs_var = safe_read(lambda: read_u16(inst, 0x0615, 4), "digital_inputs_status", errors, retries=retries)

    data["relay_variable_raw"] = relay_var
    data["alarm_variable_raw"] = alarm_var
    data["outputs_status_raw"] = outputs_var
    data["digital_inputs_status_raw"] = inputs_var

    data.update(decode_relays(relay_var, max_relays=max_relays))
    data.update(decode_alarms(alarm_var))
    data.update(decode_outputs(outputs_var))
    data.update(decode_inputs(inputs_var))

    return data, errors


def read_device_config_basic(
    inst: minimalmodbus.Instrument,
    retries: int = 1,
) -> Tuple[Dict[str, Any], Dict[str, str]]:
    """Lectura básica de configuración de comunicaciones con función 04."""
    errors: Dict[str, str] = {}
    data: Dict[str, Any] = {}

    config_regs = [
        ("config_slave_id", 0x1071),
        ("config_speed_code", 0x1072),     # 0=9600, 1=19200
        ("config_parity_code", 0x1073),    # 0=none, 1=odd, 2=even
        ("config_length_code", 0x1074),    # 0=8 bits, 1=7 bits
        ("config_stopbits_code", 0x1075),  # 0=1 bit, 1=2 bits
    ]

    for name, addr in config_regs:
        data[name] = safe_read(lambda a=addr: read_u16(inst, a, 4), name, errors, retries=retries)

    return data, errors


def append_csv(path: str, row: Dict[str, Any]) -> None:
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)

    file_exists = os.path.exists(path) and os.path.getsize(path) > 0

    if file_exists:
        with open(path, "r", newline="", encoding="utf-8") as f:
            reader = csv.reader(f)
            existing_header = next(reader)
        fieldnames = existing_header
        # Si aparecen nuevas columnas, reescribimos encabezado preservando datos previos.
        missing = [k for k in row.keys() if k not in fieldnames]
        if missing:
            with open(path, "r", newline="", encoding="utf-8") as f:
                old_rows = list(csv.DictReader(f))
            fieldnames = fieldnames + missing
            with open(path, "w", newline="", encoding="utf-8") as f:
                writer = csv.DictWriter(f, fieldnames=fieldnames)
                writer.writeheader()
                writer.writerows(old_rows)
    else:
        fieldnames = list(row.keys())

    with open(path, "a", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames, extrasaction="ignore")
        if not file_exists:
            writer.writeheader()
        writer.writerow(row)


def print_human(data: Dict[str, Any], errors: Dict[str, str]) -> None:
    selected = [
        "timestamp_local",
        "timestamp_utc",
        "voltage_L1_V",
        "voltage_L2_V",
        "voltage_L3_V",
        "voltage_L1_L2_V",
        "voltage_L2_L3_V",
        "voltage_L3_L1_V",
        "current_L1_A",
        "current_L2_A",
        "current_L3_A",
        "current_3ph_A",
        "active_power_3ph_kW_signed",
        "reactive_power_3ph_kvar_signed",
        "apparent_power_3ph_kVA",
        "power_factor_3ph",
        "cos_phi_3ph",
        "frequency_Hz",
        "temperature_C",
        "active_energy_import_kWh",
        "active_energy_export_kWh",
    ]

    print("\n--- Lectura Computer SMART III ---")
    for key in selected:
        if key in data:
            print(f"{key}: {data.get(key)}")

    relay_keys = [k for k in data if k.startswith("relay_") and k.endswith("_on")]
    if relay_keys:
        active_relays = [k.replace("_on", "") for k in relay_keys if data.get(k) is True]
        print(f"relays_active: {active_relays}")

    alarm_keys = [k for k in data if k.startswith("alarm_E") and k.endswith("_on")]
    if alarm_keys:
        active_alarms = [k.replace("_on", "") for k in alarm_keys if data.get(k) is True]
        print(f"alarms_active: {active_alarms}")

    if errors:
        print("\n--- Errores de lectura ---")
        for k, v in errors.items():
            print(f"{k}: {v}")


def run_once(args: argparse.Namespace) -> Tuple[Dict[str, Any], Dict[str, str]]:
    parity = parse_parity(args.parity)
    inst = make_instrument(
        port=args.port,
        slave=args.slave,
        baudrate=args.baud,
        parity=parity,
        bytesize=args.bytesize,
        stopbits=args.stopbits,
        timeout=args.timeout,
        debug=args.debug,
    )

    row, errors = read_measurements(
        inst,
        voltage_scale=args.voltage_scale,
        retries=args.retries,
    )

    if args.status:
        status, status_errors = read_status(inst, retries=args.retries, max_relays=args.max_relays)
        row.update(status)
        errors.update(status_errors)

    if args.config_read:
        config, config_errors = read_device_config_basic(inst, retries=args.retries)
        row.update(config)
        errors.update(config_errors)

    row["slave_id"] = args.slave
    row["serial_port"] = args.port

    return row, errors


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Lectura Modbus RTU RS485 para Circutor Computer SMART III / Smart Computer III."
    )

    # El único argumento realmente necesario es --config si quieres usar otro archivo.
    # Si no lo pasas, el script busca config.ini en la misma carpeta que read.py.
    parser.add_argument(
        "--config",
        default=str(DEFAULT_CONFIG_PATH),
        help="Ruta del archivo de configuración. Default: config.ini junto a read.py",
    )

    # Todos los siguientes argumentos son opcionales y sirven para sobrescribir config.ini.
    parser.add_argument("--port", default=None, help="Puerto serial. Ej: COM5, /dev/ttyUSB0")
    parser.add_argument("--slave", type=int, default=None, help="ID Modbus / Num. periférico")
    parser.add_argument("--baud", type=int, default=None, help="Baudios. Ej: 19200")
    parser.add_argument("--parity", default=None, help="Paridad: none, even, odd")
    parser.add_argument("--bytesize", type=int, default=None, help="Bits de datos. Normalmente 8")
    parser.add_argument("--stopbits", type=int, default=None, help="Stop bits. Normalmente 1")
    parser.add_argument("--timeout", type=float, default=None, help="Timeout serial en segundos")
    parser.add_argument("--retries", type=int, default=None, help="Reintentos por registro")
    parser.add_argument("--interval", type=float, default=None, help="Intervalo entre lecturas en segundos")
    parser.add_argument("--once", action="store_true", help="Forzar una sola lectura")
    parser.add_argument("--csv", default=None, help="Ruta de salida CSV")
    parser.add_argument("--json", action="store_true", help="Imprime JSON completo")
    parser.add_argument("--status", action="store_true", help="Incluye relés, alarmas, salidas y entradas digitales")
    parser.add_argument("--config-read", action="store_true", help="Lee configuración básica de comunicaciones")
    parser.add_argument("--max-relays", type=int, default=None, choices=[6, 12, 14], help="Modelo por número de relés")
    parser.add_argument("--voltage-scale", type=float, default=None, help="Factor para tensión. Default recomendado: 0.1")
    parser.add_argument("--debug", action="store_true", help="Activa debug de minimalmodbus")

    cli_args = parser.parse_args()

    try:
        args = load_runtime_args(cli_args)

        if not (1 <= args.slave <= 254):
            print("ERROR: slave_id debe estar entre 1 y 254.", file=sys.stderr)
            return 2

        print("\n--- Configuración activa ---")
        print(f"Config: {args.config_path}")
        print(f"Puerto: {args.port}")
        print(f"Slave ID: {args.slave}")
        print(f"Baudrate: {args.baud}")
        print(f"CSV: {args.csv}")
        print(f"Once: {args.once}")
        print(f"Intervalo: {args.interval} s")

        while True:
            row, errors = run_once(args)

            if args.json:
                print(json.dumps({"data": row, "errors": errors}, indent=2, ensure_ascii=False))
            else:
                print_human(row, errors)

            append_csv(args.csv, row)
            print(f"\nCSV actualizado: {args.csv}")

            if args.once or args.interval <= 0:
                break

            time.sleep(args.interval)

    except KeyboardInterrupt:
        print("\nInterrumpido por usuario.")
        return 0
    except serial.SerialException as exc:
        print(f"ERROR serial: {exc}", file=sys.stderr)
        return 1
    except Exception as exc:
        print(f"ERROR general: {exc}", file=sys.stderr)
        return 1

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
