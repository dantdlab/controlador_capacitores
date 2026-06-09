#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Lectura Modbus RTU RS485 para Circutor Computer SMART III / Smart Computer III.

Instalacion:
    pip install minimalmodbus pyserial

Ejemplos:
    python read.py
    python read.py --port COM9 --interval 5

Para activar el entorno virtual en PowerShell:
    ./.venv/Scripts/Activate.ps1

Notas importantes:
- El mapa del manual usa direcciones hexadecimales. Este script usa esas direcciones directamente.
- Para variables de medida se usa funcion Modbus 04.
- Por defecto: 19200 baudios, 8N1, slave/periferico 1.
- El manual limita las tramas a 80 bytes; por seguridad aqui se leen registros de forma individual.
"""

from __future__ import annotations

import argparse
import configparser
import csv
import json
import sys
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Callable, Dict, List, Optional, Tuple

import minimalmodbus
import serial


DEFAULT_VOLTAGE_SCALE = 0.1
SCRIPT_DIR = Path(__file__).resolve().parent
DEFAULT_CONFIG_PATH = SCRIPT_DIR / "config.ini"
DEFAULT_CSV_PATH = SCRIPT_DIR / "data" / "raw" / "smart_computer_iii_data.csv"


class Reg:
    def __init__(
        self,
        name: str,
        address: int,
        kind: str = "u32",
        scale: float = 1.0,
        unit: str = "",
        description: str = "",
    ) -> None:
        self.name = name
        self.address = address
        self.kind = kind
        self.scale = scale
        self.unit = unit
        self.description = description


MEASUREMENTS: List[Reg] = [
    Reg("voltage_L1_V", 0x0000, "u32", DEFAULT_VOLTAGE_SCALE, "V", "Voltage L1"),
    Reg("current_L1_A", 0x0002, "u32", 0.001, "A", "Current L1"),
    Reg("active_power_L1_kW", 0x0004, "u32", 0.001, "kW", "Active power L1"),
    Reg("reactive_inductive_L1_kvar", 0x0006, "u32", 0.001, "kvarL", "Inductive reactive power L1"),
    Reg("reactive_capacitive_L1_kvar", 0x0008, "u32", 0.001, "kvarC", "Capacitive reactive power L1"),
    Reg("reactive_power_L1_kvar", 0x000A, "u32", 0.001, "kvar", "Reactive power L1"),
    Reg("apparent_power_L1_kVA", 0x000C, "u32", 0.001, "kVA", "Apparent power L1"),
    Reg("power_factor_L1", 0x0012, "u32", 0.01, "", "Power factor L1"),
    Reg("cos_phi_L1", 0x0014, "u32", 0.01, "", "Cos phi L1"),
    Reg("sign_kw_L1", 0x0016, "s32", 1.0, "", "Sign kW L1"),
    Reg("sign_kvar_L1", 0x0018, "s32", 1.0, "", "Sign kvar L1"),

    Reg("voltage_L2_V", 0x001A, "u32", DEFAULT_VOLTAGE_SCALE, "V", "Voltage L2"),
    Reg("current_L2_A", 0x001C, "u32", 0.001, "A", "Current L2"),
    Reg("active_power_L2_kW", 0x001E, "u32", 0.001, "kW", "Active power L2"),
    Reg("reactive_inductive_L2_kvar", 0x0020, "u32", 0.001, "kvarL", "Inductive reactive power L2"),
    Reg("reactive_capacitive_L2_kvar", 0x0022, "u32", 0.001, "kvarC", "Capacitive reactive power L2"),
    Reg("reactive_power_L2_kvar", 0x0024, "u32", 0.001, "kvar", "Reactive power L2"),
    Reg("apparent_power_L2_kVA", 0x0026, "u32", 0.001, "kVA", "Apparent power L2"),
    Reg("power_factor_L2", 0x002C, "u32", 0.01, "", "Power factor L2"),
    Reg("cos_phi_L2", 0x002E, "u32", 0.01, "", "Cos phi L2"),
    Reg("sign_kw_L2", 0x0030, "s32", 1.0, "", "Sign kW L2"),
    Reg("sign_kvar_L2", 0x0032, "s32", 1.0, "", "Sign kvar L2"),

    Reg("voltage_L3_V", 0x0034, "u32", DEFAULT_VOLTAGE_SCALE, "V", "Voltage L3"),
    Reg("current_L3_A", 0x0036, "u32", 0.001, "A", "Current L3"),
    Reg("active_power_L3_kW", 0x0038, "u32", 0.001, "kW", "Active power L3"),
    Reg("reactive_inductive_L3_kvar", 0x003A, "u32", 0.001, "kvarL", "Inductive reactive power L3"),
    Reg("reactive_capacitive_L3_kvar", 0x003C, "u32", 0.001, "kvarC", "Capacitive reactive power L3"),
    Reg("reactive_power_L3_kvar", 0x003E, "u32", 0.001, "kvar", "Reactive power L3"),
    Reg("apparent_power_L3_kVA", 0x0040, "u32", 0.001, "kVA", "Apparent power L3"),
    Reg("power_factor_L3", 0x0046, "u32", 0.01, "", "Power factor L3"),
    Reg("cos_phi_L3", 0x0048, "u32", 0.01, "", "Cos phi L3"),
    Reg("sign_kw_L3", 0x004A, "s32", 1.0, "", "Sign kW L3"),
    Reg("sign_kvar_L3", 0x004C, "s32", 1.0, "", "Sign kvar L3"),

    Reg("voltage_3ph_V", 0x004E, "u32", DEFAULT_VOLTAGE_SCALE, "V", "Three phase voltage"),
    Reg("current_3ph_A", 0x0050, "u32", 0.001, "A", "Three phase current"),
    Reg("active_power_3ph_kW", 0x0052, "u32", 0.001, "kW", "Three phase active power"),
    Reg("reactive_inductive_3ph_kvar", 0x0054, "u32", 0.001, "kvarL", "Three phase inductive reactive power"),
    Reg("reactive_capacitive_3ph_kvar", 0x0056, "u32", 0.001, "kvarC", "Three phase capacitive reactive power"),
    Reg("reactive_power_3ph_kvar", 0x0058, "u32", 0.001, "kvar", "Three phase reactive power"),
    Reg("apparent_power_3ph_kVA", 0x005A, "u32", 0.001, "kVA", "Three phase apparent power"),
    Reg("power_factor_3ph", 0x0060, "u32", 0.01, "", "Three phase power factor"),
    Reg("cos_phi_3ph", 0x0062, "u32", 0.01, "", "Three phase cos phi"),
    Reg("sign_kw_3ph", 0x0064, "s32", 1.0, "", "Three phase sign kW"),
    Reg("sign_kvar_3ph", 0x0066, "s32", 1.0, "", "Three phase sign kvar"),
    Reg("frequency_Hz", 0x0068, "u32", 0.1, "Hz", "Frequency"),
    Reg("voltage_L1_L2_V", 0x006A, "u32", DEFAULT_VOLTAGE_SCALE, "V", "L1-L2 voltage"),
    Reg("voltage_L2_L3_V", 0x006C, "u32", DEFAULT_VOLTAGE_SCALE, "V", "L2-L3 voltage"),
    Reg("voltage_L3_L1_V", 0x006E, "u32", DEFAULT_VOLTAGE_SCALE, "V", "L3-L1 voltage"),
    Reg("neutral_current_A", 0x0070, "u32", 0.001, "A", "Neutral current"),
    Reg("leakage_current_mA", 0x0072, "u32", 1.0, "mA", "Leakage current"),
    Reg("temperature_C", 0x0074, "u32", 0.1, "C", "Temperature"),
    Reg("thd_voltage_L1_pct", 0x007C, "u32", 1.0, "%", "Voltage THD L1"),
    Reg("thd_voltage_L2_pct", 0x007E, "u32", 1.0, "%", "Voltage THD L2"),
    Reg("thd_voltage_L3_pct", 0x0080, "u32", 1.0, "%", "Voltage THD L3"),
    Reg("thd_current_L1_pct", 0x0082, "u32", 1.0, "%", "Current THD L1"),
    Reg("thd_current_L2_pct", 0x0084, "u32", 1.0, "%", "Current THD L2"),
    Reg("thd_current_L3_pct", 0x0086, "u32", 1.0, "%", "Current THD L3"),
]


ENERGY_PAIRS: List[Tuple[str, int, int, str]] = [
    ("active_energy_import_kWh", 0x0088, 0x008A, "Imported active energy"),
    ("reactive_inductive_import_kvarh", 0x008C, 0x008E, "Imported inductive reactive energy"),
    ("reactive_capacitive_import_kvarh", 0x0090, 0x0092, "Imported capacitive reactive energy"),
    ("apparent_energy_import_kVAh", 0x0094, 0x0096, "Imported apparent energy"),
    ("active_energy_export_kWh", 0x0098, 0x009A, "Exported active energy"),
    ("reactive_inductive_export_kvarh", 0x009C, 0x009E, "Exported inductive reactive energy"),
    ("reactive_capacitive_export_kvarh", 0x00A0, 0x00A2, "Exported capacitive reactive energy"),
    ("apparent_energy_export_kVAh", 0x00A4, 0x00A6, "Exported apparent energy"),
]


def parse_parity(value: str) -> str:
    value = value.lower().strip()
    if value in ("n", "none", "sin", "no"):
        return serial.PARITY_NONE
    if value in ("e", "even", "par"):
        return serial.PARITY_EVEN
    if value in ("o", "odd", "impar"):
        return serial.PARITY_ODD
    raise ValueError("Paridad no valida. Usa: none, even u odd.")


def parse_int(parser: configparser.ConfigParser, section: str, option: str, fallback: int) -> int:
    try:
        return parser.getint(section, option, fallback=fallback)
    except (ValueError, TypeError):
        return fallback


def parse_float(parser: configparser.ConfigParser, section: str, option: str, fallback: float) -> float:
    try:
        return parser.getfloat(section, option, fallback=fallback)
    except (ValueError, TypeError):
        return fallback


def parse_text(parser: configparser.ConfigParser, section: str, option: str, fallback: str) -> str:
    try:
        return parser.get(section, option, fallback=fallback).strip()
    except (ValueError, TypeError, AttributeError):
        return fallback


def parse_flag(parser: configparser.ConfigParser, section: str, option: str, fallback: bool = False) -> bool:
    raw = parse_text(parser, section, option, "1" if fallback else "0").lower()
    return raw in ("1", "true", "yes", "on")


def resolve_path(path_value: str, base_dir: Path) -> Path:
    candidate = Path(path_value).expanduser()
    if not candidate.is_absolute():
        candidate = base_dir / candidate
    return candidate.resolve()


def load_config(config_path: Path = DEFAULT_CONFIG_PATH) -> Dict[str, Any]:
    if not config_path.exists():
        raise FileNotFoundError(f"No se encontro config.ini en: {config_path}")

    parser = configparser.ConfigParser()
    parser.read(config_path, encoding="utf-8")

    csv_value = parse_text(
        parser,
        "logging",
        "csv_path",
        str((SCRIPT_DIR / "data" / "raw" / "smart_computer_iii_data.csv").relative_to(SCRIPT_DIR)),
    )

    return {
        "config_path": config_path,
        "project_root": config_path.parent,
        "port": parse_text(parser, "serial", "port", "COM8"),
        "baud": parse_int(parser, "serial", "baudrate", 19200),
        "parity": parse_text(parser, "serial", "parity", "none"),
        "bytesize": parse_int(parser, "serial", "bytesize", 8),
        "stopbits": parse_int(parser, "serial", "stopbits", 1),
        "timeout": parse_float(parser, "serial", "timeout", 1.0),
        "slave": parse_int(parser, "device", "slave_id", 1),
        "max_relays": parse_int(parser, "device", "max_relays", 14),
        "once": parse_flag(parser, "read", "once", False),
        "interval_seconds": parse_float(parser, "read", "interval_seconds", 10.0),
        "retries": parse_int(parser, "read", "retries", 1),
        "status": parse_flag(parser, "read", "status", True),
        "config_read": parse_flag(parser, "read", "config_read", False),
        "csv_path": resolve_path(csv_value, config_path.parent),
        "print_json": parse_flag(parser, "logging", "print_json", False),
        "debug": parse_flag(parser, "logging", "debug", False),
        "voltage_scale": parse_float(parser, "scaling", "voltage_scale", DEFAULT_VOLTAGE_SCALE),
    }


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Lectura Modbus RTU RS485 para Circutor Computer SMART III / Smart Computer III."
    )
    parser.add_argument("--port", default=None, help="Puerto serial. Ej: COM5, /dev/ttyUSB0, /dev/serial/by-id/...")
    parser.add_argument("--slave", type=int, default=None, help="ID Modbus / Num. periferico")
    parser.add_argument("--baud", type=int, default=None, help="Baudios")
    parser.add_argument("--parity", default=None, help="Paridad: none, even, odd")
    parser.add_argument("--bytesize", type=int, default=None, help="Bits de datos")
    parser.add_argument("--stopbits", type=int, default=None, help="Stop bits")
    parser.add_argument("--timeout", type=float, default=None, help="Timeout serial en segundos")
    parser.add_argument("--retries", type=int, default=None, help="Reintentos por registro")
    parser.add_argument("--interval", type=float, default=None, help="Intervalo en segundos")
    parser.add_argument("--once", action="store_true", default=None, help="Ejecuta una sola lectura")
    parser.add_argument("--csv", default=None, help="Ruta de salida CSV")
    parser.add_argument("--json", action="store_true", default=None, help="Imprime JSON completo")
    parser.add_argument("--status", action="store_true", default=None, help="Incluye relays, alarmas, salidas e inputs")
    parser.add_argument("--config-read", action="store_true", default=None, help="Lee configuracion basica")
    parser.add_argument("--max-relays", type=int, default=None, choices=[6, 12, 14], help="Modelo por numero de relays")
    parser.add_argument("--voltage-scale", type=float, default=None, help="Factor para tension")
    parser.add_argument("--debug", action="store_true", default=None, help="Activa debug de minimalmodbus")
    return parser


def merge_settings(base: Dict[str, Any], args: argparse.Namespace) -> Dict[str, Any]:
    settings = dict(base)

    arg_to_setting = {
        "port": "port",
        "slave": "slave",
        "baud": "baud",
        "parity": "parity",
        "bytesize": "bytesize",
        "stopbits": "stopbits",
        "timeout": "timeout",
        "retries": "retries",
        "interval": "interval_seconds",
        "max_relays": "max_relays",
        "voltage_scale": "voltage_scale",
    }

    for arg_name, setting_name in arg_to_setting.items():
        value = getattr(args, arg_name, None)
        if value is not None:
            settings[setting_name] = value

    csv_value = getattr(args, "csv", None)
    if csv_value is not None:
        settings["csv_path"] = resolve_path(str(csv_value), base["project_root"])

    for flag_name, setting_name in (
        ("once", "once"),
        ("json", "print_json"),
        ("status", "status"),
        ("config-read", "config_read"),
        ("debug", "debug"),
    ):
        value = getattr(args, flag_name.replace("-", "_"), None)
        if value is not None:
            settings[setting_name] = bool(value)

    return settings


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
    last_error: Optional[Exception] = None
    for attempt in range(retries + 1):
        try:
            return read_func()
        except Exception as exc:
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
        "timestamp_local": datetime.now().astimezone().isoformat(timespec="seconds"),
        "timestamp_utc": datetime.now(timezone.utc).isoformat(timespec="seconds"),
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

    for phase in ("L1", "L2", "L3", "3ph"):
        p = data.get(f"active_power_{phase}_kW")
        sign_kw = data.get(f"sign_kw_{phase}")
        q = data.get(f"reactive_power_{phase}_kvar")
        sign_kvar = data.get(f"sign_kvar_{phase}")

        if isinstance(p, (int, float)) and sign_kw in (-1, 1):
            data[f"active_power_{phase}_kW_signed"] = p * sign_kw
        if isinstance(q, (int, float)) and sign_kvar in (-1, 1):
            data[f"reactive_power_{phase}_kvar_signed"] = q * sign_kvar

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

    return {
        "fan_relay_on": bool(value & (1 << 0)),
        "alarm_relay_on": bool(value & (1 << 1)),
        "digital_output_1_on": not bool(value & (1 << 2)),
        "digital_output_2_on": not bool(value & (1 << 3)),
    }


def decode_inputs(value: Optional[int]) -> Dict[str, Optional[bool]]:
    if value is None:
        return {"digital_input_1_on": None, "digital_input_2_on": None}

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
    errors: Dict[str, str] = {}
    data: Dict[str, Any] = {}

    config_regs = [
        ("config_slave_id", 0x1071),
        ("config_speed_code", 0x1072),
        ("config_parity_code", 0x1073),
        ("config_length_code", 0x1074),
        ("config_stopbits_code", 0x1075),
    ]

    for name, addr in config_regs:
        data[name] = safe_read(lambda a=addr: read_u16(inst, a, 4), name, errors, retries=retries)

    return data, errors


def append_csv(path: str, row: Dict[str, Any]) -> None:
    path_obj = Path(path)
    path_obj.parent.mkdir(parents=True, exist_ok=True)

    file_exists = path_obj.exists() and path_obj.stat().st_size > 0
    if file_exists:
        with path_obj.open("r", newline="", encoding="utf-8") as f:
            reader = csv.reader(f)
            try:
                fieldnames = next(reader)
            except StopIteration:
                fieldnames = list(row.keys())
    else:
        fieldnames = list(row.keys())

    with path_obj.open("a", newline="", encoding="utf-8") as f:
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


def collect_row(inst: minimalmodbus.Instrument, settings: Dict[str, Any]) -> Tuple[Dict[str, Any], Dict[str, str]]:
    parity = parse_parity(settings["parity"])
    inst.serial.baudrate = settings["baud"]
    inst.serial.bytesize = settings["bytesize"]
    inst.serial.parity = parity
    inst.serial.stopbits = settings["stopbits"]
    inst.serial.timeout = settings["timeout"]
    inst.debug = settings["debug"]

    row, errors = read_measurements(
        inst,
        voltage_scale=settings["voltage_scale"],
        retries=settings["retries"],
    )

    if settings["status"]:
        status, status_errors = read_status(inst, retries=settings["retries"], max_relays=settings["max_relays"])
        row.update(status)
        errors.update(status_errors)

    if settings["config_read"]:
        config, config_errors = read_device_config_basic(inst, retries=settings["retries"])
        row.update(config)
        errors.update(config_errors)

    row["slave_id"] = settings["slave"]
    row["serial_port"] = settings["port"]
    return row, errors


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()

    try:
        base_settings = load_config(DEFAULT_CONFIG_PATH)
    except FileNotFoundError as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1

    settings = merge_settings(base_settings, args)

    if not (1 <= settings["slave"] <= 254):
        print("ERROR: --slave debe estar entre 1 y 254.", file=sys.stderr)
        return 2

    if settings["interval_seconds"] is None:
        settings["interval_seconds"] = 10.0

    inst: Optional[minimalmodbus.Instrument] = None
    try:
        parity = parse_parity(settings["parity"])
        inst = make_instrument(
            port=settings["port"],
            slave=settings["slave"],
            baudrate=settings["baud"],
            parity=parity,
            bytesize=settings["bytesize"],
            stopbits=settings["stopbits"],
            timeout=settings["timeout"],
            debug=settings["debug"],
        )

        while True:
            cycle_start = time.monotonic()

            row, errors = collect_row(inst, settings)

            if settings["print_json"]:
                print(json.dumps({"data": row, "errors": errors}, indent=2, ensure_ascii=False), flush=True)
            else:
                print_human(row, errors)

            append_csv(str(settings["csv_path"]), row)
            print(f"\nCSV actualizado: {settings['csv_path']}", flush=True)

            if settings["once"]:
                break

            elapsed = time.monotonic() - cycle_start
            sleep_seconds = max(0.0, settings["interval_seconds"] - elapsed)

            if sleep_seconds > 0:
                print(f"Siguiente lectura en {sleep_seconds:.1f} segundos...", flush=True)
                time.sleep(sleep_seconds)
            else:
                print(
                    f"ADVERTENCIA: el ciclo tardo {elapsed:.1f} s, mas que el intervalo configurado de "
                    f"{settings['interval_seconds']:.1f} s. Iniciando siguiente lectura inmediatamente.",
                    flush=True,
                )

    except KeyboardInterrupt:
        print("\nInterrumpido por usuario.")
        return 0
    except serial.SerialException as exc:
        print(f"ERROR serial: {exc}", file=sys.stderr)
        return 1
    except Exception as exc:
        print(f"ERROR general: {exc}", file=sys.stderr)
        return 1
    finally:
        if inst is not None and hasattr(inst, "serial"):
            try:
                inst.serial.close()
            except Exception:
                pass

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
