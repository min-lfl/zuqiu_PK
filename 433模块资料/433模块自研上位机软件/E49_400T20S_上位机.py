# -*- coding: utf-8 -*-
"""
E49-400T20S 专用串口上位机（单文件版）

运行：
    python E49_400T20S_上位机.py

依赖：
    python -m pip install pyserial

离线协议自测：
    python E49_400T20S_上位机.py --self-test

普通 USB-UART 不能控制模块 M0/M1：
    参数读取/修改：M1=1、M0=0，本机串口固定 9600 8N1
    透传数据收发：M1=0、M0=0，本机串口使用模块配置的 UART 参数
"""

from __future__ import annotations

import argparse
import ctypes
import re
import sys
import time
from dataclasses import dataclass, field
from datetime import datetime
from typing import Any, Dict, List, Optional, Sequence, Tuple

import tkinter as tk
from tkinter import messagebox, scrolledtext, ttk

try:
    import serial  # type: ignore
    from serial.tools import list_ports  # type: ignore

    SERIAL_AVAILABLE = hasattr(serial, "Serial")
    SERIAL_IMPORT_ERROR: Optional[BaseException] = None
except BaseException as exc:
    serial = None  # type: ignore
    list_ports = None  # type: ignore
    SERIAL_AVAILABLE = False
    SERIAL_IMPORT_ERROR = exc


APP_NAME = "E49-400T20S 参数与透传工具"
APP_VERSION = "2.1.0"

READ_PARAMETERS_COMMAND = b"\xC1\xC1\xC1"
READ_VERSION_COMMAND = b"\xC3\xC3\xC3"

BG = "#F3F4F6"
PANEL = "#FFFFFF"
TEXT = "#202124"
MUTED = "#61666D"
BLUE = "#1769AA"
BLUE_DARK = "#0D4F84"
GREEN = "#177245"
ORANGE = "#A85B00"
RED = "#B3261E"
NOTICE_BG = "#FFF4CE"
NOTICE_FG = "#7A4B00"
READ_BG = "#F2F8FD"
READ_BORDER = "#6E9FC8"
EDIT_BG = "#F4FAF1"
EDIT_BORDER = "#76A66A"
DATA_BG = "#FFF8EE"
DATA_BORDER = "#C9914C"


PARITY_CHOICES: List[Tuple[str, int, str]] = [
    ("8N1（编码 00，默认）", 0, "N"),
    ("8O1（编码 01）", 1, "O"),
    ("8E1（编码 10）", 2, "E"),
    ("8N1（编码 11）", 3, "N"),
]

UART_CHOICES: List[Tuple[str, int, int]] = [
    ("1200 bps", 0, 1200),
    ("2400 bps", 1, 2400),
    ("4800 bps", 2, 4800),
    ("9600 bps（默认）", 3, 9600),
    ("19200 bps", 4, 19200),
    ("38400 bps", 5, 38400),
    ("57600 bps", 6, 57600),
    ("115200 bps", 7, 115200),
]

AIR_CHOICES: List[Tuple[str, int, int]] = [
    ("1.2 kbps", 0, 1200),
    ("2.4 kbps（默认）", 1, 2400),
    ("4.8 kbps", 2, 4800),
    ("9.6 kbps", 3, 9600),
    ("19.2 kbps", 4, 19200),
    ("50 kbps", 5, 50000),
    ("100 kbps", 6, 100000),
    ("200 kbps", 7, 200000),
]

POWER_CHOICES: List[Tuple[str, int, int]] = [
    ("20 dBm / 100 mW（默认）", 0, 20),
    ("17 dBm", 1, 17),
    ("14 dBm", 2, 14),
    ("10 dBm", 3, 10),
]

TRANSFER_CHOICES: List[Tuple[str, bool]] = [
    ("透明传输（默认）", False),
    ("定点传输", True),
]

SERIAL_BAUD_VALUES = [str(item[2]) for item in UART_CHOICES]
SERIAL_FORMAT_VALUES = ["8N1", "8O1", "8E1"]


def choice_by_code(choices: Sequence[Sequence[Any]], code: int) -> str:
    for item in choices:
        if int(item[1]) == code:
            return str(item[0])
    raise ValueError("未知编码：{}".format(code))


def code_by_choice(choices: Sequence[Sequence[Any]], label: str) -> int:
    for item in choices:
        if str(item[0]) == label:
            return int(item[1])
    raise ValueError("未知选项：{}".format(label))


def bool_by_choice(choices: Sequence[Sequence[Any]], label: str) -> bool:
    for item in choices:
        if str(item[0]) == label:
            return bool(item[1])
    raise ValueError("未知选项：{}".format(label))


def require_int(name: str, value: Any, minimum: int, maximum: int) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise ValueError("{}必须是整数".format(name))
    if not minimum <= value <= maximum:
        raise ValueError("{}必须在 {}~{} 之间".format(name, minimum, maximum))
    return value


def parse_hex_bytes(text: str) -> bytes:
    """严格解析 HEX；支持空格、逗号、换行、0x 前缀和连续字节。"""
    if not text or not text.strip():
        raise ValueError("请输入要发送的 HEX 数据")
    tokens = [part for part in re.split(r"[\s,;:_]+", text.strip()) if part]
    chunks: List[str] = []
    for token in tokens:
        value = token[2:] if token.lower().startswith("0x") else token
        if not value:
            raise ValueError("0x 后面必须有十六进制数字")
        if re.fullmatch(r"[0-9A-Fa-f]+", value) is None:
            raise ValueError("非法 HEX 片段：{}".format(token))
        if len(value) % 2:
            raise ValueError("HEX 片段位数必须为偶数：{}".format(token))
        chunks.append(value)
    return bytes.fromhex("".join(chunks))


def parse_address(text: str) -> int:
    value = text.strip().upper()
    if value.startswith("0X"):
        value = value[2:]
    if not value or len(value) > 4 or re.search(r"[^0-9A-F]", value):
        raise ValueError("地址应为 0000~FFFF 的 1~4 位十六进制数")
    return int(value, 16)


def frequency_to_channel(value: Any) -> int:
    try:
        frequency = float(str(value).strip())
    except ValueError as exc:
        raise ValueError("频率必须是数字，例如 433.0") from exc
    if not 410.0 <= frequency <= 510.0:
        raise ValueError("频率范围必须是 410.0~510.0 MHz")
    doubled = (frequency - 410.0) * 2.0
    rounded = round(doubled)
    if abs(doubled - rounded) > 1e-7:
        raise ValueError("频率必须按 0.5 MHz 步进")
    return int(rounded)


def channel_to_frequency(channel: int) -> float:
    channel = require_int("信道", channel, 0, 0xC8)
    return 410.0 + channel * 0.5


def hex_bytes(data: bytes) -> str:
    return bytes(data).hex(" ").upper()


@dataclass(frozen=True)
class E49Config:
    head: int
    address: int
    parity_code: int
    uart_code: int
    air_code: int
    channel: int
    fixed_mode: bool
    power_code: int
    reserved_bits: int = 0

    @property
    def uart_bps(self) -> int:
        return UART_CHOICES[self.uart_code][2]

    @property
    def air_bps(self) -> int:
        return AIR_CHOICES[self.air_code][2]

    @property
    def uart_format(self) -> str:
        return PARITY_CHOICES[self.parity_code][0].split("（", 1)[0]

    @property
    def frequency_mhz(self) -> float:
        return channel_to_frequency(self.channel)

    @property
    def power_dbm(self) -> int:
        return POWER_CHOICES[self.power_code][2]

    def to_frame(self) -> bytes:
        sped = (
            ((self.parity_code & 0x03) << 6)
            | ((self.uart_code & 0x07) << 3)
            | (self.air_code & 0x07)
        )
        option = (
            (0x80 if self.fixed_mode else 0x00)
            | (self.reserved_bits & 0x7C)
            | (self.power_code & 0x03)
        )
        return bytes(
            [
                self.head,
                (self.address >> 8) & 0xFF,
                self.address & 0xFF,
                sped,
                self.channel,
                option,
            ]
        )


def decode_config_frame(frame: bytes) -> E49Config:
    frame = bytes(frame)
    if len(frame) != 6:
        raise ValueError("配置帧必须正好 6 字节")
    head, addh, addl, sped, channel, option = frame
    if head not in (0xC0, 0xC2):
        raise ValueError("配置帧 HEAD 应为 C0 或 C2")
    if channel > 0xC8:
        raise ValueError("信道 0x{:02X} 超出 0x00~0xC8".format(channel))
    return E49Config(
        head=head,
        address=(addh << 8) | addl,
        parity_code=(sped >> 6) & 0x03,
        uart_code=(sped >> 3) & 0x07,
        air_code=sped & 0x07,
        channel=channel,
        fixed_mode=bool(option & 0x80),
        power_code=option & 0x03,
        reserved_bits=option & 0x7C,
    )


def build_config_frame(
    persistent: bool,
    address: int,
    parity_code: int,
    uart_code: int,
    air_code: int,
    channel: int,
    fixed_mode: bool,
    power_code: int,
) -> bytes:
    if not isinstance(persistent, bool):
        raise ValueError("持久化标志必须是 bool")
    if not isinstance(fixed_mode, bool):
        raise ValueError("传输方式标志必须是 bool")
    config = E49Config(
        head=0xC0 if persistent else 0xC2,
        address=require_int("地址", address, 0, 0xFFFF),
        parity_code=require_int("串口校验编码", parity_code, 0, 3),
        uart_code=require_int("UART 速率编码", uart_code, 0, 7),
        air_code=require_int("空中速率编码", air_code, 0, 7),
        channel=require_int("信道", channel, 0, 0xC8),
        fixed_mode=fixed_mode,
        power_code=require_int("功率编码", power_code, 0, 3),
    )
    return config.to_frame()


def decode_version_frame(frame: bytes) -> Tuple[int, int, int]:
    frame = bytes(frame)
    if len(frame) != 4 or frame[:2] != b"\xC3\x49":
        raise ValueError("版本返回应为 C3 49 xx yy")
    return frame[1], frame[2], frame[3]


@dataclass
class PendingRequest:
    kind: str
    signatures: Tuple[bytes, ...]
    expected_len: int
    deadline: float
    buffer: bytearray = field(default_factory=bytearray)
    discarded_count: int = 0
    invalid_candidates: int = 0


class E49ToolApp:
    POLL_INTERVAL_MS = 25
    RESPONSE_TIMEOUT_SECONDS = 1.5
    RX_FRAME_GAP_SECONDS = 0.030
    MAX_READ_CHUNK = 4096
    MAX_LOG_CHARS = 1_000_000

    def __init__(self, root: tk.Tk) -> None:
        self.root = root
        self.ser: Any = None
        self.pending: Optional[PendingRequest] = None
        self.last_read_config: Optional[E49Config] = None
        self.closing = False
        self.poll_job: Optional[str] = None
        self.port_display_to_device: Dict[str, str] = {}
        self.log_sizes: Dict[str, int] = {}
        self.tx_total = 0
        self.rx_total = 0
        self.rx_frame_buffer = bytearray()
        self.rx_frame_started_at: Optional[datetime] = None
        self.rx_last_byte_at: Optional[float] = None

        self._configure_window()
        self._configure_style()
        self._create_variables()
        self._build_ui()
        self._attach_traces()
        self.refresh_ports()
        self.update_edit_preview()
        self._update_connection_ui()

        self.root.protocol("WM_DELETE_WINDOW", self.on_close)
        self.root.bind("<Configure>", self._resize_notice, add="+")
        self.poll_job = self.root.after(self.POLL_INTERVAL_MS, self.poll_serial)
        if not SERIAL_AVAILABLE:
            self.root.after(250, self.show_pyserial_help)

    def _configure_window(self) -> None:
        self.root.title("{}  v{}".format(APP_NAME, APP_VERSION))
        screen_w = self.root.winfo_screenwidth()
        screen_h = self.root.winfo_screenheight()
        width = min(1380, max(1080, screen_w - 60))
        height = min(850, max(680, screen_h - 80))
        x = max(0, (screen_w - width) // 2)
        y = max(0, (screen_h - height) // 2)
        self.root.geometry("{}x{}+{}+{}".format(width, height, x, y))
        self.root.minsize(1040, 650)
        self.root.configure(bg=BG)
        if sys.platform == "win32":
            try:
                self.root.state("zoomed")
            except tk.TclError:
                pass

    def _configure_style(self) -> None:
        style = ttk.Style(self.root)
        if "vista" in style.theme_names():
            style.theme_use("vista")
        elif "clam" in style.theme_names():
            style.theme_use("clam")
        default_font = ("Microsoft YaHei UI", 9)
        self.root.option_add("*Font", default_font)
        style.configure(".", font=default_font)
        style.configure("TFrame", background=BG)
        style.configure("Panel.TFrame", background=PANEL)
        style.configure("TLabel", background=BG, foreground=TEXT)
        style.configure("Panel.TLabel", background=PANEL, foreground=TEXT)
        style.configure("Muted.Panel.TLabel", background=PANEL, foreground=MUTED)
        style.configure("TLabelFrame", background=PANEL, padding=8)
        style.configure(
            "TLabelFrame.Label",
            background=BG,
            foreground=TEXT,
            font=("Microsoft YaHei UI", 10, "bold"),
        )
        style.configure("Primary.TButton", padding=(13, 7))
        style.configure("Small.TButton", padding=(8, 4))

    def _create_variables(self) -> None:
        self.port_var = tk.StringVar()
        self.host_baud_var = tk.StringVar(value="9600")
        self.host_format_var = tk.StringVar(value="8N1")
        self.connection_var = tk.StringVar(value="未连接")
        self.status_var = tk.StringVar(value="准备就绪")

        self.clear_rx_before_read_var = tk.BooleanVar(value=False)
        self.read_frame_var = tk.StringVar()
        self.read_status_var = tk.StringVar(value="读取结果为空")
        self.read_version_var = tk.StringVar()
        self.read_vars: Dict[str, tk.StringVar] = {
            name: tk.StringVar() for name in (
                "address",
                "parity",
                "uart",
                "air",
                "channel",
                "frequency",
                "transfer",
                "power",
            )
        }

        self.edit_address_var = tk.StringVar(value="0000")
        self.edit_parity_var = tk.StringVar(value=PARITY_CHOICES[0][0])
        self.edit_uart_var = tk.StringVar(value=UART_CHOICES[3][0])
        self.edit_air_var = tk.StringVar(value=AIR_CHOICES[1][0])
        self.edit_frequency_var = tk.StringVar(value="433.0")
        self.edit_transfer_var = tk.StringVar(value=TRANSFER_CHOICES[0][0])
        self.edit_power_var = tk.StringVar(value=POWER_CHOICES[0][0])
        self.edit_channel_var = tk.StringVar(value="CHAN 0x2E（46）")
        self.edit_preview_var = tk.StringVar()
        self.write_status_var = tk.StringVar(value="修改区与读取区互不影响")
        self.temporary_write_var = tk.BooleanVar(value=False)
        self.write_button_text_var = tk.StringVar(value="写入参数（C0，掉电保存）")

        self.send_format_var = tk.StringVar(value="HEX 原始字节")
        self.append_crlf_var = tk.BooleanVar(value=False)
        self.rx_timestamp_var = tk.BooleanVar(value=True)
        self.tx_count_var = tk.StringVar(value="0 B")
        self.rx_count_var = tk.StringVar(value="0 B")

    def _build_ui(self) -> None:
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(1, weight=1)
        self._build_serial_panel()

        main = ttk.Panedwindow(self.root, orient="horizontal")
        main.grid(row=1, column=0, sticky="nsew")
        self.main_paned = main

        panel_font = ("Microsoft YaHei UI", 10, "bold")
        self.read_panel = tk.LabelFrame(
            main,
            text="读取模块参数（只读区）",
            bg=READ_BG,
            fg="#225F91",
            font=panel_font,
            bd=2,
            relief="groove",
            highlightthickness=1,
            highlightbackground=READ_BORDER,
            padx=8,
            pady=8,
        )
        self.edit_panel = tk.LabelFrame(
            main,
            text="修改模块参数（编辑区）",
            bg=EDIT_BG,
            fg="#336B2B",
            font=panel_font,
            bd=2,
            relief="groove",
            highlightthickness=1,
            highlightbackground=EDIT_BORDER,
            padx=8,
            pady=8,
        )
        self.data_panel = tk.LabelFrame(
            main,
            text="透传与底层原始数据",
            bg=DATA_BG,
            fg="#865313",
            font=panel_font,
            bd=2,
            relief="groove",
            highlightthickness=1,
            highlightbackground=DATA_BORDER,
            padx=8,
            pady=8,
        )
        main.add(self.read_panel, weight=30)
        main.add(self.edit_panel, weight=36)
        main.add(self.data_panel, weight=34)

        self._build_read_panel()
        self._build_edit_panel()
        self._build_data_panel()
        self._build_status_bar()
        # 最大化完成后再设置初始分栏；用户仍可用鼠标拖动两条分隔线。
        self.root.after(120, self._set_initial_sashes)

    def _set_initial_sashes(self) -> None:
        try:
            width = self.main_paned.winfo_width()
            if width >= 900:
                self.main_paned.sashpos(0, int(width * 0.30))
                self.main_paned.sashpos(1, int(width * 0.66))
        except tk.TclError:
            pass

    def _build_serial_panel(self) -> None:
        panel = ttk.LabelFrame(self.root, text="串口设置", padding=(10, 7))
        panel.grid(row=0, column=0, sticky="ew", padx=10, pady=(8, 7))
        panel.columnconfigure(9, weight=1)

        ttk.Label(panel, text="端口").grid(row=0, column=0, padx=(0, 5), sticky="w")
        self.port_combo = ttk.Combobox(panel, textvariable=self.port_var, width=25)
        self.port_combo.grid(row=0, column=1, padx=(0, 5), sticky="ew")
        ttk.Button(panel, text="刷新", command=self.refresh_ports, style="Small.TButton").grid(
            row=0, column=2, padx=(0, 14)
        )

        ttk.Label(panel, text="本机波特率").grid(row=0, column=3, padx=(0, 5))
        self.host_baud_combo = ttk.Combobox(
            panel,
            textvariable=self.host_baud_var,
            values=SERIAL_BAUD_VALUES,
            width=10,
            state="readonly",
        )
        self.host_baud_combo.grid(row=0, column=4, padx=(0, 10))
        ttk.Label(panel, text="格式").grid(row=0, column=5, padx=(0, 5))
        self.host_format_combo = ttk.Combobox(
            panel,
            textvariable=self.host_format_var,
            values=SERIAL_FORMAT_VALUES,
            width=7,
            state="readonly",
        )
        self.host_format_combo.grid(row=0, column=6, padx=(0, 8))
        self.apply_serial_button = ttk.Button(
            panel,
            text="应用本机串口参数",
            command=self.apply_serial_settings,
            style="Small.TButton",
        )
        self.apply_serial_button.grid(row=0, column=7, padx=(0, 8))
        self.connect_button = ttk.Button(
            panel,
            text="打开串口",
            command=self.toggle_connection,
            style="Primary.TButton",
        )
        self.connect_button.grid(row=0, column=8, padx=(0, 12))
        self.connection_label = tk.Label(
            panel,
            textvariable=self.connection_var,
            bg="#ECEFF1",
            fg=MUTED,
            padx=10,
            pady=5,
            anchor="center",
        )
        self.connection_label.grid(row=0, column=9, sticky="e")

        self.mode_notice_label = tk.Label(
            panel,
            text=(
                "硬件模式：读取/修改参数时 M1=1、M0=0（软件自动使用 9600 8N1）；"
                "透传收发时 M1=0、M0=0。普通 USB-UART 不能替你切换 M0/M1。"
            ),
            bg=NOTICE_BG,
            fg=NOTICE_FG,
            padx=9,
            pady=6,
            anchor="w",
            justify="left",
        )
        self.mode_notice_label.grid(row=1, column=0, columnspan=10, sticky="ew", pady=(7, 0))

    def _build_read_panel(self) -> None:
        panel = self.read_panel
        panel.columnconfigure(1, weight=1)

        button_row = ttk.Frame(panel, style="Panel.TFrame")
        button_row.grid(row=0, column=0, columnspan=2, sticky="ew", pady=(0, 5))
        self.read_button = ttk.Button(
            button_row,
            text="读取模块参数",
            command=self.read_parameters,
            style="Primary.TButton",
        )
        self.read_button.pack(side="left", fill="x", expand=True)
        self.clear_read_button = ttk.Button(
            button_row,
            text="清空读取结果",
            command=self.clear_read_result,
            style="Small.TButton",
        )
        self.clear_read_button.pack(side="left", padx=(7, 0))

        ttk.Checkbutton(
            panel,
            text="读取前清空右侧 RX（默认不清空）",
            variable=self.clear_rx_before_read_var,
        ).grid(row=1, column=0, columnspan=2, sticky="w", pady=(0, 7))

        ttk.Separator(panel).grid(row=2, column=0, columnspan=2, sticky="ew", pady=(0, 7))
        self._read_row(panel, 3, "返回原始帧", self.read_frame_var)
        self._read_row(panel, 4, "模块地址", self.read_vars["address"])
        self._read_row(panel, 5, "串口校验", self.read_vars["parity"])
        self._read_row(panel, 6, "UART 波特率", self.read_vars["uart"])
        self._read_row(panel, 7, "空中速率", self.read_vars["air"])
        self._read_row(panel, 8, "信道", self.read_vars["channel"])
        self._read_row(panel, 9, "中心频率", self.read_vars["frequency"])
        self._read_row(panel, 10, "传输方式", self.read_vars["transfer"])
        self._read_row(panel, 11, "发射功率", self.read_vars["power"])

        ttk.Separator(panel).grid(row=12, column=0, columnspan=2, sticky="ew", pady=7)
        version_row = ttk.Frame(panel, style="Panel.TFrame")
        version_row.grid(row=13, column=0, columnspan=2, sticky="ew")
        version_row.columnconfigure(1, weight=1)
        self.version_button = ttk.Button(
            version_row,
            text="读取版本",
            command=self.read_version,
            style="Small.TButton",
        )
        self.version_button.grid(row=0, column=0, padx=(0, 7))
        ttk.Entry(version_row, textvariable=self.read_version_var, state="readonly").grid(
            row=0, column=1, sticky="ew"
        )

        self.read_status_label = tk.Label(
            panel,
            textvariable=self.read_status_var,
            bg="#F7F8FA",
            fg=MUTED,
            anchor="w",
            justify="left",
            padx=7,
            pady=7,
        )
        self.read_status_label.grid(row=14, column=0, columnspan=2, sticky="ew", pady=(9, 7))

        self.copy_to_edit_button = ttk.Button(
            panel,
            text="把本次读取值复制到修改区 →",
            command=self.copy_read_to_edit,
            state="disabled",
        )
        self.copy_to_edit_button.grid(row=15, column=0, columnspan=2, sticky="ew")

        panel.rowconfigure(16, weight=1)
        mode_help = tk.LabelFrame(
            panel,
            text="M1 / M0 模式与接线速查",
            bg="#E8F3FC",
            fg="#225F91",
            font=("Microsoft YaHei UI", 9, "bold"),
            bd=1,
            relief="solid",
            padx=8,
            pady=6,
        )
        mode_help.grid(row=16, column=0, columnspan=2, sticky="nsew", pady=(10, 0))
        self.mode_help_label = tk.Label(
            mode_help,
            text=(
                "M1=0，M0=0  传输模式：串口和无线都打开，正常收发数据。\n"
                "M1=0，M0=1  RSSI 模式：无线收发关闭，每 100 ms 输出一次强度值。\n"
                "M1=1，M0=0  设置模式：无线关闭，用固定 9600 8N1 读写参数。\n"
                "M1=1，M0=1  休眠模式：串口和无线均关闭，超低功耗。\n\n"
                "切换建议：先等 AUX=高，改变 M1/M0 后再等 AUX=高（或至少 2 ms）。\n"
                "接线始终不变：USB-TTL TX → 模块 RXD，USB-TTL RX ← 模块 TXD，GND 共地。"
            ),
            bg="#E8F3FC",
            fg=TEXT,
            justify="left",
            anchor="nw",
            wraplength=520,
        )
        self.mode_help_label.pack(fill="both", expand=True)
        panel.bind(
            "<Configure>",
            lambda event: self.mode_help_label.configure(
                wraplength=max(230, event.width - 55)
            ),
            add="+",
        )

    def _read_row(self, parent: tk.LabelFrame, row: int, title: str, variable: tk.StringVar) -> None:
        ttk.Label(parent, text=title).grid(row=row, column=0, sticky="w", padx=(0, 8), pady=4)
        ttk.Entry(parent, textvariable=variable, state="readonly").grid(
            row=row, column=1, sticky="ew", pady=4
        )

    def _build_edit_panel(self) -> None:
        panel = self.edit_panel
        panel.columnconfigure(1, weight=1)
        ttk.Label(
            panel,
            text="这里是准备写入的值；读取参数不会自动覆盖这里。",
            style="Muted.Panel.TLabel",
        ).grid(row=0, column=0, columnspan=2, sticky="w", pady=(0, 7))

        ttk.Label(panel, text="模块地址").grid(row=1, column=0, sticky="w", padx=(0, 8), pady=4)
        ttk.Entry(panel, textvariable=self.edit_address_var).grid(row=1, column=1, sticky="ew", pady=4)

        self._edit_combo_row(panel, 2, "串口校验", self.edit_parity_var, [item[0] for item in PARITY_CHOICES])
        self._edit_combo_row(panel, 3, "UART 波特率", self.edit_uart_var, [item[0] for item in UART_CHOICES])
        self._edit_combo_row(panel, 4, "空中速率", self.edit_air_var, [item[0] for item in AIR_CHOICES])

        ttk.Label(panel, text="中心频率 MHz").grid(row=5, column=0, sticky="w", padx=(0, 8), pady=4)
        frequency_row = ttk.Frame(panel, style="Panel.TFrame")
        frequency_row.grid(row=5, column=1, sticky="ew", pady=4)
        frequency_row.columnconfigure(0, weight=1)
        self.frequency_spinbox = ttk.Spinbox(
            frequency_row,
            from_=410.0,
            to=510.0,
            increment=0.5,
            textvariable=self.edit_frequency_var,
            width=10,
        )
        self.frequency_spinbox.grid(row=0, column=0, sticky="ew")
        ttk.Label(
            frequency_row,
            textvariable=self.edit_channel_var,
            style="Muted.Panel.TLabel",
        ).grid(row=0, column=1, padx=(8, 0))

        self._edit_combo_row(
            panel,
            6,
            "传输方式",
            self.edit_transfer_var,
            [item[0] for item in TRANSFER_CHOICES],
        )
        self._edit_combo_row(panel, 7, "发射功率", self.edit_power_var, [item[0] for item in POWER_CHOICES])

        ttk.Separator(panel).grid(row=8, column=0, columnspan=2, sticky="ew", pady=8)
        ttk.Label(panel, text="将发送的配置帧", font=("Microsoft YaHei UI", 9, "bold")).grid(
            row=9, column=0, columnspan=2, sticky="w"
        )
        self.preview_label = tk.Label(
            panel,
            textvariable=self.edit_preview_var,
            bg="#F2F6FA",
            fg=BLUE_DARK,
            font=("Consolas", 10, "bold"),
            anchor="w",
            justify="left",
            padx=8,
            pady=7,
        )
        self.preview_label.grid(row=10, column=0, columnspan=2, sticky="ew", pady=(4, 8))

        write_row = ttk.Frame(panel, style="Panel.TFrame")
        write_row.grid(row=11, column=0, columnspan=2, sticky="ew")
        write_row.columnconfigure(0, weight=1)
        self.write_button = ttk.Button(
            write_row,
            textvariable=self.write_button_text_var,
            command=self.write_parameters,
            style="Primary.TButton",
        )
        self.write_button.grid(row=0, column=0, sticky="ew", padx=(0, 10))
        self.temporary_write_checkbox = ttk.Checkbutton(
            write_row,
            text="临时设置（C2，不保存）",
            variable=self.temporary_write_var,
        )
        self.temporary_write_checkbox.grid(row=0, column=1, sticky="e")

        utility_row = ttk.Frame(panel, style="Panel.TFrame")
        utility_row.grid(row=12, column=0, columnspan=2, sticky="ew", pady=(8, 0))
        utility_row.columnconfigure(0, weight=1)
        utility_row.columnconfigure(1, weight=1)
        ttk.Button(utility_row, text="载入默认值（不发送）", command=self.load_defaults).grid(
            row=0, column=0, sticky="ew", padx=(0, 4)
        )
        self.business_uart_button = ttk.Button(
            utility_row,
            text="本机切到所选业务串口",
            command=self.apply_business_uart,
        )
        self.business_uart_button.grid(row=0, column=1, sticky="ew", padx=(4, 0))

        self.write_status_label = tk.Label(
            panel,
            textvariable=self.write_status_var,
            bg="#F7F8FA",
            fg=MUTED,
            anchor="w",
            justify="left",
            padx=7,
            pady=7,
        )
        self.write_status_label.grid(row=13, column=0, columnspan=2, sticky="ew", pady=(9, 0))

        panel.rowconfigure(14, weight=1)
        parameter_help = tk.LabelFrame(
            panel,
            text="参数作用简述",
            bg="#EAF5E6",
            fg="#336B2B",
            font=("Microsoft YaHei UI", 9, "bold"),
            bd=1,
            relief="solid",
            padx=8,
            pady=6,
        )
        parameter_help.grid(row=14, column=0, columnspan=2, sticky="nsew", pady=(10, 0))
        self.parameter_help_label = tk.Label(
            parameter_help,
            text=(
                "模块地址：16 位本机地址，用于寻址、定点及广播/监听；FFFF 有广播/监听含义。\n"
                "串口校验：模块与电脑/单片机的有线串口格式，双方必须一致。\n"
                "UART 波特率：只影响模块与电脑/单片机之间；无线双方可以不同。\n"
                "空中速率：无线双方必须一致；越低通常距离越远、抗干扰更强，但更慢。\n"
                "中心频率：无线双方必须一致；频率 = 410 + CHAN × 0.5 MHz。\n"
                "传输方式：透明模式原样收发；定点模式把每帧前 3 字节作为目标信息。\n"
                "发射功率：越高通常距离越远、耗电越大；20 dBm 时电源应能提供 100 mA 以上。\n"
                "写入方式：默认 C0，掉电保存；勾选“临时设置”后使用 C2，断电即丢失。"
            ),
            bg="#EAF5E6",
            fg=TEXT,
            justify="left",
            anchor="nw",
            wraplength=650,
        )
        self.parameter_help_label.pack(fill="both", expand=True)
        panel.bind(
            "<Configure>",
            lambda event: self.parameter_help_label.configure(
                wraplength=max(280, event.width - 55)
            ),
            add="+",
        )

    def _edit_combo_row(
        self,
        parent: tk.LabelFrame,
        row: int,
        title: str,
        variable: tk.StringVar,
        values: Sequence[str],
    ) -> None:
        ttk.Label(parent, text=title).grid(row=row, column=0, sticky="w", padx=(0, 8), pady=4)
        ttk.Combobox(parent, textvariable=variable, values=list(values), state="readonly").grid(
            row=row, column=1, sticky="ew", pady=4
        )

    def _build_data_panel(self) -> None:
        panel = self.data_panel
        panel.columnconfigure(0, weight=1)
        panel.rowconfigure(2, weight=1)
        panel.rowconfigure(3, weight=2)

        send_box = ttk.LabelFrame(panel, text="发送透传报文 / 手动命令", padding=7)
        send_box.grid(row=0, column=0, sticky="ew")
        send_box.columnconfigure(0, weight=1)

        send_options = ttk.Frame(send_box, style="Panel.TFrame")
        send_options.grid(row=0, column=0, sticky="ew", pady=(0, 5))
        ttk.Label(send_options, text="输入格式", style="Panel.TLabel").pack(side="left")
        self.send_format_combo = ttk.Combobox(
            send_options,
            textvariable=self.send_format_var,
            values=["HEX 原始字节", "UTF-8 文本"],
            state="readonly",
            width=14,
        )
        self.send_format_combo.pack(side="left", padx=(6, 10))
        self.crlf_checkbox = ttk.Checkbutton(
            send_options,
            text="文本末尾加 CRLF",
            variable=self.append_crlf_var,
            state="disabled",
        )
        self.crlf_checkbox.pack(side="left")

        self.send_text = tk.Text(
            send_box,
            height=3,
            wrap="word",
            font=("Consolas", 10),
            undo=True,
            relief="solid",
            borderwidth=1,
        )
        self.send_text.grid(row=1, column=0, sticky="ew")
        self.send_text.insert("1.0", "01 02 03 04")
        send_actions = ttk.Frame(send_box, style="Panel.TFrame")
        send_actions.grid(row=2, column=0, sticky="ew", pady=(7, 0))
        send_actions.columnconfigure(1, weight=1)
        ttk.Button(
            send_actions,
            text="清空输入",
            command=self.clear_send_input,
            style="Small.TButton",
        ).grid(row=0, column=0, padx=(0, 7))
        self.send_button = ttk.Button(
            send_actions,
            text="发送",
            command=self.send_user_data,
            style="Primary.TButton",
        )
        self.send_button.grid(row=0, column=1, sticky="ew")

        pure_notice = ttk.Label(
            panel,
            text="持续追加实际 HEX 字节；RX 按连续字节后空闲 30 ms 作为一帧。",
            style="Muted.Panel.TLabel",
        )
        pure_notice.grid(row=1, column=0, sticky="w", pady=(7, 4))

        tx_box = ttk.LabelFrame(panel, text="底层发送数据（TX）", padding=6)
        tx_box.grid(row=2, column=0, sticky="nsew", pady=(0, 5))
        tx_box.columnconfigure(0, weight=1)
        tx_box.rowconfigure(1, weight=1)
        tx_head = ttk.Frame(tx_box, style="Panel.TFrame")
        tx_head.grid(row=0, column=0, sticky="ew", pady=(0, 4))
        ttk.Label(tx_head, textvariable=self.tx_count_var, style="Muted.Panel.TLabel").pack(side="left")
        ttk.Button(tx_head, text="清空 TX", command=self.clear_tx_log, style="Small.TButton").pack(side="right")
        self.tx_log = scrolledtext.ScrolledText(
            tx_box,
            height=3,
            wrap="word",
            state="disabled",
            font=("Consolas", 10),
            background="#FBFCFD",
        )
        self.tx_log.grid(row=1, column=0, sticky="nsew")

        rx_box = ttk.LabelFrame(panel, text="底层接收数据（RX）", padding=6)
        rx_box.grid(row=3, column=0, sticky="nsew")
        rx_box.columnconfigure(0, weight=1)
        rx_box.rowconfigure(1, weight=1)
        rx_head = ttk.Frame(rx_box, style="Panel.TFrame")
        rx_head.grid(row=0, column=0, sticky="ew", pady=(0, 4))
        ttk.Label(rx_head, textvariable=self.rx_count_var, style="Muted.Panel.TLabel").pack(side="left")
        ttk.Checkbutton(
            rx_head,
            text="显示帧时间戳",
            variable=self.rx_timestamp_var,
        ).pack(side="left", padx=(12, 4))
        ttk.Button(rx_head, text="清空 RX", command=self.clear_rx_log, style="Small.TButton").pack(side="right")
        self.rx_log = scrolledtext.ScrolledText(
            rx_box,
            height=6,
            wrap="word",
            state="disabled",
            font=("Consolas", 10),
            background="#FBFCFD",
        )
        self.rx_log.grid(row=1, column=0, sticky="nsew")

    def _build_status_bar(self) -> None:
        bar = tk.Frame(self.root, bg="#E8EAED", height=28)
        bar.grid(row=2, column=0, sticky="ew")
        bar.grid_propagate(False)
        self.status_label = tk.Label(
            bar,
            textvariable=self.status_var,
            bg="#E8EAED",
            fg=TEXT,
            anchor="w",
            padx=12,
        )
        self.status_label.pack(fill="both", expand=True)

    def _attach_traces(self) -> None:
        for variable in (
            self.edit_address_var,
            self.edit_parity_var,
            self.edit_uart_var,
            self.edit_air_var,
            self.edit_frequency_var,
            self.edit_transfer_var,
            self.edit_power_var,
            self.temporary_write_var,
        ):
            variable.trace_add("write", lambda *_args: self.update_edit_preview())
        self.send_format_var.trace_add("write", lambda *_args: self._update_send_format())

    def _resize_notice(self, event: tk.Event) -> None:
        if event.widget is self.root and hasattr(self, "mode_notice_label"):
            self.mode_notice_label.configure(wraplength=max(500, event.width - 45))

    def _update_send_format(self) -> None:
        if self.send_format_var.get() == "UTF-8 文本":
            self.crlf_checkbox.configure(state="normal")
        else:
            self.append_crlf_var.set(False)
            self.crlf_checkbox.configure(state="disabled")

    def set_status(self, message: str, level: str = "info") -> None:
        colors = {
            "info": TEXT,
            "success": GREEN,
            "warning": ORANGE,
            "error": RED,
        }
        self.status_var.set(message)
        self.status_label.configure(fg=colors.get(level, TEXT))

    def show_pyserial_help(self) -> None:
        detail = "\n\n导入错误：{}".format(SERIAL_IMPORT_ERROR) if SERIAL_IMPORT_ERROR else ""
        messagebox.showerror(
            "缺少串口组件",
            "当前 Python 没有可用的 pyserial。请执行：\n\n{} -m pip install pyserial{}".format(
                sys.executable,
                detail,
            ),
            parent=self.root,
        )

    def refresh_ports(self) -> None:
        previous_device = self._selected_device(silent=True)
        self.port_display_to_device.clear()
        values: List[str] = []
        if SERIAL_AVAILABLE and list_ports is not None:
            try:
                ports = sorted(list_ports.comports(), key=lambda item: item.device)
                for port in ports:
                    description = port.description or "串口设备"
                    display = "{} — {}".format(port.device, description)
                    self.port_display_to_device[display] = port.device
                    values.append(display)
            except Exception as exc:
                self.set_status("刷新串口失败：{}".format(exc), "error")
        self.port_combo.configure(values=values)
        chosen = ""
        for display, device in self.port_display_to_device.items():
            if device == previous_device:
                chosen = display
                break
        if not chosen and values:
            chosen = values[0]
        if chosen:
            self.port_var.set(chosen)
            self.set_status("发现 {} 个串口".format(len(values)), "info")
        elif not self.port_var.get().strip():
            self.port_var.set("未发现串口" if SERIAL_AVAILABLE else "未安装 pyserial")

    def _selected_device(self, silent: bool = False) -> str:
        value = self.port_var.get().strip()
        if value in self.port_display_to_device:
            return self.port_display_to_device[value]
        if " — " in value:
            return value.split(" — ", 1)[0].strip()
        if value and value not in ("未发现串口", "未安装 pyserial"):
            return value
        if silent:
            return ""
        raise ValueError("请选择或输入串口，例如 COM3")

    def _serial_parity(self, value: str) -> Any:
        mapping = {
            "8N1": serial.PARITY_NONE if SERIAL_AVAILABLE else "N",
            "8O1": serial.PARITY_ODD if SERIAL_AVAILABLE else "O",
            "8E1": serial.PARITY_EVEN if SERIAL_AVAILABLE else "E",
        }
        if value not in mapping:
            raise ValueError("不支持的串口格式：{}".format(value))
        return mapping[value]

    def toggle_connection(self) -> None:
        if self._is_connected():
            self.disconnect("串口已断开")
        else:
            self.connect()

    def connect(self) -> None:
        if not SERIAL_AVAILABLE:
            self.show_pyserial_help()
            return
        try:
            device = self._selected_device()
            baud = int(self.host_baud_var.get())
            parity = self._serial_parity(self.host_format_var.get())
            self.ser = serial.Serial(
                port=device,
                baudrate=baud,
                bytesize=serial.EIGHTBITS,
                parity=parity,
                stopbits=serial.STOPBITS_ONE,
                timeout=0,
                write_timeout=0.3,
            )
            self._update_connection_ui()
            self.set_status(
                "已打开 {}，本机串口 {} {}".format(device, baud, self.host_format_var.get()),
                "success",
            )
        except Exception as exc:
            self.ser = None
            self._update_connection_ui()
            messagebox.showerror("打开串口失败", str(exc), parent=self.root)
            self.set_status("打开串口失败：{}".format(exc), "error")

    def disconnect(self, reason: str) -> None:
        pending_kind = self.pending.kind if self.pending is not None else ""
        self.pending = None
        self._flush_rx_frame_if_idle(force=True)
        if self.ser is not None:
            try:
                self.ser.close()
            except Exception:
                pass
        self.ser = None
        if pending_kind == "config":
            self.read_status_var.set("读取已取消：{}".format(reason))
        elif pending_kind == "version":
            self.read_version_var.set("读取已取消")
        self._update_connection_ui()
        self.set_status(reason, "warning")

    def _is_connected(self) -> bool:
        return self.ser is not None and bool(getattr(self.ser, "is_open", False))

    def _update_connection_ui(self) -> None:
        connected = self._is_connected()
        if connected:
            self.connection_var.set("已连接 {}".format(getattr(self.ser, "port", "")))
            self.connection_label.configure(bg="#E5F3EA", fg=GREEN)
            self.connect_button.configure(text="关闭串口")
            self.port_combo.configure(state="disabled")
        else:
            self.connection_var.set("未连接")
            self.connection_label.configure(bg="#ECEFF1", fg=MUTED)
            self.connect_button.configure(text="打开串口")
            self.port_combo.configure(state="normal")
        self._update_control_states()

    def _update_control_states(self) -> None:
        if not hasattr(self, "read_button"):
            return
        connected = self._is_connected()
        idle = self.pending is None
        command_state = "normal" if connected and idle else "disabled"
        for widget in (
            self.read_button,
            self.version_button,
            self.write_button,
            self.send_button,
        ):
            widget.configure(state=command_state)
        self.temporary_write_checkbox.configure(state="normal" if idle else "disabled")
        self.apply_serial_button.configure(state="normal" if idle else "disabled")
        self.business_uart_button.configure(state="normal" if idle else "disabled")
        self.host_baud_combo.configure(state="readonly" if idle else "disabled")
        self.host_format_combo.configure(state="readonly" if idle else "disabled")
        copy_state = "normal" if self.last_read_config is not None and idle else "disabled"
        self.copy_to_edit_button.configure(state=copy_state)

    def _require_connection(self) -> bool:
        if self._is_connected():
            return True
        if not SERIAL_AVAILABLE:
            self.show_pyserial_help()
        else:
            messagebox.showwarning("串口未打开", "请先在顶部选择端口并点击“打开串口”。", parent=self.root)
        self.set_status("请先打开串口", "error")
        return False

    def apply_serial_settings(self, quiet: bool = False) -> bool:
        if self.pending is not None:
            self.set_status("正在等待模块返回，暂不能更改串口参数", "warning")
            return False
        try:
            baud = int(self.host_baud_var.get())
            parity = self._serial_parity(self.host_format_var.get())
            if self._is_connected():
                self.ser.baudrate = baud
                self.ser.bytesize = serial.EIGHTBITS
                self.ser.parity = parity
                self.ser.stopbits = serial.STOPBITS_ONE
                if not quiet:
                    self.set_status(
                        "本机串口已切换为 {} {}".format(baud, self.host_format_var.get()),
                        "success",
                    )
            elif not quiet:
                self.set_status(
                    "已预选 {} {}，打开串口时生效".format(baud, self.host_format_var.get()),
                    "info",
                )
            return True
        except Exception as exc:
            if not quiet:
                messagebox.showerror("应用串口参数失败", str(exc), parent=self.root)
            self.set_status("应用串口参数失败：{}".format(exc), "error")
            return False

    def _use_config_uart(self) -> bool:
        self.host_baud_var.set("9600")
        self.host_format_var.set("8N1")
        return self.apply_serial_settings(quiet=True)

    def apply_business_uart(self) -> None:
        if self.pending is not None:
            self.set_status("正在等待模块返回，暂不能更改串口参数", "warning")
            return
        try:
            parity_code = code_by_choice(PARITY_CHOICES, self.edit_parity_var.get())
            uart_code = code_by_choice(UART_CHOICES, self.edit_uart_var.get())
            baud = UART_CHOICES[uart_code][2]
            serial_format = PARITY_CHOICES[parity_code][0].split("（", 1)[0]
            self.host_baud_var.set(str(baud))
            self.host_format_var.set(serial_format)
            if self.apply_serial_settings(quiet=True):
                if self._is_connected():
                    self.set_status(
                        "本机已切到业务串口 {} {}；模块应切到 M1=0、M0=0".format(
                            baud, serial_format
                        ),
                        "success",
                    )
                else:
                    self.set_status(
                        "已预选业务串口 {} {}，打开串口时生效".format(baud, serial_format),
                        "info",
                    )
        except ValueError as exc:
            messagebox.showerror("参数有误", str(exc), parent=self.root)

    def poll_serial(self) -> None:
        if self.closing:
            return
        try:
            if self._is_connected():
                waiting = int(self.ser.in_waiting)
                if waiting > 0:
                    data = bytes(self.ser.read(min(waiting, self.MAX_READ_CHUNK)))
                    if data:
                        self._record_raw("RX", data)
                        self._feed_pending(data)
            self._flush_rx_frame_if_idle()
            self._check_pending_timeout()
        except Exception as exc:
            self.disconnect("串口异常断开：{}".format(exc))
        finally:
            if not self.closing:
                self.poll_job = self.root.after(self.POLL_INTERVAL_MS, self.poll_serial)

    def send_raw(self, data: bytes, note: str) -> bool:
        if not self._require_connection():
            return False
        if not data:
            messagebox.showwarning("没有数据", "发送数据不能为空。", parent=self.root)
            return False
        try:
            written = int(self.ser.write(data))
            if written:
                self._record_raw("TX", bytes(data[:written]))
            if written != len(data):
                raise IOError("串口只写入 {} / {} 字节".format(written, len(data)))
            self.set_status("已发送 {} 字节：{}".format(len(data), note), "success")
            return True
        except Exception as exc:
            messagebox.showerror("发送失败", str(exc), parent=self.root)
            self.set_status("发送失败：{}".format(exc), "error")
            return False

    def _record_raw(self, direction: str, data: bytes) -> None:
        if direction == "TX":
            self._append_log(self.tx_log, hex_bytes(data) + "\n")
            self.tx_total += len(data)
            self.tx_count_var.set("{} B".format(self.tx_total))
        else:
            now = time.monotonic()
            # 若 GUI 曾短暂忙碌而错过定时检查，先结束上一帧再接收新帧。
            if (
                self.rx_frame_buffer
                and self.rx_last_byte_at is not None
                and now - self.rx_last_byte_at >= self.RX_FRAME_GAP_SECONDS
            ):
                self._flush_rx_frame_if_idle(force=True)
            if not self.rx_frame_buffer:
                self.rx_frame_started_at = datetime.now()
            self.rx_frame_buffer.extend(data)
            self.rx_last_byte_at = now
            self.rx_total += len(data)
            self.rx_count_var.set("{} B".format(self.rx_total))

    def _flush_rx_frame_if_idle(self, force: bool = False) -> None:
        if not self.rx_frame_buffer:
            return
        if not force:
            if self.rx_last_byte_at is None:
                return
            if time.monotonic() - self.rx_last_byte_at < self.RX_FRAME_GAP_SECONDS:
                return
        frame = bytes(self.rx_frame_buffer)
        started_at = self.rx_frame_started_at or datetime.now()
        self.rx_frame_buffer.clear()
        self.rx_frame_started_at = None
        self.rx_last_byte_at = None
        if self.rx_timestamp_var.get():
            timestamp = started_at.strftime("%H:%M:%S.") + "{:03d}".format(
                started_at.microsecond // 1000
            )
            line = "[{}] {}\n".format(timestamp, hex_bytes(frame))
        else:
            line = hex_bytes(frame) + "\n"
        self._append_log(self.rx_log, line)

    def _append_log(self, widget: scrolledtext.ScrolledText, text: str) -> None:
        key = str(widget)
        widget.configure(state="normal")
        widget.insert("end", text)
        size = self.log_sizes.get(key, 0) + len(text)
        if size > self.MAX_LOG_CHARS or int(widget.index("end-1c").split(".")[0]) > 5000:
            widget.delete("1.0", "501.0")
            counted = widget.count("1.0", "end-1c", "chars")
            size = int(counted[0]) if counted else 0
        self.log_sizes[key] = size
        widget.see("end")
        widget.configure(state="disabled")

    def _clear_log(self, widget: scrolledtext.ScrolledText) -> None:
        widget.configure(state="normal")
        widget.delete("1.0", "end")
        widget.configure(state="disabled")
        self.log_sizes[str(widget)] = 0

    def clear_tx_log(self, show_status: bool = True) -> None:
        self._clear_log(self.tx_log)
        self.tx_total = 0
        self.tx_count_var.set("0 B")
        if show_status:
            self.set_status("TX 发送记录已清空", "info")

    def clear_rx_log(self, show_status: bool = True) -> None:
        self.rx_frame_buffer.clear()
        self.rx_frame_started_at = None
        self.rx_last_byte_at = None
        self._clear_log(self.rx_log)
        self.rx_total = 0
        self.rx_count_var.set("0 B")
        if show_status:
            self.set_status("RX 接收记录已清空", "info")

    def clear_send_input(self) -> None:
        self.send_text.delete("1.0", "end")
        self.send_text.focus_set()

    def send_user_data(self) -> None:
        if self.pending is not None:
            self.set_status("正在等待模块返回，请稍后再发送", "warning")
            return
        content = self.send_text.get("1.0", "end-1c")
        try:
            if self.send_format_var.get() == "HEX 原始字节":
                data = parse_hex_bytes(content)
            else:
                if not content:
                    raise ValueError("请输入要发送的文本")
                data = content.encode("utf-8")
                if self.append_crlf_var.get():
                    data += b"\r\n"
            if self.send_raw(data, "透传/手动数据") and len(data) > 54:
                self.set_status(
                    "已发送 {} 字节；超过模块单个 54 字节无线包".format(len(data)),
                    "warning",
                )
        except ValueError as exc:
            messagebox.showerror("发送内容有误", str(exc), parent=self.root)
            self.set_status("发送内容有误：{}".format(exc), "error")

    def _config_from_edit(self, persistent: bool) -> Tuple[bytes, E49Config]:
        frame = build_config_frame(
            persistent=persistent,
            address=parse_address(self.edit_address_var.get()),
            parity_code=code_by_choice(PARITY_CHOICES, self.edit_parity_var.get()),
            uart_code=code_by_choice(UART_CHOICES, self.edit_uart_var.get()),
            air_code=code_by_choice(AIR_CHOICES, self.edit_air_var.get()),
            channel=frequency_to_channel(self.edit_frequency_var.get()),
            fixed_mode=bool_by_choice(TRANSFER_CHOICES, self.edit_transfer_var.get()),
            power_code=code_by_choice(POWER_CHOICES, self.edit_power_var.get()),
        )
        return frame, decode_config_frame(frame)

    def update_edit_preview(self) -> None:
        try:
            c0, config = self._config_from_edit(True)
            c2 = bytes([0xC2]) + c0[1:]
            if self.temporary_write_var.get():
                self.edit_preview_var.set("当前临时 C2：{}\n断电后不会保留".format(hex_bytes(c2)))
                self.write_button_text_var.set("写入参数（C2，临时不保存）")
            else:
                self.edit_preview_var.set("当前永久 C0：{}\n掉电后仍然保存".format(hex_bytes(c0)))
                self.write_button_text_var.set("写入参数（C0，掉电保存）")
            self.edit_channel_var.set("CHAN 0x{:02X}（{}）".format(config.channel, config.channel))
        except (ValueError, tk.TclError) as exc:
            self.edit_preview_var.set("参数未完成：{}".format(exc))
            self.edit_channel_var.set("CHAN --")

    def load_defaults(self) -> None:
        self._populate_edit(decode_config_frame(bytes.fromhex("C0 00 00 19 2E 00")))
        self.write_status_var.set("已载入默认值，但尚未发送")
        self.set_status("已载入默认值（未发送）", "info")

    def _populate_edit(self, config: E49Config) -> None:
        self.edit_address_var.set("{:04X}".format(config.address))
        self.edit_parity_var.set(choice_by_code(PARITY_CHOICES, config.parity_code))
        self.edit_uart_var.set(choice_by_code(UART_CHOICES, config.uart_code))
        self.edit_air_var.set(choice_by_code(AIR_CHOICES, config.air_code))
        self.edit_frequency_var.set("{:.1f}".format(config.frequency_mhz))
        self.edit_transfer_var.set(TRANSFER_CHOICES[1 if config.fixed_mode else 0][0])
        self.edit_power_var.set(choice_by_code(POWER_CHOICES, config.power_code))
        self.update_edit_preview()

    def copy_read_to_edit(self) -> None:
        if self.last_read_config is None:
            self.set_status("当前没有可复制的读取结果", "warning")
            return
        self._populate_edit(self.last_read_config)
        self.write_status_var.set("已把本次读取值复制到修改区；尚未写入模块")
        self.set_status("读取值已复制到修改区（未发送）", "success")

    def clear_read_result(self, show_status: bool = True) -> None:
        self.read_frame_var.set("")
        self.read_version_var.set("")
        for variable in self.read_vars.values():
            variable.set("")
        self.last_read_config = None
        self.read_status_var.set("读取结果为空")
        self._update_control_states()
        if show_status:
            self.set_status("参数读取结果已清空；修改区未改变", "info")

    def _populate_read_result(self, config: E49Config, frame: bytes) -> None:
        self.read_frame_var.set(hex_bytes(frame))
        self.read_vars["address"].set("0x{:04X}".format(config.address))
        self.read_vars["parity"].set(
            "{}（编码 {:02b}）".format(config.uart_format, config.parity_code)
        )
        self.read_vars["uart"].set("{} bps".format(config.uart_bps))
        self.read_vars["air"].set(
            "{:.1f} kbps".format(config.air_bps / 1000.0)
            if config.air_bps < 100000
            else "{} kbps".format(config.air_bps // 1000)
        )
        self.read_vars["channel"].set("0x{:02X}（{}）".format(config.channel, config.channel))
        self.read_vars["frequency"].set("{:.1f} MHz".format(config.frequency_mhz))
        self.read_vars["transfer"].set("定点传输" if config.fixed_mode else "透明传输")
        self.read_vars["power"].set("{} dBm".format(config.power_dbm))
        self.last_read_config = config
        self._update_control_states()

    def write_parameters(self) -> None:
        if self.pending is not None:
            self.set_status("请先等待当前读取结束", "warning")
            return
        if not self._require_connection():
            return
        persistent = not self.temporary_write_var.get()
        try:
            frame, config = self._config_from_edit(persistent)
        except ValueError as exc:
            messagebox.showerror("修改参数有误", str(exc), parent=self.root)
            self.set_status("修改参数有误：{}".format(exc), "error")
            return
        if not self._use_config_uart():
            return
        name = "永久写入 C0" if persistent else "临时写入 C2"
        if self.send_raw(frame, name):
            self.write_status_var.set(
                "已发送 {}：{}\n手册未明确规定写入回包，请再点左侧“读取模块参数”核对。".format(
                    name, hex_bytes(frame)
                )
            )
            self.set_status("{} 已发送；模块应处于 M1=1、M0=0".format(name), "success")

    def _prepare_read(self, name: str) -> bool:
        if not self._require_connection():
            return False
        if self.pending is not None:
            self.set_status("已有读取命令等待返回，暂不能执行{}".format(name), "warning")
            return False
        return True

    def _drain_old_rx(self) -> int:
        if not self._is_connected():
            return 0
        total = 0
        try:
            for _ in range(16):
                waiting = int(self.ser.in_waiting)
                if waiting <= 0:
                    break
                chunk = bytes(self.ser.read(min(waiting, self.MAX_READ_CHUNK)))
                if not chunk:
                    break
                total += len(chunk)
                self._record_raw("RX", chunk)
            self.ser.reset_input_buffer()
        except Exception as exc:
            self.set_status("清理旧接收数据失败：{}".format(exc), "warning")
        # 无论驱动里是否还有字节，都在发送命令前结束当前显示帧，避免新响应粘到旧帧。
        self._flush_rx_frame_if_idle(force=True)
        return total

    def read_parameters(self) -> None:
        if not self._prepare_read("读取参数"):
            return
        if not self._use_config_uart():
            return
        self.clear_read_result(show_status=False)
        old_count = self._drain_old_rx()
        if self.clear_rx_before_read_var.get():
            self.clear_rx_log(show_status=False)
        if not self.send_raw(READ_PARAMETERS_COMMAND, "读取参数 C1"):
            return
        self._start_pending("config", (b"\xC0", b"\xC2"), 6)
        self.read_status_var.set("已清空旧结果，正在等待新的 6 字节返回……")
        suffix = "；已丢弃 {} 个请求前旧字节".format(old_count) if old_count else ""
        self.set_status("已发送 C1 C1 C1，等待参数返回{}".format(suffix), "info")

    def read_version(self) -> None:
        if not self._prepare_read("读取版本"):
            return
        if not self._use_config_uart():
            return
        self.read_version_var.set("")
        old_count = self._drain_old_rx()
        if self.clear_rx_before_read_var.get():
            self.clear_rx_log(show_status=False)
        if not self.send_raw(READ_VERSION_COMMAND, "读取版本 C3"):
            return
        self._start_pending("version", (b"\xC3\x49",), 4)
        self.read_version_var.set("等待返回……")
        suffix = "；已丢弃 {} 个请求前旧字节".format(old_count) if old_count else ""
        self.set_status("已发送 C3 C3 C3，等待版本返回{}".format(suffix), "info")

    def _start_pending(self, kind: str, signatures: Sequence[bytes], expected_len: int) -> None:
        normalized = tuple(bytes(item) for item in signatures)
        if not normalized or any(not item for item in normalized):
            raise ValueError("等待签名不能为空")
        self.pending = PendingRequest(
            kind=kind,
            signatures=normalized,
            expected_len=expected_len,
            deadline=time.monotonic() + self.RESPONSE_TIMEOUT_SECONDS,
        )
        self._update_control_states()

    @staticmethod
    def _signature_suffix_length(data: bytes, signature: bytes) -> int:
        maximum = min(len(data), max(0, len(signature) - 1))
        for length in range(maximum, 0, -1):
            if data[-length:] == signature[:length]:
                return length
        return 0

    @staticmethod
    def _find_earliest_signature(
        data: bytes, signatures: Sequence[bytes]
    ) -> Optional[Tuple[int, bytes]]:
        matches: List[Tuple[int, bytes]] = []
        for signature in signatures:
            index = data.find(signature)
            if index >= 0:
                matches.append((index, signature))
        return min(matches, key=lambda item: item[0]) if matches else None

    def _feed_pending(self, data: bytes) -> None:
        pending = self.pending
        if pending is None:
            return
        pending.buffer.extend(data)
        while self.pending is pending:
            match = self._find_earliest_signature(bytes(pending.buffer), pending.signatures)
            if match is None:
                keep = max(
                    self._signature_suffix_length(bytes(pending.buffer), signature)
                    for signature in pending.signatures
                )
                removed = len(pending.buffer) - keep
                pending.discarded_count += removed
                pending.buffer[:] = pending.buffer[-keep:] if keep else b""
                return
            index, _signature = match
            if index:
                pending.discarded_count += index
                del pending.buffer[:index]
            if len(pending.buffer) < pending.expected_len:
                return
            frame = bytes(pending.buffer[: pending.expected_len])
            try:
                parsed: Any
                if pending.kind == "config":
                    parsed = decode_config_frame(frame)
                elif pending.kind == "version":
                    parsed = decode_version_frame(frame)
                else:
                    raise ValueError("未知读取类型")
            except ValueError:
                pending.invalid_candidates += 1
                pending.discarded_count += 1
                del pending.buffer[0]
                continue
            kind = pending.kind
            discarded = pending.discarded_count
            invalid = pending.invalid_candidates
            trailing = len(pending.buffer) - pending.expected_len
            self.pending = None
            self._update_control_states()
            self._handle_read_success(kind, frame, parsed, discarded, invalid, trailing)
            return

    def _handle_read_success(
        self,
        kind: str,
        frame: bytes,
        parsed: Any,
        discarded: int,
        invalid: int,
        trailing: int,
    ) -> None:
        details: List[str] = []
        if discarded:
            details.append("返回前忽略 {} 字节".format(discarded))
        if invalid:
            details.append("跳过 {} 个无效候选".format(invalid))
        if trailing:
            details.append("返回后另有 {} 字节，仍保留在 RX".format(trailing))

        if kind == "config":
            config: E49Config = parsed
            self._populate_read_result(config, frame)
            if config.head == 0xC2:
                details.append("返回头为 C2（临时参数）")
            if config.reserved_bits:
                details.append("OPTION 保留位非零：0x{:02X}".format(config.reserved_bits))
            self.read_status_var.set(
                "读取成功" + ("；" + "；".join(details) if details else "")
            )
            self.set_status(
                "参数读取成功；修改区没有被改变",
                "warning" if details else "success",
            )
        else:
            family, version, feature = parsed
            value = "C3 49 {:02X} {:02X}｜系列 0x{:02X}，版本 0x{:02X}，特性 0x{:02X}".format(
                version, feature, family, version, feature
            )
            if details:
                value += "｜" + "；".join(details)
            self.read_version_var.set(value)
            self.set_status("版本读取成功", "warning" if details else "success")
        self.root.bell()

    def _check_pending_timeout(self) -> None:
        pending = self.pending
        if pending is None or time.monotonic() < pending.deadline:
            return
        candidate = hex_bytes(bytes(pending.buffer)) if pending.buffer else "无"
        kind = pending.kind
        self.pending = None
        self._update_control_states()
        if kind == "config":
            self.read_status_var.set(
                "读取超时（候选数据：{}）。结果区保持为空，不会显示旧参数。".format(candidate)
            )
        else:
            self.read_version_var.set("读取超时")
        self.set_status("读取超时：检查 M1=1、M0=0、TX/RX、共地及 9600 8N1", "error")
        self.root.bell()

    def on_close(self) -> None:
        self.closing = True
        if self.poll_job is not None:
            try:
                self.root.after_cancel(self.poll_job)
            except tk.TclError:
                pass
            self.poll_job = None
        self._flush_rx_frame_if_idle(force=True)
        if self.ser is not None:
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None
        self.root.destroy()


def run_self_test() -> None:
    assert READ_PARAMETERS_COMMAND == bytes.fromhex("C1 C1 C1")
    assert READ_VERSION_COMMAND == bytes.fromhex("C3 C3 C3")
    assert parse_hex_bytes("C0 00,00;19_2E:00") == bytes.fromhex("C0 00 00 19 2E 00")
    assert parse_hex_bytes("0xC1 0XC1 0xc1") == READ_PARAMETERS_COMMAND
    assert parse_hex_bytes("c1c1c1") == READ_PARAMETERS_COMMAND
    assert parse_address("0x1234") == 0x1234
    assert frequency_to_channel("433.0") == 0x2E
    assert channel_to_frequency(0x2E) == 433.0

    default = bytes.fromhex("C0 00 00 19 2E 00")
    config = decode_config_frame(default)
    assert config.address == 0x0000
    assert config.parity_code == 0
    assert config.uart_bps == 9600
    assert config.air_bps == 2400
    assert config.channel == 0x2E
    assert config.frequency_mhz == 433.0
    assert not config.fixed_mode
    assert config.power_dbm == 20
    assert config.reserved_bits == 0
    assert config.to_frame() == default

    assert [item[2] for item in UART_CHOICES] == [
        1200, 2400, 4800, 9600, 19200, 38400, 57600, 115200
    ]
    assert [item[2] for item in AIR_CHOICES] == [
        1200, 2400, 4800, 9600, 19200, 50000, 100000, 200000
    ]
    assert [item[2] for item in POWER_CHOICES] == [20, 17, 14, 10]

    for parity_code in range(4):
        for uart_code in range(8):
            for air_code in range(8):
                frame = build_config_frame(
                    True, 0xABCD, parity_code, uart_code, air_code, 0x2E, False, 0
                )
                decoded = decode_config_frame(frame)
                assert decoded.parity_code == parity_code
                assert decoded.uart_code == uart_code
                assert decoded.air_code == air_code

    for fixed_mode in (False, True):
        for power_code in range(4):
            frame = build_config_frame(True, 0, 0, 3, 1, 0, fixed_mode, power_code)
            decoded = decode_config_frame(frame)
            assert decoded.fixed_mode is fixed_mode
            assert decoded.power_code == power_code
            assert decoded.reserved_bits == 0

    assert build_config_frame(True, 0x1234, 0, 3, 1, 0x2E, True, 0) == bytes.fromhex(
        "C0 12 34 19 2E 80"
    )
    assert build_config_frame(False, 0x1234, 0, 3, 1, 0x2E, True, 0) == bytes.fromhex(
        "C2 12 34 19 2E 80"
    )
    assert decode_version_frame(bytes.fromhex("C3 49 12 34")) == (0x49, 0x12, 0x34)
    assert channel_to_frequency(0x00) == 410.0
    assert channel_to_frequency(0xC8) == 510.0

    # 不创建 GUI，也验证串口分段、前导噪声和无效候选后的重新同步。
    probe = object.__new__(E49ToolApp)
    captured: List[Tuple[Any, ...]] = []
    probe._update_control_states = lambda: None  # type: ignore
    probe._handle_read_success = lambda *args: captured.append(args)  # type: ignore
    probe.pending = PendingRequest(
        kind="config",
        signatures=(b"\xC0", b"\xC2"),
        expected_len=6,
        deadline=time.monotonic() + 1.0,
    )
    probe._feed_pending(bytes.fromhex("99 C0 00 00 19 FF 00 C0 00"))
    assert probe.pending is not None
    probe._feed_pending(bytes.fromhex("00 19 2E 00"))
    assert probe.pending is None
    assert captured[-1][0] == "config"
    assert captured[-1][1] == default

    probe.pending = PendingRequest(
        kind="version",
        signatures=(b"\xC3\x49",),
        expected_len=4,
        deadline=time.monotonic() + 1.0,
    )
    probe._feed_pending(bytes.fromhex("AA C3"))
    assert probe.pending is not None
    probe._feed_pending(bytes.fromhex("49 12 34"))
    assert probe.pending is None
    assert captured[-1][0] == "version"
    assert captured[-1][1] == bytes.fromhex("C3 49 12 34")

    invalid_cases = [
        lambda: parse_hex_bytes(""),
        lambda: parse_hex_bytes("ABC"),
        lambda: parse_hex_bytes("GG"),
        lambda: parse_hex_bytes("00x1"),
        lambda: parse_hex_bytes("0xC10xC1"),
        lambda: parse_hex_bytes("-01"),
        lambda: parse_address("10000"),
        lambda: frequency_to_channel("433.1"),
        lambda: channel_to_frequency(0xC9),
        lambda: channel_to_frequency(46.9),
        lambda: decode_config_frame(bytes.fromhex("C1 00 00 19 2E 00")),
        lambda: decode_config_frame(bytes.fromhex("C0 00 00 19 C9 00")),
        lambda: decode_version_frame(bytes.fromhex("C3 48 01 00")),
        lambda: build_config_frame(True, 1.9, 0, 3, 1, 0, False, 0),
        lambda: build_config_frame(1, 0, 0, 3, 1, 0, False, 0),
    ]
    for invalid in invalid_cases:
        try:
            invalid()
        except ValueError:
            pass
        else:
            raise AssertionError("预期 ValueError，但测试未失败：{}".format(invalid))

    print("E49 协议自测通过：{}".format(APP_VERSION))


def enable_windows_dpi_awareness() -> None:
    if sys.platform != "win32":
        return
    try:
        ctypes.windll.shcore.SetProcessDpiAwareness(1)
    except Exception:
        try:
            ctypes.windll.user32.SetProcessDPIAware()
        except Exception:
            pass


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=APP_NAME)
    parser.add_argument("--self-test", action="store_true", help="运行协议自测，不打开 GUI")
    args = parser.parse_args(argv)
    if args.self_test:
        run_self_test()
        return 0

    enable_windows_dpi_awareness()
    root = tk.Tk()
    E49ToolApp(root)
    root.mainloop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
