#!/usr/bin/env python3
"""Small GUI monitor/plotter/CSV exporter for CCtrl RS232 RM 0x0302 frames."""

from __future__ import annotations

import csv
import math
import queue
import threading
import time
import tkinter as tk
from collections import deque
from dataclasses import dataclass
from datetime import datetime
from tkinter import filedialog, messagebox, ttk
from typing import Callable

import serial
from serial.tools import list_ports

from protocol_v2 import HostPayload, HostStreamDecoder


STATE_NAMES = {0: "INIT", 1: "ACTIVE", 2: "DEGRADED", 3: "DISCONNECTED"}
STATE_NAMES_FRIENDLY = {0: "初始化", 1: "正常", 2: "降级", 3: "断开"}
COLORS = (
    "#2563eb", "#dc2626", "#16a34a", "#9333ea", "#ea580c", "#0891b2",
    "#be123c", "#4f46e5", "#15803d", "#a16207", "#7e22ce", "#0f766e",
    "#475569", "#e11d48", "#0369a1", "#65a30d", "#c2410c", "#6d28d9",
    "#0e7490", "#334155", "#b91c1c", "#047857", "#7c3aed",
)


@dataclass(frozen=True)
class Channel:
    key: str
    label: str
    read: Callable[[HostPayload], int]
    color: str
    selected: bool = False
    fmt: str = "dec"


@dataclass
class Sample:
    wall_time: float
    mono_time: float
    sequence: int
    values: dict[str, int]


def make_channels() -> list[Channel]:
    channels: list[Channel] = []

    def add(key: str, label: str, read: Callable[[HostPayload], int],
            selected: bool = False, fmt: str = "dec") -> None:
        channels.append(Channel(key, label, read, COLORS[len(channels) % len(COLORS)],
                                selected, fmt))

    for i in range(6):
        add(f"encoder_{i + 1}", f"Encoder {i + 1}",
            lambda payload, i=i: payload.encoder_value[i], True)
    add("joystick_x", "Joystick X", lambda p: p.joystick_x)
    add("joystick_y", "Joystick Y", lambda p: p.joystick_y)
    add("trigger", "Trigger", lambda p: p.trigger)
    add("main_buttons", "Main buttons", lambda p: p.main_buttons, fmt="hex")
    add("handle_buttons", "Handle buttons", lambda p: p.handle_buttons, fmt="hex")
    for i in range(6):
        add(f"encoder_status_{i + 1}", f"Enc {i + 1} status",
            lambda payload, i=i: payload.encoder_status[i], fmt="hex")
    add("system_status", "System state", lambda p: p.system_status, fmt="state")
    add("error_flags", "Error flags", lambda p: p.error_flags, fmt="hex")
    add("warning_flags", "Warning flags", lambda p: p.warning_flags, fmt="hex")
    add("node_valid_flags", "Node valid", lambda p: p.node_valid_flags, fmt="hex")
    return channels


class SerialReader(threading.Thread):
    def __init__(self, port_name: str, output: queue.Queue):
        super().__init__(daemon=True)
        self.port_name = port_name
        self.output = output
        self.stop_event = threading.Event()
        self.port: serial.Serial | None = None

    def stop(self) -> None:
        self.stop_event.set()
        if self.port and self.port.is_open:
            self.port.close()

    def run(self) -> None:
        decoder = HostStreamDecoder()
        try:
            self.port = serial.Serial(self.port_name, 115200, timeout=0.1)
            self.output.put(("connected", self.port_name))
            while not self.stop_event.is_set():
                data = self.port.read(self.port.in_waiting or 1)
                if not data:
                    continue
                before = decoder.rejected
                for frame in decoder.feed(data):
                    self.output.put(("frame", time.time(), time.monotonic(), frame))
                if decoder.rejected != before:
                    self.output.put(("rejected", decoder.rejected - before))
        except (serial.SerialException, OSError) as exc:
            if not self.stop_event.is_set():
                self.output.put(("error", str(exc)))
        finally:
            if self.port and self.port.is_open:
                self.port.close()
            self.output.put(("closed", self.port_name))


class ChannelMonitor(tk.Tk):
    MAX_HISTORY_SECONDS = 600
    EXPECTED_HZ = 30

    def __init__(self) -> None:
        super().__init__()
        self.title("CCtrl RS232 Channel Monitor")
        self.geometry("1220x780")
        self.minsize(900, 620)
        self.protocol("WM_DELETE_WINDOW", self.on_close)

        self.channels = make_channels()
        self.channel_by_key = {channel.key: channel for channel in self.channels}
        self.selected = {channel.key: tk.BooleanVar(value=channel.selected)
                         for channel in self.channels}
        self.value_labels: dict[str, ttk.Label] = {}
        self.history: deque[Sample] = deque(
            maxlen=self.MAX_HISTORY_SECONDS * self.EXPECTED_HZ * 2)
        self.events: queue.Queue = queue.Queue()
        self.reader: SerialReader | None = None
        self.frame_count = 0
        self.rejected_count = 0
        self.frame_times: deque[float] = deque()
        self.last_sequence: int | None = None
        self.sequence_gaps = 0
        self.port_display_to_name: dict[str, str] = {}

        self.port_var = tk.StringVar()
        self.status_var = tk.StringVar(value="未连接")
        self.history_seconds = tk.IntVar(value=30)
        self.independent_scale = tk.BooleanVar(value=False)
        self.friendly_mode = tk.BooleanVar(value=True)
        self.pause_time: float | None = None

        self._build_ui()
        self.refresh_ports()
        self.after(30, self.process_events)
        self.after(100, self.redraw_plot)

    def _build_ui(self) -> None:
        controls = ttk.Frame(self, padding=(8, 8, 8, 4))
        controls.pack(fill=tk.X)
        ttk.Label(controls, text="RS232串口").pack(side=tk.LEFT)
        self.port_combo = ttk.Combobox(controls, textvariable=self.port_var,
                                       state="readonly", width=40)
        self.port_combo.pack(side=tk.LEFT, padx=(6, 4))
        ttk.Button(controls, text="刷新", command=self.refresh_ports).pack(side=tk.LEFT)
        self.connect_button = ttk.Button(controls, text="连接", command=self.toggle_connection)
        self.connect_button.pack(side=tk.LEFT, padx=(8, 16))
        ttk.Label(controls, text="曲线窗口").pack(side=tk.LEFT)
        ttk.Combobox(controls, textvariable=self.history_seconds, state="readonly",
                     values=(5, 10, 30, 60, 120, 300, 600), width=5).pack(
                         side=tk.LEFT, padx=(5, 2))
        ttk.Label(controls, text="秒").pack(side=tk.LEFT)
        ttk.Checkbutton(controls, text="各通道独立缩放",
                        variable=self.independent_scale).pack(side=tk.LEFT, padx=10)
        ttk.Checkbutton(controls, text="友好值",
                        variable=self.friendly_mode,
                        command=self.refresh_value_labels).pack(side=tk.LEFT, padx=4)
        ttk.Button(controls, text="清空历史", command=self.clear_history).pack(side=tk.RIGHT)
        ttk.Button(controls, text="导出当前窗口", command=self.export_csv).pack(
            side=tk.RIGHT, padx=8)
        self.pause_button = ttk.Button(controls, text="暂停曲线",
                                       command=self.toggle_pause)
        self.pause_button.pack(side=tk.RIGHT)

        status = ttk.Label(self, textvariable=self.status_var, padding=(10, 2), anchor=tk.W)
        status.pack(fill=tk.X)

        body = ttk.Panedwindow(self, orient=tk.HORIZONTAL)
        body.pack(fill=tk.BOTH, expand=True, padx=8, pady=(4, 8))

        channel_frame = ttk.LabelFrame(body, text="通道（勾选后绘图/导出）", padding=6)
        body.add(channel_frame, weight=0)
        for index, channel in enumerate(self.channels):
            column = index // 12
            row = index % 12
            cell = ttk.Frame(channel_frame)
            cell.grid(row=row, column=column, sticky="ew", padx=(0, 12), pady=1)
            ttk.Checkbutton(cell, variable=self.selected[channel.key]).pack(side=tk.LEFT)
            ttk.Label(cell, text=channel.label, width=15).pack(side=tk.LEFT)
            value = ttk.Label(cell, text="--", width=24, anchor=tk.E)
            value.pack(side=tk.LEFT)
            self.value_labels[channel.key] = value
        for col in range((len(self.channels) + 11) // 12):
            channel_frame.columnconfigure(col, weight=1)

        plot_frame = ttk.LabelFrame(body, text="最近一段时间", padding=4)
        body.add(plot_frame, weight=1)
        self.plot = tk.Canvas(plot_frame, background="#ffffff", highlightthickness=0)
        self.plot.pack(fill=tk.BOTH, expand=True)

    def refresh_ports(self) -> None:
        current_name = self.port_display_to_name.get(self.port_var.get(), "")
        self.port_display_to_name.clear()
        displays = []
        for item in sorted(list_ports.comports(), key=lambda port: port.device):
            display = f"{item.device} — {item.description}"
            displays.append(display)
            self.port_display_to_name[display] = item.device
        self.port_combo["values"] = displays
        selected = next((display for display, name in self.port_display_to_name.items()
                         if name == current_name), None)
        if selected:
            self.port_var.set(selected)
        elif displays:
            preferred = next((display for display in displays
                              if self.port_display_to_name[display] == "COM6"), displays[0])
            self.port_var.set(preferred)
        else:
            self.port_var.set("")

    def toggle_connection(self) -> None:
        if self.reader:
            self.disconnect()
            return
        port_name = self.port_display_to_name.get(self.port_var.get())
        if not port_name:
            messagebox.showwarning("没有串口", "请选择可用的RS232串口。")
            return
        self.connect_button.configure(text="断开")
        self.port_combo.configure(state="disabled")
        self.status_var.set(f"正在连接 {port_name} @ 115200 8N1…")
        self.reader = SerialReader(port_name, self.events)
        self.reader.start()

    def disconnect(self) -> None:
        reader, self.reader = self.reader, None
        if reader:
            reader.stop()
        self.connect_button.configure(text="连接")
        self.port_combo.configure(state="readonly")
        self.status_var.set("已断开")

    def process_events(self) -> None:
        try:
            while True:
                event = self.events.get_nowait()
                kind = event[0]
                if kind == "frame":
                    _, wall, mono, frame = event
                    self.accept_frame(wall, mono, frame.sequence, frame.payload)
                elif kind == "rejected":
                    self.rejected_count += event[1]
                elif kind == "connected":
                    self.status_var.set(f"已连接 {event[1]} @ 115200 8N1，等待0x0302帧")
                elif kind == "error":
                    messagebox.showerror("串口错误", event[1])
                    self.disconnect()
                elif kind == "closed" and self.reader and not self.reader.is_alive():
                    self.disconnect()
        except queue.Empty:
            pass
        self.update_status()
        self.after(30, self.process_events)

    def accept_frame(self, wall: float, mono: float, sequence: int,
                     payload: HostPayload) -> None:
        values = {channel.key: int(channel.read(payload)) for channel in self.channels}
        self.history.append(Sample(wall, mono, sequence, values))
        self.frame_count += 1
        self.frame_times.append(mono)
        if self.last_sequence is not None:
            delta = (sequence - self.last_sequence) & 0xFF
            if delta != 1:
                self.sequence_gaps += (delta - 1) & 0xFF
        self.last_sequence = sequence
        for channel in self.channels:
            self.value_labels[channel.key].configure(
                text=self.format_value(channel, values[channel.key]))

    def refresh_value_labels(self) -> None:
        if not self.history:
            return
        values = self.history[-1].values
        for channel in self.channels:
            self.value_labels[channel.key].configure(
                text=self.format_value(channel, values[channel.key]))

    @staticmethod
    def bit_names(value: int, names: tuple[tuple[int, str], ...],
                  empty: str = "无") -> str:
        active = [name for bit, name in names if value & bit]
        return " ".join(active) if active else empty

    def format_value(self, channel: Channel, value: int) -> str:
        if self.friendly_mode.get():
            if channel.key.startswith("encoder_") and not channel.key.startswith("encoder_status_"):
                return f"{value * 360.0 / 4096.0:+.1f}°"
            if channel.key in ("joystick_x", "joystick_y", "trigger"):
                return f"{value * 100.0 / 4095.0:.1f}%"
            if channel.key.startswith("encoder_status_"):
                if value == 3:
                    return "正常"
                return self.bit_names(value, ((1, "I2C"), (2, "磁铁"),
                                              (4, "弱"), (8, "强"),
                                              (16, "旧")), "不可用")
            if channel.key == "system_status":
                return STATE_NAMES_FRIENDLY.get(value, f"未知({value})")
            if channel.key == "error_flags":
                return self.bit_names(value, ((1, "超时"),), "正常")
            if channel.key == "warning_flags":
                return self.bit_names(value, ((1, "CRC"), (2, "格式"),
                                              (4, "磁场"), (8, "节点数")),
                                      "正常")
            if channel.key == "node_valid_flags":
                names = tuple((1 << i, f"E{i + 1}") for i in range(6)) + ((0x40, "Handle"),)
                return self.bit_names(value, names)
            if channel.key == "main_buttons":
                return self.bit_names(value, tuple((1 << i, f"K{i + 1}") for i in range(4)))
            if channel.key == "handle_buttons":
                return self.bit_names(value, tuple((1 << i, f"S{i + 1}") for i in range(4)))
        if channel.fmt == "hex":
            return f"0x{value:02X}"
        if channel.fmt == "state":
            return f"{value} {STATE_NAMES.get(value, '?')}"
        return str(value)

    def plot_value(self, channel: Channel, value: int) -> float:
        if not self.friendly_mode.get():
            return float(value)
        if channel.key.startswith("encoder_") and not channel.key.startswith("encoder_status_"):
            return value * 360.0 / 4096.0
        if channel.key in ("joystick_x", "joystick_y", "trigger"):
            return value * 100.0 / 4095.0
        return float(value)

    @staticmethod
    def plot_number(value: float) -> str:
        return f"{value:.1f}" if not value.is_integer() else str(int(value))

    def update_status(self) -> None:
        now = time.monotonic()
        while self.frame_times and now - self.frame_times[0] > 2.0:
            self.frame_times.popleft()
        hz = len(self.frame_times) / 2.0
        if self.reader:
            port = self.reader.port_name
            pause_text = "  曲线:已暂停" if self.pause_time is not None else ""
            self.status_var.set(
                f"{port}  接收:{self.frame_count}  速率:{hz:.1f} Hz  "
                f"坏帧:{self.rejected_count}  序号缺失:{self.sequence_gaps}  "
                f"历史:{len(self.history)}帧{pause_text}")

    def visible_samples(self) -> list[Sample]:
        end = self.pause_time if self.pause_time is not None else time.monotonic()
        cutoff = end - max(1, self.history_seconds.get())
        return [sample for sample in self.history
                if cutoff <= sample.mono_time <= end]

    def selected_channels(self) -> list[Channel]:
        return [channel for channel in self.channels if self.selected[channel.key].get()]

    def redraw_plot(self) -> None:
        canvas = self.plot
        canvas.delete("all")
        width = max(canvas.winfo_width(), 100)
        height = max(canvas.winfo_height(), 100)
        left, right, top, bottom = 54, 12, 28, 34
        plot_w, plot_h = max(1, width - left - right), max(1, height - top - bottom)
        canvas.create_rectangle(left, top, left + plot_w, top + plot_h,
                                outline="#94a3b8")
        for i in range(1, 5):
            y = top + plot_h * i / 5
            canvas.create_line(left, y, left + plot_w, y, fill="#e2e8f0")
        window_s = max(1, self.history_seconds.get())
        for fraction in (0.0, 0.25, 0.5, 0.75, 1.0):
            x = left + plot_w * fraction
            canvas.create_line(x, top, x, top + plot_h, fill="#f1f5f9")
            canvas.create_text(x, top + plot_h + 16,
                               text=f"{-window_s * (1.0 - fraction):.0f}s",
                               fill="#64748b")

        samples = self.visible_samples()
        channels = self.selected_channels()
        if not samples or not channels:
            canvas.create_text(left + plot_w / 2, top + plot_h / 2,
                               text="连接串口并勾选需要查看的通道",
                               fill="#64748b", font=("Segoe UI", 12))
            self.after(100, self.redraw_plot)
            return

        now = self.pause_time if self.pause_time is not None else time.monotonic()
        step = max(1, math.ceil(len(samples) / max(plot_w, 1)))
        draw_samples = samples[::step]
        if draw_samples[-1] is not samples[-1]:
            draw_samples.append(samples[-1])
        shared_values = [self.plot_value(channel, sample.values[channel.key])
                         for sample in samples for channel in channels]
        shared_min, shared_max = min(shared_values), max(shared_values)
        if shared_min == shared_max:
            shared_min -= 1
            shared_max += 1

        legend_x = left + 6
        for channel in channels:
            channel_values = [self.plot_value(channel, sample.values[channel.key])
                              for sample in samples]
            if self.independent_scale.get():
                low, high = min(channel_values), max(channel_values)
                if low == high:
                    low -= 1
                    high += 1
            else:
                low, high = shared_min, shared_max
            points = []
            for sample in draw_samples:
                x = left + plot_w * (1.0 - min(window_s, now - sample.mono_time) / window_s)
                value = self.plot_value(channel, sample.values[channel.key])
                y = top + plot_h * (high - value) / (high - low)
                points.extend((x, y))
            if len(points) >= 4:
                canvas.create_line(*points, fill=channel.color, width=2, smooth=False)
            legend = channel.label
            if self.independent_scale.get():
                legend += (f" [{self.plot_number(min(channel_values))}.."
                           f"{self.plot_number(max(channel_values))}]")
            text_id = canvas.create_text(legend_x, 13, text=legend, anchor=tk.W,
                                         fill=channel.color, font=("Segoe UI", 9, "bold"))
            legend_x += canvas.bbox(text_id)[2] - canvas.bbox(text_id)[0] + 12
            if legend_x > width - 140:
                break
        if not self.independent_scale.get():
            canvas.create_text(left - 5, top, text=self.plot_number(shared_max), anchor=tk.E,
                               fill="#475569")
            canvas.create_text(left - 5, top + plot_h,
                               text=self.plot_number(shared_min), anchor=tk.E,
                               fill="#475569")
        self.after(100, self.redraw_plot)

    def clear_history(self) -> None:
        self.history.clear()
        self.frame_count = 0
        self.rejected_count = 0
        self.sequence_gaps = 0
        self.last_sequence = None

    def toggle_pause(self) -> None:
        if self.pause_time is None:
            self.pause_time = time.monotonic()
            self.pause_button.configure(text="继续曲线")
        else:
            self.pause_time = None
            self.pause_button.configure(text="暂停曲线")

    def export_csv(self) -> None:
        channels = self.selected_channels()
        samples = self.visible_samples()
        if not channels:
            messagebox.showwarning("没有通道", "请先勾选至少一个需要导出的通道。")
            return
        if not samples:
            messagebox.showwarning("没有数据", "当前时间窗口内没有可导出的数据。")
            return
        default_name = datetime.now().strftime("cctrl-rs232-%Y%m%d-%H%M%S.csv")
        path = filedialog.asksaveasfilename(
            title="导出当前时间窗口", defaultextension=".csv",
            initialfile=default_name,
            filetypes=(("CSV文件", "*.csv"), ("所有文件", "*.*")))
        if not path:
            return
        first_mono = samples[0].mono_time
        with open(path, "w", newline="", encoding="utf-8-sig") as output:
            writer = csv.writer(output)
            writer.writerow(["timestamp", "elapsed_s", "sequence"] +
                            [channel.key for channel in channels])
            for sample in samples:
                writer.writerow([
                    datetime.fromtimestamp(sample.wall_time).isoformat(timespec="milliseconds"),
                    f"{sample.mono_time - first_mono:.6f}", sample.sequence,
                    *[sample.values[channel.key] for channel in channels],
                ])
        messagebox.showinfo("导出完成", f"已导出 {len(samples)} 帧、{len(channels)} 个通道。\n{path}")

    def on_close(self) -> None:
        if self.reader:
            self.reader.stop()
        self.destroy()


if __name__ == "__main__":
    ChannelMonitor().mainloop()
