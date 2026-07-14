#!/usr/bin/env python3
"""
pico-rgb2hdmi control panel.
(Generated 100% by Opus 4.8)

A single-page Tk GUI to drive the converter over its USB serial console.
Every console command exposed by the firmware (integrationTest.c / commands.c)
has a control here: screen position, horizontal alignment, sampling timing,
AFE calibration, display slots, settings dump/restore/decode, diagnostics,
capture and raw commands. A background poller reads the `status` command and
keeps a live measurements panel + every spinner in sync with the device.
"""

import os
import sys
import time
import queue
import struct
import argparse
import threading
from datetime import datetime

import serial
import tkinter as tk
from tkinter import ttk, messagebox
from tkinter.scrolledtext import ScrolledText
from tkinter.filedialog import asksaveasfile

import serialCmd
import csv2png

os.environ['TK_SILENCE_DEPRECATION'] = '1'

# Firmware limits, mirrored from settings.h / videoAdjust.h so the widgets clamp
# the same way the device does.
DISPLAY_SLOT_MAX = 4          # SETTINGS_DISPLAY_MAX
FINE_TUNE_MAX_STEPS = 100     # VIDEO_FINE_TUNE_MAX / 1000
PHASE_MAX = 11                # 0..11 twelfths of a pixel
GAIN_MAX = 511                # WM8213_GAIN_MAX
OFFSET_MAX = 255              # WM8213_POS_OFFSET_MAX
NEG_OFFSET_MAX = 15           # WM8213_NEG_OFFSET_MAX
PORCH_MAX = 300               # firmware clamp in command_apply_h_porch

DEFAULT_PORT = "/dev/tty.usbmodem1101"

# The firmware runs an interactive shell (integrationTest.c command_line_loop):
# it prints this prompt, echoes typed characters, accepts '\r' or '\n' as Enter,
# then emits a "Request <name><c>(<arg>)" ack line before the reply. So a command
# exchange looks like:
#   rgb2hdmi> version\r\n            <- prompt + echoed command
#   Request version<v>()\r\n         <- ack
#   pico-rgb2hdmi - ... version 0.7.0\r\n   <- reply
#   rgb2hdmi>                        <- next prompt (no trailing newline)
# We send '\r' and read until that next prompt re-appears.
PROMPT = "rgb2hdmi>"

SYNC_NAMES = {"0": "none", "1": "csync", "2": "hvsync"}


# ---------------------------------------------------------------------------
# Serial worker: owns the port, runs every command on one background thread so
# the UI never blocks and commands never overlap on the wire.
# ---------------------------------------------------------------------------
class SerialWorker:
    def __init__(self, log_fn, status_fn, verbose=False):
        self._log = log_fn                # thread-safe callable(str)
        self._status = status_fn          # thread-safe callable(connected, port, extra)
        self._verbose = verbose
        self._jobs = queue.Queue()
        self._port = None
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def _vlog(self, msg):
        """Log only when --verbose: connection chatter, idle checks, etc."""
        if self._verbose:
            self._log(msg)

    # ---- prompt-aware serial conversation ----
    def _converse(self, cmd, initial_timeout=2.0, idle_gap=0.4):
        """Send one command and read the reply up to the next prompt.

        Raises on I/O error (caller treats that as a disconnect). Returns the
        raw text received (echo + ack + reply + next prompt).
        """
        ser = self._port
        ser.reset_input_buffer()
        ser.write((cmd + "\r").encode("ascii"))
        buf = bytearray()
        start = last = time.time()
        while True:
            n = ser.in_waiting
            if n:
                buf += ser.read(n)
                last = time.time()
                # Done once the next prompt has been re-emitted at the tail.
                tail = bytes(buf[-len(PROMPT) - 4:]).decode("ascii", "replace")
                if tail.rstrip().endswith(PROMPT):
                    break
            else:
                now = time.time()
                if buf and now - last > idle_gap:
                    break                      # data stopped without a prompt
                if not buf and now - start > initial_timeout:
                    break                      # device silent
                time.sleep(0.01)
        return buf.decode("ascii", "replace")

    def _parse(self, cmd, raw):
        """Split a raw exchange into (ok, reply_text, ack_line)."""
        idx = raw.rfind(PROMPT)                # drop the trailing prompt (may be
        if idx != -1:                          # glued to the last reply line)
            raw = raw[:idx]
        ack = None
        body = []
        for ln in raw.replace("\r", "").split("\n"):
            s = ln.strip()
            if s.startswith(PROMPT):           # strip a prompt glued to the echo
                s = s[len(PROMPT):].strip()
            if not s or s == cmd:              # blank or the echoed command
                continue
            if s.startswith("Request ") and "<" in s and s.endswith(")"):
                ack = s
                continue
            body.append(s)
        ok = ack is not None or bool(body)
        return ok, "\n".join(body), ack

    # ---- public API (called from the UI thread) ----
    def connect(self, port=None, skip_scan=False):
        self._jobs.put(('connect', (port, skip_scan), None))

    def send(self, cmd, on_done=None, quiet=False):
        """Queue a console command. on_done(ok, text) runs on the UI thread."""
        self._jobs.put(('cmd', cmd, (on_done, quiet)))

    def stop(self):
        self._running = False
        self._jobs.put(('quit', None, None))

    @property
    def connected(self):
        return self._port is not None

    # ---- worker thread ----
    def _loop(self):
        last_check = 0.0
        while self._running:
            try:
                kind, arg, meta = self._jobs.get(timeout=1.0)
            except queue.Empty:
                # Idle: cheaply verify the link is still up every couple seconds.
                now = time.time()
                if self._port is not None and now - last_check > 2.0:
                    last_check = now
                    self._vlog("* idle link check")
                    if not self._alive():
                        self._log("* Device disconnected")
                        self._close_port()
                        self._status(False, "", "")
                continue

            if kind == 'quit':
                break
            if kind == 'connect':
                self._do_connect(arg[0], arg[1])
            elif kind == 'cmd':
                self._do_cmd(arg, meta)

    def _open(self, port):
        return serial.Serial(port=port, baudrate=115200,
                             parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE,
                             bytesize=serial.EIGHTBITS, timeout=1, write_timeout=2)

    def _close_port(self):
        if self._port is not None:
            try:
                self._port.close()
            except Exception:
                pass
            self._port = None

    def _alive(self):
        try:
            return self._port.is_open and self._port.in_waiting >= 0
        except Exception:
            return False

    def _do_connect(self, port, skip_scan):
        self._close_port()

        if skip_scan:
            if not port:
                self._log("No port set; cannot connect without auto-detect")
                self._status(False, "", "")
                return
            self._vlog("Opening %s directly (auto-detect disabled)" % port)
            try:
                self._port = self._open(port)
            except Exception as e:
                self._log("Cannot open %s: %s" % (port, e))
                self._status(False, "", "")
                return
            self._log("Connected on %s (auto-detect disabled)" % port)
            self._status(True, port, self._probe_identity())
            return

        self._vlog("Auto-detecting a pico-rgb2hdmi ...")
        found = self._scan_for_device(port)
        if not found:
            self._log("No device found. Enable the USB console on the unit, "
                      "or set the port and tick 'No auto-detect'.")
            self._status(False, "", "")
            return
        self._log("Connected on %s" % found)
        self._status(True, found, self._probe_identity())

    def _try_port(self, port):
        """Open + probe one port. Leaves it open and returns True if it answers
        'version' as a pico-rgb2hdmi; otherwise closes it and returns False."""
        self._vlog("Probing %s ..." % port)
        try:
            self._port = self._open(port)
        except Exception as e:
            self._vlog("  open failed: %s" % e)
            return False
        try:
            ok, body, _ack = self._parse("version", self._converse("version"))
        except Exception as e:
            self._vlog("  no reply: %s" % e)
            ok, body = False, ""
        if ok and "pico-rgb2hdmi" in body:
            return True
        self._close_port()
        return False

    def _scan_for_device(self, preferred=None):
        """Try the preferred (default) port first, then every other serial port."""
        if preferred:
            self._vlog("Trying default port %s first" % preferred)
            if self._try_port(preferred):
                return preferred
        for port in serialCmd.get_serial_ports():
            if port == preferred or "bluetooth" in port.lower():
                continue
            if self._try_port(port):
                return port
        return None

    def _probe_identity(self):
        """Read id + mode so the status bar can show them. Returns 'id | mode'."""
        parts = []
        for cmd, marker in (("id", "Device is:"), ("mode", "pico-rgb2hdmi")):
            try:
                _ok, body, _ack = self._parse(cmd, self._converse(cmd))
            except Exception:
                body = ""
            self._vlog("%s -> %s" % (cmd, body or "no reply"))
            if marker in body:
                parts.append(body.split(marker, 1)[1].strip())
        return "  |  ".join(parts)

    def _do_cmd(self, cmd, meta):
        on_done, quiet = meta
        if self._port is None:
            if not quiet:
                self._log("! Not connected: %s" % cmd)
            if on_done:
                _ui_call(on_done, False, "")
            return
        if not quiet:
            self._log(">> %s" % cmd)
        # 'capture' streams a whole frame (tens of seconds), so let its idle
        # window be generous. 'probe' collects silently for ~20ms per frame
        # before printing anything, so scale its initial timeout with the
        # frame count or long probes would be reported as "no response".
        idle = 3.0 if cmd == "capture" else 0.4
        initial = 2.0
        if cmd.startswith("probe"):
            try:
                frames = int(cmd.split(",")[-1])
            except ValueError:
                frames = 50
            initial = frames * 0.02 + 3.0
        try:
            raw = self._converse(cmd, initial_timeout=initial, idle_gap=idle)
        except Exception as e:
            self._log("!! I/O error on '%s': %s" % (cmd, e))
            self._close_port()
            self._status(False, "", "")
            if on_done:
                _ui_call(on_done, False, "")
            return
        ok, body, ack = self._parse(cmd, raw)
        if self._verbose and ack:
            self._log(ack)
        if not ok and not quiet:
            self._log("!! no response to '%s'" % cmd)
        elif body and not quiet:
            self._log(body)
        if on_done:
            _ui_call(on_done, ok, body)


# ---------------------------------------------------------------------------
# UI-thread marshalling helpers
# ---------------------------------------------------------------------------
_root = None


def _ui_call(fn, *args):
    if _root is not None:
        _root.after(0, lambda: fn(*args))


# ---------------------------------------------------------------------------
# Main application
# ---------------------------------------------------------------------------
class App:
    def __init__(self, root, verbose=False, default_port=DEFAULT_PORT):
        global _root
        _root = root
        self.root = root
        self.verbose = verbose
        self.default_port = default_port
        self.worker = SerialWorker(self.log, self.set_status, verbose=verbose)
        self.last_image = None            # PIL image from last capture
        self.last_capture_text = ""       # raw capture CSV text
        self.auto_set = None              # BooleanVar, built with the status bar
        self._poll_busy = False
        self._missed_prev = None          # (missed_count, timestamp) for the rate
        self.raw_history = []             # raw command history (Up/Down arrows)
        self.raw_hist_pos = None          # None = not browsing; else history index
        self.raw_draft = ""               # in-progress text saved while browsing

        root.title("pico-rgb2hdmi control panel")
        root.minsize(780, 680)

        outer = ttk.Frame(root, padding=8)
        outer.pack(fill="both", expand=True)

        self._build_status_bar(outer)

        # Vertical paned window: controls on top, console below - drag the sash
        # to resize the console, or detach it into its own window.
        self.paned = ttk.PanedWindow(outer, orient="vertical")
        self.paned.pack(fill="both", expand=True)
        top = ttk.Frame(self.paned)
        self.bottom = ttk.Frame(self.paned)
        self.paned.add(top, weight=0)
        self.paned.add(self.bottom, weight=1)
        self.console_win = None           # Toplevel while the console is detached

        body = ttk.Frame(top)
        body.pack(fill="x")
        # Two columns of grouped controls.
        left = ttk.Frame(body)
        left.pack(side="left", fill="both", expand=True, padx=(0, 4))
        right = ttk.Frame(body)
        right.pack(side="left", fill="both", expand=True, padx=(4, 0))

        self._build_measurements(left)
        self._build_position(left)
        self._build_horizontal(left)
        self._build_sampling(left)

        self._build_calibration(right)
        self._build_display_system(right)
        self._build_settings(right)
        self._build_diagnostics(right)

        self._build_capture(top)
        self._build_raw(self.bottom)
        self._build_console(self.bottom)

        root.protocol("WM_DELETE_WINDOW", self.on_close)
        if verbose:
            self.log("Verbose logging enabled")
        # Kick off connect shortly after the window is up, then start polling.
        root.after(200, self.do_connect)
        root.after(1500, self._poll_tick)

    # ---- small widget helpers ----
    def _group(self, parent, title):
        f = ttk.LabelFrame(parent, text=title, padding=6)
        f.pack(fill="x", pady=4)
        return f

    def _entry(self, parent, default="", width=8):
        e = ttk.Entry(parent, width=width)
        e.insert(0, str(default))
        return e

    def _spinbox(self, parent, lo, hi, default, width=6):
        s = ttk.Spinbox(parent, width=width, from_=lo, to=hi)
        s.set(default)
        return s

    def _autowire(self, spinbox, send_fn):
        """Make a spinner send its command when 'Auto-set' is on: fires on the
        up/down arrows, and always on Enter."""
        spinbox.configure(command=lambda: self._auto(send_fn))
        spinbox.bind("<Return>", lambda _e: send_fn())
        return send_fn

    def _auto(self, send_fn):
        if self.auto_set.get():
            send_fn()

    def set_enabled(self, container, enabled):
        """Enable/disable (gray out) every widget inside a frame or group,
        recursively. Works on ttk widgets (state flags) and classic tk widgets
        (configure state). Containers themselves have no state - only their
        children are touched."""
        state_flags = ["!disabled"] if enabled else ["disabled"]
        state_value = "normal" if enabled else "disabled"
        for child in container.winfo_children():
            try:
                child.state(state_flags)              # ttk widgets
            except (AttributeError, tk.TclError):
                try:
                    child.configure(state=state_value)  # classic tk widgets
                except tk.TclError:
                    pass                              # pure containers
            self.set_enabled(child, enabled)

    def _set_widget(self, widget, value):
        """Update a spinner/entry from polled data - but never while the user
        is editing it (it has focus), and only when the value changed."""
        if value is None:
            return
        try:
            if self.root.focus_get() is widget:
                return
        except (KeyError, tk.TclError):
            pass
        if str(widget.get()) == str(value):
            return
        try:
            widget.set(str(value))                 # ttk.Spinbox
        except AttributeError:
            widget.delete(0, "end")                # ttk.Entry
            widget.insert(0, str(value))

    # ---- status bar ----
    def _build_status_bar(self, parent):
        bar = ttk.Frame(parent)
        bar.pack(fill="x", pady=(0, 6))
        self.status_dot = ttk.Label(bar, text="●", foreground="#c0392b")
        self.status_dot.pack(side="left")
        self.status_label = ttk.Label(bar, text="Disconnected")
        self.status_label.pack(side="left", padx=6)
        self.auto_set = tk.BooleanVar(value=False)
        ttk.Checkbutton(bar, text="Auto-set spinners", variable=self.auto_set).pack(side="left", padx=12)

        ttk.Button(bar, text="Reconnect", command=self.do_connect).pack(side="right")
        self.no_scan = tk.BooleanVar(value=False)
        ttk.Checkbutton(bar, text="No auto-detect", variable=self.no_scan).pack(side="right", padx=6)
        self.port_entry = ttk.Entry(bar, width=22)
        self.port_entry.insert(0, self.default_port)
        self.port_entry.pack(side="right")
        ttk.Label(bar, text="Port:").pack(side="right", padx=(0, 3))

    def do_connect(self):
        """Read the port box + checkbox and (re)connect via the worker."""
        port = self.port_entry.get().strip()
        self.worker.connect(port=port, skip_scan=self.no_scan.get())

    def set_status(self, connected, port, extra):
        if connected:
            self.status_dot.config(foreground="#27ae60")
            text = "Connected  %s" % port
            if extra:
                text += "   " + extra
            self.status_label.config(text=text)
        else:
            self.status_dot.config(foreground="#c0392b")
            self.status_label.config(text="Disconnected")

    # ---- live readings + poller ----
    def _build_measurements(self, parent):
        g = self._group(parent, "Live readings")

        top = ttk.Frame(g)
        top.pack(fill="x")
        self.poll_enabled = tk.BooleanVar(value=True)
        ttk.Checkbutton(top, text="Poll every", variable=self.poll_enabled).pack(side="left")
        self.poll_int = self._spinbox(top, 1, 30, 2, width=3)
        self.poll_int.pack(side="left", padx=2)
        ttk.Label(top, text="s").pack(side="left")

        grid = ttk.Frame(g)
        grid.pack(fill="x", pady=(4, 0))
        self.meas = {}
        rows = [("Mode", "mode"), ("Slot", "slot"),
                ("Sampling rate", "rate"), ("Sync", "sync"),
                ("Line rate", "linerate"), ("Frame rate", "framerate"),
                ("Lines/frame", "lines"), ("Missed arms", "missed")]
        for i, (label, key) in enumerate(rows):
            r, c = divmod(i, 2)
            ttk.Label(grid, text=label + ":").grid(row=r, column=c * 2, sticky="w", padx=(0, 4), pady=1)
            v = ttk.Label(grid, text="—", width=17)
            v.grid(row=r, column=c * 2 + 1, sticky="w", padx=(0, 10), pady=1)
            self.meas[key] = v

    def _poll_tick(self):
        """Periodic status poll; reschedules itself. Skips a round while a
        previous poll is still in flight or a capture may be running."""
        if self.worker.connected and self.poll_enabled.get() and not self._poll_busy:
            self._poll_busy = True
            self.worker.send("status", on_done=self._on_status, quiet=True)
        try:
            interval = max(1, int(float(self.poll_int.get())))
        except (ValueError, tk.TclError):
            interval = 2
        self.root.after(interval * 1000, self._poll_tick)

    def _on_status(self, ok, text):
        self._poll_busy = False
        st = parse_status(text)
        if not ok or not st:
            return
        now = time.time()
        try:
            self.meas["mode"].config(text="%sx%s@%sbpp" % (st["w"], st["h"], st["bpp"]))
            self.meas["slot"].config(text=st["slot"])
            self.meas["rate"].config(text="%s Hz" % st["rate"])
            self.meas["sync"].config(text=SYNC_NAMES.get(st.get("sync"), st.get("sync", "—")))
            hsync_ns = int(st.get("hsyncns", "0"))
            vsync_ns = int(st.get("vsyncns", "0"))
            self.meas["linerate"].config(text=("%.0f Hz" % (1e9 / hsync_ns)) if hsync_ns else "—")
            self.meas["framerate"].config(text=("%.1f Hz" % (1e9 / vsync_ns)) if vsync_ns else "—")
            lines = st.get("lines", "0")
            self.meas["lines"].config(text=lines if lines != "0" else "—")
            missed = int(st.get("missed", "0"))
            note = ""
            if self._missed_prev is not None:
                prev_missed, prev_time = self._missed_prev
                if missed < prev_missed:
                    note = "  (device rebooted?)"
                elif now > prev_time:
                    note = "  (+%.0f/s)" % ((missed - prev_missed) / (now - prev_time))
            self.meas["missed"].config(text="%d%s" % (missed, note))
            self._missed_prev = (missed, now)
        except (KeyError, ValueError, ZeroDivisionError) as e:
            if self.verbose:
                self.log("! status parse: %s" % e)
            return

        # Reflect device state into the control widgets (focus-safe).
        self._set_widget(self.porch_front, st.get("hf"))
        self._set_widget(self.porch_back, st.get("hb"))
        try:  # keep the sum-lock reference in step with what the widgets show
            self._porch_last = [int(float(self.porch_front.get())),
                                int(float(self.porch_back.get()))]
        except (ValueError, tk.TclError):
            pass
        try:
            self._set_widget(self.pixelw, int(st["hf"]) + int(st["hb"]))
        except (KeyError, ValueError):
            pass
        self._set_widget(self.phase, st.get("phase"))
        self._set_widget(self.finetune, st.get("finetune"))
        self._set_widget(self.refresh, st.get("refresh"))
        self._set_widget(self.slot, st.get("slot"))
        gain = st.get("gain", "").split(",")
        offset = st.get("offset", "").split(",")
        if len(gain) == 3:
            for widget, value in zip(self.gain_rgb, gain):
                self._set_widget(widget, value)
        if len(offset) == 3:
            for widget, value in zip(self.offset_rgb, offset):
                self._set_widget(widget, value)
        self._set_widget(self.negoffset, st.get("neg"))

    # ---- position (D-pad) ----
    def _build_position(self, parent):
        g = self._group(parent, "Screen position")
        step_row = ttk.Frame(g)
        step_row.pack(anchor="w")
        ttk.Label(step_row, text="Step").pack(side="left")
        self.step = self._spinbox(step_row, 1, 50, 1, width=4)
        self.step.pack(side="left", padx=4)

        pad = ttk.Frame(g)
        pad.pack(pady=4)
        ttk.Button(pad, text="↑ Up", width=8,
                   command=lambda: self.move("up")).grid(row=0, column=1, pady=1)
        ttk.Button(pad, text="← Left", width=8,
                   command=lambda: self.move("left")).grid(row=1, column=0, padx=1)
        ttk.Button(pad, text="Right →", width=8,
                   command=lambda: self.move("right")).grid(row=1, column=2, padx=1)
        ttk.Button(pad, text="↓ Down", width=8,
                   command=lambda: self.move("down")).grid(row=2, column=1, pady=1)

    def move(self, direction):
        try:
            step = int(float(self.step.get()))
        except ValueError:
            step = 1
        self.worker.send("%s %d" % (direction, max(1, step)))

    def _porch_spun(self, which):
        """Porch spinner arrow handler: applies the sum-lock, then auto-sends."""
        try:
            front = int(float(self.porch_front.get()))
            back = int(float(self.porch_back.get()))
        except (ValueError, tk.TclError):
            return
        if self.porch_lock.get():
            if which == 0:
                back = max(0, min(PORCH_MAX, self._porch_last[1] - (front - self._porch_last[0])))
                self.porch_back.set(back)
            else:
                front = max(1, min(PORCH_MAX, self._porch_last[0] - (back - self._porch_last[1])))
                self.porch_front.set(front)
        self._porch_last = [front, back]
        self._auto(self._send_porch)

    # ---- horizontal alignment ----
    def _build_horizontal(self, parent):
        g = self._group(parent, "Horizontal alignment")

        row = ttk.Frame(g)
        row.pack(fill="x", pady=1)
        ttk.Label(row, text="Porch front,back").pack(side="left")
        self.porch_front = self._spinbox(row, 1, PORCH_MAX, 100, width=5)
        self.porch_front.pack(side="left", padx=2)
        self.porch_back = self._spinbox(row, 0, PORCH_MAX, 50, width=5)
        self.porch_back.pack(side="left", padx=2)
        send_porch = lambda: self.worker.send(
            "porch %s,%s" % (_spin(self.porch_front), _spin(self.porch_back)))
        self._send_porch = send_porch
        self._porch_last = [100, 50]
        # Locked: moving one porch counter-moves the other, keeping the sum
        # (i.e. the sampling rate) constant - pure position moves.
        self.porch_lock = tk.BooleanVar(value=True)
        self.porch_front.configure(command=lambda: self._porch_spun(0))
        self.porch_back.configure(command=lambda: self._porch_spun(1))
        self.porch_front.bind("<Return>", lambda _e: send_porch())
        self.porch_back.bind("<Return>", lambda _e: send_porch())
        ttk.Checkbutton(row, text="Lock sum", variable=self.porch_lock).pack(side="left", padx=2)
        ttk.Button(row, text="Set", width=5, command=send_porch).pack(side="left", padx=4)

        row = ttk.Frame(g)
        row.pack(fill="x", pady=1)
        ttk.Label(row, text="Pixel width (sum)").pack(side="left")
        self.pixelw = self._spinbox(row, 1, 2 * PORCH_MAX, 150, width=5)
        self.pixelw.pack(side="left", padx=2)
        send_pixelw = self._autowire(self.pixelw,
                                     lambda: self.worker.send("pixelw %s" % _spin(self.pixelw)))
        ttk.Button(row, text="Set", width=5, command=send_pixelw).pack(side="left", padx=4)

        row = ttk.Frame(g)
        row.pack(fill="x", pady=1)
        ttk.Label(row, text="Phase 0-%d" % PHASE_MAX).pack(side="left")
        self.phase = self._spinbox(row, 0, PHASE_MAX, 0, width=4)
        self.phase.pack(side="left", padx=4)
        send_phase = self._autowire(self.phase, lambda: self.worker.send("phase %s" % _spin(self.phase)))
        ttk.Button(row, text="Set", width=5, command=send_phase).pack(side="left")
        # self.set_enabled(row, False)

        row = ttk.Frame(g)
        row.pack(fill="x", pady=(4, 1))
        self.lock_row = row      # e.g. self.set_enabled(self.lock_row, False)
       
        # self.set_enabled(self.lock_row, False)

    # ---- sampling timing ----
    def _build_sampling(self, parent):
        g = self._group(parent, "Sampling timing")

        row = ttk.Frame(g)
        row.pack(fill="x", pady=1)
        ttk.Label(row, text="Fine tune ±%d (1kHz @8bpp, 0.5kHz @16bpp)" % FINE_TUNE_MAX_STEPS).pack(side="left")
        self.finetune = self._spinbox(row, -FINE_TUNE_MAX_STEPS, FINE_TUNE_MAX_STEPS, 0, width=6)
        self.finetune.pack(side="left", padx=4)
        send_ft = self._autowire(self.finetune, lambda: self.worker.send("finetune %s" % _spin(self.finetune)))
        ttk.Button(row, text="Set", width=5, command=send_ft).pack(side="left")

        row = ttk.Frame(g)
        row.pack(fill="x", pady=1)
        ttk.Label(row, text="Refresh Hz (1-255)").pack(side="left")
        self.refresh = self._spinbox(row, 1, 255, 50, width=6)
        self.refresh.pack(side="left", padx=4)
        send_refresh = self._autowire(self.refresh, lambda: self.worker.send("refresh %s" % _spin(self.refresh)))
        ttk.Button(row, text="Set", width=5, command=send_refresh).pack(side="left")

    # ---- AFE calibration ----
    def _build_calibration(self, parent):
        g = self._group(parent, "AFE calibration")

        def rgb_row(label, cmd, hi):
            row = ttk.Frame(g)
            row.pack(fill="x", pady=1)
            ttk.Label(row, text=label, width=8).pack(side="left")
            spinners = tuple(self._spinbox(row, 0, hi, 0, width=5) for _ in range(3))
            for s in spinners:
                s.pack(side="left", padx=1)
            send = lambda: self.worker.send(
                "%s %s,%s,%s" % (cmd, _spin(spinners[0]), _spin(spinners[1]), _spin(spinners[2])))
            for s in spinners:
                self._autowire(s, send)
            ttk.Button(row, text="Set", width=5, command=send).pack(side="left", padx=4)
            return spinners

        ttk.Label(g, text="            R      G      B").pack(anchor="w")
        self.gain_rgb = rgb_row("Gain", "gain", GAIN_MAX)
        self.offset_rgb = rgb_row("Offset", "offset", OFFSET_MAX)

        row = ttk.Frame(g)
        row.pack(fill="x", pady=1)
        ttk.Label(row, text="Neg offset", width=8).pack(side="left")
        self.negoffset = self._spinbox(row, 0, NEG_OFFSET_MAX, 0, width=5)
        self.negoffset.pack(side="left", padx=1)
        send_neg = self._autowire(self.negoffset,
                                  lambda: self.worker.send("negoffset %s" % _spin(self.negoffset)))
        ttk.Button(row, text="Set", width=5, command=send_neg).pack(side="left", padx=4)

        row = ttk.Frame(g)
        row.pack(fill="x", pady=(4, 1))
        ttk.Label(row, text="AFE reg idx,hex").pack(side="left")
        self.afe_idx = self._entry(row, "", 4)
        self.afe_idx.pack(side="left", padx=1)
        self.afe_val = self._entry(row, "", 4)
        self.afe_val.pack(side="left", padx=1)
        ttk.Button(row, text="Set", width=5, command=self.set_afereg).pack(side="left", padx=2)
        ttk.Button(row, text="Dump", width=5,
                   command=lambda: self.worker.send("afereg dump")).pack(side="left")


    def set_afereg(self):
        idx = self.afe_idx.get().strip()
        val = self.afe_val.get().strip()
        if not idx or not val:
            self.log("! AFE reg needs index and hex value")
            return
        self.worker.send("afereg %s,%s" % (idx, val))

    # ---- display slots + system ----
    def _build_display_system(self, parent):
        g = self._group(parent, "Display slot & system")

        row = ttk.Frame(g)
        row.pack(fill="x", pady=1)
        ttk.Label(row, text="Slot 1-%d" % DISPLAY_SLOT_MAX).pack(side="left")
        self.slot = self._spinbox(row, 1, DISPLAY_SLOT_MAX, 1, width=4)
        self.slot.pack(side="left", padx=4)
        send_slot = self._autowire(self.slot, lambda: self.worker.send("slot %s" % _spin(self.slot)))
        ttk.Button(row, text="Select", width=7, command=send_slot).pack(side="left")
        ttk.Button(row, text="Save", width=6,
                   command=lambda: self.worker.send("save")).pack(side="left", padx=2)

        row = ttk.Frame(g)
        row.pack(fill="x", pady=1)
        for label, cmd in (("Show", "show"), ("Mode", "mode"), ("ID", "id"), ("Version", "version")):
            ttk.Button(row, text=label, width=7,
                       command=lambda c=cmd: self.worker.send(c)).pack(side="left", padx=1)

        row = ttk.Frame(g)
        row.pack(fill="x", pady=(4, 1))
        ttk.Label(row, text="DVI").pack(side="left")
        ttk.Button(row, text="Start", width=5,
                   command=lambda: self.worker.send("display true")).pack(side="left", padx=1)
        ttk.Button(row, text="Stop", width=5,
                   command=lambda: self.worker.send("display false")).pack(side="left", padx=1)
        ttk.Label(row, text="  Info").pack(side="left")
        ttk.Button(row, text="On", width=4,
                   command=lambda: self.worker.send("info true")).pack(side="left", padx=1)
        ttk.Button(row, text="Off", width=4,
                   command=lambda: self.worker.send("info false")).pack(side="left", padx=1)
        
        self.set_enabled(row, False)

        row = ttk.Frame(g)
        row.pack(fill="x", pady=1)
        ttk.Label(row, text="TMDS drive,slew").pack(side="left")
        self.tmds_drive = self._spinbox(row, 0, 3, 0, width=3)
        self.tmds_drive.pack(side="left", padx=1)
        self.tmds_slew = self._spinbox(row, 0, 1, 0, width=3)
        self.tmds_slew.pack(side="left", padx=1)
        send_tmds = lambda: self.worker.send("tmds %s,%s" % (_spin(self.tmds_drive), _spin(self.tmds_slew)))
        self._autowire(self.tmds_drive, send_tmds)
        self._autowire(self.tmds_slew, send_tmds)
        ttk.Button(row, text="Set", width=5, command=send_tmds).pack(side="left", padx=2)

        self.set_enabled(row, False)

        row = ttk.Frame(g)
        row.pack(fill="x", pady=(4, 1))
        ttk.Label(row, text="USB console on boot").pack(side="left")
        ttk.Button(row, text="Enable", width=7,
                   command=lambda: self.worker.send("usb true")).pack(side="left", padx=1)
        ttk.Button(row, text="Disable (reboots)", width=15,
                   command=lambda: self.worker.send("usb false")).pack(side="left", padx=1)
        ttk.Button(row, text="Reboot", width=7,
                   command=self.confirm_reboot).pack(side="left", padx=6)

        self.set_enabled(row, False)


    def confirm_reboot(self):
        if messagebox.askyesno("Reboot", "Reboot the device now?"):
            self.worker.send("reboot")

    # ---- settings dump/restore/decode + video mode ----
    def _build_settings(self, parent):
        g = self._group(parent, "Settings & mode")
        row = ttk.Frame(g)
        row.pack(fill="x", pady=1)
        ttk.Button(row, text="Dump", width=6, command=self.dump_settings).pack(side="left")
        ttk.Button(row, text="Decode", width=7, command=self.decode_settings).pack(side="left", padx=2)
        ttk.Button(row, text="Restore from clipboard", command=self.restore_settings).pack(side="left", padx=2)

        row = ttk.Frame(g)
        row.pack(fill="x", pady=(4, 1))
        ttk.Label(row, text="Mode (reboots)").pack(side="left")
        ttk.Button(row, text="320x240@16", width=10,
                   command=lambda: self.switch_mode(16)).pack(side="left", padx=2)
        ttk.Button(row, text="640x240@8", width=10,
                   command=lambda: self.switch_mode(8)).pack(side="left", padx=2)
        
        self.set_enabled(g, False)

    def switch_mode(self, bpp):
        name = "320x240@16bpp" if bpp == 16 else "640x240@8bpp"
        if messagebox.askyesno("Switch mode", "Switch to %s?\nThe device saves and reboots." % name):
            self.worker.send("mode %d" % bpp)

    def dump_settings(self):
        """Fetch the settings blob; log it decoded and put the hex on the clipboard."""
        self.worker.send("dump", on_done=self._on_dump, quiet=True)

    def _on_dump(self, ok, text):
        blob = self._extract_blob(text)
        if not blob:
            self.log("! dump failed: %s" % (text or "no reply"))
            return
        self.last_settings_hex = blob
        self.root.clipboard_clear()
        self.root.clipboard_append(blob)
        self.log("Settings dump: %d bytes (hex copied to clipboard, keep it as a backup)" % (len(blob) // 2))
        self._log_decoded(blob)

    @staticmethod
    def _extract_blob(text):
        for ln in (text or "").splitlines():
            if ln.startswith("SETTINGS "):
                parts = ln.split()
                if len(parts) == 3 and len(parts[2]) == int(parts[1]) * 2:
                    return parts[2]
        return None

    def decode_settings(self):
        blob = getattr(self, "last_settings_hex", None)
        if blob:
            self._log_decoded(blob)
        else:
            self.dump_settings()

    def restore_settings(self):
        try:
            blob = self.root.clipboard_get()
        except tk.TclError:
            blob = ""
        blob = "".join(blob.split())
        if not blob or any(c not in "0123456789abcdefABCDEF" for c in blob):
            self.log("! Clipboard does not contain a settings hex dump")
            return
        if messagebox.askyesno("Restore settings",
                               "Write %d bytes of settings to the device?\n"
                               "It saves to flash and reboots." % (len(blob) // 2)):
            self.worker.send("restore %s" % blob)

    def _log_decoded(self, blob):
        try:
            self.log(decode_settings_blob(blob))
        except Exception as e:
            self.log("! settings decode error: %s" % e)

    # ---- diagnostics ----
    def _build_diagnostics(self, parent):
        g = self._group(parent, "Diagnostics")

        row = ttk.Frame(g)
        row.pack(fill="x", pady=1)
        ttk.Button(row, text="Levels", width=7,
                   command=lambda: self.worker.send("levels")).pack(side="left")
        ttk.Label(row, text="  Probe x,y,frames").pack(side="left")
        self.probe_x = self._entry(row, "0", 4)
        self.probe_x.pack(side="left", padx=1)
        self.probe_y = self._entry(row, "0", 4)
        self.probe_y.pack(side="left", padx=1)
        self.probe_n = self._entry(row, "50", 4)
        self.probe_n.pack(side="left", padx=1)
        ttk.Button(row, text="Run", width=5, command=self.run_probe).pack(side="left", padx=2)
        
        self.set_enabled(row, False)

    def run_probe(self):
        x = self.probe_x.get().strip() or "0"
        y = self.probe_y.get().strip() or "0"
        n = self.probe_n.get().strip() or "50"
        self.worker.send("probe %s,%s,%s" % (x, y, n))

    # ---- capture ----
    def _build_capture(self, parent):
        g = self._group(parent, "Capture")
        row = ttk.Frame(g)
        row.pack(fill="x")
        ttk.Button(row, text="Capture frame", command=self.capture).pack(side="left")
        self.save_btn = ttk.Button(row, text="Save PNG...", command=self.save_capture, state="disabled")
        self.save_btn.pack(side="left", padx=4)
        self.open_btn = ttk.Button(row, text="Open PNG", command=self.open_capture_popup, state="disabled")
        self.open_btn.pack(side="left", padx=4)
        self.copy_btn = ttk.Button(row, text="Copy CSV", command=self.copy_capture_csv, state="disabled")
        self.copy_btn.pack(side="left", padx=4)
        self.capture_note = ttk.Label(row, text="")
        self.capture_note.pack(side="left", padx=6)

    def capture(self):
        # Pause polling during the long capture so the poller can't interleave.
        self.capture_note.config(text="capturing...")
        self._poll_busy = True
        self.worker.send("capture", on_done=self._on_capture, quiet=True)

    def _on_capture(self, ok, text):
        self._poll_busy = False
        if not ok or not text:
            self.capture_note.config(text="capture failed")
            return
        self.last_capture_text = text          # raw CSV (header + hex rows)
        self.copy_btn.config(state="normal")
        try:
            img = csv2png.processRGBFromStrArray(text)
        except Exception as e:
            self.capture_note.config(text="decode error: %s (CSV still copyable)" % e)
            return
        self.last_image = img
        self.capture_note.config(text="%dx%d captured — click Open PNG to view" % (img.width, img.height))
        self.save_btn.config(state="normal")
        self.open_btn.config(state="normal")

    def copy_capture_csv(self):
        if not getattr(self, "last_capture_text", ""):
            self.log("No capture text to copy")
            return
        self.root.clipboard_clear()
        self.root.clipboard_append(self.last_capture_text)
        self.log("Capture CSV copied to clipboard (%d chars)" % len(self.last_capture_text))

    def open_capture_popup(self):
        if self.last_image is None:
            self.log("No capture to open")
            return
        top = tk.Toplevel(self.root)
        top.title("Capture  %dx%d" % (self.last_image.width, self.last_image.height))
        top.resizable(True, True)
        top.minsize(320, 240)
        from PIL import ImageTk
        # Cap the on-screen size but keep the image scrollable at native pixels.
        photo = ImageTk.PhotoImage(self.last_image)
        top._photo = photo                     # keep a ref so Tk doesn't GC it
        view_w = min(self.last_image.width, 1100)
        view_h = min(self.last_image.height, 800)
        canvas = tk.Canvas(top, width=view_w, height=view_h,
                           scrollregion=(0, 0, self.last_image.width, self.last_image.height))
        hbar = ttk.Scrollbar(top, orient="horizontal", command=canvas.xview)
        vbar = ttk.Scrollbar(top, orient="vertical", command=canvas.yview)
        canvas.configure(xscrollcommand=hbar.set, yscrollcommand=vbar.set)
        canvas.grid(row=0, column=0, sticky="nsew")
        vbar.grid(row=0, column=1, sticky="ns")
        hbar.grid(row=1, column=0, sticky="ew")
        top.rowconfigure(0, weight=1)
        top.columnconfigure(0, weight=1)
        canvas.create_image(0, 0, anchor="nw", image=photo)

    def save_capture(self):
        if self.last_image is None:
            return
        now = datetime.now().strftime("%Y_%m_%d-%H_%M_%S")
        f = asksaveasfile(initialfile="rgb2hdmi-%s.png" % now,
                          defaultextension=".png",
                          filetypes=[("PNG image", "*.png")])
        if f:
            path = os.path.abspath(f.name)
            f.close()
            self.last_image.save(path)
            self.log("Saved %s" % path)

    # ---- raw command (lives with the console, follows it when detached) ----
    def _build_raw(self, parent):
        self._build_raw_row(parent, embedded=True)

    def _build_raw_row(self, parent, embedded):
        row = ttk.Frame(parent)
        row.pack(fill="x", pady=(6, 2))
        ttk.Label(row, text="Raw command").pack(side="left")
        entry = ttk.Entry(row)
        entry.pack(side="left", fill="x", expand=True, padx=6)
        entry.bind("<Return>", lambda _e: self.send_raw())
        # Shell-style history: Up/Down browse past commands (shared between the
        # embedded and detached incarnations of the box).
        entry.bind("<Up>", lambda _e: self._raw_history_step(-1))
        entry.bind("<Down>", lambda _e: self._raw_history_step(+1))
        ttk.Button(row, text="Send", command=self.send_raw).pack(side="left")
        if embedded:
            self.raw_embedded = entry
        self.raw = entry
        return entry

    def send_raw(self):
        cmd = self.raw.get().strip()
        if not cmd:
            return
        self.worker.send(cmd)
        self.raw.delete(0, "end")
        if not self.raw_history or self.raw_history[-1] != cmd:
            self.raw_history.append(cmd)
            del self.raw_history[:-100]        # keep the last 100
        self.raw_hist_pos = None

    def _raw_history_step(self, direction):
        """Replace the raw entry content with the previous/next history item.
        Browsing starts from a saved draft of whatever was being typed, and
        stepping past the newest entry restores that draft."""
        if not self.raw_history:
            return "break"
        if self.raw_hist_pos is None:
            if direction > 0:
                return "break"                 # nothing newer to go to
            self.raw_draft = self.raw.get()
            self.raw_hist_pos = len(self.raw_history)
        self.raw_hist_pos += direction
        if self.raw_hist_pos < 0:
            self.raw_hist_pos = 0
        if self.raw_hist_pos >= len(self.raw_history):
            self.raw_hist_pos = None
            text = self.raw_draft
        else:
            text = self.raw_history[self.raw_hist_pos]
        self.raw.delete(0, "end")
        self.raw.insert(0, text)
        self.raw.icursor("end")
        return "break"

    # ---- console (attachable/detachable) ----
    def _build_console(self, parent):
        row = ttk.Frame(parent)
        row.pack(fill="x")
        ttk.Label(row, text="Console").pack(side="left")
        ttk.Button(row, text="Clear", command=self.clear_console).pack(side="right")
        ttk.Button(row, text="Detach", command=self.detach_console).pack(side="right", padx=4)
        self.console_embedded = ScrolledText(parent, height=8, state="disabled", wrap="word")
        self.console_embedded.pack(fill="both", expand=True, pady=(2, 0))
        self.console = self.console_embedded   # where log() writes right now

    def detach_console(self):
        """Pop the console out into its own resizable window. The embedded pane
        is hidden and logging is redirected to the new widget (with the
        scrollback carried over); closing the window re-attaches."""
        if self.console_win is not None:
            self.console_win.lift()
            return
        win = tk.Toplevel(self.root)
        win.title("pico-rgb2hdmi console")
        win.geometry("900x400")
        win.minsize(400, 150)
        row = ttk.Frame(win, padding=(6, 6, 6, 0))
        row.pack(fill="x")
        ttk.Label(row, text="Console").pack(side="left")
        ttk.Button(row, text="Clear", command=self.clear_console).pack(side="right")
        ttk.Button(row, text="Attach", command=self.attach_console).pack(side="right", padx=4)
        if not hasattr(self, "console_topmost"):
            self.console_topmost = tk.BooleanVar(value=False)
        ttk.Checkbutton(row, text="Always on top", variable=self.console_topmost,
                        command=lambda: win.attributes("-topmost", self.console_topmost.get())
                        ).pack(side="right", padx=4)
        win.attributes("-topmost", self.console_topmost.get())

        raw_holder = ttk.Frame(win, padding=(6, 0))
        raw_holder.pack(fill="x")
        pending = self.raw.get()
        detached_raw = self._build_raw_row(raw_holder, embedded=False)
        detached_raw.insert(0, pending)

        detached = ScrolledText(win, state="disabled", wrap="word")
        detached.pack(fill="both", expand=True, padx=6, pady=6)

        self._copy_console_text(self.console_embedded, detached)
        self.console = detached
        self.console_win = win
        self.paned.forget(self.bottom)
        win.protocol("WM_DELETE_WINDOW", self.attach_console)
        detached_raw.focus_set()

    def attach_console(self):
        """Bring the console back into the main window's bottom pane."""
        if self.console_win is None:
            return
        self._copy_console_text(self.console, self.console_embedded)
        self.console = self.console_embedded
        pending = self.raw.get()
        self.raw = self.raw_embedded
        self.raw.delete(0, "end")
        self.raw.insert(0, pending)
        win, self.console_win = self.console_win, None
        win.destroy()
        self.paned.add(self.bottom, weight=1)
        # macOS Tk can leave the app without keyboard focus after destroying a
        # focused Toplevel - the raw command entry then ignores typing until
        # focus is forced back.
        self.root.lift()
        self.root.focus_force()
        self.raw.focus_set()

    @staticmethod
    def _copy_console_text(src, dst):
        text = src.get("1.0", "end-1c")
        dst.config(state="normal")
        dst.delete("1.0", "end")
        dst.insert("end", text)
        dst.see("end")
        dst.config(state="disabled")

    def clear_console(self):
        self.console.config(state="normal")
        self.console.delete("1.0", "end")
        self.console.config(state="disabled")

    def log(self, text):
        # May be called from the worker thread -> marshal to the UI thread.
        _ui_call(self._log_ui, text)

    def _log_ui(self, text):
        self.console.config(state="normal")
        self.console.insert("end", text.rstrip("\n") + "\n")
        self.console.see("end")
        self.console.config(state="disabled")

    def on_close(self):
        self.worker.stop()
        self.root.destroy()


def _spin(spinbox):
    """Spinbox value as an int string, tolerating stray float text."""
    try:
        return str(int(float(spinbox.get())))
    except (ValueError, tk.TclError):
        return spinbox.get()


def parse_status(text):
    """Parse the firmware `status` line into a dict, or None if absent."""
    for ln in (text or "").splitlines():
        if ln.startswith("STATUS "):
            try:
                return dict(kv.split("=", 1) for kv in ln[7:].split())
            except ValueError:
                return None
    return None


def decode_settings_blob(hexstr):
    """Decode the firmware `dump` blob (settings_t minus the security key).

    Layout mirrors src/settings/settings.h of the matching firmware build:
      0   menu_colors[6]      6 x uint32
      24  menu_reserved[7]    7 x uint32
      52  capture_phase       uint32
      56  flags               uint16 (bit0 auto_shut_down, bits1-2 default_display)
      58                      uint8  (bit0 scan_line, bit1 symbols_per_word, bit2 usb_enabled)
      60  displays[4]         24 bytes each:
            +0  gain r,g,b       3 x uint16
            +6  offset r,g,b,neg 4 x uint8
            +10 v_front, v_back, h_front, h_back  4 x uint16
            +18 refresh          uint8 (+1 pad)
            +20 fine_tune        int32
      156 eof_canary          uint8
    """
    b = bytes.fromhex(hexstr)
    if len(b) < 160:
        raise ValueError("blob too short: %d bytes, expected 160" % len(b))
    phase, = struct.unpack_from("<I", b, 52)
    flags16, = struct.unpack_from("<H", b, 56)
    flags8 = b[58]
    mode = "320x240@16bpp" if (flags8 >> 1) & 1 else "640x240@8bpp"
    lines = [
        "Mode: %s   default slot: %d   usb on boot: %s" % (
            mode, ((flags16 >> 1) & 3) + 1, "yes" if (flags8 >> 2) & 1 else "no"),
        "Scanline effect: %s   auto shutdown: %s   capture phase: %d/12" % (
            "on" if flags8 & 1 else "off", "on" if flags16 & 1 else "off", phase),
    ]
    for i in range(4):
        o = 60 + i * 24
        gain = struct.unpack_from("<3H", b, o)
        off = struct.unpack_from("<4B", b, o + 6)
        vf, vb, hf, hb = struct.unpack_from("<4H", b, o + 10)
        refresh = b[o + 18]
        ft, = struct.unpack_from("<i", b, o + 20)
        lines.append(
            "Slot %d: gain %d,%d,%d  offset %d,%d,%d neg %d  "
            "H porch %d/%d  V porch %d/%d  refresh %dHz  finetune %d" % (
                i + 1, gain[0], gain[1], gain[2], off[0], off[1], off[2], off[3],
                hf, hb, vf, vb, refresh, ft))
    canary = b[156]
    lines.append("Canary: 0x%02X (%s)" % (canary, "valid" if canary == 0 else "INVALID - factory reset on boot"))
    return "\n".join(lines)


def main():
    parser = argparse.ArgumentParser(description="pico-rgb2hdmi control panel")
    parser.add_argument("-v", "--verbose", action="store_true",
                        help="print extra logs (connection details, idle checks, probes)")
    parser.add_argument("-p", "--port", default=DEFAULT_PORT,
                        help="default serial port shown in the Port box (default: %(default)s)")
    parser.add_argument("--no-scan", action="store_true",
                        help="start with auto-detect disabled (open the port directly)")
    args = parser.parse_args()

    root = tk.Tk()
    app = App(root, verbose=args.verbose, default_port=args.port)
    if args.no_scan:
        app.no_scan.set(True)
    root.mainloop()


if __name__ == "__main__":
    main()
