"""
MSB LUMES JIG – Test Session Logger v5.1
- Passive: connects, listens, never sends anything.
- START (0xA0) → begin recording session.
- DATA  (0x10) → accumulate frames, update Live tab.
- END   (0xA1) → finalise session, push to Test Result accordion.
- System Log: only shows serial-level events (connect/errors), NOT protocol events.
- Test Result: collapsible accordion; unlimited sessions kept in memory.
"""

import customtkinter as ctk
import tkinter.ttk as ttk
import serial.tools.list_ports
from serial_manager import SerialManager
from protocol import JigProtocol
import time

# ── Palette ───────────────────────────────────────────────────────────────────
CH_COLORS  = ["#2b9ede", "#00cc88", "#f0a500", "#cc44aa"]
REL_COLORS = ["#00d4ff", "#00ff88", "#ffcc00"]
REL_NAMES  = ["A", "B", "C"]

GAIN_EXPECTED = 0x333300   # x16 correct


# ─────────────────────────────────────────────────────────────────────────────
# Accordion Session Card
# ─────────────────────────────────────────────────────────────────────────────
# ─────────────────────────────────────────────────────────────────────────────
# Custom Table Widget (Per-Cell Highlighting Support)
# ─────────────────────────────────────────────────────────────────────────────
class JigTable(ctk.CTkFrame):
    """A high-performance text-based table with dual-axis zoom (Font & Width)."""
    def __init__(self, master, columns, title="", **kwargs):
        super().__init__(master, fg_color="transparent", **kwargs)
        self.columns = columns
        self.col_w   = 8  
        self.font_size = 11
        self._frames_ref = [] # Local ref for re-rendering
        
        if title:
            ctk.CTkLabel(self, text=title, font=ctk.CTkFont(size=11, weight="bold"), text_color="#777").pack(pady=(2,0))

        self.txt = ctk.CTkTextbox(self, fg_color="#0a0a12", text_color="#ccc",
                                    font=ctk.CTkFont(family="Consolas", size=self.font_size),
                                    wrap="none", corner_radius=0)
        self.txt.pack(fill="both", expand=True)
        
        self._update_tag_config()
        self._write_header()

    def _update_tag_config(self):
        self.txt.tag_config("fail", foreground="#ff4444")
        self.txt.tag_config("hdr",  foreground="#2b9ede")

    def _write_header(self):
        self.txt.configure(state="normal")
        h_str = ""
        for i, col in enumerate(self.columns):
            w = (self.col_w + 2) if "Gain" in col else self.col_w
            if col in ("Frm#", "T(s)"): w = self.col_w - 3
            if w < 4: w = 4
            h_str += col.ljust(w)
        self.txt.insert("end", h_str + "\n", "hdr")
        self.txt.configure(state="disabled")

    def set_zoom(self, font_size=None, col_width=None):
        """Update zoom and re-render if width changed."""
        re_render_needed = False
        if font_size is not None:
            self.font_size = font_size
            self.txt.configure(font=ctk.CTkFont(family="Consolas", size=font_size))
        
        if col_width is not None and col_width != self.col_w:
            self.col_w = col_width
            re_render_needed = True

        return re_render_needed

    def append_row(self, values, highlight_indices=None):
        self.txt.configure(state="normal")
        if highlight_indices is None: highlight_indices = []

        pos = self.txt.index("end-1c")
        line_no = pos.split(".")[0]
        
        current_line_str = ""
        cell_positions = [] 
        
        for i, val in enumerate(values):
            col_name = self.columns[i]
            w = (self.col_w + 2) if "Gain" in col_name else self.col_w
            if col_name in ("Frm#", "T(s)"): w = self.col_w - 3
            if w < 4: w = 4
            
            s = str(val).ljust(w)
            start_pos = len(current_line_str)
            current_line_str += s
            end_pos   = len(current_line_str)
            cell_positions.append((start_pos, end_pos))

        self.txt.insert("end", current_line_str + "\n")
        
        for col_idx in highlight_indices:
            if col_idx < len(cell_positions):
                sp, ep = cell_positions[col_idx]
                self.txt.tag_add("fail", f"{line_no}.{sp}", f"{line_no}.{ep}")

        self.txt.see("end")
        self.txt.configure(state="disabled")

    def clear(self):
        self.txt.configure(state="normal")
        self.txt.delete("1.0", "end")
        self.txt.configure(state="disabled")
        self._write_header()





# ─────────────────────────────────────────────────────────────────────────────
# Main Application
# ─────────────────────────────────────────────────────────────────────────────
class LumesJigApp(ctk.CTk):
    def __init__(self):
        super().__init__()
        self.title("MSB LUMES JIG – Test Session Logger v5.1")
        self.geometry("1500x960")
        ctk.set_appearance_mode("dark")

        style = ttk.Style()
        style.theme_use("clam")
        style.configure("Treeview", background="#1e1e1e", foreground="white",
                        fieldbackground="#1e1e1e", borderwidth=0, rowheight=22)
        style.map("Treeview", background=[("selected", "#2b719e")])
        style.configure("Treeview.Heading", background="#111", foreground="#aaa", relief="flat")

        # Session state
        self._session_frames   = []
        self._in_session       = False
        self._session_start    = 0.0
        self._session_no       = 0       # auto-incremented per session
        self._all_sessions     = []      # list of dicts {frames, timestamp, duration}

        self.serial_manager = SerialManager(self._on_packet, self._sys_log)
        self._build_ui()
        self._refresh_ports()
        self._tick_status()

    # ──────────────────────────────────────────────
    # UI
    # ──────────────────────────────────────────────
    def _build_ui(self):
        # Sidebar
        sb = ctk.CTkFrame(self, width=210, corner_radius=0, fg_color="#0e0e18")
        sb.pack(side="left", fill="y")
        sb.pack_propagate(False)

        ctk.CTkLabel(sb, text="LUMES JIG",
                     font=ctk.CTkFont(size=20, weight="bold"),
                     text_color="#2b9ede").pack(pady=(25, 2))
        ctk.CTkLabel(sb, text="Session Logger v5.1",
                     font=ctk.CTkFont(size=10), text_color="#555").pack()
        ctk.CTkFrame(sb, height=1, fg_color="#222").pack(fill="x", padx=16, pady=14)

        self._status_lbl = ctk.CTkLabel(sb, text="● DISCONNECTED",
                                         font=ctk.CTkFont(size=12, weight="bold"),
                                         text_color="#444")
        self._status_lbl.pack(pady=4)
        self._session_badge = ctk.CTkLabel(sb, text="",
                                             font=ctk.CTkFont(size=10), text_color="#ffd700")
        self._session_badge.pack()
        ctk.CTkFrame(sb, height=1, fg_color="#222").pack(fill="x", padx=16, pady=10)

        port_f = ctk.CTkFrame(sb, fg_color="transparent")
        port_f.pack(padx=12, fill="x")
        self._port_var = ctk.StringVar()
        self._port_cb  = ctk.CTkComboBox(port_f, variable=self._port_var, values=[], width=120)
        self._port_cb.pack(side="left")
        ctk.CTkButton(port_f, text="🔄", width=34, fg_color="#222",
                      command=self._refresh_ports).pack(side="right", padx=(4, 0))

        self._btn_connect = ctk.CTkButton(sb, text="CONNECT",
                                           fg_color="#1a5fa0",
                                           command=self._toggle_connect)
        self._btn_connect.pack(pady=10, padx=12, fill="x")

        self._btn_mock = ctk.CTkButton(sb, text="MOCK UP DATA",
                                         fg_color="#444", text_color="#aaa",
                                         command=self._load_mock_data)
        self._btn_mock.pack(pady=(0, 10), padx=12, fill="x")

        ctk.CTkFrame(sb, height=1, fg_color="#222").pack(fill="x", padx=16, pady=8)

        self._frame_lbl = ctk.CTkLabel(sb, text="Sessions: 0  |  Frames: 0",
                                        font=ctk.CTkFont(family="Consolas", size=10),
                                        text_color="#666")
        self._frame_lbl.pack()

        ctk.CTkLabel(sb, text="FONT SIZE", font=ctk.CTkFont(size=10, weight="bold"), text_color="#555").pack(pady=(12,0))
        self._zoom_v = ctk.CTkSlider(sb, from_=8, to=24, number_of_steps=16, height=16, command=self._on_zoom_v)
        self._zoom_v.set(11)
        self._zoom_v.pack(padx=10, pady=(2,4))

        ctk.CTkLabel(sb, text="COL WIDTH", font=ctk.CTkFont(size=10, weight="bold"), text_color="#555").pack(pady=(4,0))
        self._zoom_h = ctk.CTkSlider(sb, from_=5, to=20, number_of_steps=15, height=16, command=self._on_zoom_h)
        self._zoom_h.set(8)
        self._zoom_h.pack(padx=10, pady=(2,12))

        ctk.CTkLabel(sb,
                     text="Passive listener.\nFirmware sends:\nSTART → DATA → END",
                     font=ctk.CTkFont(size=9), text_color="#333",
                     justify="center").pack(side="bottom", pady=20)

        # Tabs
        self._tabs = ctk.CTkTabview(self, fg_color="#111118")
        self._tabs.pack(side="right", fill="both", expand=True, padx=6, pady=6)

        self._tab_live    = self._tabs.add("LIVE")
        self._tab_summary = self._tabs.add("CALIBRATION")
        self._tab_log     = self._tabs.add("SESSION LOG")
        self._tab_history = self._tabs.add("TEST HISTORY")
        self._tab_syslog  = self._tabs.add("SYSTEM LOGS")

        self._build_live_tab()
        self._build_summary_tab()
        self._build_log_tab()
        self._build_history_tab()
        self._build_syslog_tab()

    # ── Summary Tab ────────────────────────────
    def _build_summary_tab(self):
        scroll = ctk.CTkScrollableFrame(self._tab_summary, fg_color="transparent")
        scroll.pack(fill="both", expand=True, padx=4, pady=4)
        
        self._summary_cards = []
        for ch in range(4):
            card = ctk.CTkFrame(scroll, border_width=1, border_color="#222",
                                fg_color="#0d0d1a", corner_radius=10)
            card.pack(fill="x", padx=10, pady=8)
            
            # Header
            hdr = ctk.CTkFrame(card, fg_color="#1a1a2e", corner_radius=8, height=36)
            hdr.pack(fill="x", padx=2, pady=2)
            hdr.pack_propagate(False)
            
            ctk.CTkLabel(hdr, text=f"CHANNEL {ch+1} SUMMARY", 
                         font=ctk.CTkFont(size=14, weight="bold"),
                         text_color=CH_COLORS[ch]).pack(side="left", padx=12)
            
            res_lbl = ctk.CTkLabel(hdr, text="WAITING...", font=ctk.CTkFont(size=12, weight="bold"), text_color="#555")
            res_lbl.pack(side="right", padx=12)
            
            # Content Grid
            body = ctk.CTkFrame(card, fg_color="transparent")
            body.pack(fill="x", padx=15, pady=10)
            
            # Left side: Status checks
            col_l = ctk.CTkFrame(body, fg_color="transparent")
            col_l.pack(side="left", fill="both", expand=True)
            
            checks = {}
            for i, label_txt in enumerate(["Zero Detect", "Calibration Range", "EEPROM Write"]):
                row = ctk.CTkFrame(col_l, fg_color="transparent")
                row.pack(fill="x", pady=2)
                ctk.CTkLabel(row, text=f"• {label_txt}:", font=ctk.CTkFont(size=11), text_color="#aaa").pack(side="left")
                val = ctk.CTkLabel(row, text="---", font=ctk.CTkFont(size=11, weight="bold"), text_color="#444")
                val.pack(side="right", padx=10)
                checks[label_txt] = val
                
            # Right side: Relay mechanical checks
            col_r = ctk.CTkFrame(body, fg_color="transparent")
            col_r.pack(side="right", fill="both", expand=True)
            relays = []
            for r in range(3):
                row = ctk.CTkFrame(col_r, fg_color="transparent")
                row.pack(fill="x", pady=2)
                ctk.CTkLabel(row, text=f"Relay {REL_NAMES[r]}:", font=ctk.CTkFont(size=11), text_color="#aaa").pack(side="left")
                val = ctk.CTkLabel(row, text="---", font=ctk.CTkFont(size=11, weight="bold"), text_color="#444")
                val.pack(side="right", padx=10)
                relays.append(val)
                
            # Bottom: Calibration Factors Table
            k_f = ctk.CTkFrame(card, fg_color="#05050a", corner_radius=6)
            k_f.pack(fill="x", padx=10, pady=(0,10))
            
            k_txt = ctk.CTkTextbox(k_f, height=80, font=ctk.CTkFont(family="Consolas", size=11), fg_color="transparent", text_color="#88ccee")
            k_txt.pack(fill="x", padx=5, pady=5)
            k_txt.insert("1.0", "Calibration Factors (K) will appear here after end of test.")
            k_txt.configure(state="disabled")
            
            self._summary_cards.append({
                "header": res_lbl,
                "checks": checks,
                "relays": relays,
                "k_box": k_txt
            })

    # ── Live Tab ──────────────────────────────────
    def _build_live_tab(self):
        # Use a scrollable frame so all 4 channels fit on any screen size
        scroll = ctk.CTkScrollableFrame(self._tab_live, fg_color="transparent", label_text="")
        scroll.pack(fill="both", expand=True, padx=2, pady=2)

        grid = ctk.CTkFrame(scroll, fg_color="transparent")
        grid.pack(fill="both", expand=True, padx=4, pady=4)
        self._live_cards = []
        for ch in range(4):
            card = ctk.CTkFrame(grid, border_width=1, border_color="#222",
                                fg_color="#0d0d1a", corner_radius=10)
            card.grid(row=ch//2, column=ch%2, padx=6, pady=6, sticky="nsew")
            grid.grid_columnconfigure(ch%2, weight=1)
            grid.grid_rowconfigure(ch//2, weight=1)

            hdr = ctk.CTkFrame(card, fg_color="#111128", corner_radius=8, height=40)
            hdr.pack(fill="x", padx=3, pady=3)
            hdr.pack_propagate(False)
            ctk.CTkLabel(hdr, text=f"CHANNEL {ch+1}",
                         font=ctk.CTkFont(size=16, weight="bold"),
                         text_color=CH_COLORS[ch]).pack(side="left", padx=12)
            g_lbl = ctk.CTkLabel(hdr, text="GAIN: ──────",
                                  font=ctk.CTkFont(family="Consolas", size=13),
                                  text_color="#776699")
            g_lbl.pack(side="left", padx=6)
            v_lbl = ctk.CTkLabel(hdr, text="─── V",
                                  font=ctk.CTkFont(family="Consolas", size=26, weight="bold"),
                                  text_color="#00ffcc")
            v_lbl.pack(side="right", padx=12)

            tbl = ctk.CTkFrame(card, fg_color="transparent")
            tbl.pack(fill="both", expand=True, padx=12, pady=6)
            rows = []
            for r in range(3):
                ctk.CTkLabel(tbl, text=f"{ch+1}{REL_NAMES[r]}",
                              font=ctk.CTkFont(size=13, weight="bold"),
                              text_color=REL_COLORS[r]).grid(row=r, column=0, padx=8, pady=8)
                i_lbl = ctk.CTkLabel(tbl, text="─── mA",
                                      font=ctk.CTkFont(family="Consolas", size=28, weight="bold"))
                i_lbl.grid(row=r, column=1, padx=15)
                p_lbl = ctk.CTkLabel(tbl, text="─── W",
                                      font=ctk.CTkFont(family="Consolas", size=28, weight="bold"))
                p_lbl.grid(row=r, column=2, padx=15)
                rows.append({"v": v_lbl, "g": g_lbl, "i": i_lbl, "p": p_lbl})
            self._live_cards.append(rows)

    # ── Session Log Tab ────────────────────────────
    def _build_log_tab(self):
        top = ctk.CTkFrame(self._tab_log, fg_color="transparent", height=34)
        top.pack(fill="x", padx=10, pady=(6, 2))
        self._log_status_lbl = ctk.CTkLabel(
            top, text="No session yet.", font=ctk.CTkFont(size=11), text_color="#555")
        self._log_status_lbl.pack(side="left")
        ctk.CTkButton(top, text="Clear", width=70, height=26, fg_color="#222",
                      command=self._clear_log).pack(side="right")

        # Sub-tabs for each channel to make it easy to read
        self._log_tabs = ctk.CTkTabview(self._tab_log, fg_color="#0a0a12", height=40)
        self._log_tabs.pack(fill="both", expand=True, padx=4, pady=4)
        
        self._log_tables = []
        cols = ("Frm#", "T(s)", "Voltage", "IA(mA)", "PA(W)", "IB(mA)", "PB(W)", "IC(mA)", "PC(W)", "Gain")
        
        for ch in range(4):
            nm = f"CHANNEL {ch+1}"
            t = self._log_tabs.add(nm)
            tbl = JigTable(t, cols)
            tbl.pack(fill="both", expand=True)
            self._log_tables.append(tbl)

    # ── Test History Tab (Master-Detail) ──────────
    def _build_history_tab(self):
        paned = ttk.PanedWindow(self._tab_history, orient="vertical")
        paned.pack(fill="both", expand=True, padx=4, pady=4)

        # 1. Master: Session List
        master_f = ctk.CTkFrame(paned, fg_color="transparent")
        paned.add(master_f, weight=1)

        hdr = ctk.CTkFrame(master_f, fg_color="#1a1a2e", height=32)
        hdr.pack(fill="x")
        ctk.CTkLabel(hdr, text="PAST SESSIONS", font=ctk.CTkFont(size=11, weight="bold")).pack(side="left", padx=10)
        ctk.CTkButton(hdr, text="CLEAR ALL", width=70, height=20, fg_color="#441111", 
                       command=self._clear_results).pack(side="right", padx=10)

        cols = ("ID", "Timestamp", "Frames", "Duration", "Gain Status")
        self._hist_list = ttk.Treeview(master_f, columns=cols, show="headings", height=8)
        self._hist_list.heading("ID", text="#")
        self._hist_list.heading("Timestamp", text="Date/Time")
        self._hist_list.heading("Frames", text="Frames")
        self._hist_list.heading("Duration", text="Time(s)")
        self._hist_list.heading("Gain Status", text="Gain Summary")
        
        self._hist_list.column("ID", width=40, anchor="center")
        self._hist_list.column("Timestamp", width=180, anchor="center")
        self._hist_list.column("Frames", width=100, anchor="center")
        self._hist_list.column("Duration", width=100, anchor="center")
        self._hist_list.column("Gain Status", width=200, anchor="center")
        
        self._hist_list.pack(fill="both", expand=True)
        self._hist_list.bind("<<TreeviewSelect>>", self._on_history_select)

        # 2. Detail: Frame Log for selected session
        detail_f = ctk.CTkFrame(paned, fg_color="transparent")
        paned.add(detail_f, weight=3)

        hdr_d = ctk.CTkFrame(detail_f, fg_color="#0e0e18", height=32)
        hdr_d.pack(fill="x")
        self._hist_detail_lbl = ctk.CTkLabel(hdr_d, text="SELECT A SESSION TO VIEW DETAILS", 
                                             font=ctk.CTkFont(size=11, weight="bold"), text_color="#777")
        self._hist_detail_lbl.pack(side="left", padx=10)

        self._hist_tabs = ctk.CTkTabview(detail_f, fg_color="#0a0a12", height=40)
        self._hist_tabs.pack(fill="both", expand=True, padx=4, pady=2)
        
        self._hist_tables = []
        cols = ("Frm#", "T(s)", "Voltage", "IA(mA)", "PA(W)", "IB(mA)", "PB(W)", "IC(mA)", "PC(W)", "Gain")
        for ch in range(4):
            t = self._hist_tabs.add(f"CH {ch+1}")
            tbl = JigTable(t, cols)
            tbl.pack(fill="both", expand=True)
            self._hist_tables.append(tbl)

    def _on_history_select(self, _e):
        sel = self._hist_list.selection()
        if not sel: return
        
        selected_item = self._hist_list.item(sel[0])
        # Values[0] is the 1-based ID
        idx = int(selected_item["values"][0]) - 1
        if idx < 0 or idx >= len(self._all_sessions): return

        session = self._all_sessions[idx]
        self._hist_detail_lbl.configure(text=f"DETAILS: SESSION #{idx+1:03d} ({session['timestamp']})", text_color="#aaccff")
        
        # Clear & Repopulate detail tables
        for tbl in self._hist_tables:
            tbl.clear()

        for i, frame in enumerate(session['frames']):
            ts = frame.get("ts_offset", 0)
            ch = frame["channel"]
            row, errs = self._format_frame_row(frame, i, ts, ch)
            self._hist_tables[ch].append_row(row, errs)

    def _format_frame_row(self, pkt, index, ts, channel):
        v = pkt["voltage"]; cs = pkt["currents"]
        ps = pkt["powers"];   g = pkt["gain"]
        mask = pkt.get("relay_mask", 0)
        
        row = [index, f"{ts:.1f}"]
        errs = []
        
        # V
        row.append(f"{v:.2f}")
        # I & P for 3 relays
        for r in range(3):
            status = JigProtocol.row_status(mask, channel, r, cs[r], ps[r])
            row.append(f"{cs[r]:.1f}")
            if status in ('fail_low', 'fail_leak'): errs.append(len(row)-1)
            row.append(f"{ps[r]:.2f}")
            if status in ('fail_low', 'fail_leak'): errs.append(len(row)-1)
        # G
        row.append(f"0x{g:06X}")
        if v > 50.0 and (g & 0xFFFF00) != GAIN_EXPECTED:
            errs.append(len(row)-1)

        return row, errs

    # ── System Logs Tab ────────────────────────────
    def _build_syslog_tab(self):
        self._syslog = ctk.CTkTextbox(
            self._tab_syslog, font=ctk.CTkFont(family="Consolas", size=11))
        self._syslog.pack(fill="both", expand=True, padx=10, pady=10)

    # ──────────────────────────────────────────────
    # Helpers
    # ──────────────────────────────────────────────
    def _sys_log(self, msg):
        """Only serial-level messages (connect/error). NOT protocol events."""
        self._syslog.insert("end", time.strftime("[%H:%M:%S] ") + msg + "\n")
        self._syslog.see("end")

    def _refresh_ports(self):
        ports = [p.device for p in serial.tools.list_ports.comports()]
        self._port_cb.configure(values=ports)
        if ports:
            self._port_var.set(ports[0])

    def _toggle_connect(self):
        if not self.serial_manager.running:
            self.serial_manager.connect(self._port_var.get())
        else:
            self.serial_manager.disconnect()

    def _tick_status(self):
        health = self.serial_manager.is_healthy()
        if not self.serial_manager.running:
            self._status_lbl.configure(text="● DISCONNECTED", text_color="#444")
            self._btn_connect.configure(text="CONNECT", fg_color="#1a5fa0")
        elif self._in_session:
            self._status_lbl.configure(text="● RECORDING…", text_color="#ffd700")
            self._btn_connect.configure(text="DISCONNECT", fg_color="#6b1a1a")
        elif health is None:
            self._status_lbl.configure(text="● LISTENING…", text_color="#557799")
            self._btn_connect.configure(text="DISCONNECT", fg_color="#6b1a1a")
        elif health:
            self._status_lbl.configure(text="● CONNECTED", text_color="#00cc88")
            self._btn_connect.configure(text="DISCONNECT", fg_color="#6b1a1a")
        else:
            self._status_lbl.configure(text="● NO DATA", text_color="#cc3333")
        self.after(400, self._tick_status)

    def _update_frame_lbl(self):
        total_frames = sum(len(s["frames"]) for s in self._all_sessions) + len(self._session_frames)
        self._frame_lbl.configure(
            text=f"Sessions: {self._session_no}  |  Frames: {total_frames}")

    def _load_mock_data(self):
        """Generates a complete fake test session to verify UI."""
        self._sys_log("Simulating mock test session...")
        self._on_test_start()
        
        # 1. Simulate 5 data frames
        import random
        for i in range(12): # simulate 12 packets (3 per channel)
            ch = i % 4
            mock_pkt = {
                "type": "ch_data",
                "channel": ch,
                "voltage": 220.5 + random.uniform(-1, 1),
                "currents": [500.0 + random.uniform(-5, 5) for _ in range(3)],
                "powers":   [110.0 + random.uniform(-2, 2) for _ in range(3)],
                "gain":     0x333300,
                "relay_mask": 0b111000111000
            }
            self._on_ch_data_frame(mock_pkt)
            self.update()
            time.sleep(0.1)

        # 2. Simulate summary frame
        mock_summary = {
            "type":      "summary",
            "zcd_ok":    [1, 1, 0, 1],
            "calib_ok":  [1, 1, 1, 1],
            "eeprom_ok": [1, 1, 1, 0],
            "relay_ok":  [1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1],
            "zcd_counts":[100, 101, 45, 99],
            "k_u":       [1050, 1048, 1052, 1049],
            "k_i":       [1010 + random.randint(0,20) for _ in range(12)],
            "k_p":       [1020 + random.randint(0,20) for _ in range(12)],
            "cnt_u":     [5, 5, 5, 5],
            "cnt_i":     [5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5]
        }
        self._on_summary_frame(mock_summary)
        
        # 3. End session
        self._on_test_end()
        self._sys_log("Mock session complete (Check CALIBRATION tab).")

    # ──────────────────────────────────────────────
    # Packet Routing (called from serial thread → after() to main thread)
    # ──────────────────────────────────────────────
    def _on_packet(self, pkt):
        t = pkt.get("type")
        if t == "event":
            cmd = pkt["cmd"]
            if cmd == JigProtocol.CMD_TEST_START:
                self.after(0, self._on_test_start)
            elif cmd == JigProtocol.CMD_TEST_END:
                self.after(0, self._on_test_end)
            # Do NOT write START/END to sys log
        elif t == "ch_data":
            self.after(0, lambda p=pkt: self._on_ch_data_frame(p))
        elif t == "summary":
            self.after(0, lambda p=pkt: self._on_summary_frame(p))

    # ──────────────────────────────────────────────
    # Session lifecycle
    # ──────────────────────────────────────────────
    def _on_test_start(self):
        self._session_frames.clear()
        self._in_session    = True
        self._session_start = time.time()
        self._session_badge.configure(text="▶ RECORDING")
        self._log_status_lbl.configure(text="● Session in progress…", text_color="#ffd700")
        self._clear_log()
        self._tabs.set("SESSION LOG")

    def _on_ch_data_frame(self, pkt):
        elapsed = time.time() - (self._session_start or time.time())
        if self._in_session:
            pkt["_ts"] = elapsed
            self._session_frames.append(pkt)
            self._append_log_row(pkt, elapsed)
        self._update_live(pkt)
        self._update_frame_lbl()

    def _on_summary_frame(self, pkt):
        """Update the CALIBRATION tab with detailed results."""
        self._tabs.set("CALIBRATION")
        for ch in range(4):
            card = self._summary_cards[ch]
            
            # ZCD, Calib Range, EEPROM
            z_ok = pkt["zcd_ok"][ch] == 1
            z_val = pkt.get("zcd_counts", [0,0,0,0])[ch]
            c_ok = pkt["calib_ok"][ch] == 1
            e_ok = pkt["eeprom_ok"][ch] == 1
            
            z_txt = f"{'PASS' if z_ok else 'FAIL'} ({z_val})"
            card["checks"]["Zero Detect"].configure(text=z_txt, text_color="#00ff88" if z_ok else "#ff4444")
            card["checks"]["Calibration Range"].configure(text="PASS" if c_ok else "FAIL", text_color="#00ff88" if c_ok else "#ff4444")
            card["checks"]["EEPROM Write"].configure(text="PASS" if e_ok else "FAIL", text_color="#00ff88" if e_ok else "#ff4444")
            
            # Relays
            all_r_ok = True
            for r in range(3):
                idx = ch * 3 + r
                r_ok = pkt["relay_ok"][idx] == 1
                if not r_ok: all_r_ok = False
                card["relays"][r].configure(text="OK" if r_ok else "FAIL", text_color="#00ff88" if r_ok else "#ff4444")
            
            # Final Status
            overall = z_ok and c_ok and e_ok and all_r_ok
            card["header"].configure(text="[ PASS ]" if overall else "[ FAIL ]", text_color="#00ff88" if overall else "#ff4444")
            
            # Calibration Table (K-factors & Counts)
            k_box = card["k_box"]
            k_box.configure(state="normal")
            k_box.delete("1.0", "end")
            
            k_box.insert("end", f"K-Voltage: {pkt['k_u'][ch]:<8} (Count: {pkt['cnt_u'][ch]})\n")
            line_i = "K-Current: "
            line_p = "K-Power:   "
            line_c = "Counts I:  "
            for r in range(3):
                idx = ch * 3 + r
                line_i += f"{REL_NAMES[r]}:{pkt['k_i'][idx]:<6} "
                line_p += f"{REL_NAMES[r]}:{pkt['k_p'][idx]:<6} "
                line_c += f"{REL_NAMES[r]}:{pkt['cnt_i'][idx]:<6} "
            
            k_box.insert("end", line_i + "\n" + line_p + "\n" + line_c)
            k_box.configure(state="disabled")

    def _on_test_end(self):
        self._in_session = False
        self._session_no += 1
        duration = time.time() - self._session_start
        session = {
            "frames":    list(self._session_frames),
            "timestamp": time.strftime("%H:%M:%S"),
            "duration":  duration,
        }
        self._all_sessions.append(session)
        self._session_badge.configure(text=f"■ Done (#{self._session_no})")
        self._log_status_lbl.configure(
            text=f"● Session #{self._session_no} — {len(self._session_frames)} frames, {duration:.0f}s",
            text_color="#00cc88")
        self._push_result_card(session)
        self._update_frame_lbl()
        self._tabs.set("TEST HISTORY")

    # ──────────────────────────────────────────────
    # Live Tab
    # ──────────────────────────────────────────────
    def _update_live(self, pkt):
        ch = pkt["channel"]
        v = pkt["voltage"]; cs = pkt["currents"]
        ps = pkt["powers"];  g = pkt["gain"]
        mask = pkt.get("relay_mask", 0)
        
        rows = self._live_cards[ch]
        
        # Smart Gain coloring
        gain_ok = (v < 50.0) or ((g & 0xFFFF00) == GAIN_EXPECTED)
        g_text  = f"GAIN: 0x{g:06X}" + ("" if gain_ok else " ⚠")
        g_color = "#00cc88" if gain_ok else "#ff4444"
        
        rows[0]["v"].configure(text=f"{v:.2f} V")
        rows[0]["g"].configure(text=g_text, text_color=g_color)
        
        for r in range(3):
            status = JigProtocol.row_status(mask, ch, r, cs[r], ps[r])
            
            txt_color = "#ffffff"
            if status in ('fail_low', 'fail_leak'):
                txt_color = "#ff4444"
            elif status == 'ok':
                txt_color = "#00ff88"
            else:
                txt_color = "#777777"
            
            rows[r]["i"].configure(text=f"{cs[r]:.1f} mA", text_color=txt_color)
            rows[r]["p"].configure(text=f"{ps[r]:.2f} W",   text_color=txt_color)

    # ──────────────────────────────────────────────
    # Session Log Tab
    # ──────────────────────────────────────────────
    def _append_log_row(self, pkt, ts):
        n = len(self._session_frames)
        pkt["ts_offset"] = ts
        ch = pkt["channel"]
        row, errs = self._format_frame_row(pkt, n, ts, ch)
        self._log_tables[ch].append_row(row, errs)

    def _clear_log(self):
        for tbl in self._log_tables:
            tbl.clear()

    # ──────────────────────────────────────────────
    # Test History Push
    # ──────────────────────────────────────────────
    def _push_result_card(self, session):
        n = len(self._all_sessions)
        frames = session["frames"]
        ts = session["timestamp"]
        dur = f"{session['duration']:.0f}"
        
        # Calculate Gain Summary for master list
        err_count = 0
        for f in frames:
            if f.get("type") == "ch_data":
                v = f["voltage"]
                g = f["gain"]
                if v > 50.0 and (g & 0xFFFF00) != GAIN_EXPECTED:
                    err_count += 1
        
        status = "✅ OK" if err_count == 0 else f"⚠ {err_count} RESETS"
        self._hist_list.insert("", "end", values=(n, ts, len(frames), dur, status))
        self._tabs.set("TEST HISTORY")

    def _clear_results(self):
        self._hist_list.delete(*self._hist_list.get_children())
        for tbl in self._hist_tables:
            tbl.clear()
        self._all_sessions.clear()
        self._session_no = 0
        self._update_frame_lbl()
        self._hist_detail_lbl.configure(text="SELECT A SESSION TO VIEW DETAILS", text_color="#777")
        
    def _re_render_logs(self, frames):
        """Full re-render of all log tables (Live view)."""
        self._clear_log()
        for i, pkt in enumerate(frames):
            ts = pkt.get("ts_offset", 0)
            ch = pkt["channel"]
            row, errs = self._format_frame_row(pkt, i, ts, ch)
            self._log_tables[ch].append_row(row, errs)

    def _on_zoom_v(self, val):
        sz = int(float(val))
        for tbl in self._log_tables + self._hist_tables:
            tbl.set_zoom(font_size=sz)

    def _on_zoom_h(self, val):
        w = int(float(val))
        re_render = False
        for tbl in self._log_tables + self._hist_tables:
            if tbl.set_zoom(col_width=w):
                re_render = True
        
        if re_render:
            # Re-render Live view
            self._re_render_logs(self._session_frames)
            # Re-render History view (if a session is selected)
            sel = self._hist_list.selection()
            if sel:
                self._on_history_select(None)


# ─────────────────────────────────────────────────────────────────────────────
if __name__ == "__main__":
    app = LumesJigApp()
    app.mainloop()
