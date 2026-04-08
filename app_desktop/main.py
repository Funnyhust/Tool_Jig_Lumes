import customtkinter as ctk
import tkinter.ttk as ttk
import serial.tools.list_ports
from serial_manager import SerialManager
from protocol import JigProtocol
import time
import collections
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
from matplotlib.figure import Figure

# Matplotlib configuration for Dark Mode
plt.style.use('dark_background')

class MultiLineChart(ctk.CTkFrame):
    def __init__(self, master, title, **kwargs):
        super().__init__(master, border_width=1, border_color="#333", **kwargs)
        self.is_frozen = False
        self.start_time = time.time()
        
        header = ctk.CTkFrame(self, fg_color="transparent")
        header.pack(fill="x", padx=5, pady=2)
        ctk.CTkLabel(header, text=title, font=ctk.CTkFont(size=12, weight="bold")).pack(side="left")
        
        self.btn_hold = ctk.CTkButton(header, text="LIVE", width=60, height=22, 
                                      fg_color="#2b719e", command=self.toggle_hold, font=ctk.CTkFont(size=10, weight="bold"))
        self.btn_hold.pack(side="right", padx=5)
        
        self.param_var = ctk.StringVar(value="Power")
        self.selector = ctk.CTkOptionMenu(header, values=["Power", "Current", "Voltage"],
                                         variable=self.param_var, height=22, width=80, font=ctk.CTkFont(size=11),
                                         command=self.force_redraw)
        self.selector.pack(side="right")
        
        self.fig = Figure(figsize=(5, 3), dpi=80)
        self.ax = self.fig.add_subplot(111)
        self.ax.set_facecolor('#1a1a1a')
        self.fig.patch.set_facecolor('#1a1a1a')
        self.ax.tick_params(colors='#888', labelsize=8)
        self.ax.grid(True, color='#333', linestyle='--', linewidth=0.5)
        self.ax.set_xlabel("Time (s)", color='#666', fontsize=8)
        
        self.line_a, = self.ax.plot([], [], color='#00d4ff', label='A', linewidth=1.5)
        self.line_b, = self.ax.plot([], [], color='#00ff88', label='B', linewidth=1.5)
        self.line_c, = self.ax.plot([], [], color='#ffcc00', label='C', linewidth=1.5)
        self.ax.legend(loc='upper right', fontsize=7, framealpha=0.3)
        
        self.canvas = FigureCanvasTkAgg(self.fig, master=self)
        self.canvas_widget = self.canvas.get_tk_widget()
        self.canvas_widget.pack(fill="both", expand=True, padx=5, pady=0)
        
        self.toolbar = NavigationToolbar2Tk(self.canvas, self, pack_toolbar=False)
        self.toolbar.update()
        self.toolbar.pack(side="bottom", fill="x", padx=5)
        self.canvas.mpl_connect('scroll_event', self.on_scroll)
        
        # Comprehensive Data storage
        self.history_t = collections.deque(maxlen=300)
        self.history_v = collections.deque(maxlen=300)
        self.history_i_a = collections.deque(maxlen=300)
        self.history_i_b = collections.deque(maxlen=300)
        self.history_i_c = collections.deque(maxlen=300)
        self.history_p_a = collections.deque(maxlen=300)
        self.history_p_b = collections.deque(maxlen=300)
        self.history_p_c = collections.deque(maxlen=300)

    def toggle_hold(self):
        self.is_frozen = not self.is_frozen
        if self.is_frozen:
            self.btn_hold.configure(text="HOLD", fg_color="#ff6600")
        else:
            self.btn_hold.configure(text="LIVE", fg_color="#2b719e")
            self.reset_axes()

    def reset_axes(self):
        if not self.history_t: return
        self.ax.relim()
        self.ax.autoscale_view()
        # Keep X-axis showing at least a 30s window
        cur_xlim = self.ax.get_xlim()
        latest_t = self.history_t[-1]
        if latest_t > 30:
            self.ax.set_xlim(latest_t - 30, latest_t + 2)
        self.canvas.draw_idle()

    def on_scroll(self, event):
        if event.inaxes != self.ax: return
        base_scale = 1.2
        cur_xlim = self.ax.get_xlim()
        xdata = event.xdata
        if event.button == 'up':
            scale_factor = 1 / base_scale
        else:
            scale_factor = base_scale
        new_width = (cur_xlim[1] - cur_xlim[0]) * scale_factor
        relx = (cur_xlim[1] - xdata) / (cur_xlim[1] - cur_xlim[0])
        self.ax.set_xlim([xdata - new_width * (1 - relx), xdata + new_width * relx])
        # Auto-hold đã bị tắt theo yêu cầu. Người dùng phải bấm HOLD bằng tay nếu muốn zoom.
        self.canvas.draw_idle()

    def reset_history(self):
        self.history_t.clear()
        self.history_v.clear()
        self.history_i_a.clear()
        self.history_i_b.clear()
        self.history_i_c.clear()
        self.history_p_a.clear()
        self.history_p_b.clear()
        self.history_p_c.clear()
        self.start_time = time.time()

    def force_redraw(self, _=None):
        if not self.history_t: return
        t_data = list(self.history_t)
        mode = self.param_var.get()
        if mode == "Voltage":
            self.line_a.set_data(t_data, list(self.history_v))
            self.line_b.set_data([], [])
            self.line_c.set_data([], [])
            self.ax.set_ylabel("Voltage (V)", color='#888', fontsize=8)
        elif mode == "Current":
            self.line_a.set_data(t_data, list(self.history_i_a))
            self.line_b.set_data(t_data, list(self.history_i_b))
            self.line_c.set_data(t_data, list(self.history_i_c))
            self.ax.set_ylabel("Current (mA)", color='#888', fontsize=8)
        else:
            self.line_a.set_data(t_data, list(self.history_p_a))
            self.line_b.set_data(t_data, list(self.history_p_b))
            self.line_c.set_data(t_data, list(self.history_p_c))
            self.ax.set_ylabel("Power (W)", color='#888', fontsize=8)
            
        self.ax.relim()
        self.ax.autoscale_view()
        self.canvas.draw_idle()

    def update_chart(self, v, i_a, i_b, i_c, p_a, p_b, p_c):
        rel_time = time.time() - self.start_time
        
        self.history_t.append(rel_time)
        self.history_v.append(v)
        self.history_i_a.append(i_a)
        self.history_i_b.append(i_b)
        self.history_i_c.append(i_c)
        self.history_p_a.append(p_a)
        self.history_p_b.append(p_b)
        self.history_p_c.append(p_c)
        
        if self.is_frozen: return
        
        mode = self.param_var.get()
        t_data = list(self.history_t)
        if mode == "Voltage":
            self.line_a.set_data(t_data, list(self.history_v))
        elif mode == "Current":
            self.line_a.set_data(t_data, list(self.history_i_a))
            self.line_b.set_data(t_data, list(self.history_i_b))
            self.line_c.set_data(t_data, list(self.history_i_c))
        else:
            self.line_a.set_data(t_data, list(self.history_p_a))
            self.line_b.set_data(t_data, list(self.history_p_b))
            self.line_c.set_data(t_data, list(self.history_p_c))
        
        self.reset_axes()

class LumesJigApp(ctk.CTk):
    def __init__(self):
        super().__init__()
        self.title("MSB LUMES JIG - ADVANCED PASSIVE MONITOR v4.6")
        self.geometry("1400x950")
        ctk.set_appearance_mode("dark")
        
        style = ttk.Style()
        style.theme_use("clam")
        style.configure("Treeview", background="#222", foreground="white", fieldbackground="#222", borderwidth=0, rowheight=25)
        style.map("Treeview", background=[('selected', '#2b719e')])
        style.configure("Treeview.Heading", background="#111", foreground="white", relief="flat")

        self.serial_manager = SerialManager(self.on_data_received, self.log)
        self.setup_ui()
        self.refresh_ports()
        self.update_connection_status()

    def setup_ui(self):
        self.sidebar = ctk.CTkFrame(self, width=280, corner_radius=0)
        self.sidebar.pack(side="left", fill="y")
        ctk.CTkLabel(self.sidebar, text="LUMES JIG v4.6", font=ctk.CTkFont(size=22, weight="bold")).pack(pady=30)
        
        self.status_box = ctk.CTkFrame(self.sidebar, border_width=1, border_color="#333")
        self.status_box.pack(padx=20, pady=10, fill="x")
        self.status_indicator = ctk.CTkLabel(self.status_box, text="● DISCONNECTED", text_color="gray", font=ctk.CTkFont(size=13, weight="bold"))
        self.status_indicator.pack(pady=10)

        self.port_var = ctk.StringVar()
        port_f = ctk.CTkFrame(self.sidebar, fg_color="transparent")
        port_f.pack(pady=5, padx=20, fill="x")
        self.port_cb = ctk.CTkComboBox(port_f, variable=self.port_var, values=[], width=160)
        self.port_cb.pack(side="left", padx=(0,5))
        ctk.CTkButton(port_f, text="🔄", width=40, command=self.refresh_ports, fg_color="#444").pack(side="right")
        
        self.btn_connect = ctk.CTkButton(self.sidebar, text="OPEN MONITOR", command=self.toggle_connect, fg_color="#2b719e")
        self.btn_connect.pack(pady=10, padx=20, fill="x")
        
        ctk.CTkLabel(self.sidebar, text="PASSIVE MONITOR MODE\nX-Axis units: Seconds\nUse Scroll Wheel to Zoom", 
                     text_color="#00ffcc", font=ctk.CTkFont(size=11, slant="italic")).pack(pady=30, side="bottom")

        self.tabview = ctk.CTkTabview(self, fg_color="#1a1a1a")
        self.tabview.pack(side="right", fill="both", expand=True, padx=10, pady=10)
        self.tab_dashboard = self.tabview.add("DASHBOARD")
        self.tab_analytics = self.tabview.add("GRAPHS")
        self.tab_history = self.tabview.add("HISTORY LOG")
        self.tab_system = self.tabview.add("SYSTEM LOGS")
        
        self.setup_dashboard_tab()
        self.setup_analytics_tab()
        self.setup_history_tab()
        self.setup_logs_tab()

    def setup_history_tab(self):
        container = ctk.CTkFrame(self.tab_history)
        container.pack(fill="both", expand=True, padx=10, pady=10)
        cols = ("Time", "Channel", "Gain", "Voltage", "IA(mA)", "PA(W)", "IB(mA)", "PB(W)", "IC(mA)", "PC(W)")
        self.tree = ttk.Treeview(container, columns=cols, show="headings")
        for col in cols:
            self.tree.heading(col, text=col)
            self.tree.column(col, width=100, anchor="center")
        scrollbar = ttk.Scrollbar(container, orient="vertical", command=self.tree.yview)
        self.tree.configure(yscrollcommand=scrollbar.set)
        self.tree.pack(side="left", fill="both", expand=True)
        scrollbar.pack(side="right", fill="y")
        btn_f = ctk.CTkFrame(self.tab_history, fg_color="transparent")
        btn_f.pack(fill="x", padx=10, pady=5)
        ctk.CTkButton(btn_f, text="Clear History", command=lambda: self.tree.delete(*self.tree.get_children()), fg_color="#444", width=120).pack(side="right")

    def setup_dashboard_tab(self):
        self.monitor_grid = ctk.CTkFrame(self.tab_dashboard, fg_color="transparent")
        self.monitor_grid.pack(fill="both", expand=True, padx=10, pady=10)
        self.channel_uis = []
        self.channel_v_labels = []
        self.channel_g_labels = []
        for ch_idx in range(4):
            ch_frame = ctk.CTkFrame(self.monitor_grid, border_width=1, border_color="#333", fg_color="#121212")
            ch_frame.grid(row=ch_idx//2, column=ch_idx%2, padx=10, pady=10, sticky="nsew")
            self.monitor_grid.grid_columnconfigure(ch_idx%2, weight=1)
            self.monitor_grid.grid_rowconfigure(ch_idx//2, weight=1)
            top_f = ctk.CTkFrame(ch_frame, fg_color="#1e1e1e", height=40)
            top_f.pack(fill="x", padx=2, pady=2)
            ctk.CTkLabel(top_f, text=f"CHANNEL {ch_idx+1}", font=ctk.CTkFont(size=14, weight="bold"), text_color="#2b719e").pack(side="left", padx=15)
            g_lbl = ctk.CTkLabel(top_f, text="GAIN: 0x000000", font=ctk.CTkFont(family="Consolas", size=13), text_color="#aa88ff")
            g_lbl.pack(side="left", padx=5)
            v_lbl = ctk.CTkLabel(top_f, text="0.0 V", font=ctk.CTkFont(family="Consolas", size=18, weight="bold"), text_color="#00ffcc")
            v_lbl.pack(side="right", padx=15)
            self.channel_g_labels.append(g_lbl)
            self.channel_v_labels.append(v_lbl)
            table_f = ctk.CTkFrame(ch_frame, fg_color="transparent")
            table_f.pack(fill="both", expand=True, padx=10, pady=10)
            relay_rows = []
            for r_idx in range(3):
                name = f"{ch_idx+1}{'ABC'[r_idx]}"
                clr = ["#00d4ff", "#00ff88", "#ffcc00"][r_idx]
                ctk.CTkLabel(table_f, text=name, font=ctk.CTkFont(size=14, weight="bold"), text_color=clr).grid(row=r_idx, column=0, padx=10, pady=10)
                i_lbl = ctk.CTkLabel(table_f, text="0", font=ctk.CTkFont(family="Consolas", size=22))
                i_lbl.grid(row=r_idx, column=1, padx=15)
                p_lbl = ctk.CTkLabel(table_f, text="0.0", font=ctk.CTkFont(family="Consolas", size=22))
                p_lbl.grid(row=r_idx, column=2, padx=15)
                relay_rows.append({"i": i_lbl, "p": p_lbl})
            self.channel_uis.append(relay_rows)

    def setup_analytics_tab(self):
        self.charts_frame = ctk.CTkFrame(self.tab_analytics, fg_color="transparent")
        self.charts_frame.pack(fill="both", expand=True, padx=5, pady=5)
        self.charts = []
        for i in range(4):
            chart = MultiLineChart(self.charts_frame, title=f"CH {i+1}")
            chart.grid(row=i//2, column=i%2, padx=5, pady=5, sticky="nsew")
            self.charts_frame.grid_columnconfigure(i%2, weight=1)
            self.charts_frame.grid_rowconfigure(i//2, weight=1)
            self.charts.append(chart)

    def setup_logs_tab(self):
        self.log_text = ctk.CTkTextbox(self.tab_system, font=ctk.CTkFont(family="Consolas", size=11))
        self.log_text.pack(fill="both", expand=True, padx=10, pady=10)

    def log(self, message):
        self.log_text.insert("end", time.strftime("[%H:%M:%S] ") + message + "\n")
        self.log_text.see("end")

    def refresh_ports(self):
        ports = [p.device for p in serial.tools.list_ports.comports()]
        self.port_cb.configure(values=ports)
        if ports: self.port_var.set(ports[0])

    def update_connection_status(self):
        health = self.serial_manager.is_healthy()
        if not self.serial_manager.running:
            self.status_indicator.configure(text="● DISCONNECTED", text_color="gray")
            self.btn_connect.configure(text="OPEN MONITOR", fg_color="#2b719e")
        elif health is None:
            self.status_indicator.configure(text="● LISTENING...", text_color="#ffd700")
        elif health:
            self.status_indicator.configure(text="● MONITORING", text_color="#00ff00")
        else:
            self.status_indicator.configure(text="● NO DATA (TIMEOUT)", text_color="#ff4444")
        self.after(500, self.update_connection_status)

    def toggle_connect(self):
        if not self.serial_manager.running: 
            if self.serial_manager.connect(self.port_var.get()):
                for chart in self.charts: chart.reset_history()
        else: 
            self.serial_manager.disconnect()

    def on_data_received(self, data):
        if data["type"] == "data":
            v_list = data.get("voltages", [0.0]*4)
            currents = data.get("currents", [0]*12)
            powers = data.get("powers", [0.0]*12)
            gains = data.get("gains", [0]*4)
            ts = time.strftime("%H:%M:%S")
            for ch in range(4):
                self.channel_v_labels[ch].configure(text=f"{v_list[ch]:.1f} V")
                self.channel_g_labels[ch].configure(text=f"GAIN: 0x{gains[ch]:06X}")
                
                # Pass all data: V, Ix3, Px3
                self.charts[ch].update_chart(
                    v_list[ch],
                    currents[ch*3], currents[ch*3+1], currents[ch*3+2],
                    powers[ch*3], powers[ch*3+1], powers[ch*3+2]
                )
                
                for r in range(3):
                    idx = ch*3 + r
                    self.channel_uis[ch][r]["i"].configure(text=f"{currents[idx]}")
                    self.channel_uis[ch][r]["p"].configure(text=f"{powers[idx]:.1f}")
                row = (ts, f"CH{ch+1}", f"0x{gains[ch]:06X}", f"{v_list[ch]:.1f}", currents[ch*3], f"{powers[ch*3]:.1f}", 
                       currents[ch*3+1], f"{powers[ch*3+1]:.1f}", currents[ch*3+2], f"{powers[ch*3+2]:.1f}")
                item = self.tree.insert("", "end", values=row)
                self.tree.see(item)
                if len(self.tree.get_children()) > 1000: self.tree.delete(self.tree.get_children()[0])

if __name__ == "__main__":
    app = LumesJigApp()
    app.mainloop()
