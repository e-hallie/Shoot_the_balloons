"""
LSM6DSL Gyroscoop Visualisatie
==============================
Dit programma ontvangt gyroscoop/accelerometer data van een STM32F401RET6
via UART en visualiseert de beweging met een punt op het scherm.

Hardware setup:
- STM32F401RET6 microcontroller
- LSM6DSL 6-DOF sensor (via TCA9548APWR I2C multiplexer op kanaal 3)
- UART communicatie op 115200 baud

Auteur: Claude AI
Datum: December 2024
"""

import tkinter as tk
from tkinter import ttk, messagebox
import serial
import serial.tools.list_ports
import threading
import re
import math
import time
from collections import deque


class GyroVisualizer:
    """
    Hoofdklasse voor de gyroscoop visualisatie applicatie.
    
    Deze klasse:
    1. Leest UART data van de STM32
    2. Parseert de gyroscoop en accelerometer waarden
    3. Visualiseert de beweging in realtime
    """
    
    def __init__(self, root):
        self.root = root
        self.root.title("LSM6DSL Gyroscoop Visualisatie")
        self.root.geometry("900x700")
        self.root.configure(bg='#1a1a2e')
        
        # Seriële poort variabelen
        self.serial_port = None
        self.is_connected = False
        self.read_thread = None
        self.running = False
        
        # Sensor data (initiële waarden)
        self.ax = 0.0  # Accelerometer X (g)
        self.ay = 0.0  # Accelerometer Y (g)
        self.az = 0.0  # Accelerometer Z (g)
        self.gx = 0.0  # Gyroscoop X (dps - degrees per second)
        self.gy = 0.0  # Gyroscoop Y (dps)
        self.gz = 0.0  # Gyroscoop Z (dps)
        
        # Positie van het punt (begint in het midden)
        self.dot_x = 0.0
        self.dot_y = 0.0
        
        # Gevoeligheid instelling (hoe snel het punt beweegt)
        self.sensitivity = 5.0
        
        # Historie voor trail effect
        self.trail_history = deque(maxlen=50)
        
        # Bouw de GUI op
        self.setup_gui()
        
        # Start de update loop
        self.update_visualization()
    
    def setup_gui(self):
        """Bouwt de grafische interface op."""
        
        # === BOVENSTE FRAME: Connectie instellingen ===
        connection_frame = tk.Frame(self.root, bg='#16213e', pady=10, padx=10)
        connection_frame.pack(fill=tk.X, padx=10, pady=(10, 5))
        
        # Titel
        title_label = tk.Label(
            connection_frame, 
            text="🎯 LSM6DSL Gyroscoop Visualisatie",
            font=('Segoe UI', 16, 'bold'),
            fg='#e94560',
            bg='#16213e'
        )
        title_label.pack(pady=(0, 10))
        
        # COM poort selectie
        port_frame = tk.Frame(connection_frame, bg='#16213e')
        port_frame.pack(fill=tk.X)
        
        tk.Label(
            port_frame, 
            text="COM Poort:",
            font=('Segoe UI', 10),
            fg='#eee',
            bg='#16213e'
        ).pack(side=tk.LEFT, padx=(0, 10))
        
        # Dropdown voor COM poorten
        self.port_var = tk.StringVar()
        self.port_combo = ttk.Combobox(
            port_frame, 
            textvariable=self.port_var,
            width=15,
            state='readonly'
        )
        self.port_combo.pack(side=tk.LEFT, padx=(0, 10))
        
        # Vernieuw knop voor poorten
        refresh_btn = tk.Button(
            port_frame,
            text="🔄",
            command=self.refresh_ports,
            font=('Segoe UI', 10),
            bg='#0f3460',
            fg='white',
            relief=tk.FLAT,
            cursor='hand2'
        )
        refresh_btn.pack(side=tk.LEFT, padx=(0, 10))
        
        # Connect/Disconnect knop
        self.connect_btn = tk.Button(
            port_frame,
            text="Verbinden",
            command=self.toggle_connection,
            font=('Segoe UI', 10, 'bold'),
            bg='#4ecca3',
            fg='#1a1a2e',
            relief=tk.FLAT,
            width=12,
            cursor='hand2'
        )
        self.connect_btn.pack(side=tk.LEFT, padx=(0, 20))
        
        # Status indicator
        self.status_label = tk.Label(
            port_frame,
            text="● Niet verbonden",
            font=('Segoe UI', 10),
            fg='#ff6b6b',
            bg='#16213e'
        )
        self.status_label.pack(side=tk.LEFT)
        
        # Gevoeligheid slider
        sensitivity_frame = tk.Frame(connection_frame, bg='#16213e')
        sensitivity_frame.pack(fill=tk.X, pady=(10, 0))
        
        tk.Label(
            sensitivity_frame,
            text="Gevoeligheid:",
            font=('Segoe UI', 10),
            fg='#eee',
            bg='#16213e'
        ).pack(side=tk.LEFT, padx=(0, 10))
        
        self.sensitivity_scale = tk.Scale(
            sensitivity_frame,
            from_=1,
            to=20,
            orient=tk.HORIZONTAL,
            length=200,
            bg='#16213e',
            fg='#eee',
            highlightthickness=0,
            troughcolor='#0f3460',
            command=self.update_sensitivity
        )
        self.sensitivity_scale.set(5)
        self.sensitivity_scale.pack(side=tk.LEFT)
        
        # Reset knop
        reset_btn = tk.Button(
            sensitivity_frame,
            text="Reset Positie",
            command=self.reset_position,
            font=('Segoe UI', 10),
            bg='#e94560',
            fg='white',
            relief=tk.FLAT,
            cursor='hand2'
        )
        reset_btn.pack(side=tk.LEFT, padx=(20, 0))
        
        # === MIDDEN FRAME: Visualisatie canvas ===
        canvas_frame = tk.Frame(self.root, bg='#1a1a2e')
        canvas_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=5)
        
        # Canvas voor de visualisatie
        self.canvas_size = 500
        self.canvas = tk.Canvas(
            canvas_frame,
            width=self.canvas_size,
            height=self.canvas_size,
            bg='#0f0f23',
            highlightthickness=2,
            highlightbackground='#e94560'
        )
        self.canvas.pack(pady=10)
        
        # Teken het raster en assen
        self.draw_grid()
        
        # Maak de bewegende punt
        center = self.canvas_size // 2
        self.dot = self.canvas.create_oval(
            center - 12, center - 12,
            center + 12, center + 12,
            fill='#4ecca3',
            outline='#7effc9',
            width=3
        )
        
        # === ONDERSTE FRAME: Data weergave ===
        data_frame = tk.Frame(self.root, bg='#16213e', pady=10, padx=10)
        data_frame.pack(fill=tk.X, padx=10, pady=(5, 10))
        
        # Gyroscoop data labels
        gyro_frame = tk.Frame(data_frame, bg='#16213e')
        gyro_frame.pack(side=tk.LEFT, padx=(0, 40))
        
        tk.Label(
            gyro_frame,
            text="GYROSCOOP (°/s)",
            font=('Segoe UI', 10, 'bold'),
            fg='#e94560',
            bg='#16213e'
        ).pack()
        
        self.gyro_labels = {}
        for axis, color in [('X', '#ff6b6b'), ('Y', '#4ecdc4'), ('Z', '#ffe66d')]:
            frame = tk.Frame(gyro_frame, bg='#16213e')
            frame.pack(fill=tk.X)
            tk.Label(
                frame,
                text=f"{axis}:",
                font=('Consolas', 11, 'bold'),
                fg=color,
                bg='#16213e',
                width=3
            ).pack(side=tk.LEFT)
            self.gyro_labels[axis] = tk.Label(
                frame,
                text="+0.00",
                font=('Consolas', 11),
                fg='#eee',
                bg='#16213e',
                width=10
            )
            self.gyro_labels[axis].pack(side=tk.LEFT)
        
        # Accelerometer data labels
        acc_frame = tk.Frame(data_frame, bg='#16213e')
        acc_frame.pack(side=tk.LEFT, padx=(0, 40))
        
        tk.Label(
            acc_frame,
            text="ACCELEROMETER (g)",
            font=('Segoe UI', 10, 'bold'),
            fg='#e94560',
            bg='#16213e'
        ).pack()
        
        self.acc_labels = {}
        for axis, color in [('X', '#ff6b6b'), ('Y', '#4ecdc4'), ('Z', '#ffe66d')]:
            frame = tk.Frame(acc_frame, bg='#16213e')
            frame.pack(fill=tk.X)
            tk.Label(
                frame,
                text=f"{axis}:",
                font=('Consolas', 11, 'bold'),
                fg=color,
                bg='#16213e',
                width=3
            ).pack(side=tk.LEFT)
            self.acc_labels[axis] = tk.Label(
                frame,
                text="+0.000",
                font=('Consolas', 11),
                fg='#eee',
                bg='#16213e',
                width=10
            )
            self.acc_labels[axis].pack(side=tk.LEFT)
        
        # Positie weergave
        pos_frame = tk.Frame(data_frame, bg='#16213e')
        pos_frame.pack(side=tk.LEFT)
        
        tk.Label(
            pos_frame,
            text="PUNT POSITIE",
            font=('Segoe UI', 10, 'bold'),
            fg='#e94560',
            bg='#16213e'
        ).pack()
        
        self.pos_label = tk.Label(
            pos_frame,
            text="X: 0.0  Y: 0.0",
            font=('Consolas', 11),
            fg='#4ecca3',
            bg='#16213e'
        )
        self.pos_label.pack()
        
        # Raw data weergave
        self.raw_label = tk.Label(
            data_frame,
            text="",
            font=('Consolas', 8),
            fg='#666',
            bg='#16213e',
            wraplength=200
        )
        self.raw_label.pack(side=tk.RIGHT)
        
        # Initialiseer de COM poort lijst
        self.refresh_ports()
    
    def draw_grid(self):
        """Tekent het raster en de assen op het canvas."""
        center = self.canvas_size // 2
        grid_spacing = 50
        
        # Teken rasterlijnen
        for i in range(0, self.canvas_size + 1, grid_spacing):
            # Verticale lijnen
            color = '#1a1a3a' if i != center else '#3a3a5a'
            width = 1 if i != center else 2
            self.canvas.create_line(i, 0, i, self.canvas_size, fill=color, width=width)
            # Horizontale lijnen
            self.canvas.create_line(0, i, self.canvas_size, i, fill=color, width=width)
        
        # Teken de hoofdassen (dikker)
        self.canvas.create_line(center, 0, center, self.canvas_size, fill='#e94560', width=2)
        self.canvas.create_line(0, center, self.canvas_size, center, fill='#e94560', width=2)
        
        # As labels
        self.canvas.create_text(center + 15, 15, text="Y+", fill='#4ecdc4', font=('Segoe UI', 10, 'bold'))
        self.canvas.create_text(center + 15, self.canvas_size - 15, text="Y-", fill='#4ecdc4', font=('Segoe UI', 10, 'bold'))
        self.canvas.create_text(15, center - 15, text="X-", fill='#ff6b6b', font=('Segoe UI', 10, 'bold'))
        self.canvas.create_text(self.canvas_size - 15, center - 15, text="X+", fill='#ff6b6b', font=('Segoe UI', 10, 'bold'))
        
        # Centrum marker
        self.canvas.create_oval(
            center - 5, center - 5,
            center + 5, center + 5,
            outline='#e94560',
            width=2
        )
    
    def refresh_ports(self):
        """Vernieuwt de lijst met beschikbare COM poorten."""
        ports = serial.tools.list_ports.comports()
        port_list = [port.device for port in ports]
        
        self.port_combo['values'] = port_list
        
        if port_list:
            # Probeer COM4 te selecteren als die beschikbaar is
            if 'COM4' in port_list:
                self.port_combo.set('COM4')
            else:
                self.port_combo.set(port_list[0])
    
    def toggle_connection(self):
        """Schakelt de seriële verbinding aan/uit."""
        if self.is_connected:
            self.disconnect()
        else:
            self.connect()
    
    def connect(self):
        """Maakt verbinding met de geselecteerde COM poort."""
        port = self.port_var.get()
        
        if not port:
            messagebox.showerror("Fout", "Selecteer eerst een COM poort!")
            return
        
        try:
            # Open de seriële poort
            # Baudrate 115200 komt overeen met je STM32 code
            self.serial_port = serial.Serial(
                port=port,
                baudrate=115200,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.1
            )
            
            self.is_connected = True
            self.running = True
            
            # Start de lees-thread
            self.read_thread = threading.Thread(target=self.read_serial_data, daemon=True)
            self.read_thread.start()
            
            # Update GUI
            self.connect_btn.config(text="Verbreken", bg='#ff6b6b')
            self.status_label.config(text=f"● Verbonden met {port}", fg='#4ecca3')
            
        except serial.SerialException as e:
            messagebox.showerror("Verbindingsfout", f"Kan niet verbinden met {port}:\n{e}")
    
    def disconnect(self):
        """Verbreekt de seriële verbinding."""
        self.running = False
        
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
        
        self.is_connected = False
        self.serial_port = None
        
        # Update GUI
        self.connect_btn.config(text="Verbinden", bg='#4ecca3')
        self.status_label.config(text="● Niet verbonden", fg='#ff6b6b')
    
    def read_serial_data(self):
        """
        Leest continu data van de seriële poort.
        Draait in een aparte thread om de GUI niet te blokkeren.
        
        Verwacht data format van je STM32:
        Acc(g): X=+0.012 Y=-0.034 Z=+0.987 | Gyro(dps): X=+1.23 Y=-0.45 Z=+0.12
        """
        # Regex patroon om de sensor waarden te extracten
        # Dit patroon matcht het format van je UART_Print output
        pattern = r'Acc\(g\):\s*X=([+-]?\d+\.?\d*)\s*Y=([+-]?\d+\.?\d*)\s*Z=([+-]?\d+\.?\d*)\s*\|\s*Gyro\(dps\):\s*X=([+-]?\d+\.?\d*)\s*Y=([+-]?\d+\.?\d*)\s*Z=([+-]?\d+\.?\d*)'
        
        buffer = ""
        
        while self.running:
            try:
                if self.serial_port and self.serial_port.is_open:
                    # Lees beschikbare bytes
                    if self.serial_port.in_waiting > 0:
                        data = self.serial_port.read(self.serial_port.in_waiting)
                        buffer += data.decode('utf-8', errors='ignore')
                        
                        # Zoek naar complete regels
                        while '\n' in buffer:
                            line, buffer = buffer.split('\n', 1)
                            line = line.strip()
                            
                            # Update raw data label (in main thread)
                            self.root.after(0, lambda l=line: self.raw_label.config(text=l[-60:] if len(l) > 60 else l))
                            
                            # Parse de data
                            match = re.search(pattern, line)
                            if match:
                                # Haal de waarden op
                                self.ax = float(match.group(1))
                                self.ay = float(match.group(2))
                                self.az = float(match.group(3))
                                self.gx = float(match.group(4))
                                self.gy = float(match.group(5))
                                self.gz = float(match.group(6))
                                
            except Exception as e:
                print(f"Leesfout: {e}")
                time.sleep(0.1)
            
            time.sleep(0.01)  # Kleine pauze om CPU te sparen
    
    def update_sensitivity(self, value):
        """Update de gevoeligheid instelling."""
        self.sensitivity = float(value)
    
    def reset_position(self):
        """Reset de punt naar het midden."""
        self.dot_x = 0.0
        self.dot_y = 0.0
        self.trail_history.clear()
    
    def update_visualization(self):
        """
        Update de visualisatie.
        Deze functie wordt elke 20ms aangeroepen (50 FPS).
        """
        # Update punt positie gebaseerd op gyroscoop data
        # gx draait rond X-as -> beweegt Y
        # gy draait rond Y-as -> beweegt X
        # We gebruiken een kleine tijdstap voor integratie
        dt = 0.02  # 20ms
        
        # Integreer de gyroscoop rotatie naar positie
        # Dit is een vereenvoudigde benadering
        self.dot_x += self.gy * dt * self.sensitivity
        self.dot_y -= self.gx * dt * self.sensitivity  # Negatief voor juiste richting
        
        # Begrens de positie tot het canvas
        max_offset = (self.canvas_size / 2) - 20
        self.dot_x = max(-max_offset, min(max_offset, self.dot_x))
        self.dot_y = max(-max_offset, min(max_offset, self.dot_y))
        
        # Bereken canvas coördinaten
        center = self.canvas_size // 2
        canvas_x = center + self.dot_x
        canvas_y = center + self.dot_y
        
        # Update trail history
        self.trail_history.append((canvas_x, canvas_y))
        
        # Teken trail (oude punten)
        self.canvas.delete("trail")
        if len(self.trail_history) > 1:
            for i, (tx, ty) in enumerate(self.trail_history):
                # Fade effect: oudere punten zijn transparanter (kleiner en lichter)
                alpha = i / len(self.trail_history)
                size = 2 + alpha * 6
                # Maak kleur lichter voor oudere punten
                green_val = int(0x4e + (0xcc - 0x4e) * alpha)
                color = f'#{0x4e:02x}{green_val:02x}{0xa3:02x}'
                self.canvas.create_oval(
                    tx - size/2, ty - size/2,
                    tx + size/2, ty + size/2,
                    fill=color,
                    outline='',
                    tags="trail"
                )
        
        # Update hoofdpunt positie
        self.canvas.coords(
            self.dot,
            canvas_x - 12, canvas_y - 12,
            canvas_x + 12, canvas_y + 12
        )
        
        # Breng punt naar voorgrond
        self.canvas.tag_raise(self.dot)
        
        # Update data labels
        self.gyro_labels['X'].config(text=f"{self.gx:+.2f}")
        self.gyro_labels['Y'].config(text=f"{self.gy:+.2f}")
        self.gyro_labels['Z'].config(text=f"{self.gz:+.2f}")
        
        self.acc_labels['X'].config(text=f"{self.ax:+.3f}")
        self.acc_labels['Y'].config(text=f"{self.ay:+.3f}")
        self.acc_labels['Z'].config(text=f"{self.az:+.3f}")
        
        self.pos_label.config(text=f"X: {self.dot_x:.1f}  Y: {self.dot_y:.1f}")
        
        # Plan de volgende update
        self.root.after(20, self.update_visualization)
    
    def on_closing(self):
        """Wordt aangeroepen wanneer het venster wordt gesloten."""
        self.running = False
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
        self.root.destroy()


def main():
    """Hoofdfunctie om de applicatie te starten."""
    root = tk.Tk()
    
    # Stel het window icoon in (als beschikbaar)
    try:
        root.iconbitmap(default='')
    except:
        pass
    
    app = GyroVisualizer(root)
    
    # Handel window sluiting netjes af
    root.protocol("WM_DELETE_WINDOW", app.on_closing)
    
    # Start de GUI event loop
    root.mainloop()


if __name__ == "__main__":
    print("=" * 50)
    print("LSM6DSL Gyroscoop Visualisatie")
    print("=" * 50)
    print("\nBenodigde library: pyserial")
    print("Installeer met: pip install pyserial")
    print("\nStarten van de applicatie...")
    print("-" * 50)
    
    main()
