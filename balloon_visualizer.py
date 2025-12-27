"""
LSM6DSL Ballonnenschieter Visualisatie
======================================
Dit programma ontvangt game data van een STM32F401RET6
via UART en visualiseert het schietspel:
- Richt-positie (groen vizier)
- Ballon positie (rode ballon)
- Plof-animatie wanneer ballon geraakt wordt!
- Score en game status

Hardware setup:
- STM32F401RET6 microcontroller
- LSM6DSL 6-DOF sensor (via TCA9548A I2C multiplexer op kanaal 3)
- UART communicatie op 115200 baud

Data formaat van STM32:
GAME:Az=+12.3,El=+5.6,Baz=+30.0,Bel=+15.0,Score=3,Active=1,Hit=0,Pop=0

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
import random
from collections import deque


class BalloonShooterVisualizer:
    """
    Hoofdklasse voor de ballonnenschieter visualisatie applicatie.
    """
    
    def __init__(self, root):
        self.root = root
        self.root.title("🎈 Ballonnenschieter Visualisatie")
        self.root.geometry("950x750")
        self.root.configure(bg='#1a1a2e')
        
        # Seriële poort variabelen
        self.serial_port = None
        self.is_connected = False
        self.read_thread = None
        self.running = False
        
        # Game data (van STM32)
        self.azimuth = 0.0       # Richt-positie horizontaal (graden)
        self.elevation = 0.0     # Richt-positie verticaal (graden)
        self.balloon_az = 0.0    # Ballon positie horizontaal
        self.balloon_el = 0.0    # Ballon positie verticaal
        self.score = 0           # Huidige score
        self.game_active = False # Game actief?
        self.in_hit_zone = False # Binnen trefzone?
        self.balloon_popped = False  # Ballon zojuist geraakt?
        self.pop_position_az = 0.0   # Positie waar ballon geplofd is (azimuth)
        self.pop_position_el = 0.0   # Positie waar ballon geplofd is (elevatie)
        
        # VPA configuratie (moet overeenkomen met STM32!)
        self.VPA_AZIMUTH_RANGE = 60.0
        self.VPA_ELEVATION_MIN = 0.0
        self.VPA_ELEVATION_MAX = 25.0
        self.HIT_RADIUS = 15.0
        
        # Trail voor vizier beweging
        self.trail_history = deque(maxlen=30)
        
        # Animatie variabelen
        self.balloon_pulse = 0.0
        
        # === PLOF ANIMATIE VARIABELEN ===
        self.pop_animation_active = False
        self.pop_animation_frame = 0
        self.pop_animation_max_frames = 20  # ~0.6 seconden bij 30fps
        self.pop_x = 0  # Positie waar de plof plaatsvindt
        self.pop_y = 0
        self.pop_particles = []  # Lijst met particles voor de explosie
        
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
            text="🎈 Virtuele Ballonnenschieter",
            font=('Segoe UI', 18, 'bold'),
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
        
        self.port_var = tk.StringVar()
        self.port_combo = ttk.Combobox(
            port_frame, 
            textvariable=self.port_var,
            width=15,
            state='readonly'
        )
        self.port_combo.pack(side=tk.LEFT, padx=(0, 10))
        
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
        
        self.status_label = tk.Label(
            port_frame,
            text="● Niet verbonden",
            font=('Segoe UI', 10),
            fg='#ff6b6b',
            bg='#16213e'
        )
        self.status_label.pack(side=tk.LEFT)
        
        self.game_status_label = tk.Label(
            port_frame,
            text="⏸ IDLE",
            font=('Segoe UI', 12, 'bold'),
            fg='#888',
            bg='#16213e'
        )
        self.game_status_label.pack(side=tk.RIGHT, padx=(20, 0))
        
        # === MIDDEN FRAME: Visualisatie canvas ===
        canvas_frame = tk.Frame(self.root, bg='#1a1a2e')
        canvas_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=5)
        
        self.canvas_width = 700
        self.canvas_height = 400
        self.canvas = tk.Canvas(
            canvas_frame,
            width=self.canvas_width,
            height=self.canvas_height,
            bg='#0a0a1a',
            highlightthickness=2,
            highlightbackground='#e94560'
        )
        self.canvas.pack(pady=10)
        
        self.draw_playfield()
        
        # === ONDERSTE FRAME: Score en data weergave ===
        data_frame = tk.Frame(self.root, bg='#16213e', pady=15, padx=15)
        data_frame.pack(fill=tk.X, padx=10, pady=(5, 10))
        
        # Score display
        score_frame = tk.Frame(data_frame, bg='#16213e')
        score_frame.pack(side=tk.LEFT, padx=(0, 40))
        
        tk.Label(
            score_frame,
            text="SCORE",
            font=('Segoe UI', 12, 'bold'),
            fg='#e94560',
            bg='#16213e'
        ).pack()
        
        self.score_label = tk.Label(
            score_frame,
            text="0",
            font=('Consolas', 48, 'bold'),
            fg='#4ecca3',
            bg='#16213e'
        )
        self.score_label.pack()
        
        # Positie data
        pos_frame = tk.Frame(data_frame, bg='#16213e')
        pos_frame.pack(side=tk.LEFT, padx=(0, 40))
        
        tk.Label(
            pos_frame,
            text="VIZIER POSITIE",
            font=('Segoe UI', 10, 'bold'),
            fg='#e94560',
            bg='#16213e'
        ).pack()
        
        self.aim_label = tk.Label(
            pos_frame,
            text="Az: 0.0°  El: 0.0°",
            font=('Consolas', 14),
            fg='#4ecca3',
            bg='#16213e'
        )
        self.aim_label.pack()
        
        # Ballon positie
        balloon_frame = tk.Frame(data_frame, bg='#16213e')
        balloon_frame.pack(side=tk.LEFT, padx=(0, 40))
        
        tk.Label(
            balloon_frame,
            text="BALLON POSITIE",
            font=('Segoe UI', 10, 'bold'),
            fg='#e94560',
            bg='#16213e'
        ).pack()
        
        self.balloon_label = tk.Label(
            balloon_frame,
            text="Az: 0.0°  El: 0.0°",
            font=('Consolas', 14),
            fg='#ff6b6b',
            bg='#16213e'
        )
        self.balloon_label.pack()
        
        # Afstand tot doel
        distance_frame = tk.Frame(data_frame, bg='#16213e')
        distance_frame.pack(side=tk.LEFT)
        
        tk.Label(
            distance_frame,
            text="AFSTAND",
            font=('Segoe UI', 10, 'bold'),
            fg='#e94560',
            bg='#16213e'
        ).pack()
        
        self.distance_label = tk.Label(
            distance_frame,
            text="--.-°",
            font=('Consolas', 14),
            fg='#ffe66d',
            bg='#16213e'
        )
        self.distance_label.pack()
        
        # Legenda
        legend_frame = tk.Frame(data_frame, bg='#16213e')
        legend_frame.pack(side=tk.RIGHT)
        
        tk.Label(
            legend_frame,
            text="LEGENDA",
            font=('Segoe UI', 10, 'bold'),
            fg='#e94560',
            bg='#16213e'
        ).pack()
        
        tk.Label(
            legend_frame,
            text="🟢 Vizier    🔴 Ballon    💥 Raak!",
            font=('Segoe UI', 10),
            fg='#aaa',
            bg='#16213e'
        ).pack()
        
        self.refresh_ports()
    
    def draw_playfield(self):
        """Tekent het speelveld (VPA) op het canvas."""
        self.canvas.delete("playfield")
        
        margin = 40
        usable_width = self.canvas_width - 2 * margin
        usable_height = self.canvas_height - 2 * margin
        
        self.scale_x = usable_width / (2 * self.VPA_AZIMUTH_RANGE)
        self.scale_y = usable_height / (self.VPA_ELEVATION_MAX - self.VPA_ELEVATION_MIN)
        
        self.origin_x = self.canvas_width / 2
        self.origin_y = self.canvas_height - margin
        
        # Achtergrond
        for i in range(10):
            y1 = margin + i * (usable_height / 10)
            y2 = margin + (i + 1) * (usable_height / 10)
            shade = int(10 + i * 1.5)
            color = f'#{shade:02x}{shade:02x}{int(shade*1.5):02x}'
            self.canvas.create_rectangle(
                margin, y1, self.canvas_width - margin, y2,
                fill=color, outline='', tags="playfield"
            )
        
        # Rasterlijnen verticaal
        for az in range(-60, 61, 15):
            x = self.origin_x + az * self.scale_x
            color = '#2a2a4a' if az != 0 else '#4a4a6a'
            width = 1 if az != 0 else 2
            self.canvas.create_line(
                x, margin, x, self.canvas_height - margin,
                fill=color, width=width, tags="playfield"
            )
            self.canvas.create_text(
                x, self.canvas_height - margin + 15,
                text=f"{az}°", fill='#666', font=('Consolas', 8),
                tags="playfield"
            )
        
        # Rasterlijnen horizontaal
        for el in range(0, 26, 5):
            y = self.origin_y - el * self.scale_y
            color = '#2a2a4a' if el != 0 else '#4a4a6a'
            width = 1 if el != 0 else 2
            self.canvas.create_line(
                margin, y, self.canvas_width - margin, y,
                fill=color, width=width, tags="playfield"
            )
            self.canvas.create_text(
                margin - 20, y,
                text=f"{el}°", fill='#666', font=('Consolas', 8),
                tags="playfield"
            )
        
        # VPA rand
        self.canvas.create_rectangle(
            margin, margin,
            self.canvas_width - margin, self.canvas_height - margin,
            outline='#e94560', width=2, tags="playfield"
        )
        
        # Labels
        self.canvas.create_text(
            self.canvas_width / 2, 15,
            text="ELEVATIE ↑", fill='#888', font=('Segoe UI', 9),
            tags="playfield"
        )
        self.canvas.create_text(
            self.canvas_width - 60, self.canvas_height - 15,
            text="AZIMUTH →", fill='#888', font=('Segoe UI', 9),
            tags="playfield"
        )
    
    def angle_to_canvas(self, az, el):
        """Converteert hoeken naar canvas coördinaten."""
        x = self.origin_x + az * self.scale_x
        y = self.origin_y - el * self.scale_y
        return x, y
    
    def start_pop_animation(self, x, y):
        """Start de plof-animatie op de gegeven positie."""
        self.pop_animation_active = True
        self.pop_animation_frame = 0
        self.pop_x = x
        self.pop_y = y
        
        # Maak particles voor de explosie
        self.pop_particles = []
        num_particles = 20
        
        for i in range(num_particles):
            angle = (2 * math.pi * i) / num_particles + random.uniform(-0.3, 0.3)
            speed = random.uniform(3, 8)
            size = random.uniform(4, 12)
            
            # Verschillende kleuren voor de particles
            colors = ['#ff4444', '#ff6666', '#ff8888', '#ffaaaa', '#ffe66d', '#ffcc00']
            color = random.choice(colors)
            
            particle = {
                'x': x,
                'y': y,
                'vx': math.cos(angle) * speed,
                'vy': math.sin(angle) * speed,
                'size': size,
                'color': color,
                'life': 1.0  # 1.0 = vol leven, 0.0 = dood
            }
            self.pop_particles.append(particle)
        
        # Voeg ook wat confetti-achtige stukjes toe
        for _ in range(10):
            particle = {
                'x': x + random.uniform(-10, 10),
                'y': y + random.uniform(-10, 10),
                'vx': random.uniform(-2, 2),
                'vy': random.uniform(-5, -1),  # Naar boven
                'size': random.uniform(3, 6),
                'color': random.choice(['#ff0000', '#ff4444', '#cc0000']),
                'life': 1.0,
                'is_confetti': True
            }
            self.pop_particles.append(particle)
    
    def update_pop_animation(self):
        """Update de plof-animatie."""
        if not self.pop_animation_active:
            return
        
        self.pop_animation_frame += 1
        
        # Update particles
        for particle in self.pop_particles:
            particle['x'] += particle['vx']
            particle['y'] += particle['vy']
            
            # Zwaartekracht voor confetti
            if particle.get('is_confetti'):
                particle['vy'] += 0.3
            
            # Fade out
            particle['life'] -= 1.0 / self.pop_animation_max_frames
            particle['size'] *= 0.95
        
        # Stop animatie als alle frames voorbij zijn
        if self.pop_animation_frame >= self.pop_animation_max_frames:
            self.pop_animation_active = False
            self.pop_particles = []
    
    def draw_pop_animation(self):
        """Tekent de plof-animatie."""
        if not self.pop_animation_active:
            return
        
        # Teken centrale flash in de eerste frames
        if self.pop_animation_frame < 5:
            flash_size = 30 + self.pop_animation_frame * 10
            
            # Gele/oranje flash
            self.canvas.create_oval(
                self.pop_x - flash_size, self.pop_y - flash_size,
                self.pop_x + flash_size, self.pop_y + flash_size,
                fill='#ffcc00', outline='#ff6600', width=3,
                tags="game_objects"
            )
        
        # Teken "RAAK!" tekst
        if self.pop_animation_frame < 15:
            text_size = 14 + self.pop_animation_frame
            self.canvas.create_text(
                self.pop_x, self.pop_y - 40 - self.pop_animation_frame * 2,
                text="💥 RAAK!",
                font=('Segoe UI', int(text_size), 'bold'),
                fill='#ffe66d',
                tags="game_objects"
            )
        
        # Teken particles
        for particle in self.pop_particles:
            if particle['life'] > 0 and particle['size'] > 1:
                size = particle['size'] * particle['life']
                
                self.canvas.create_oval(
                    particle['x'] - size, particle['y'] - size,
                    particle['x'] + size, particle['y'] + size,
                    fill=particle['color'], outline='',
                    tags="game_objects"
                )
        
        # Teken explosie ring
        if self.pop_animation_frame < 12:
            ring_size = self.pop_animation_frame * 8
            ring_width = max(1, 4 - self.pop_animation_frame // 3)
            self.canvas.create_oval(
                self.pop_x - ring_size, self.pop_y - ring_size,
                self.pop_x + ring_size, self.pop_y + ring_size,
                outline='#ff6600', width=ring_width,
                tags="game_objects"
            )
    
    def refresh_ports(self):
        """Vernieuwt de lijst met beschikbare COM poorten."""
        ports = serial.tools.list_ports.comports()
        port_list = [port.device for port in ports]
        
        self.port_combo['values'] = port_list
        
        if port_list:
            if 'COM4' in port_list:
                self.port_combo.set('COM4')
            else:
                self.port_combo.set(port_list[0])
    
    def toggle_connection(self):
        if self.is_connected:
            self.disconnect()
        else:
            self.connect()
    
    def connect(self):
        port = self.port_var.get()
        
        if not port:
            messagebox.showerror("Fout", "Selecteer eerst een COM poort!")
            return
        
        try:
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
            
            self.read_thread = threading.Thread(target=self.read_serial_data, daemon=True)
            self.read_thread.start()
            
            self.connect_btn.config(text="Verbreken", bg='#ff6b6b')
            self.status_label.config(text=f"● Verbonden met {port}", fg='#4ecca3')
            
        except serial.SerialException as e:
            messagebox.showerror("Verbindingsfout", f"Kan niet verbinden met {port}:\n{e}")
    
    def disconnect(self):
        self.running = False
        
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
        
        self.is_connected = False
        self.serial_port = None
        
        self.connect_btn.config(text="Verbinden", bg='#4ecca3')
        self.status_label.config(text="● Niet verbonden", fg='#ff6b6b')
    
    def read_serial_data(self):
        """Leest continu data van de seriële poort."""
        # Regex met Pop veld en Pop positie (Paz, Pel)
        pattern = r'GAME:Az=([+-]?\d+\.?\d*),El=([+-]?\d+\.?\d*),Baz=([+-]?\d+\.?\d*),Bel=([+-]?\d+\.?\d*),Score=(\d+),Active=(\d+),Hit=(\d+),Pop=(\d+),Paz=([+-]?\d+\.?\d*),Pel=([+-]?\d+\.?\d*)'
        
        buffer = ""
        
        while self.running:
            try:
                if self.serial_port and self.serial_port.is_open:
                    if self.serial_port.in_waiting > 0:
                        data = self.serial_port.read(self.serial_port.in_waiting)
                        buffer += data.decode('utf-8', errors='ignore')
                        
                        while '\n' in buffer:
                            line, buffer = buffer.split('\n', 1)
                            line = line.strip()
                            
                            match = re.search(pattern, line)
                            if match:
                                self.azimuth = float(match.group(1))
                                self.elevation = float(match.group(2))
                                self.balloon_az = float(match.group(3))
                                self.balloon_el = float(match.group(4))
                                self.score = int(match.group(5))
                                self.game_active = (match.group(6) == '1')
                                self.in_hit_zone = (match.group(7) == '1')
                                
                                # Check voor pop event
                                if match.group(8) == '1':
                                    # Gebruik de meegstuurde pop positie (waar ballon was)
                                    self.pop_position_az = float(match.group(9))
                                    self.pop_position_el = float(match.group(10))
                                    self.balloon_popped = True
                                
            except Exception as e:
                print(f"Leesfout: {e}")
                time.sleep(0.1)
            
            time.sleep(0.01)
    
    def update_visualization(self):
        """Update de visualisatie (~30 FPS)."""
        # Update animatie counters
        self.balloon_pulse += 0.15
        
        # Check of er een nieuwe pop event is
        if self.balloon_popped and not self.pop_animation_active:
            # Start plof animatie op de OPGESLAGEN positie (waar ballon was)
            pop_x, pop_y = self.angle_to_canvas(self.pop_position_az, self.pop_position_el)
            self.start_pop_animation(pop_x, pop_y)
            self.balloon_popped = False  # Reset flag
        
        # Update plof animatie
        self.update_pop_animation()
        
        # Verwijder oude game objecten
        self.canvas.delete("game_objects")
        
        # Bereken posities
        aim_x, aim_y = self.angle_to_canvas(self.azimuth, self.elevation)
        balloon_x, balloon_y = self.angle_to_canvas(self.balloon_az, self.balloon_el)
        
        # Bereken afstand
        distance = math.sqrt(
            (self.azimuth - self.balloon_az) ** 2 + 
            (self.elevation - self.balloon_el) ** 2
        )
        
        # === TEKEN TREFZONE ===
        if self.game_active and not self.pop_animation_active:
            hit_radius_pixels = self.HIT_RADIUS * self.scale_x
            zone_color = '#3a5a3a' if not self.in_hit_zone else '#5a8a5a'
            self.canvas.create_oval(
                balloon_x - hit_radius_pixels, balloon_y - hit_radius_pixels,
                balloon_x + hit_radius_pixels, balloon_y + hit_radius_pixels,
                outline=zone_color, width=2, dash=(5, 5),
                tags="game_objects"
            )
        
        # === TEKEN LIJN VAN VIZIER NAAR BALLON ===
        if self.game_active and not self.pop_animation_active:
            line_color = '#4ecca3' if self.in_hit_zone else '#444'
            self.canvas.create_line(
                aim_x, aim_y, balloon_x, balloon_y,
                fill=line_color, width=1, dash=(3, 3),
                tags="game_objects"
            )
        
        # === TEKEN BALLON (alleen als er geen plof animatie is) ===
        if self.game_active and not self.pop_animation_active:
            pulse = math.sin(self.balloon_pulse) * 3
            balloon_size = 20 + pulse
            
            if self.in_hit_zone:
                balloon_color = '#ff4444'
                balloon_outline = '#ffaaaa'
            else:
                balloon_color = '#cc3333'
                balloon_outline = '#ff6666'
            
            # Ballon body
            self.canvas.create_oval(
                balloon_x - balloon_size, balloon_y - balloon_size,
                balloon_x + balloon_size, balloon_y + balloon_size,
                fill=balloon_color, outline=balloon_outline, width=2,
                tags="game_objects"
            )
            
            # Touwtje
            self.canvas.create_line(
                balloon_x, balloon_y + balloon_size,
                balloon_x, balloon_y + balloon_size + 15,
                fill='#888', width=1,
                tags="game_objects"
            )
            
            # Highlight
            self.canvas.create_oval(
                balloon_x - balloon_size * 0.5, balloon_y - balloon_size * 0.6,
                balloon_x - balloon_size * 0.2, balloon_y - balloon_size * 0.3,
                fill='#ff8888', outline='',
                tags="game_objects"
            )
        
        # === TEKEN PLOF ANIMATIE ===
        self.draw_pop_animation()
        
        # === TEKEN TRAIL ===
        self.trail_history.append((aim_x, aim_y))
        if len(self.trail_history) > 1:
            for i, (tx, ty) in enumerate(self.trail_history):
                alpha = i / len(self.trail_history)
                size = 2 + alpha * 4
                green_val = int(0x4e + (0xcc - 0x4e) * alpha)
                color = f'#{0x4e:02x}{green_val:02x}{0xa3:02x}'
                self.canvas.create_oval(
                    tx - size/2, ty - size/2,
                    tx + size/2, ty + size/2,
                    fill=color, outline='',
                    tags="game_objects"
                )
        
        # === TEKEN VIZIER ===
        vizier_size = 15
        vizier_color = '#4ecca3' if not self.in_hit_zone else '#7fff7f'
        
        self.canvas.create_oval(
            aim_x - vizier_size, aim_y - vizier_size,
            aim_x + vizier_size, aim_y + vizier_size,
            outline=vizier_color, width=2,
            tags="game_objects"
        )
        
        self.canvas.create_line(
            aim_x - vizier_size - 5, aim_y,
            aim_x + vizier_size + 5, aim_y,
            fill=vizier_color, width=2,
            tags="game_objects"
        )
        self.canvas.create_line(
            aim_x, aim_y - vizier_size - 5,
            aim_x, aim_y + vizier_size + 5,
            fill=vizier_color, width=2,
            tags="game_objects"
        )
        
        self.canvas.create_oval(
            aim_x - 3, aim_y - 3,
            aim_x + 3, aim_y + 3,
            fill=vizier_color, outline='',
            tags="game_objects"
        )
        
        # === UPDATE LABELS ===
        self.score_label.config(text=str(self.score))
        self.aim_label.config(text=f"Az: {self.azimuth:+.1f}°  El: {self.elevation:+.1f}°")
        self.balloon_label.config(text=f"Az: {self.balloon_az:+.1f}°  El: {self.balloon_el:+.1f}°")
        self.distance_label.config(text=f"{distance:.1f}°")
        
        if distance <= self.HIT_RADIUS:
            self.distance_label.config(fg='#4ecca3')
        elif distance <= self.HIT_RADIUS * 2:
            self.distance_label.config(fg='#ffe66d')
        else:
            self.distance_label.config(fg='#ff6b6b')
        
        if self.game_active:
            self.game_status_label.config(text="▶ ACTIEF", fg='#4ecca3')
        else:
            self.game_status_label.config(text="⏸ IDLE", fg='#888')
        
        # Plan de volgende update
        self.root.after(33, self.update_visualization)
    
    def on_closing(self):
        self.running = False
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
        self.root.destroy()


def main():
    root = tk.Tk()
    
    try:
        root.iconbitmap(default='')
    except:
        pass
    
    app = BalloonShooterVisualizer(root)
    
    root.protocol("WM_DELETE_WINDOW", app.on_closing)
    
    root.mainloop()


if __name__ == "__main__":
    print("=" * 50)
    print("🎈 Ballonnenschieter Visualisatie")
    print("=" * 50)
    print("\nBenodigde library: pyserial")
    print("Installeer met: pip install pyserial")
    print("\nStarten van de applicatie...")
    print("-" * 50)
    
    main()
