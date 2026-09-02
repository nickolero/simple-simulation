import tkinter as tk
import subprocess
import threading
import time
import random
import queue 

class DroneVisualizer:
    def __init__(self, root):
        self.root = root
        self.root.title("MCTS Drone - Active Frontiers")
        
        # Default starting values
        self.width = 30
        self.height = 30
        self.max_canvas_dim = 700 
        self.drone_pos = (0, 0)
        self.running = False
        self.frontiers = set() # NEW: Store active frontiers
        
        self.ui_queue = queue.Queue()
        
        # --- Top Control Panel ---
        control_frame = tk.Frame(root)
        control_frame.pack(pady=8)
        
        tk.Label(control_frame, text="W:").grid(row=0, column=0)
        self.width_input = tk.Entry(control_frame, width=4)
        self.width_input.insert(0, str(self.width))
        self.width_input.grid(row=0, column=1, padx=2)
        
        tk.Label(control_frame, text="H:").grid(row=0, column=2)
        self.height_input = tk.Entry(control_frame, width=4)
        self.height_input.insert(0, str(self.height))
        self.height_input.grid(row=0, column=3, padx=2)

        tk.Label(control_frame, text="Density (%):").grid(row=0, column=4, padx=(8, 2))
        self.density_input = tk.Entry(control_frame, width=4)
        self.density_input.insert(0, "25")
        self.density_input.grid(row=0, column=5, padx=2)

        tk.Label(control_frame, text="Block Size:").grid(row=0, column=6, padx=(8, 2))
        self.block_size_input = tk.Entry(control_frame, width=3)
        self.block_size_input.insert(0, "3")
        self.block_size_input.grid(row=0, column=7, padx=2)
        
        self.apply_btn = tk.Button(control_frame, text="Resize", command=self.apply_new_size)
        self.apply_btn.grid(row=0, column=8, padx=4)

        self.gen_btn = tk.Button(control_frame, text="Generate Map", command=self.generate_random_map)
        self.gen_btn.grid(row=0, column=9, padx=4)
        
        self.start_btn = tk.Button(control_frame, text="Start Simulation", command=self.start_simulation, bg="#bbf7d0")
        self.start_btn.grid(row=0, column=10, padx=6)
        
        # --- The Canvas ---
        self.canvas = tk.Canvas(root, bg="white")
        self.canvas.pack(pady=5)
        self.canvas.bind("<Button-1>", self.toggle_obstacle)
        
        self.apply_new_size()

    def calculate_tile_size(self):
        self.tile_size = max(8, min(self.max_canvas_dim // self.width, self.max_canvas_dim // self.height))

    def apply_new_size(self):
        if self.running: return 
        
        try:
            new_w = int(self.width_input.get())
            new_h = int(self.height_input.get())
            if new_w < 5 or new_h < 5: return
            self.width = new_w
            self.height = new_h
        except ValueError:
            return 
            
        self.calculate_tile_size()
        self.grid = [[0 for _ in range(self.width)] for _ in range(self.height)]
        self.visit_counts = [[0 for _ in range(self.width)] for _ in range(self.height)]
        self.explored = [[False for _ in range(self.width)] for _ in range(self.height)]
        self.frontiers = set() # Reset frontiers on resize
        
        self.drone_pos = (0, 0)
        self.visit_counts[0][0] = 1 
        
        self.canvas.config(width=self.width * self.tile_size, height=self.height * self.tile_size)
        self.draw_grid()

    def generate_random_map(self):
        if self.running: return
        self.apply_new_size()

        try:
            density = float(self.density_input.get()) / 100.0
            density = max(0.0, min(0.60, density))
        except ValueError:
            density = 0.25

        try:
            block_size = int(self.block_size_input.get())
            block_size = max(1, block_size)
        except ValueError:
            block_size = 3

        macro_density = density * 0.6  
        micro_density = density * 0.4  

        coarse_w = self.width // block_size
        coarse_h = self.height // block_size
        coarse_grid = [[0 for _ in range(coarse_w)] for _ in range(coarse_h)]

        for cy in range(coarse_h):
            for cx in range(coarse_w):
                if max(abs(cx - 0), abs(cy - 0)) <= 1: 
                    continue
                if random.random() < macro_density:
                    coarse_grid[cy][cx] = 1

        for y in range(self.height):
            for x in range(self.width):
                cy = y // block_size
                cx = x // block_size
                if cy < coarse_h and cx < coarse_w:
                    self.grid[y][x] = coarse_grid[cy][cx]
                else:
                    self.grid[y][x] = 1 

        for y in range(self.height):
            for x in range(self.width):
                if max(abs(x - 0), abs(y - 0)) <= 2: 
                    continue
                if self.grid[y][x] == 0 and random.random() < micro_density:
                    self.grid[y][x] = 1

        self.grid[0][0] = 0
        self.grid[0][1] = 0
        self.grid[1][0] = 0
        self.grid[1][1] = 0 
        self.draw_grid()

    def draw_grid(self):
        self.canvas.delete("all")
        show_numbers = self.tile_size >= 24  

        for y in range(self.height):
            for x in range(self.width):
                x1, y1 = x * self.tile_size, y * self.tile_size
                x2, y2 = x1 + self.tile_size, y1 + self.tile_size
                
                color = "white"
                if self.grid[y][x] == 1:
                    color = "#333333"
                elif self.visit_counts[y][x] > 0:
                    color = "#a8f0b1"
                elif self.explored[y][x]:
                    color = "#fffac8"
                
                # OVERRIDE: If this tile is an active frontier, paint it purple
                if hasattr(self, 'frontiers') and (x, y) in self.frontiers:
                    color = "#c084fc"
                    
                self.canvas.create_rectangle(x1, y1, x2, y2, fill=color, outline="#e5e7eb" if self.tile_size < 15 else "gray")
                
                if show_numbers and self.visit_counts[y][x] > 0 and self.grid[y][x] == 0:
                    center_x = x1 + (self.tile_size / 2)
                    center_y = y1 + (self.tile_size / 2)
                    self.canvas.create_text(center_x, center_y, text=str(self.visit_counts[y][x]), fill="black", font=("Arial", max(7, self.tile_size // 3), "bold"))
                
                if (x, y) == self.drone_pos:
                    pad = max(2, self.tile_size // 8)
                    self.canvas.create_oval(x1 + pad, y1 + pad, x2 - pad, y2 - pad, fill="blue")

                # NEW: Draw the paths as a persistent layer so they don't get deleted!
                if hasattr(self, 'mcts_plan') and self.mcts_plan:
                    line_points = [self.drone_pos[0] * self.tile_size + (self.tile_size / 2),
                                self.drone_pos[1] * self.tile_size + (self.tile_size / 2)]
                    for i in range(0, len(self.mcts_plan) - 1, 2):
                        px, py = self.mcts_plan[i], self.mcts_plan[i+1]
                        line_points.extend([px * self.tile_size + (self.tile_size / 2), py * self.tile_size + (self.tile_size / 2)])
                        x1, y1 = px * self.tile_size, py * self.tile_size
                        x2, y2 = x1 + self.tile_size, y1 + self.tile_size
                        pad = max(2, self.tile_size // 4)
                        self.canvas.create_rectangle(x1 + pad, y1 + pad, x2 - pad, y2 - pad, fill="red", tags="plan")
                    if len(line_points) >= 4:
                        self.canvas.create_line(line_points, fill="red", width=2, tags="plan")

                if hasattr(self, 'astar_plan') and self.astar_plan:
                    line_points = []
                    for i in range(0, len(self.astar_plan), 2):
                        px, py = self.astar_plan[i], self.astar_plan[i+1]
                        line_points.extend([px * self.tile_size + (self.tile_size / 2), py * self.tile_size + (self.tile_size / 2)])
                    if len(line_points) >= 4:
                        self.canvas.create_line(line_points, fill="magenta", width=2, dash=(4, 2), tags="astar_plan")

    def toggle_obstacle(self, event):
        if self.running: return 
        x = event.x // self.tile_size
        y = event.y // self.tile_size
        if (x, y) != self.drone_pos and 0 <= x < self.width and 0 <= y < self.height:
            self.grid[y][x] = 1 if self.grid[y][x] == 0 else 0
            self.draw_grid()

    def start_simulation(self):
        self.running = True
        self.start_btn.config(state=tk.DISABLED)
        self.apply_btn.config(state=tk.DISABLED)
        self.gen_btn.config(state=tk.DISABLED)
        
        self.root.after(20, self.process_queue)
        threading.Thread(target=self.run_cpp_brain, daemon=True).start()

    def has_line_of_sight(self, x0, y0, x1, y1):
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy

        while True:
            # 1. Target reached
            if x0 == x1 and y0 == y1:
                return True
            
            # 2. Collision check (Python only tracks 0 and 1 for obstacles)
            if self.grid[y0][x0] == 1:
                return False
                
            e2 = 2 * err
            prev_x, prev_y = x0, y0
            
            # 3. Strict < and >
            if e2 > -dy:
                err -= dy
                x0 += sx
            if e2 < dx:
                err += dx
                y0 += sy
                
            # 4. Diagonal check
            if x0 != prev_x or y0 != prev_y:
                if self.grid[prev_y][x0] == 1 or self.grid[y0][prev_x] == 1:
                    return False

    def process_queue(self):
        try:
            while True:
                msg_type, data = self.ui_queue.get_nowait()
                
                if msg_type == "FRONTIERS":
                    self.frontiers = data
                    self.draw_grid()

                elif msg_type == "ASTAR":
                    self.astar_plan = data
                    self.mcts_plan = [] # Clear old MCTS plans
                    self.draw_grid()
                        
                elif msg_type == "PATH":
                    self.mcts_plan = data
                    self.astar_plan = [] # Clear old A* plans
                    self.draw_grid()
                        
                elif msg_type == "MOVE":
                    new_x, new_y = data
                    self.drone_pos = (new_x, new_y)
                    self.visit_counts[new_y][new_x] += 1

                    radius = 4
                    radius_squared = radius ** 2
                    
                    for dy in range(-radius, radius + 1):
                        for dx in range(-radius, radius + 1):
                            if (dx**2 + dy**2) > radius_squared:
                                continue
                            ex, ey = new_x + dx, new_y + dy
                            if 0 <= ex < self.width and 0 <= ey < self.height:
                                if self.has_line_of_sight(new_x, new_y, ex, ey):
                                    self.explored[ey][ex] = True
                    self.draw_grid()
                    
                elif msg_type == "HALT":
                    print(f"\n>>> SIMULATION HALTED: {data} <<<\n")
                    self.start_btn.config(state=tk.NORMAL)
                    self.apply_btn.config(state=tk.NORMAL)
                    self.gen_btn.config(state=tk.NORMAL)
                    self.running = False
                    
                elif msg_type == "PRINT":
                    print(f"[C++] {data}")

        except queue.Empty:
            pass
            
        if self.running:
            self.root.after(20, self.process_queue)

    def run_cpp_brain(self):
        process = subprocess.Popen(['./drone'], stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True)
        setup_data = f"{self.width} {self.height} {self.drone_pos[0]} {self.drone_pos[1]}\n"
        for row in self.grid:
            setup_data += " ".join(map(str, row)) + "\n"
        process.stdin.write(setup_data)
        process.stdin.flush()
        
        for line in process.stdout:
            line = line.strip()

            if line.startswith("FRONTIERS"):
                coords = list(map(int, line.split()[1:]))
                frontier_set = set()
                for i in range(0, len(coords), 2):
                    frontier_set.add((coords[i], coords[i+1]))
                self.ui_queue.put(("FRONTIERS", frontier_set))
                continue

            if line.startswith("ASTAR_PATH"):
                coords = list(map(int, line.split()[1:]))
                self.ui_queue.put(("ASTAR", coords))
                continue

            if line.startswith("PATH"):
                coords = list(map(int, line.split()[1:]))
                self.ui_queue.put(("PATH", coords))
                continue

            if line == "DONE" or line == "STUCK":
                self.ui_queue.put(("HALT", line))
                break
                
            parts = line.split()
            if len(parts) == 2 and parts[0].lstrip('-').isdigit() and parts[1].lstrip('-').isdigit():
                new_x, new_y = int(parts[0]), int(parts[1])
                self.ui_queue.put(("MOVE", (new_x, new_y)))
                time.sleep(0.04) 
            else:
                if not line.startswith("PATH") and not line.startswith("ASTAR_PATH") and not line.startswith("FRONTIERS"):
                    self.ui_queue.put(("PRINT", line))

if __name__ == "__main__":
    root = tk.Tk()
    app = DroneVisualizer(root)
    root.mainloop()