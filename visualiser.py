import tkinter as tk
import subprocess
import threading
import random
import queue
import csv
import json
import os
import statistics
from datetime import datetime
from tkinter import filedialog, messagebox

BASE_DIR = os.path.dirname(os.path.abspath(__file__))


class DroneVisualizer:
    DRONE_COLORS = [
        "#1d4ed8",  # 0: Royal Blue
        "#dc2626",  # 1: Crimson Red
        "#7c3aed",  # 2: Vivid Violet
        "#0d9488",  # 3: Teal
        "#d97706",  # 4: Amber
        "#db2777",  # 5: Magenta
        "#4f46e5",  # 6: Indigo
        "#059669",  # 7: Emerald
    ]

    # Lighter pastel tints for each drone's 8 candidate sparse paths
    DRONE_LIGHT_COLORS = [
        "#93c5fd",  # 0: Light Blue
        "#fca5a5",  # 1: Light Red
        "#c4b5fd",  # 2: Light Violet
        "#5eead4",  # 3: Light Teal
        "#fcd34d",  # 4: Light Amber
        "#f9a8d4",  # 5: Light Pink
        "#a5b4fc",  # 6: Light Indigo
        "#6ee7b7",  # 7: Light Emerald
    ]

    def __init__(self, root):
        self.root = root
        self.root.title("Dec-MCTS Multi-UAV - Active Frontiers, SaR & Checkpoints")

        self.width = 30
        self.height = 30
        self.max_canvas_dim = 700
        self.drone_positions = [(0, 0)]
        self.launch_positions = [(0, 0)]
        self.running = False
        self.frontiers = set()
        self.found_sar = set()
        self.found_cp = set()
        self.mcts_plans = {}
        self.best_full_plans = {}
        self.sparse_plans = {}
        self.astar_plans = {}
        self.tsdf_grid = []
        self.current_map_name = "unsaved_map"
        self.current_run_metrics = []
        self.last_run_stats = None
        self.last_run_heatmap = []
        self.run_id = None

        self.ui_queue = queue.Queue()

        # --- Top Control Panel (Row 0: Map & Sensors) ---
        control_frame = tk.Frame(root)
        control_frame.pack(pady=4)

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

        self.speed_slider = tk.Scale(control_frame, from_=5, to=1000, orient=tk.HORIZONTAL, label="Delay (ms)", resolution=5)
        self.speed_slider.set(100)
        self.speed_slider.grid(row=0, column=11, padx=8)

        self.show_tsdf = tk.BooleanVar(value=False)
        self.tsdf_btn = tk.Checkbutton(control_frame, text="Show TSDF", variable=self.show_tsdf, command=self.draw_grid)
        self.tsdf_btn.grid(row=0, column=12, padx=4)

        self.fov_slider = tk.Scale(control_frame, from_=2, to=10, orient=tk.HORIZONTAL, label="FOV", resolution=1)
        self.fov_slider.set(7)
        self.fov_slider.grid(row=0, column=13, padx=8)

        self.camera_fov_slider = tk.Scale(control_frame, from_=1, to=5, orient=tk.HORIZONTAL, label="Camera FOV", resolution=1)
        self.camera_fov_slider.set(3)
        self.camera_fov_slider.grid(row=0, column=14, padx=8)

        # --- Second Control Panel (Row 1: UAV Placement, SaR, Checkpoints & Path Debuggers) ---
        mission_frame = tk.Frame(root)
        mission_frame.pack(pady=4)

        self.uav_mode = tk.BooleanVar(value=False)
        self.uav_cb = tk.Checkbutton(
            mission_frame, text="UAVs", variable=self.uav_mode,
            font=("Arial", 10, "bold"), fg="#1d4ed8"
        )
        self.uav_cb.grid(row=0, column=0, padx=(4, 6))

        tk.Label(mission_frame, text="|").grid(row=0, column=1, padx=4)

        # SaR Controls (Grid value = 3)
        self.sar_enabled = tk.BooleanVar(value=False)
        self.sar_cb = tk.Checkbutton(mission_frame, text="SaR", variable=self.sar_enabled, font=("Arial", 10, "bold"))
        self.sar_cb.grid(row=0, column=2, padx=(4, 2))

        tk.Label(mission_frame, text="Qty:").grid(row=0, column=3)
        self.sar_qty_input = tk.Entry(mission_frame, width=3)
        self.sar_qty_input.insert(0, "3")
        self.sar_qty_input.grid(row=0, column=4, padx=2)

        self.sar_rtl = tk.BooleanVar(value=True)
        self.sar_rtl_cb = tk.Checkbutton(mission_frame, text="RTL", variable=self.sar_rtl)
        self.sar_rtl_cb.grid(row=0, column=5, padx=(2, 8))

        tk.Label(mission_frame, text="|").grid(row=0, column=6, padx=4)

        # Checkpoints Controls (Grid value = 4)
        self.cp_enabled = tk.BooleanVar(value=False)
        self.cp_cb = tk.Checkbutton(mission_frame, text="Checkpoints", variable=self.cp_enabled, font=("Arial", 10, "bold"))
        self.cp_cb.grid(row=0, column=7, padx=(4, 2))

        tk.Label(mission_frame, text="Qty:").grid(row=0, column=8)
        self.cp_qty_input = tk.Entry(mission_frame, width=3)
        self.cp_qty_input.insert(0, "3")
        self.cp_qty_input.grid(row=0, column=9, padx=2)

        self.cp_coords = tk.BooleanVar(value=True)
        self.cp_coords_cb = tk.Checkbutton(mission_frame, text="Coordinates", variable=self.cp_coords)
        self.cp_coords_cb.grid(row=0, column=10, padx=2)

        self.cp_rtl = tk.BooleanVar(value=True)
        self.cp_rtl_cb = tk.Checkbutton(mission_frame, text="RTL", variable=self.cp_rtl)
        self.cp_rtl_cb.grid(row=0, column=11, padx=(2, 8))

        tk.Label(mission_frame, text="|").grid(row=0, column=12, padx=4)

        # New Rollout Visualization Checkboxes
        self.show_best_path = tk.BooleanVar(value=False)
        self.best_path_cb = tk.Checkbutton(
            mission_frame, text="Best Path (10)", variable=self.show_best_path,
            command=self.draw_grid, font=("Arial", 9, "bold"), fg="#b91c1c"
        )
        self.best_path_cb.grid(row=0, column=13, padx=4)

        self.show_all_paths = tk.BooleanVar(value=False)
        self.all_paths_cb = tk.Checkbutton(
            mission_frame, text="All Paths (8)", variable=self.show_all_paths,
            command=self.draw_grid, font=("Arial", 9, "bold"), fg="#6d28d9"
        )
        self.all_paths_cb.grid(row=0, column=14, padx=4)

        benchmark_frame = tk.Frame(root)
        benchmark_frame.pack(pady=4)
        self.save_csv = tk.BooleanVar(value=True)
        tk.Checkbutton(benchmark_frame, text="Save CSV", variable=self.save_csv).grid(row=0, column=0, padx=4)
        tk.Label(benchmark_frame, text="Algorithm:").grid(row=0, column=1)
        self.algorithm_input = tk.Entry(benchmark_frame, width=12)
        self.algorithm_input.insert(0, "Dec-MCTS")
        self.algorithm_input.grid(row=0, column=2, padx=3)
        tk.Label(benchmark_frame, text="Trials:").grid(row=0, column=3)
        self.trials_input = tk.Entry(benchmark_frame, width=5)
        self.trials_input.insert(0, "10")
        self.trials_input.grid(row=0, column=4, padx=3)
        self.save_map_btn = tk.Button(benchmark_frame, text="Save Map", command=self.save_map)
        self.save_map_btn.grid(row=0, column=5, padx=3)
        self.load_map_btn = tk.Button(benchmark_frame, text="Load Map", command=self.load_map)
        self.load_map_btn.grid(row=0, column=6, padx=3)
        self.reset_map_btn = tk.Button(benchmark_frame, text="Reset Map", command=self.reset_map)
        self.reset_map_btn.grid(row=0, column=7, padx=3)
        self.batch_btn = tk.Button(benchmark_frame, text="Run Batch", command=self.run_batch)
        self.batch_btn.grid(row=0, column=8, padx=3)
        self.benchmark_status = tk.Label(root, text="Ready")
        self.benchmark_status.pack()

        # --- The Canvas ---
        self.canvas = tk.Canvas(root, bg="white")
        self.canvas.pack(pady=5)
        self.canvas.bind("<Button-1>", self.on_canvas_click)

        self.apply_new_size()

    def reset_runtime_state(self):
        self.visit_counts = [[0 for _ in range(self.width)] for _ in range(self.height)]
        self.explored = [[False for _ in range(self.width)] for _ in range(self.height)]
        self.camera_explored = [[False for _ in range(self.width)] for _ in range(self.height)]
        self.frontiers = set()
        self.found_sar = set()
        self.found_cp = set()
        self.mcts_plans = {}
        self.best_full_plans = {}
        self.sparse_plans = {}
        self.astar_plans = {}
        self.tsdf_grid = []
        self.current_run_metrics = []
        self.last_run_stats = None
        self.last_run_heatmap = []

    def set_benchmark_controls(self, state):
        for button in (self.save_map_btn, self.load_map_btn, self.reset_map_btn, self.batch_btn):
            button.config(state=state)

    def save_map(self):
        if self.running:
            return
        path = filedialog.asksaveasfilename(defaultextension=".json", filetypes=[("JSON Maps", "*.json")])
        if not path:
            return
        data = {
            "name": os.path.splitext(os.path.basename(path))[0],
            "width": self.width, "height": self.height, "grid": self.grid,
            "launch_positions": [list(p) for p in self.launch_positions],
            "fov": self.fov_slider.get(), "camera_fov": self.camera_fov_slider.get(),
            "sar_enabled": self.sar_enabled.get(), "sar_rtl": self.sar_rtl.get(),
            "cp_enabled": self.cp_enabled.get(), "cp_coords": self.cp_coords.get(),
            "cp_rtl": self.cp_rtl.get(),
        }
        with open(path, "w", encoding="utf-8") as output:
            json.dump(data, output, indent=2)
        self.current_map_name = data["name"]

    def load_map(self):
        if self.running:
            return
        path = filedialog.askopenfilename(filetypes=[("JSON Maps", "*.json")])
        if not path:
            return
        with open(path, encoding="utf-8") as source:
            data = json.load(source)
        self.width = int(data["width"])
        self.height = int(data["height"])
        self.width_input.delete(0, tk.END)
        self.width_input.insert(0, str(self.width))
        self.height_input.delete(0, tk.END)
        self.height_input.insert(0, str(self.height))
        self.grid = data["grid"]
        self.launch_positions = [tuple(p) for p in data["launch_positions"]]
        self.drone_positions = list(self.launch_positions)
        self.fov_slider.set(data.get("fov", self.fov_slider.get()))
        self.camera_fov_slider.set(data.get("camera_fov", self.camera_fov_slider.get()))
        self.sar_enabled.set(data.get("sar_enabled", False))
        self.sar_rtl.set(data.get("sar_rtl", True))
        self.cp_enabled.set(data.get("cp_enabled", False))
        self.cp_coords.set(data.get("cp_coords", True))
        self.cp_rtl.set(data.get("cp_rtl", True))
        self.current_map_name = data.get("name", os.path.splitext(os.path.basename(path))[0])
        self.calculate_tile_size()
        self.canvas.config(width=self.width * self.tile_size, height=self.height * self.tile_size)
        self.reset_runtime_state()
        self.draw_grid()

    def reset_map(self):
        if self.running:
            return
        self.drone_positions = list(self.launch_positions)
        self.reset_runtime_state()
        self.draw_grid()

    def calculate_tile_size(self):
        self.tile_size = max(8, min(self.max_canvas_dim // self.width, self.max_canvas_dim // self.height))

    def apply_new_size(self):
        if self.running:
            return

        try:
            new_w = int(self.width_input.get())
            new_h = int(self.height_input.get())
            if new_w < 5 or new_h < 5:
                return
            self.width = new_w
            self.height = new_h
        except ValueError:
            return

        self.calculate_tile_size()
        self.grid = [[0 for _ in range(self.width)] for _ in range(self.height)]
        self.visit_counts = [[0 for _ in range(self.width)] for _ in range(self.height)]
        self.explored = [[False for _ in range(self.width)] for _ in range(self.height)]
        self.camera_explored = [[False for _ in range(self.width)] for _ in range(self.height)]
        self.frontiers = set()
        self.found_sar = set()
        self.found_cp = set()
        self.mcts_plans = {}
        self.best_full_plans = {}
        self.sparse_plans = {}
        self.astar_plans = {}

        self.drone_positions = [(0, 0)]
        self.launch_positions = [(0, 0)]

        self.canvas.config(width=self.width * self.tile_size, height=self.height * self.tile_size)
        self.draw_grid()

    def is_near_any_drone(self, x, y, buffer_dist=2):
        for dx, dy in self.drone_positions:
            if max(abs(x - dx), abs(y - dy)) <= buffer_dist:
                return True
        return False

    def place_random_objects(self, tile_value, count):
        candidates = []
        for y in range(self.height):
            for x in range(self.width):
                if self.is_near_any_drone(x, y, 2):
                    continue
                if self.grid[y][x] != 0:
                    continue
                has_free_neighbor = False
                for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                    nx, ny = x + dx, y + dy
                    if 0 <= nx < self.width and 0 <= ny < self.height and self.grid[ny][nx] == 0:
                        has_free_neighbor = True
                        break
                if has_free_neighbor:
                    candidates.append((x, y))

        random.shuffle(candidates)
        for x, y in candidates[:max(0, count)]:
            self.grid[y][x] = tile_value

    def generate_random_map(self):
        if self.running:
            return
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

        macro_density = density * 1.0
        micro_density = density * 0.0

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

        if self.sar_enabled.get():
            try:
                sar_qty = int(self.sar_qty_input.get())
            except ValueError:
                sar_qty = 0
            self.place_random_objects(3, sar_qty)

        if self.cp_enabled.get():
            try:
                cp_qty = int(self.cp_qty_input.get())
            except ValueError:
                cp_qty = 0
            self.place_random_objects(4, cp_qty)

        self.draw_grid()

    def draw_grid(self):
        self.canvas.delete("all")
        show_numbers = self.tile_size >= 20

        for y in range(self.height):
            for x in range(self.width):
                x1, y1 = x * self.tile_size, y * self.tile_size
                x2, y2 = x1 + self.tile_size, y1 + self.tile_size

                tile_val = self.grid[y][x]
                color = "white"

                if self.show_tsdf.get() and hasattr(self, 'tsdf_grid') and self.tsdf_grid:
                    val = self.tsdf_grid[y][x]
                    if val <= 0.0:
                        color = "#333333"
                    elif val >= 5.0:
                        color = "white"
                    else:
                        intensity = int((val / 5.0) * 255)
                        color = f"#ff{intensity:02x}{intensity:02x}"
                elif self.visit_counts[y][x] > 0 or self.explored[y][x]:
                    color = "#a8f0b1"

                if (x, y) in self.frontiers:
                    color = "#c084fc"
                elif self.camera_explored[y][x]:
                    color = "#a8d8ff"

                if tile_val == 1:
                    color = "#333333"
                elif tile_val == 3:
                    color = "#39ff14" if (x, y) in self.found_sar else "#6b8e6b"
                elif tile_val == 4:
                    color = "#ff6700" if (x, y) in self.found_cp else "#a3704c"

                self.canvas.create_rectangle(
                    x1, y1, x2, y2,
                    fill=color,
                    outline="#e5e7eb" if self.tile_size < 15 else "gray"
                )

                if show_numbers and self.visit_counts[y][x] > 0 and tile_val not in (1, 3):
                    center_x = x1 + (self.tile_size / 2)
                    center_y = y1 + (self.tile_size / 2)
                    self.canvas.create_text(
                        center_x, center_y,
                        text=str(self.visit_counts[y][x]),
                        fill="black",
                        font=("Arial", max(7, self.tile_size // 3), "bold")
                    )

        # LAYER 1 (Bottom): Draw all 8 Sparse Candidate Paths if "All Paths (8)" is checked
        if self.show_all_paths.get():
            for drone_id, paths_list in self.sparse_plans.items():
                if not paths_list or drone_id >= len(self.drone_positions):
                    continue
                light_col = self.DRONE_LIGHT_COLORS[drone_id % len(self.DRONE_LIGHT_COLORS)]
                dark_col = self.DRONE_COLORS[drone_id % len(self.DRONE_COLORS)]
                dx, dy = self.drone_positions[drone_id]
                start_cx = dx * self.tile_size + (self.tile_size / 2)
                start_cy = dy * self.tile_size + (self.tile_size / 2)

                for cand_coords in paths_list:
                    if len(cand_coords) < 2:
                        continue
                    pts = [start_cx, start_cy]
                    for i in range(0, len(cand_coords) - 1, 2):
                        px, py = cand_coords[i], cand_coords[i + 1]
                        pts.extend([
                            px * self.tile_size + (self.tile_size / 2),
                            py * self.tile_size + (self.tile_size / 2)
                        ])
                    if len(pts) >= 4:
                        self.canvas.create_line(pts, fill=dark_col, width=1, dash=(2, 3), tags="sparse_plan")
                        end_x, end_y = pts[-2], pts[-1]
                        r = max(2, self.tile_size // 7)
                        self.canvas.create_oval(
                            end_x - r, end_y - r, end_x + r, end_y + r,
                            fill=light_col, outline=dark_col, tags="sparse_plan"
                        )

        # LAYER 2 (Middle): Draw Full 10-Step Winning Rollout if "Best Path (10)" is checked
        if self.show_best_path.get():
            for drone_id, full_plan in self.best_full_plans.items():
                if not full_plan or drone_id >= len(self.drone_positions):
                    continue
                d_color = self.DRONE_COLORS[drone_id % len(self.DRONE_COLORS)]
                dx, dy = self.drone_positions[drone_id]
                line_points = [
                    dx * self.tile_size + (self.tile_size / 2),
                    dy * self.tile_size + (self.tile_size / 2)
                ]
                for i in range(0, len(full_plan) - 1, 2):
                    px, py = full_plan[i], full_plan[i + 1]
                    cx = px * self.tile_size + (self.tile_size / 2)
                    cy = py * self.tile_size + (self.tile_size / 2)
                    line_points.extend([cx, cy])
                    r = max(2, self.tile_size // 6)
                    self.canvas.create_oval(
                        cx - r, cy - r, cx + r, cy + r,
                        outline=d_color, width=1, tags="best_full_plan"
                    )
                if len(line_points) >= 4:
                    self.canvas.create_line(
                        line_points, fill=d_color, width=2, dash=(5, 2), tags="best_full_plan"
                    )

        # LAYER 3 (Top): Draw each drone's active 3-step committed chain
        for drone_id, plan in self.mcts_plans.items():
            if not plan or drone_id >= len(self.drone_positions):
                continue
            d_color = self.DRONE_COLORS[drone_id % len(self.DRONE_COLORS)]
            dx, dy = self.drone_positions[drone_id]
            line_points = [
                dx * self.tile_size + (self.tile_size / 2),
                dy * self.tile_size + (self.tile_size / 2)
            ]
            for i in range(0, len(plan) - 1, 2):
                px, py = plan[i], plan[i + 1]
                line_points.extend([
                    px * self.tile_size + (self.tile_size / 2),
                    py * self.tile_size + (self.tile_size / 2)
                ])
                x1, y1 = px * self.tile_size, py * self.tile_size
                x2, y2 = x1 + self.tile_size, y1 + self.tile_size
                pad = max(2, self.tile_size // 4)
                self.canvas.create_rectangle(x1 + pad, y1 + pad, x2 - pad, y2 - pad, fill="red", outline=d_color, tags="plan")
            if len(line_points) >= 4:
                self.canvas.create_line(line_points, fill="red", width=2, tags="plan")

        # Draw each drone's A* RTL path
        for drone_id, astar_plan in self.astar_plans.items():
            if not astar_plan or drone_id >= len(self.drone_positions):
                continue
            d_color = self.DRONE_COLORS[drone_id % len(self.DRONE_COLORS)]
            dx, dy = self.drone_positions[drone_id]
            line_points = [
                dx * self.tile_size + (self.tile_size / 2),
                dy * self.tile_size + (self.tile_size / 2)
            ]
            for i in range(0, len(astar_plan) - 1, 2):
                px, py = astar_plan[i], astar_plan[i + 1]
                line_points.extend([
                    px * self.tile_size + (self.tile_size / 2),
                    py * self.tile_size + (self.tile_size / 2)
                ])
            if len(line_points) >= 4:
                self.canvas.create_line(line_points, fill=d_color, width=3, dash=(2, 4), tags="astar_plan")

        # Draw UAVs
        for idx, (dx, dy) in enumerate(self.drone_positions):
            x1, y1 = dx * self.tile_size, dy * self.tile_size
            x2, y2 = x1 + self.tile_size, y1 + self.tile_size
            pad = max(2, self.tile_size // 8)
            d_color = self.DRONE_COLORS[idx % len(self.DRONE_COLORS)]
            self.canvas.create_oval(x1 + pad, y1 + pad, x2 - pad, y2 - pad, fill=d_color, outline="white", width=1)
            if self.tile_size >= 14:
                self.canvas.create_text(
                    x1 + self.tile_size / 2,
                    y1 + self.tile_size / 2,
                    text=str(idx),
                    fill="white",
                    font=("Arial", max(7, self.tile_size // 3), "bold")
                )

    def on_canvas_click(self, event):
        if self.running:
            return
        x = event.x // self.tile_size
        y = event.y // self.tile_size
        if not (0 <= x < self.width and 0 <= y < self.height):
            return

        if self.uav_mode.get():
            if (x, y) in self.drone_positions:
                if len(self.drone_positions) > 1:
                    self.drone_positions.remove((x, y))
            else:
                if self.grid[y][x] in (1, 3):
                    self.grid[y][x] = 0
                self.drone_positions.append((x, y))
            self.launch_positions = list(self.drone_positions)
            self.draw_grid()
            return

        if (x, y) not in self.drone_positions:
            curr = self.grid[y][x]
            if curr == 0:
                self.grid[y][x] = 1
            elif curr == 1 and self.sar_enabled.get():
                self.grid[y][x] = 3
            elif curr in (1, 3) and self.cp_enabled.get():
                self.grid[y][x] = 4
            else:
                self.grid[y][x] = 0
            self.draw_grid()

    def start_simulation(self):
        if not self.drone_positions:
            self.drone_positions = [(0, 0)]
        if not self.running and hasattr(self, 'launch_positions') and len(self.launch_positions) == len(self.drone_positions):
            self.drone_positions = list(self.launch_positions)
        else:
            self.launch_positions = list(self.drone_positions)

        self.reset_runtime_state()

        for dx, dy in self.drone_positions:
            if self.grid[dy][dx] in (1, 3):
                self.grid[dy][dx] = 0

        has_sar_on_grid = any(self.grid[y][x] == 3 for y in range(self.height) for x in range(self.width))
        if self.sar_enabled.get() and not has_sar_on_grid:
            try:
                self.place_random_objects(3, int(self.sar_qty_input.get()))
            except ValueError:
                pass

        has_cp_on_grid = any(self.grid[y][x] == 4 for y in range(self.height) for x in range(self.width))
        if self.cp_enabled.get() and not has_cp_on_grid:
            try:
                self.place_random_objects(4, int(self.cp_qty_input.get()))
            except ValueError:
                pass

        self.draw_grid()

        self.running = True
        self.start_btn.config(state=tk.DISABLED)
        self.apply_btn.config(state=tk.DISABLED)
        self.gen_btn.config(state=tk.DISABLED)
        self.set_benchmark_controls(tk.DISABLED)
        self.run_id = datetime.now().strftime("%Y%m%d_%H%M%S")

        self.root.after(20, self.process_queue)
        threading.Thread(target=self.run_cpp_brain, daemon=True).start()

    def is_opaque(self, x, y):
        return self.grid[y][x] in (1, 3)

    def has_line_of_sight(self, x0, y0, x1, y1):
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy

        while True:
            if x0 == x1 and y0 == y1:
                return True
            if self.is_opaque(x0, y0):
                return False
            e2 = 2 * err
            prev_x, prev_y = x0, y0
            if e2 > -dy:
                err -= dy
                x0 += sx
            if e2 < dx:
                err += dx
                y0 += sy
            if x0 != prev_x and y0 != prev_y:
                if self.is_opaque(x0, prev_y) or self.is_opaque(prev_x, y0):
                    return False

    def observe_from_position(self, new_x, new_y):
        radius = self.fov_slider.get()
        radius_squared = radius ** 2
        for dy in range(-radius, radius + 1):
            for dx in range(-radius, radius + 1):
                if (dx ** 2 + dy ** 2) > radius_squared:
                    continue
                ex, ey = new_x + dx, new_y + dy
                if 0 <= ex < self.width and 0 <= ey < self.height:
                    if not self.is_opaque(ex, ey) and self.has_line_of_sight(new_x, new_y, ex, ey):
                        self.explored[ey][ex] = True

        camera_radius = self.camera_fov_slider.get()
        camera_radius_squared = camera_radius ** 2
        for dy in range(-camera_radius, camera_radius + 1):
            for dx in range(-camera_radius, camera_radius + 1):
                if (dx ** 2 + dy ** 2) > camera_radius_squared:
                    continue
                ex, ey = new_x + dx, new_y + dy
                if 0 <= ex < self.width and 0 <= ey < self.height:
                    if self.has_line_of_sight(new_x, new_y, ex, ey):
                        if not self.is_opaque(ex, ey):
                            self.camera_explored[ey][ex] = True
                        if self.grid[ey][ex] == 3:
                            self.found_sar.add((ex, ey))
                        elif self.grid[ey][ex] == 4:
                            self.found_cp.add((ex, ey))

    def format_stats_summary(self):
        if not self.last_run_stats:
            return "No statistics received."
        stats = self.last_run_stats
        return (f"Status: {stats.get('status', '?')}\n"
                f"Coverage: {stats.get('coverage_pct', 0.0):.2f}%\n"
                f"Steps: {stats.get('total_steps', 0)}\n"
                f"Distance: {stats.get('total_swarm_distance', 0.0):.2f}\n"
                f"Planning: {stats.get('avg_planning_ms', 0.0):.2f} ms/call")

    def _csv_append(self, filename, fieldnames, row):
        filename = os.path.join(BASE_DIR, filename)
        exists = os.path.exists(filename)
        if exists:
            with open(filename, newline="", encoding="utf-8") as source:
                current_header = next(csv.reader(source), [])
            if current_header != list(fieldnames):
                stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                os.replace(filename, f"{os.path.splitext(filename)[0]}_legacy_{stamp}.csv")
                exists = False
        with open(filename, "a", newline="", encoding="utf-8") as output:
            writer = csv.DictWriter(output, fieldnames=fieldnames)
            if not exists:
                writer.writeheader()
            writer.writerow({key: row.get(key, "") for key in fieldnames})

    def save_completed_run(self):
        if not self.save_csv.get() or not self.last_run_stats:
            return
        run_id = self.run_id or datetime.now().strftime("%Y%m%d_%H%M%S")
        base = {
            "timestamp": datetime.now().isoformat(timespec="seconds"),
            "run_id": run_id, "algorithm": self.algorithm_input.get(),
            "map_name": self.current_map_name, "width": self.width, "height": self.height,
            "num_drones": len(self.launch_positions), "fov": self.fov_slider.get(),
            "camera_fov": self.camera_fov_slider.get(), "sar_enabled": int(self.sar_enabled.get()),
            "cp_enabled": int(self.cp_enabled.get()), "cp_coords": int(self.cp_coords.get()),
        }
        summary_fields = list(base) + list(self.last_run_stats)
        summary_row = dict(base)
        summary_row.update(self.last_run_stats)
        self._csv_append("results_summary.csv", summary_fields, summary_row)
        curve_fields = ["run_id", "algorithm", "map_name", "num_drones", "step",
                "lidar_covered", "camera_covered", "total_reachable",
                "lidar_coverage_pct", "camera_coverage_pct", "retaken_steps",
                "sar_found", "cp_found"]
        for metric in self.current_run_metrics:
            row = {"run_id": run_id, "algorithm": self.algorithm_input.get(),
                   "map_name": self.current_map_name, "num_drones": len(self.launch_positions)}
            row.update(metric)
            self._csv_append("results_curves.csv", curve_fields, row)
        self.export_pgfplots([{"stats": self.last_run_stats,
                               "metrics": self.current_run_metrics,
                               "heatmap": self.last_run_heatmap}], run_id)

    def export_pgfplots(self, records, timestamp):
        if not records: return
        if records:
            data_dir = os.path.join(BASE_DIR, "pgfplots_data")
            os.makedirs(data_dir, exist_ok=True)
            clean = lambda value: "".join(ch if ch.isalnum() or ch in "_-" else "_" for ch in str(value))
            prefix = f"{clean(self.current_map_name)}_{clean(self.algorithm_input.get())}_{clean(timestamp)}"
            curve_path = os.path.join(data_dir, f"pgf_curves_{prefix}.dat")
            max_step = max((metric.get("step", 0) for record in records for metric in record["metrics"]), default=0)
            series = []
            for record in records:
                by_step = {metric["step"]: metric for metric in record["metrics"]}
                last = {}
                values = []
                for step in range(1, max_step + 1):
                    if step in by_step:
                        last = by_step[step]
                    if last:
                        values.append((100.0 * last.get("lidar_covered", 0) / max(1, last.get("total_reachable", 1)),
                                       100.0 * last.get("camera_covered", 0) / max(1, last.get("total_reachable", 1)),
                                       last.get("retaken_steps", 0), last.get("sar_found", 0), last.get("cp_found", 0)))
                series.append(values)
            with open(curve_path, "w", encoding="utf-8") as output:
                output.write("step lidar_pct_mean lidar_pct_std cam_pct_mean cam_pct_std retaken_mean retaken_std sar_mean cp_mean\n")
                for index in range(max_step):
                    row = [values[index] for values in series if index < len(values)]
                    cols = list(zip(*row)) if row else [[] for _ in range(5)]
                    means = [statistics.mean(col) if col else 0.0 for col in cols]
                    stds = [statistics.stdev(col) if len(col) > 1 else 0.0 for col in cols[:3]]
                    output.write(f"{index + 1} {means[0]:.4f} {stds[0]:.4f} {means[1]:.4f} {stds[1]:.4f} {means[2]:.4f} {stds[2]:.4f} {means[3]:.4f} {means[4]:.4f}\n")

            heatmap_path = os.path.join(data_dir, f"pgf_heatmap_{prefix}.dat")
            with open(heatmap_path, "w", encoding="utf-8") as output:
                output.write("x y visits\n")
                for y in range(self.height):
                    for x in range(self.width):
                        index = y * self.width + x
                        if self.grid[y][x] in (1, 3):
                            value = "nan"
                        else:
                            visits = [record["heatmap"][index] for record in records if len(record["heatmap"]) > index]
                            value = f"{statistics.mean(visits):.4f}" if visits else "0.0000"
                        output.write(f"{x} {y} {value}\n")
                    output.write("\n")

            obstacle_path = os.path.join(data_dir, f"pgf_obstacles_{clean(self.current_map_name)}.dat")
            with open(obstacle_path, "w", encoding="utf-8") as output:
                output.write("x y\n")
                for y in range(self.height):
                    for x in range(self.width):
                        if self.grid[y][x] in (1, 3): output.write(f"{x} {y}\n")

            tex_path = os.path.join(data_dir, f"pgf_plots_{prefix}.tex")
            with open(tex_path, "w", encoding="utf-8") as output:
                output.write("% Requires pgfplots and patterns\n% \\usepackage{pgfplots}\n% \\usetikzlibrary{patterns}\n")
                output.write("\\begin{tikzpicture}\n\\begin{axis}[title={LIDAR vs. Camera Coverage},xlabel={Steps},ylabel={Coverage (\\%)},legend pos=south east]\n")
                output.write(f"\\addplot table[x=step,y=lidar_pct_mean] {{{os.path.basename(curve_path)}}};\\addlegendentry{{LIDAR}}\n")
                output.write(f"\\addplot table[x=step,y=cam_pct_mean] {{{os.path.basename(curve_path)}}};\\addlegendentry{{Camera}}\n\\end{{axis}}\\end{{tikzpicture}}\n\n")
                output.write("\\begin{tikzpicture}\n\\begin{axis}[title={Cumulative Retaken Steps},xlabel={Steps},ylabel={Retaken steps}]\n")
                output.write(f"\\addplot table[x=step,y=retaken_mean] {{{os.path.basename(curve_path)}}};\n\\end{{axis}}\\end{{tikzpicture}}\n\n")
                output.write(f"\\begin{{tikzpicture}}\\begin{{axis}}[title={{2D Visit Heatmap}},y dir=reverse,axis equal image,colorbar,point meta min=0,point meta max=5,xmin=-0.5,xmax={self.width - 0.5},ymin=-0.5,ymax={self.height - 0.5},colormap={{whiteRed}}{{color=(white) color=(red!60!white) color=(red!85!black)}},unbounded coords=jump]\n")
                output.write(f"\\addplot[matrix plot*,point meta=explicit,mesh/cols={self.width},mesh/rows={self.height}] table[x=x,y=y,meta=visits] {{{os.path.basename(heatmap_path)}}};\n")
                for y in range(self.height):
                    for x in range(self.width):
                        if self.grid[y][x] in (1, 3):
                            output.write(f"\\draw[pattern=north east lines,pattern color=black,draw=black,line width=0.2pt] (axis cs:{x - 0.5},{y - 0.5}) rectangle (axis cs:{x + 0.5},{y + 0.5});\n")
                output.write("\\end{axis}\n\\end{tikzpicture}\n\n")
                output.write(f"\\begin{{tikzpicture}}\\begin{{axis}}[title={{3D Visit Surface}},point meta min=0,point meta max=5,zmin=0,colorbar,colormap={{whiteRed}}{{color=(white) color=(red!60!white) color=(red!85!black)}},unbounded coords=jump]\n")
                output.write(f"\\addplot3[surf,shader=interp] table[x=x,y=y,z=visits] {{{os.path.basename(heatmap_path)}}};\n\\end{{axis}}\\end{{tikzpicture}}\n")

    def _build_stdin(self):
        flags = (int(self.sar_enabled.get()), int(self.sar_rtl.get()), int(self.cp_enabled.get()),
                 int(self.cp_coords.get()), int(self.cp_rtl.get()))
        text = f"{self.width} {self.height} {len(self.launch_positions)} {self.fov_slider.get()} {self.camera_fov_slider.get()} {' '.join(map(str, flags))}\n"
        text += "".join(f"{x} {y}\n" for x, y in self.launch_positions)
        text += "".join(" ".join(map(str, row)) + "\n" for row in self.grid)
        return text

    def run_batch(self):
        if self.running:
            return
        try:
            trials = max(1, int(self.trials_input.get()))
        except ValueError:
            trials = 1
        self.set_benchmark_controls(tk.DISABLED)
        threading.Thread(target=self._run_batch_worker, args=(trials,), daemon=True).start()

    def _run_batch_worker(self, trials):
        results = []
        for trial in range(trials):
            process = subprocess.run([os.path.join(BASE_DIR, "drone")], input=self._build_stdin(), text=True,
                                     capture_output=True, check=False)
            metrics = []
            stats = None
            heatmap = []
            for line in process.stdout.splitlines():
                if line.startswith("METRIC "):
                    parts = line.split()
                    if len(parts) == 8:
                        step, lidar, camera, total, retaken, sar, cp = map(int, parts[1:])
                        metrics.append({"step": step, "lidar_covered": lidar,
                                        "camera_covered": camera, "total_reachable": total,
                                        "retaken_steps": retaken,
                                        "lidar_coverage_pct": 100.0 * lidar / max(1, total),
                                        "camera_coverage_pct": 100.0 * camera / max(1, total),
                                        "sar_found": sar, "cp_found": cp})
                elif line.startswith("HEATMAP "):
                    try:
                        heatmap = list(map(int, line.split()[1:]))
                    except ValueError:
                        heatmap = []
                elif line.startswith("STATS "):
                    try:
                        stats = json.loads(line[6:])
                    except json.JSONDecodeError:
                        pass
            if stats:
                run_id = f"{datetime.now().strftime('%Y%m%d_%H%M%S')}_{trial + 1}"
                self._log_batch_result(run_id, stats, metrics)
                results.append({"stats": stats, "metrics": metrics, "heatmap": heatmap})
            self.root.after(0, self.benchmark_status.config, {"text": f"Batch: {trial + 1} / {trials} completed..."})
        self.root.after(0, self._finish_batch, results)

    def _log_batch_result(self, run_id, stats, metrics):
        if not self.save_csv.get():
            return
        base = {"timestamp": datetime.now().isoformat(timespec="seconds"), "run_id": run_id,
                "algorithm": self.algorithm_input.get(), "map_name": self.current_map_name,
                "width": self.width, "height": self.height, "num_drones": len(self.launch_positions),
                "fov": self.fov_slider.get(), "camera_fov": self.camera_fov_slider.get(),
                "sar_enabled": int(self.sar_enabled.get()), "cp_enabled": int(self.cp_enabled.get()),
                "cp_coords": int(self.cp_coords.get())}
        self._csv_append("results_summary.csv", list(base) + list(stats), {**base, **stats})
        fields = ["run_id", "algorithm", "map_name", "num_drones", "step", "lidar_covered",
              "camera_covered", "total_reachable", "lidar_coverage_pct",
              "camera_coverage_pct", "retaken_steps", "sar_found", "cp_found"]
        for metric in metrics:
            self._csv_append("results_curves.csv", fields,
                             {"run_id": run_id, "algorithm": self.algorithm_input.get(),
                              "map_name": self.current_map_name, "num_drones": len(self.launch_positions), **metric})

    def _finish_batch(self, results):
        self.set_benchmark_controls(tk.NORMAL)
        if not results:
            self.benchmark_status.config(text="Batch finished: no valid STATS")
            return
        stats_records = [record["stats"] for record in results]
        batch_timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        average = {}
        for key in stats_records[0]:
            values = [float(item[key]) for item in stats_records if isinstance(item.get(key), (int, float))]
            if values:
                valid = [value for value in values if not key.startswith("time_to_") or value >= 0]
                average[key] = f"{statistics.mean(valid) if valid else -1:.4f}"
        base = {"timestamp": datetime.now().isoformat(timespec="seconds"),
                "run_id": f"BATCH_AVG_{batch_timestamp}", "algorithm": self.algorithm_input.get(),
                "map_name": self.current_map_name, "width": self.width, "height": self.height,
                "num_drones": len(self.launch_positions), "fov": self.fov_slider.get(),
                "camera_fov": self.camera_fov_slider.get(), "sar_enabled": int(self.sar_enabled.get()),
                "cp_enabled": int(self.cp_enabled.get()), "cp_coords": int(self.cp_coords.get()),
                "status": "AVERAGE", **average}
        if self.save_csv.get():
            self._csv_append("results_summary.csv", list(base) + [key for key in stats_records[0] if key not in base], base)
            self.export_pgfplots(results, batch_timestamp)

        keys = ["exploration_steps", "total_steps", "coverage_pct", "camera_overlap_ratio",
                "revisited_steps", "inter_drone_overlap_tiles", "total_swarm_distance",
                "total_hovers", "tsdf_exposure_pct", "avg_planning_ms"]
        lines = []
        for key in keys:
            values = [float(item["stats"][key]) for item in results if key in item["stats"]]
            if values:
                lines.append(f"{key}: {statistics.mean(values):.2f} +/- {statistics.stdev(values) if len(values) > 1 else 0.0:.2f}")
        success = 100.0 * sum(item["stats"].get("status") == "DONE" for item in results) / len(results)
        lines.append(f"success_rate: {success:.2f}%")
        summary = "\n".join(lines)
        self.benchmark_status.config(text=f"Batch complete: {len(results)} / {len(results)}")
        print(summary)
        messagebox.showinfo("Batch summary", summary)

    def process_queue(self):
        try:
            while True:
                msg_type, data = self.ui_queue.get_nowait()

                if msg_type == "FRONTIERS":
                    self.frontiers = data
                elif msg_type == "FOUND_SAR":
                    self.found_sar.add(data)
                elif msg_type == "FOUND_CP":
                    self.found_cp.add(data)
                elif msg_type == "TSDF":
                    self.tsdf_grid = [data[i * self.width:(i + 1) * self.width] for i in range(self.height)]
                elif msg_type == "ASTAR":
                    drone_id, coords = data
                    self.astar_plans[drone_id] = coords
                    self.mcts_plans[drone_id] = []
                    self.best_full_plans[drone_id] = []
                    self.sparse_plans[drone_id] = []
                elif msg_type == "PATH":
                    drone_id, coords = data
                    self.mcts_plans[drone_id] = coords
                    self.astar_plans[drone_id] = []
                elif msg_type == "BEST_PATH":
                    drone_id, coords = data
                    self.best_full_plans[drone_id] = coords
                elif msg_type == "SPARSE_PATHS":
                    drone_id, paths_list = data
                    self.sparse_plans[drone_id] = paths_list
                elif msg_type == "METRIC":
                    self.current_run_metrics.append(data)
                elif msg_type == "STATS":
                    self.last_run_stats = data
                elif msg_type == "HEATMAP":
                    self.last_run_heatmap = data

                elif msg_type == "MOVE":
                    drone_id, new_x, new_y = data
                    if 0 <= drone_id < len(self.drone_positions):
                        self.drone_positions[drone_id] = (new_x, new_y)
                    self.visit_counts[new_y][new_x] += 1
                    self.observe_from_position(new_x, new_y)

                    # Trim the first step from best_full_plans if it matches the step just taken
                    if drone_id in self.best_full_plans and len(self.best_full_plans[drone_id]) >= 2:
                        if self.best_full_plans[drone_id][0] == new_x and self.best_full_plans[drone_id][1] == new_y:
                            self.best_full_plans[drone_id] = self.best_full_plans[drone_id][2:]

                    self.draw_grid()

                    if self.running:
                        self.root.after(self.speed_slider.get(), self.process_queue)
                    return

                elif msg_type == "HALT":
                    self.draw_grid()
                    print(f"\n>>> SIMULATION HALTED: {data} <<<\n")
                    self.start_btn.config(state=tk.NORMAL)
                    self.apply_btn.config(state=tk.NORMAL)
                    self.gen_btn.config(state=tk.NORMAL)
                    self.set_benchmark_controls(tk.NORMAL)
                    self.running = False
                    if self.last_run_stats:
                        print(json.dumps(self.last_run_stats, indent=2, sort_keys=True))
                        self.save_completed_run()
                        messagebox.showinfo("Run summary", self.format_stats_summary())
                    return
                elif msg_type == "PRINT":
                    print(f"[C++] {data}")

        except queue.Empty:
            pass

        if self.running:
            self.root.after(10, self.process_queue)

    def run_cpp_brain(self):
        process = subprocess.Popen(['./drone'], stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True)

        sar_en = 1 if self.sar_enabled.get() else 0
        sar_rtl_flag = 1 if self.sar_rtl.get() else 0
        cp_en = 1 if self.cp_enabled.get() else 0
        cp_coords_flag = 1 if self.cp_coords.get() else 0
        cp_rtl_flag = 1 if self.cp_rtl.get() else 0

        num_drones = len(self.drone_positions)
        setup_data = (
            f"{self.width} {self.height} {num_drones} "
            f"{self.fov_slider.get()} {self.camera_fov_slider.get()} "
            f"{sar_en} {sar_rtl_flag} {cp_en} {cp_coords_flag} {cp_rtl_flag}\n"
        )
        for dx, dy in self.drone_positions:
            setup_data += f"{dx} {dy}\n"
        for row in self.grid:
            setup_data += " ".join(map(str, row)) + "\n"

        process.stdin.write(setup_data)
        process.stdin.flush()

        for line in process.stdout:
            line = line.strip()
            if not line:
                continue

            if line.startswith("FRONTIERS"):
                coords = list(map(int, line.split()[1:]))
                frontier_set = set()
                for i in range(0, len(coords) - 1, 2):
                    frontier_set.add((coords[i], coords[i + 1]))
                self.ui_queue.put(("FRONTIERS", frontier_set))
                continue
            if line.startswith("FOUND_SAR"):
                parts = line.split()
                if len(parts) >= 3:
                    self.ui_queue.put(("FOUND_SAR", (int(parts[1]), int(parts[2]))))
                continue
            if line.startswith("FOUND_CP"):
                parts = line.split()
                if len(parts) >= 3:
                    self.ui_queue.put(("FOUND_CP", (int(parts[1]), int(parts[2]))))
                continue
            if line.startswith("ASTAR_PATH"):
                parts = list(map(int, line.split()[1:]))
                if len(parts) >= 1:
                    self.ui_queue.put(("ASTAR", (parts[0], parts[1:])))
                continue
            if line.startswith("BEST_PATH"):
                parts = list(map(int, line.split()[1:]))
                if len(parts) >= 1:
                    self.ui_queue.put(("BEST_PATH", (parts[0], parts[1:])))
                continue
            if line.startswith("SPARSE_PATHS"):
                tokens = line.split()[1:]
                if len(tokens) >= 1:
                    drone_id = int(tokens[0])
                    raw_str = " ".join(tokens[1:])
                    path_chunks = [chunk.strip() for chunk in raw_str.split("|") if chunk.strip()]
                    parsed_paths = []
                    for ch in path_chunks:
                        parsed_paths.append(list(map(int, ch.split())))
                    self.ui_queue.put(("SPARSE_PATHS", (drone_id, parsed_paths)))
                continue
            if line.startswith("PATH"):
                parts = list(map(int, line.split()[1:]))
                if len(parts) >= 1:
                    self.ui_queue.put(("PATH", (parts[0], parts[1:])))
                continue
            if line.startswith("TSDF"):
                coords = list(map(float, line.split()[1:]))
                self.ui_queue.put(("TSDF", coords))
                continue
            if line.startswith("METRIC "):
                parts = line.split()
                if len(parts) == 8:
                    step, lidar, camera, total, retaken, sar, cp = map(int, parts[1:])
                    self.ui_queue.put(("METRIC", {
                        "step": step, "lidar_covered": lidar, "camera_covered": camera,
                        "total_reachable": total, "retaken_steps": retaken,
                        "lidar_coverage_pct": 100.0 * lidar / max(1, total),
                        "camera_coverage_pct": 100.0 * camera / max(1, total),
                        "sar_found": sar, "cp_found": cp,
                    }))
                continue
            if line.startswith("HEATMAP "):
                try:
                    self.ui_queue.put(("HEATMAP", list(map(int, line.split()[1:]))))
                except ValueError:
                    self.ui_queue.put(("PRINT", line))
                continue
            if line.startswith("STATS "):
                try:
                    self.ui_queue.put(("STATS", json.loads(line[6:])))
                except json.JSONDecodeError:
                    self.ui_queue.put(("PRINT", line))
                continue
            if line.startswith("MOVE"):
                parts = line.split()
                if len(parts) >= 4:
                    self.ui_queue.put(("MOVE", (int(parts[1]), int(parts[2]), int(parts[3]))))
                continue
            if line == "DONE" or line == "STUCK":
                self.ui_queue.put(("HALT", line))
                break

            self.ui_queue.put(("PRINT", line))


if __name__ == "__main__":
    root = tk.Tk()
    app = DroneVisualizer(root)
    root.mainloop()
    