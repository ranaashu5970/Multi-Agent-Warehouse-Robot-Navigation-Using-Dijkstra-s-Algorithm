import tkinter as tk
from tkinter import messagebox
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import heapq
import time
import threading
import random


GRID_SIZE = 15
CELL_SIZE = 1
running = False


agents = {
    "A1": {"start": (0, 0), "goal": (10, 12), "color": "red"},
    "A2": {"start": (14, 14), "goal": (3, 2), "color": "blue"},
}


def generate_obstacles():
    obstacles = set()
    total_cells = GRID_SIZE * GRID_SIZE
    max_obstacles = total_cells // 5  # 20% obstacles
    safe_cells = {agents["A1"]["start"], agents["A1"]["goal"], agents["A2"]["start"], agents["A2"]["goal"]}
    while len(obstacles) < max_obstacles:
        x, y = random.randint(0, GRID_SIZE - 1), random.randint(0, GRID_SIZE - 1)
        if (x, y) not in safe_cells:
            obstacles.add((x, y))
    return obstacles

obstacles = generate_obstacles()


def dijkstra(start, goal, obstacles):
    pq = []
    heapq.heappush(pq, (0, start))
    distances = {start: 0}
    came_from = {}

    while pq:
        current_dist, current = heapq.heappop(pq)

        if current == goal:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            return path[::-1]

        for dx, dy in [(1,0), (-1,0), (0,1), (0,-1)]:
            neighbor = (current[0] + dx, current[1] + dy)
            if 0 <= neighbor[0] < GRID_SIZE and 0 <= neighbor[1] < GRID_SIZE:
                if neighbor not in obstacles:
                    new_cost = current_dist + 1
                    if new_cost < distances.get(neighbor, float("inf")):
                        distances[neighbor] = new_cost
                        came_from[neighbor] = current
                        heapq.heappush(pq, (new_cost, neighbor))
    return []



root = tk.Tk()
root.title("MULTI-AGENT WAREHOUSE ROBOT NAVIGATION (DIJKSTRA)")
root.configure(bg="#edf7ed")

title = tk.Label(root,
                 text="MULTI-AGENT WAREHOUSE ROBOT NAVIGATION (DIJKSTRA)",
                 font=("Arial", 16, "bold"), fg="green", bg="#edf7ed")
title.pack(pady=10)

subtitle = tk.Label(root,
                    text="Robots (Agents) plan and move using Dijkstra’s Algorithm while avoiding collisions.",
                    font=("Arial", 10), bg="#edf7ed")
subtitle.pack()

fig, ax = plt.subplots(figsize=(6, 6))
ax.set_xlim(0, GRID_SIZE)
ax.set_ylim(0, GRID_SIZE)
ax.set_xticks(range(GRID_SIZE + 1))
ax.set_yticks(range(GRID_SIZE + 1))
ax.grid(True)
ax.set_aspect("equal")

canvas = FigureCanvasTkAgg(fig, master=root)
canvas.get_tk_widget().pack(pady=20)


def draw_grid(agent_positions=None):
    ax.clear()
    ax.set_xlim(0, GRID_SIZE)
    ax.set_ylim(0, GRID_SIZE)
    ax.set_xticks(range(GRID_SIZE + 1))
    ax.set_yticks(range(GRID_SIZE + 1))
    ax.grid(True)
    ax.set_aspect("equal")

    
    for (x, y) in obstacles:
        ax.add_patch(plt.Rectangle((x, y), CELL_SIZE, CELL_SIZE, color="black"))

    
    for name, info in agents.items():
        sx, sy = info["start"]
        gx, gy = info["goal"]
        ax.text(sx + 0.3, sy + 0.3, "S", color="red", fontsize=10, fontweight="bold")
        ax.text(gx + 0.3, gy + 0.3, "G", color="green", fontsize=10, fontweight="bold")

    
    if agent_positions:
        for name, pos in agent_positions.items():
            ax.add_patch(plt.Circle((pos[0] + 0.5, pos[1] + 0.5),
                                    0.3, color=agents[name]["color"], label=name))

    canvas.draw()

draw_grid()


def simulate():
    global running

    paths = {}
    for name, info in agents.items():
        path = dijkstra(info["start"], info["goal"], obstacles)
        if not path:
            messagebox.showerror("Error", f"No path found for {name}. Try restarting.")
            running = False
            return
        paths[name] = path

    agent_positions = {name: info["start"] for name, info in agents.items()}
    step = 0
    max_steps = max(len(p) for p in paths.values())

    while running and step < max_steps:
        occupied = set(agent_positions.values())
        for name, path in paths.items():
            if step < len(path):
                next_pos = path[step]
                if next_pos not in occupied:
                    agent_positions[name] = next_pos
                    occupied.add(next_pos)
        draw_grid(agent_positions)
        time.sleep(0.5)
        step += 1

    if running:
        messagebox.showinfo("Simulation Complete", "All agents reached their goals!")
    running = False



def start_simulation():
    global running, obstacles
    if running:
        return
    running = True
    obstacles = generate_obstacles()
    draw_grid()
    threading.Thread(target=simulate, daemon=True).start()

def stop_simulation():
    global running
    running = False
    messagebox.showwarning("Stopped", "Simulation stopped.")

btn_frame = tk.Frame(root, bg="#edf7ed")
btn_frame.pack(pady=10)

start_btn = tk.Button(btn_frame, text="▶ START", command=start_simulation,
                      bg="lightgreen", font=("Arial", 11, "bold"))
start_btn.grid(row=0, column=0, padx=10)

stop_btn = tk.Button(btn_frame, text="■ STOP", command=stop_simulation,
                     bg="lightcoral", font=("Arial", 11, "bold"))
stop_btn.grid(row=0, column=1, padx=10)

root.mainloop()
