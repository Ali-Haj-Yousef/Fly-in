"""
visualizer.py
=============

Dual-mode visual representation of the Fly-in drone simulation.

Provides two complementary visualization modes:

1. **Terminal mode** – Colored ANSI output showing the network graph,
   drone positions, zone states, and turn-by-turn movement log.

2. **GUI mode** – Interactive Tkinter window displaying the network
   topology as a force-directed graph with animated drone tokens,
   color-coded zones, labeled connections, and playback controls.

Both modes can be used independently or together (``--mode both``).

Usage (standalone)::

    python visualizer.py <map_file> [--mode terminal|gui|both] [--speed <ms>]

Usage (as a library)::

    from visualizer import TerminalVisualizer, GUIVisualizer
    tv = TerminalVisualizer(graph, scheduler.reservations)
    tv.display()

Author: Fly-in team
"""

from __future__ import annotations

import argparse
import math
import os
import sys
import time
from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Tuple


from drone import Drone
from graph import Graph
from parser import MapParser
from scheduler import Scheduler
from turn import Turn
from zone import Zone, ZoneType


class _CanvasLike:
    """Minimal protocol for Tkinter canvas methods used by the GUI."""

    def create_line(self, *args: Any, **kwargs: Any) -> Any:
        raise NotImplementedError

    def create_text(self, *args: Any, **kwargs: Any) -> Any:
        raise NotImplementedError

    def create_oval(self, *args: Any, **kwargs: Any) -> Any:
        raise NotImplementedError

    def create_polygon(self, *args: Any, **kwargs: Any) -> Any:
        raise NotImplementedError

    def delete(self, *args: Any, **kwargs: Any) -> Any:
        raise NotImplementedError


# ── Ensure UTF-8 output on Windows ──────────────────────────────────
if sys.platform == "win32":
    try:
        if hasattr(sys.stdout, "reconfigure"):
            sys.stdout.reconfigure(encoding="utf-8")
        if hasattr(sys.stderr, "reconfigure"):
            sys.stderr.reconfigure(encoding="utf-8")
    except Exception:
        pass

# ── Windows high-DPI fix (prevents blurry Tkinter rendering) ────────
if sys.platform == "win32":
    try:
        import ctypes

        ctypes.windll.shcore.SetProcessDpiAwareness(2)  # Per-monitor DPI aware
    except Exception:
        try:
            ctypes.windll.user32.SetProcessDPIAware()
            # Fallback for older Windows.
        except Exception:
            pass


# ═══════════════════════════════════════════════════════════════════════
# ANSI Color Helpers
# ═══════════════════════════════════════════════════════════════════════


class _ANSI:
    """ANSI escape-code constants for colored terminal output."""

    RESET = "\033[0m"
    BOLD = "\033[1m"
    DIM = "\033[2m"
    ITALIC = "\033[3m"
    UNDERLINE = "\033[4m"
    BLINK = "\033[5m"
    REVERSE = "\033[7m"

    # Foreground
    BLACK = "\033[30m"
    RED = "\033[31m"
    GREEN = "\033[32m"
    YELLOW = "\033[33m"
    BLUE = "\033[34m"
    MAGENTA = "\033[35m"
    CYAN = "\033[36m"
    WHITE = "\033[37m"

    # Bright foreground
    BRIGHT_BLACK = "\033[90m"
    BRIGHT_RED = "\033[91m"
    BRIGHT_GREEN = "\033[92m"
    BRIGHT_YELLOW = "\033[93m"
    BRIGHT_BLUE = "\033[94m"
    BRIGHT_MAGENTA = "\033[95m"
    BRIGHT_CYAN = "\033[96m"
    BRIGHT_WHITE = "\033[97m"

    # Background
    BG_BLACK = "\033[40m"
    BG_RED = "\033[41m"
    BG_GREEN = "\033[42m"
    BG_YELLOW = "\033[43m"
    BG_BLUE = "\033[44m"
    BG_MAGENTA = "\033[45m"
    BG_CYAN = "\033[46m"
    BG_WHITE = "\033[47m"

    BG_BRIGHT_BLACK = "\033[100m"
    BG_BRIGHT_GREEN = "\033[102m"
    BG_BRIGHT_YELLOW = "\033[103m"
    BG_BRIGHT_BLUE = "\033[104m"
    BG_BRIGHT_CYAN = "\033[106m"


# Map zone colors (from map file) to ANSI codes
_ZONE_COLOR_MAP: Dict[Optional[str], str] = {
    "green": _ANSI.BRIGHT_GREEN,
    "red": _ANSI.BRIGHT_RED,
    "blue": _ANSI.BRIGHT_BLUE,
    "yellow": _ANSI.BRIGHT_YELLOW,
    "orange": _ANSI.BRIGHT_YELLOW,
    "cyan": _ANSI.BRIGHT_CYAN,
    "magenta": _ANSI.BRIGHT_MAGENTA,
    "white": _ANSI.BRIGHT_WHITE,
    "purple": _ANSI.BRIGHT_MAGENTA,
    "black": _ANSI.BRIGHT_BLACK,
    "brown": _ANSI.YELLOW,
    "maroon": _ANSI.RED,
    "gold": _ANSI.BRIGHT_YELLOW,
    "darkred": _ANSI.RED,
    "violet": _ANSI.BRIGHT_MAGENTA,
    "crimson": _ANSI.BRIGHT_RED,
    "rainbow": _ANSI.BRIGHT_MAGENTA,
    None: _ANSI.WHITE,
}

_ZONE_TYPE_SYMBOL: Dict[ZoneType, str] = {
    ZoneType.NORMAL: "○",
    ZoneType.BLOCKED: "✖",
    ZoneType.RESTRICTED: "◆",
    ZoneType.PRIORITY: "★",
}

_ZONE_TYPE_LABEL: Dict[ZoneType, str] = {
    ZoneType.NORMAL: "NORMAL",
    ZoneType.BLOCKED: "BLOCKED",
    ZoneType.RESTRICTED: "RESTRICTED",
    ZoneType.PRIORITY: "PRIORITY",
}

# Drone colors cycle (for terminal drone IDs)
_DRONE_COLORS = [
    _ANSI.BRIGHT_CYAN,
    _ANSI.BRIGHT_MAGENTA,
    _ANSI.BRIGHT_YELLOW,
    _ANSI.BRIGHT_GREEN,
    _ANSI.BRIGHT_RED,
    _ANSI.BRIGHT_BLUE,
    _ANSI.BRIGHT_WHITE,
]


def _drone_color(drone_id: int) -> str:
    """Returns a cycling ANSI color for a given drone ID."""
    return _DRONE_COLORS[drone_id % len(_DRONE_COLORS)]


def _zone_ansi(zone: Zone) -> str:
    """Returns the ANSI color code for a zone based on its color attribute."""
    return _ZONE_COLOR_MAP.get(zone.color, _ANSI.WHITE)


def _supports_color() -> bool:
    """Heuristic check for ANSI color support."""
    if os.getenv("NO_COLOR"):
        return False
    if os.getenv("FORCE_COLOR"):
        return True
    return hasattr(sys.stdout, "isatty") and sys.stdout.isatty()


# ═══════════════════════════════════════════════════════════════════════
# Data extraction helpers
# ═══════════════════════════════════════════════════════════════════════


@dataclass
class DronePosition:
    """Describes where a drone is during a particular turn."""

    drone_id: int
    zone_name: str  # zone the drone is *at* or heading *to*
    connection_name: str  # full connection key (e.g. "A-B") or zone name
    will_move: bool
    on_transit: bool


def extract_turn_positions(
    turn: Turn,
) -> List[DronePosition]:
    """Converts a Turn's reservations into a flat list of DronePositions."""
    positions: List[DronePosition] = []
    for conn_name, statuses in turn.reservations.items():
        # Derive the "current zone" from the connection name
        if "-" in conn_name:
            zone_name = conn_name.split("-")[1]
        else:
            zone_name = conn_name
        for status in statuses:
            positions.append(
                DronePosition(
                    drone_id=status.drone.id,
                    zone_name=zone_name,
                    connection_name=conn_name,
                    will_move=status.will_move,
                    on_transit=status.on_transit,
                )
            )
    return positions


# ═══════════════════════════════════════════════════════════════════════
# Terminal Visualizer
# ═══════════════════════════════════════════════════════════════════════


class TerminalVisualizer:
    """Renders the simulation with rich ANSI-colored terminal output.

    Features:
        - Network topology overview with zone types and colors
        - Turn-by-turn drone movement log
        - Zone occupancy bar per turn
        - Summary statistics at the end
    """

    SEPARATOR_WIDTH = 72

    def __init__(
        self,
        graph: Graph,
        reservations: List[Turn],
        use_color: bool = True,
    ) -> None:
        self.graph = graph
        self.reservations = reservations
        self.use_color = use_color and _supports_color()

    # ── pretty-print helpers ─────────────────────────────────────────

    def _c(self, code: str, text: str) -> str:
        """Wraps *text* with an ANSI *code* if color is enabled."""
        if not self.use_color:
            return text
        return f"{code}{text}{_ANSI.RESET}"

    def _separator(self, char: str = "═", width: int = 0) -> str:
        w = width or self.SEPARATOR_WIDTH
        return self._c(_ANSI.DIM, char * w)

    def _header(self, title: str) -> str:
        pad = (self.SEPARATOR_WIDTH - len(title) - 4) // 2
        line = "═" * pad + f"  {title}  " + "═" * pad
        return self._c(_ANSI.BOLD + _ANSI.BRIGHT_CYAN, line)

    # ── main display ─────────────────────────────────────────────────

    def display(self, delay: float = 0.0) -> None:
        """Runs the full terminal visualization."""
        self._print_banner()
        self._print_network_overview()
        self._print_connections_overview()
        self._print_simulation(delay)
        self._print_summary()

    # ── banner ───────────────────────────────────────────────────────

    def _print_banner(self) -> None:
        banner = r"""
     _____ _           ___
    |  ___| |_   _    |_ _|_ __
    | |_  | | | | |    | || '_ \
    |  _| | | |_| |    | || | | |
    |_|   |_|\__, |   |___|_| |_|
             |___/
        """
        print(self._c(_ANSI.BOLD + _ANSI.BRIGHT_CYAN, banner))
        print(
            self._c(
                _ANSI.DIM + _ANSI.BRIGHT_WHITE,
                "  ╔══════════════════════════════════════════════════╗",
            )
        )
        print(
            self._c(
                _ANSI.DIM + _ANSI.BRIGHT_WHITE,
                "  ║   Drone Network Simulation — Visual Output      ║",
            )
        )
        print(
            self._c(
                _ANSI.DIM + _ANSI.BRIGHT_WHITE,
                "  ╚══════════════════════════════════════════════════╝",
            )
        )
        print()

    # ── network overview ─────────────────────────────────────────────

    def _print_network_overview(self) -> None:
        print(self._header("NETWORK TOPOLOGY"))
        print()

        # Legend
        print(self._c(_ANSI.BOLD, "  Zone Types:"))
        for zt in ZoneType:
            sym = _ZONE_TYPE_SYMBOL[zt]
            label = _ZONE_TYPE_LABEL[zt]
            cost = zt.movement_cost
            cost_str = f"cost={cost}" if cost >= 0 else "impassable"
            print(f"    {sym}  {label:<12s}  ({cost_str})")
        print()

        # Zones table
        print(self._c(_ANSI.BOLD, "  Zones:"))
        print(
            self._c(
                _ANSI.DIM,
                "    ┌──────────────────┬──────────┬────────────"
                "┬────────┬───────────┐",
            )
        )
        print(
            self._c(
                _ANSI.DIM,
                "    │ Name             │ Type     │ Role       "
                "│ Cap    │ Position  │",
            )
        )
        print(
            self._c(
                _ANSI.DIM,
                "    ├──────────────────┼──────────┼────────────"
                "┼────────┼───────────┤",
            )
        )

        for zone in self.graph.zones.values():
            sym = _ZONE_TYPE_SYMBOL[zone.zone_type]
            color = _zone_ansi(zone)
            name = self._c(color + _ANSI.BOLD, f"{sym} {zone.name:<14s}")
            zt_label = zone.zone_type.value[:8].ljust(8)
            role = zone.role.value[:10].ljust(10)
            cap = str(zone.max_drones).ljust(6)
            pos = f"({zone.x},{zone.y})".ljust(9)
            print(f"    │ {name} │ {zt_label} │ {role} │ {cap} │ {pos} │")

        print(
            self._c(
                _ANSI.DIM,
                "    └──────────────────┴──────────┴────────────"
                "┴────────┴───────────┘",
            )
        )
        print()
        print(
            self._c(
                _ANSI.BOLD + _ANSI.BRIGHT_WHITE,
                f"  Drones to route: {self.graph.nb_drones}",
            )
        )
        print()

    # ── connections overview ─────────────────────────────────────────

    def _print_connections_overview(self) -> None:
        print(self._header("CONNECTIONS"))
        print()
        for conn in self.graph.connections:
            a_color = _zone_ansi(conn.zone_a)
            b_color = _zone_ansi(conn.zone_b)
            a_name = self._c(a_color, conn.zone_a.name)
            b_name = self._c(b_color, conn.zone_b.name)
            status = (
                self._c(_ANSI.RED + _ANSI.BOLD, " ✖ BLOCKED")
                if conn.blocked
                else self._c(_ANSI.GREEN, " ✔")
            )
            cap = self._c(_ANSI.DIM, f"[cap={conn.max_link_capacity}]")
            arrow = self._c(_ANSI.DIM, "───▶")
            print(f"    {a_name} {arrow} {b_name}  {cap}{status}")
        print()

    # ── per-turn simulation ──────────────────────────────────────────

    def _print_simulation(self, delay: float) -> None:
        print(self._header("SIMULATION PLAYBACK"))
        print()
        total = len(self.reservations)
        for idx, turn in enumerate(self.reservations):
            turn_num = idx + 1
            positions = extract_turn_positions(turn)

            # Turn header
            progress = f"[{turn_num}/{total}]"
            bar_fill = int((turn_num / total) * 30)
            bar = "█" * bar_fill + "░" * (30 - bar_fill)
            print(
                self._c(
                    _ANSI.BOLD + _ANSI.BRIGHT_YELLOW,
                    f"  ┌─ Turn {turn_num} {progress}",
                )
            )
            print(
                self._c(
                    _ANSI.DIM,
                    f"  │  Progress: [{bar}] " f"{turn_num * 100 // total}%",
                )
            )
            print(self._c(_ANSI.DIM, "  │"))

            # Group drones by zone
            zone_drones: Dict[str, List[DronePosition]] = {}
            for pos in positions:
                zone_drones.setdefault(pos.zone_name, []).append(pos)

            for zone_name, drones_here in zone_drones.items():
                zone = self.graph.zones.get(zone_name)
                if zone:
                    z_color = _zone_ansi(zone)
                    z_sym = _ZONE_TYPE_SYMBOL[zone.zone_type]
                    max_d: int | str = zone.max_drones
                else:
                    z_color = _ANSI.WHITE
                    z_sym = "·"
                    max_d = "?"

                zone_label = self._c(
                    z_color + _ANSI.BOLD, f"{z_sym} {zone_name}"
                )
                cap_info = self._c(_ANSI.DIM, f" [max:{max_d}]")
                drone_tags = []
                for dp in drones_here:
                    d_col = _drone_color(dp.drone_id)
                    tag = f"D{dp.drone_id}"
                    if dp.on_transit:
                        tag += self._c(_ANSI.DIM, "(transit)")
                    if not dp.will_move:
                        tag += self._c(_ANSI.BRIGHT_RED, "(waiting)")
                    drone_tags.append(self._c(d_col + _ANSI.BOLD, tag))
                drones_str = "  ".join(drone_tags)
                print(f"  │  {zone_label}{cap_info}: {drones_str}")

            print(self._c(_ANSI.DIM, "  │"))
            print(self._c(_ANSI.BOLD + _ANSI.BRIGHT_YELLOW, "  └" + "─" * 50))
            print()
            if delay > 0:
                time.sleep(delay)

    # ── summary ──────────────────────────────────────────────────────

    def _print_summary(self) -> None:
        print(self._header("SIMULATION COMPLETE"))
        print()
        total_turns = len(self.reservations)
        print(
            self._c(
                _ANSI.BOLD + _ANSI.BRIGHT_GREEN,
                f"  ✔  All {self.graph.nb_drones} drones reached "
                f"the destination in {total_turns} turns.",
            )
        )
        print()

        # Per-drone timeline
        print(self._c(_ANSI.BOLD, "  Drone Timelines:"))
        print(self._c(_ANSI.DIM, "  " + "─" * 60))
        drone_paths: Dict[int, List[str]] = {}
        for idx, turn in enumerate(self.reservations):
            positions = extract_turn_positions(turn)
            for pos in positions:
                drone_paths.setdefault(pos.drone_id, []).append(pos.zone_name)

        for d_id in sorted(drone_paths.keys()):
            path = drone_paths[d_id]
            d_col = _drone_color(d_id)
            label = self._c(d_col + _ANSI.BOLD, f"  D{d_id}")
            arrows = self._c(_ANSI.DIM, " → ").join(path)
            print(f"  {label}: {arrows}")

        print()
        print(self._separator())
        print()


# ═══════════════════════════════════════════════════════════════════════
# GUI Visualizer (Tkinter)
# ═══════════════════════════════════════════════════════════════════════

# Tkinter color palette for zones
_GUI_ZONE_COLORS: Dict[Optional[str], str] = {
    "green": "#22c55e",
    "red": "#ef4444",
    "blue": "#3b82f6",
    "yellow": "#eab308",
    "orange": "#f97316",
    "cyan": "#06b6d4",
    "magenta": "#d946ef",
    "white": "#e2e8f0",
    "purple": "#a855f7",
    "black": "#334155",
    "brown": "#92400e",
    "maroon": "#881337",
    "gold": "#eab308",
    "darkred": "#991b1b",
    "violet": "#8b5cf6",
    "crimson": "#be123c",
    "rainbow": "#ec4899",
    None: "#94a3b8",
}


def _gui_zone_color(color_name: Optional[str]) -> str:
    """Returns the Tkinter hex color for a zone's color attribute."""
    if not color_name:
        return _GUI_ZONE_COLORS[None]
    color_lower = color_name.lower()
    if color_lower in _GUI_ZONE_COLORS:
        return _GUI_ZONE_COLORS[color_lower]
    if color_name.startswith("#"):
        return color_name
    return _GUI_ZONE_COLORS.get(color_lower, "#94a3b8")


_GUI_ZONE_TYPE_OUTLINE: Dict[ZoneType, str] = {
    ZoneType.NORMAL: "#64748b",
    ZoneType.BLOCKED: "#dc2626",
    ZoneType.RESTRICTED: "#f97316",
    ZoneType.PRIORITY: "#2563eb",
}

# Drone token colors (for GUI)
_GUI_DRONE_COLORS = [
    "#06b6d4",  # cyan
    "#d946ef",  # magenta
    "#eab308",  # yellow
    "#22c55e",  # green
    "#ef4444",  # red
    "#3b82f6",  # blue
    "#f97316",  # orange
    "#ec4899",  # pink
    "#8b5cf6",  # violet
    "#14b8a6",  # teal
]


class GUIVisualizer:
    """Interactive Tkinter-based graphical visualizer for the simulation.

    Features:
        - Network graph drawn on a canvas with zone nodes and connection edges
        - Animated drone tokens moving between zones each turn
        - Color-coded zones (by zone type and map color)
        - Playback controls: Play/Pause, Step Forward, Step Back, Speed slider
        - Live legend panel and turn information overlay
    """

    CANVAS_WIDTH = 1100
    CANVAS_HEIGHT = 700
    NODE_RADIUS = 32
    DRONE_RADIUS = 10
    PADDING = 100
    MIN_ZONE_SPACING = 120  # Minimum pixel distance between any two zones

    # Dark theme palette
    BG_COLOR = "#0f172a"
    CANVAS_BG = "#1e293b"
    PANEL_BG = "#1e293b"
    TEXT_COLOR = "#e2e8f0"
    MUTED_COLOR = "#94a3b8"
    ACCENT_COLOR = "#38bdf8"
    EDGE_COLOR = "#475569"
    EDGE_BLOCKED_COLOR = "#7f1d1d"

    def __init__(
        self,
        graph: Graph,
        reservations: List[Turn],
        speed_ms: int = 800,
    ) -> None:
        self.graph = graph
        self.reservations = reservations
        self.speed_ms = speed_ms
        self.current_turn = -1  # -1 = initial state (all at start)
        self.playing = False
        self._after_id: Optional[str] = None

        # Compute zone canvas positions from their (x, y) map coords
        self._zone_positions: Dict[str, Tuple[float, float]] = {}
        self._canvas_w = self.CANVAS_WIDTH
        self._canvas_h = self.CANVAS_HEIGHT
        self._resize_after_id: Optional[str] = None
        self.canvas: Any = None
        self.root: Any = None
        self._turn_label: Any = None
        self._progress_label: Any = None
        self._drone_info_frame: Any = None
        self._btn_back: Any = None
        self._btn_play: Any = None
        self._btn_step: Any = None
        self._btn_reset: Any = None
        self._speed_var: Any = None
        self._speed_label: Any = None
        self._compute_positions(self.CANVAS_WIDTH, self.CANVAS_HEIGHT)

    # ── geometry ─────────────────────────────────────────────────────

    def _compute_positions(self, canvas_w: int, canvas_h: int) -> None:
        """Maps zone (x, y) from the map file to canvas pixel positions.

        Scales the zone coordinates to fill the available canvas area
        defined by *canvas_w* × *canvas_h*, then applies an iterative
        repulsion pass to enforce `MIN_ZONE_SPACING` between every pair
        of zones so that labels never overlap.
        """
        zones = list(self.graph.zones.values())
        if not zones:
            return

        self._canvas_w = canvas_w
        self._canvas_h = canvas_h

        xs = [z.x for z in zones]
        ys = [z.y for z in zones]
        min_x, max_x = min(xs), max(xs)
        min_y, max_y = min(ys), max(ys)
        range_x = max_x - min_x if max_x != min_x else 1
        range_y = max_y - min_y if max_y != min_y else 1

        pad = self.PADDING
        usable_w = canvas_w - 2 * pad
        usable_h = canvas_h - 2 * pad

        # Maintain aspect ratio of the original map coordinates
        scale_x = usable_w / range_x
        scale_y = usable_h / range_y
        scale = min(scale_x, scale_y)

        # Center the graph within the canvas
        graph_w = range_x * scale
        graph_h = range_y * scale
        offset_x = pad + (usable_w - graph_w) / 2
        offset_y = pad + (usable_h - graph_h) / 2

        positions: Dict[str, List[float]] = {}
        for zone in zones:
            nx = (zone.x - min_x) / range_x  # 0..1
            ny = (zone.y - min_y) / range_y  # 0..1
            cx = offset_x + nx * graph_w
            cy = offset_y + ny * graph_h
            positions[zone.name] = [cx, cy]

        # ── Iterative repulsion to enforce minimum spacing ───────────
        min_dist = self.MIN_ZONE_SPACING
        names = list(positions.keys())
        n = len(names)

        for _iteration in range(200):
            moved = False
            for i in range(n):
                for j in range(i + 1, n):
                    ax, ay = positions[names[i]]
                    bx, by = positions[names[j]]
                    dx = bx - ax
                    dy = by - ay
                    dist = math.sqrt(dx * dx + dy * dy)
                    if dist < min_dist and dist > 0:
                        # Push apart
                        overlap = (min_dist - dist) / 2.0
                        ux, uy = dx / dist, dy / dist
                        positions[names[i]][0] -= ux * overlap
                        positions[names[i]][1] -= uy * overlap
                        positions[names[j]][0] += ux * overlap
                        positions[names[j]][1] += uy * overlap
                        moved = True
                    elif dist == 0:
                        # Identical positions: nudge apart arbitrarily
                        positions[names[i]][0] -= min_dist / 2
                        positions[names[j]][0] += min_dist / 2
                        moved = True
            if not moved:
                break

        # ── Re-center after repulsion may have shifted the cloud ─────
        all_x = [p[0] for p in positions.values()]
        all_y = [p[1] for p in positions.values()]
        cur_min_x, cur_max_x = min(all_x), max(all_x)
        cur_min_y, cur_max_y = min(all_y), max(all_y)
        cur_w = cur_max_x - cur_min_x if cur_max_x != cur_min_x else 1
        cur_h = cur_max_y - cur_min_y if cur_max_y != cur_min_y else 1

        # Scale the repulsed layout to fit the usable area
        fit_scale_x = usable_w / cur_w if cur_w > usable_w else 1.0
        fit_scale_y = usable_h / cur_h if cur_h > usable_h else 1.0
        fit_scale = min(fit_scale_x, fit_scale_y)

        # Center of the current cloud
        cx_center = (cur_min_x + cur_max_x) / 2
        cy_center = (cur_min_y + cur_max_y) / 2
        # Target center
        tx_center = canvas_w / 2
        ty_center = canvas_h / 2

        for name in positions:
            px, py = positions[name]
            # Scale around cloud center, then translate to canvas center
            positions[name][0] = (px - cx_center) * fit_scale + tx_center
            positions[name][1] = (py - cy_center) * fit_scale + ty_center

        # Clamp to canvas bounds with padding
        for name in positions:
            positions[name][0] = max(
                pad, min(canvas_w - pad, positions[name][0])
            )
            positions[name][1] = max(
                pad, min(canvas_h - pad, positions[name][1])
            )

        self._zone_positions = {
            name: (pos[0], pos[1]) for name, pos in positions.items()
        }

    # ── launch ───────────────────────────────────────────────────────

    def show(self) -> None:
        """Creates the Tkinter window and starts the main loop."""
        try:
            import tkinter as tk
            from tkinter import ttk
        except ImportError:
            print(
                "ERROR: Tkinter is not available. "
                "Use --mode terminal instead."
            )
            return

        self.tk = tk
        self.root = tk.Tk()
        self.root.title("Fly-in — Drone Simulation Visualizer")
        self.root.configure(bg=self.BG_COLOR)
        self.root.resizable(True, True)

        # ── Start maximized so the network fills the screen ──────────
        try:
            self.root.state("zoomed")  # Works on Windows
        except Exception:
            try:
                # Fallback for Linux / macOS
                screen_w = self.root.winfo_screenwidth()
                screen_h = self.root.winfo_screenheight()
                self.root.geometry(f"{screen_w}x{screen_h}+0+0")
            except Exception:
                pass

        # ── High-DPI: tell Tk the actual screen DPI ──────────────────
        try:
            dpi = self.root.winfo_fpixels("1i")  # actual pixels per inch
            self.root.tk.call("tk", "scaling", dpi / 72)
        except Exception:
            pass

        # ── Main layout: sidebar + canvas ────────────────────────────

        main_frame = tk.Frame(self.root, bg=self.BG_COLOR)
        main_frame.pack(fill=tk.BOTH, expand=True)

        # Sidebar
        sidebar = tk.Frame(main_frame, bg=self.PANEL_BG, width=260)
        sidebar.pack(side=tk.LEFT, fill=tk.Y, padx=(8, 0), pady=8)
        sidebar.pack_propagate(False)

        self._build_sidebar(sidebar, tk)

        # Canvas area
        canvas_frame = tk.Frame(main_frame, bg=self.BG_COLOR)
        canvas_frame.pack(
            side=tk.LEFT, fill=tk.BOTH, expand=True, padx=8, pady=8
        )

        self.canvas = tk.Canvas(
            canvas_frame,
            width=self.CANVAS_WIDTH,
            height=self.CANVAS_HEIGHT,
            bg=self.CANVAS_BG,
            highlightthickness=1,
            highlightbackground="#334155",
        )
        self.canvas.pack(fill=tk.BOTH, expand=True)

        # ── Bind canvas resize to recompute layout ───────────────────
        self.canvas.bind("<Configure>", self._on_canvas_resize)

        # ── Controls bar ─────────────────────────────────────────────

        controls = tk.Frame(self.root, bg=self.BG_COLOR)
        controls.pack(fill=tk.X, padx=8, pady=(0, 8))
        self._build_controls(controls, tk, ttk)

        # ── Draw initial state ───────────────────────────────────────

        self._draw_graph()
        self._draw_turn()

        self.root.mainloop()

    # ── resize handling ──────────────────────────────────────────────

    def _on_canvas_resize(self, event: Any) -> None:
        """Recomputes zone positions and redraws when the canvas resizes."""
        new_w = getattr(event, "width", 0)
        new_h = getattr(event, "height", 0)
        # Ignore very small sizes (e.g. during init) and no-change events
        if new_w < 100 or new_h < 100:
            return
        if new_w == self._canvas_w and new_h == self._canvas_h:
            return

        # Debounce: cancel pending redraw and schedule a new one
        if self._resize_after_id is not None:
            self.root.after_cancel(self._resize_after_id)
        self._resize_after_id = self.root.after(
            150, self._do_resize, new_w, new_h
        )

    def _do_resize(self, new_w: int, new_h: int) -> None:
        """Actually performs the resize: recompute positions and redraw."""
        self._resize_after_id = None
        self._compute_positions(new_w, new_h)
        self.canvas.delete("all")
        self._draw_graph()
        self._draw_turn()

    # ── sidebar ──────────────────────────────────────────────────────

    def _build_sidebar(self, parent: Any, tk: Any) -> None:
        # Title
        tk.Label(
            parent,
            text="🛩  Fly-in Visualizer",
            bg=self.PANEL_BG,
            fg=self.ACCENT_COLOR,
            font=("Segoe UI", 13, "bold"),
            anchor="w",
        ).pack(fill="x", padx=12, pady=(12, 4))

        tk.Frame(parent, bg="#334155", height=1).pack(
            fill="x", padx=12, pady=6
        )

        # Info
        info_items = [
            ("Drones", str(self.graph.nb_drones)),
            ("Zones", str(len(self.graph.zones))),
            ("Connections", str(len(self.graph.connections))),
            ("Total turns", str(len(self.reservations))),
            ("Start", self.graph.start_zone.name),
            ("End", self.graph.end_zone.name),
        ]
        for label, value in info_items:
            row = tk.Frame(parent, bg=self.PANEL_BG)
            row.pack(fill="x", padx=12, pady=2)
            tk.Label(
                row,
                text=f"{label}:",
                bg=self.PANEL_BG,
                fg=self.MUTED_COLOR,
                font=("Segoe UI", 9),
                anchor="w",
            ).pack(side="left")
            tk.Label(
                row,
                text=value,
                bg=self.PANEL_BG,
                fg=self.TEXT_COLOR,
                font=("Segoe UI", 9, "bold"),
                anchor="e",
            ).pack(side="right")

        tk.Frame(parent, bg="#334155", height=1).pack(
            fill="x", padx=12, pady=6
        )

        # Legend
        tk.Label(
            parent,
            text="Legend",
            bg=self.PANEL_BG,
            fg=self.TEXT_COLOR,
            font=("Segoe UI", 10, "bold"),
            anchor="w",
        ).pack(fill="x", padx=12, pady=(4, 2))

        legend_items = [
            ("○  Normal", "#94a3b8"),
            ("★  Priority", "#2563eb"),
            ("◆  Restricted", "#f97316"),
            ("✖  Blocked", "#dc2626"),
        ]
        for text, color in legend_items:
            tk.Label(
                parent,
                text=f"  {text}",
                bg=self.PANEL_BG,
                fg=color,
                font=("Segoe UI", 9),
                anchor="w",
            ).pack(fill="x", padx=12, pady=1)

        tk.Frame(parent, bg="#334155", height=1).pack(
            fill="x", padx=12, pady=6
        )

        # Turn info (updated dynamically)
        self._turn_label = tk.Label(
            parent,
            text="Turn: — / —",
            bg=self.PANEL_BG,
            fg=self.ACCENT_COLOR,
            font=("Segoe UI", 11, "bold"),
            anchor="w",
        )
        self._turn_label.pack(fill="x", padx=12, pady=(4, 2))

        self._drone_info_frame = tk.Frame(parent, bg=self.PANEL_BG)
        self._drone_info_frame.pack(fill="x", padx=12, pady=2)

    # ── controls ─────────────────────────────────────────────────────

    def _build_controls(
        self, parent: Any, tk: Any, ttk: Any
    ) -> None:
        btn_opts = dict(
            bg="#334155",
            fg=self.TEXT_COLOR,
            activebackground=self.ACCENT_COLOR,
            activeforeground="#0f172a",
            font=("Segoe UI", 10, "bold"),
            relief="flat",
            bd=0,
            padx=14,
            pady=4,
            cursor="hand2",
        )

        self._btn_back = tk.Button(
            parent, text="⏮  Back", command=self._step_back, **btn_opts
        )
        self._btn_back.pack(side="left", padx=(0, 4))

        self._btn_play = tk.Button(
            parent, text="▶  Play", command=self._toggle_play, **btn_opts
        )
        self._btn_play.pack(side="left", padx=4)

        self._btn_step = tk.Button(
            parent, text="⏭  Step", command=self._step_forward, **btn_opts
        )
        self._btn_step.pack(side="left", padx=4)

        self._btn_reset = tk.Button(
            parent, text="⏹  Reset", command=self._reset, **btn_opts
        )
        self._btn_reset.pack(side="left", padx=4)

        # Speed slider
        tk.Label(
            parent,
            text="Speed:",
            bg=self.BG_COLOR,
            fg=self.MUTED_COLOR,
            font=("Segoe UI", 9),
        ).pack(side="left", padx=(20, 4))

        self._speed_var = tk.IntVar(value=self.speed_ms)
        speed_slider = tk.Scale(
            parent,
            from_=100,
            to=2000,
            variable=self._speed_var,
            orient="horizontal",
            length=160,
            bg=self.BG_COLOR,
            fg=self.MUTED_COLOR,
            troughcolor="#334155",
            highlightthickness=0,
            sliderrelief="flat",
            showvalue=False,
            font=("Segoe UI", 7),
        )
        speed_slider.pack(side="left", padx=4)

        self._speed_label = tk.Label(
            parent,
            text=f"{self.speed_ms} ms",
            bg=self.BG_COLOR,
            fg=self.MUTED_COLOR,
            font=("Segoe UI", 9),
        )
        self._speed_label.pack(side="left", padx=4)
        self._speed_var.trace_add("write", self._on_speed_change)

        # Turn indicator on the right
        self._progress_label = tk.Label(
            parent,
            text="",
            bg=self.BG_COLOR,
            fg=self.ACCENT_COLOR,
            font=("Segoe UI", 10, "bold"),
        )
        self._progress_label.pack(side="right", padx=8)

    # ── drawing ──────────────────────────────────────────────────────

    def _draw_graph(self) -> None:
        """Draws the static network: edges and zone nodes."""
        canvas = self.canvas
        r = self.NODE_RADIUS

        # Edges
        for conn in self.graph.connections:
            x1, y1 = self._zone_positions[conn.zone_a.name]
            x2, y2 = self._zone_positions[conn.zone_b.name]
            color = (
                self.EDGE_BLOCKED_COLOR if conn.blocked else self.EDGE_COLOR
            )
            dash = (6, 4) if conn.blocked else ()
            width = 1 if conn.blocked else 2

            canvas.create_line(
                x1,
                y1,
                x2,
                y2,
                fill=color,
                width=width,
                dash=dash,
                tags="edge",
            )

            # Edge label (capacity)
            mx, my = (x1 + x2) / 2, (y1 + y2) / 2
            # offset label slightly so it doesn't overlap the line
            angle = math.atan2(y2 - y1, x2 - x1)
            offset = 14
            lx = mx + offset * math.sin(angle)
            ly = my - offset * math.cos(angle)
            if not conn.blocked:
                canvas.create_text(
                    lx,
                    ly,
                    text=f"cap:{conn.max_link_capacity}",
                    fill=self.MUTED_COLOR,
                    font=("Segoe UI", 7),
                    tags="edge_label",
                )

            # Arrow head
            if not conn.blocked:
                self._draw_arrowhead(x1, y1, x2, y2, r, canvas)

        # Nodes
        for zone in self.graph.zones.values():
            cx, cy = self._zone_positions[zone.name]
            fill = _gui_zone_color(zone.color)
            outline = _GUI_ZONE_TYPE_OUTLINE.get(zone.zone_type, "#64748b")

            # Glow effect
            canvas.create_oval(
                cx - r - 4,
                cy - r - 4,
                cx + r + 4,
                cy + r + 4,
                fill="",
                outline=fill,
                width=1,
                stipple="gray25",
                tags="node_glow",
            )

            # Main node circle
            canvas.create_oval(
                cx - r,
                cy - r,
                cx + r,
                cy + r,
                fill=self.CANVAS_BG,
                outline=outline,
                width=3,
                tags="node",
            )

            # Zone symbol
            sym = _ZONE_TYPE_SYMBOL.get(zone.zone_type, "○")
            canvas.create_text(
                cx,
                cy - 8,
                text=sym,
                fill=fill,
                font=("Segoe UI", 14, "bold"),
                tags="node_symbol",
            )

            # Max drones capacity label inside the node
            cap_text = f"max:{zone.max_drones}"
            canvas.create_text(
                cx,
                cy + 10,
                text=cap_text,
                fill=self.MUTED_COLOR,
                font=("Segoe UI", 7),
                tags="node_cap",
            )

            # Zone name label below
            canvas.create_text(
                cx,
                cy + r + 14,
                text=zone.name,
                fill=self.TEXT_COLOR,
                font=("Segoe UI", 9, "bold"),
                tags="node_label",
            )

            # Role badge
            if zone.is_start:
                canvas.create_text(
                    cx,
                    cy - r - 10,
                    text="START",
                    fill="#22c55e",
                    font=("Segoe UI", 7, "bold"),
                    tags="role_badge",
                )
            elif zone.is_end:
                canvas.create_text(
                    cx,
                    cy - r - 10,
                    text="END",
                    fill="#22c55e",
                    font=("Segoe UI", 7, "bold"),
                    tags="role_badge",
                )

    def _draw_arrowhead(
        self,
        x1: float,
        y1: float,
        x2: float,
        y2: float,
        node_r: float,
        canvas: _CanvasLike,
    ) -> None:
        """Draws a small arrowhead at the edge of the destination node."""
        angle = math.atan2(y2 - y1, x2 - x1)
        tip_x = x2 - node_r * math.cos(angle)
        tip_y = y2 - node_r * math.sin(angle)
        arrow_len = 10
        arrow_angle = math.pi / 7
        lx = tip_x - arrow_len * math.cos(angle - arrow_angle)
        ly = tip_y - arrow_len * math.sin(angle - arrow_angle)
        rx = tip_x - arrow_len * math.cos(angle + arrow_angle)
        ry = tip_y - arrow_len * math.sin(angle + arrow_angle)
        canvas.create_polygon(
            tip_x,
            tip_y,
            lx,
            ly,
            rx,
            ry,
            fill=self.EDGE_COLOR,
            outline=self.EDGE_COLOR,
            tags="arrowhead",
        )

    def _draw_turn(self) -> None:
        """Draws drone tokens for the current turn."""
        canvas = self.canvas
        canvas.delete("drone")
        canvas.delete("drone_label")

        total = len(self.reservations)

        if self.current_turn < 0:
            # Initial state: all drones at start
            self._turn_label.config(text="Turn: Start")
            self._progress_label.config(text="Ready to begin")
            start = self.graph.start_zone.name
            n = self.graph.nb_drones
            cx, cy = self._zone_positions[start]
            for idx, i in enumerate(range(1, n + 1)):
                self._draw_drone_token(canvas, cx, cy, i, n, "idle", index=idx)
            self._update_drone_info_panel(
                [
                    DronePosition(i, start, start, False, False)
                    for i in range(1, n + 1)
                ]
            )
            return

        turn = self.reservations[self.current_turn]
        positions = extract_turn_positions(turn)
        self._turn_label.config(
            text=f"Turn: {self.current_turn + 1} / {total}"
        )
        self._progress_label.config(
            text=f"Turn {self.current_turn + 1} of {total}"
        )

        # Group by zone
        zone_drones: Dict[str, List[DronePosition]] = {}
        for pos in positions:
            zone_drones.setdefault(pos.zone_name, []).append(pos)

        for zone_name, drones in zone_drones.items():
            if zone_name not in self._zone_positions:
                continue
            cx, cy = self._zone_positions[zone_name]
            n = len(drones)
            for i, dp in enumerate(drones):
                state = (
                    "transit"
                    if dp.on_transit
                    else ("waiting" if not dp.will_move else "moving")
                )
                self._draw_drone_token(
                    canvas, cx, cy, dp.drone_id, n, state, i
                )

        self._update_drone_info_panel(positions)

    def _draw_drone_token(
        self,
        canvas: _CanvasLike,
        cx: float,
        cy: float,
        drone_id: int,
        total_at_zone: int,
        state: str,
        index: int = 0,
    ) -> None:
        """Draws a single drone token around a zone center."""
        r = float(self.DRONE_RADIUS)

        # Distribute drones in a ring around the zone center
        if total_at_zone == 1:
            dx: float = 0.0
            dy: float = 0.0
        else:
            angle = (2 * math.pi * index) / total_at_zone - math.pi / 2
            ring_r = float(self.NODE_RADIUS) + r + 6.0
            dx = ring_r * math.cos(angle)
            dy = ring_r * math.sin(angle)

        tx, ty = cx + dx, cy + dy
        color = _GUI_DRONE_COLORS[drone_id % len(_GUI_DRONE_COLORS)]

        # State-dependent styling
        outline = color
        fill = color
        if state == "transit":
            fill = self.CANVAS_BG
            outline = color
        elif state == "waiting":
            fill = "#7f1d1d"
            outline = "#ef4444"

        canvas.create_oval(
            tx - r,
            ty - r,
            tx + r,
            ty + r,
            fill=fill,
            outline=outline,
            width=2,
            tags="drone",
        )
        canvas.create_text(
            tx,
            ty,
            text=str(drone_id),
            fill="#0f172a" if state == "moving" else self.TEXT_COLOR,
            font=("Segoe UI", 8, "bold"),
            tags="drone_label",
        )

    def _update_drone_info_panel(self, positions: List[DronePosition]) -> None:
        """Updates the sidebar drone-info section."""
        import tkinter as tk

        for w in self._drone_info_frame.winfo_children():
            w.destroy()

        for pos in sorted(positions, key=lambda p: p.drone_id):
            color = _GUI_DRONE_COLORS[pos.drone_id % len(_GUI_DRONE_COLORS)]
            status = ""
            if pos.on_transit:
                status = " ⏳transit"
            elif not pos.will_move:
                status = " 🔴wait"
            else:
                status = " ✈️move"

            label = tk.Label(
                self._drone_info_frame,
                text=f"  D{pos.drone_id} → {pos.zone_name}{status}",
                bg=self.PANEL_BG,
                fg=color,
                font=("Segoe UI", 8),
                anchor="w",
            )
            label.pack(fill="x", pady=0)

    # ── playback controls ────────────────────────────────────────────

    def _toggle_play(self) -> None:
        self.playing = not self.playing
        if self.playing:
            self._btn_play.config(text="⏸  Pause")
            self._auto_play()
        else:
            self._btn_play.config(text="▶  Play")
            if self._after_id:
                self.root.after_cancel(self._after_id)
                self._after_id = None

    def _auto_play(self) -> None:
        if not self.playing:
            return
        if self.current_turn < len(self.reservations) - 1:
            self._step_forward()
            self._after_id = self.root.after(
                self._speed_var.get(), self._auto_play
            )
        else:
            self.playing = False
            self._btn_play.config(text="▶  Play")

    def _step_forward(self) -> None:
        if self.current_turn < len(self.reservations) - 1:
            self.current_turn += 1
            self._draw_turn()

    def _step_back(self) -> None:
        if self.current_turn > -1:
            self.current_turn -= 1
            self._draw_turn()

    def _reset(self) -> None:
        self.playing = False
        self._btn_play.config(text="▶  Play")
        if self._after_id:
            self.root.after_cancel(self._after_id)
            self._after_id = None
        self.current_turn = -1
        self._draw_turn()

    def _on_speed_change(self, *_args: object) -> None:
        val = self._speed_var.get()
        self._speed_label.config(text=f"{val} ms")


# ═══════════════════════════════════════════════════════════════════════
# Standalone entry-point
# ═══════════════════════════════════════════════════════════════════════


def _run_simulation(map_file: str) -> Tuple[Graph, List[Turn]]:
    """Parses the map, runs the scheduler, and returns results."""
    parser = MapParser()
    graph = parser.parse_file(map_file)
    graph.block(graph.start_zone, [])
    scheduler = Scheduler(
        graph, [Drone(i) for i in range(1, graph.nb_drones + 1)]
    )
    scheduler.schedule_drones()
    return graph, scheduler.reservations


def main() -> None:
    ap = argparse.ArgumentParser(
        description="Fly-in Drone Simulation — Visual Representation"
    )
    ap.add_argument("map_file", help="Path to the map file")
    ap.add_argument(
        "--mode",
        choices=["terminal", "gui", "both"],
        default="both",
        help="Visualization mode (default: both)",
    )
    ap.add_argument(
        "--speed",
        type=int,
        default=800,
        help="GUI playback speed in ms per turn (default: 800)",
    )
    ap.add_argument(
        "--delay",
        type=float,
        default=0.3,
        help="Terminal turn delay in seconds (default: 0.3)",
    )
    args = ap.parse_args()

    graph, reservations = _run_simulation(args.map_file)

    if args.mode in ("terminal", "both"):
        tv = TerminalVisualizer(graph, reservations)
        tv.display(delay=args.delay)

    if args.mode in ("gui", "both"):
        gv = GUIVisualizer(graph, reservations, speed_ms=args.speed)
        gv.show()


if __name__ == "__main__":
    main()
