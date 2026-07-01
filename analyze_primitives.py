"""
Analyze motion primitives from JSON files.
Designed for interactive exploration in VS Code's interactive window.
"""

import json
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.ticker import MultipleLocator
from pathlib import Path
from collections import Counter


class PrimitivesAnalyzer:
    """
    Load and analyze motion primitives.
    
    Attributes:
    json_path   (Path): Path to the primitives JSON file.
    data        (dict): Loaded JSON data.
    metadata    (dict): Metadata from the JSON file.
    primitives  (list of dict): List of motion primitives.
    
    Methods:
    load_data(): Load primitives from JSON file.
    print_metadata(): Print metadata and configuration parameters.
    print_statistics(): Print statistics about the primitive set.
    print_primitives_summary(limit=None): Print summary table of primitives.
    plot_primitive(trajectory_id, show_curvature=False): Plot a single primitive trajectory.
    plot_all_primitives_by_start_angle(start_angle_idx): Plot all primitives starting from a given angle.

    """
    
    def __init__(self, json_path):
        """Initialize analyzer with primitives JSON file."""
        self.json_path = Path(json_path)
        self.data = None
        self.metadata = None
        self.primitives = None
        self.load_data()
    
    def load_data(self):
        """Load primitives from JSON file."""
        with open(self.json_path, 'r') as f:
            self.data = json.load(f)
        self.metadata = self.data.get('lattice_metadata', {})
        self.primitives = self.data.get('primitives', [])
        
        # convert poses from list of lists to array
        [p.update({'poses': np.array(p['poses'])}) for p in self.primitives]
        
        print(f"✓ Loaded {len(self.primitives)} primitives from {self.json_path.name}")
    
    def print_metadata(self):
        """Print metadata and configuration parameters."""
        print("\n" + "="*70)
        print("LATTICE METADATA")
        print("="*70)
        print(f"Version: {self.data.get('version', 'N/A')}")
        print(f"Date Generated: {self.data.get('date_generated', 'N/A')}")
        print()
        
        print("Motion Model Parameters:")
        print(f"  • Motion Model: {self.metadata.get('motion_model', 'N/A').upper()}")
        print(f"  • Turning Radius: {self.metadata.get('turning_radius', 'N/A')} m")
        print(f"  • Grid Resolution: {self.metadata.get('grid_resolution', 'N/A')} m")
        print()
        
        print("Planning Parameters:")
        print(f"  • Number of Headings: {self.metadata.get('num_of_headings', 'N/A')}")
        print(f"  • Stopping Threshold: {self.metadata.get('stopping_threshold', 'N/A')} iterations")
        print(f"  • Max Wavefront Expansion: {self.metadata.get('max_wavefront_expansion', 'N/A')}")
        print(f"  • Verbosity: {self.metadata.get('verbosity', 'N/A')}")
        print()
        
        print("Velocity/Acceleration Limits:")
        print(f"  • Max Linear Velocity: {self.metadata.get('max_vel', 'N/A')} m/s")
        print(f"  • Min Linear Velocity: {self.metadata.get('min_vel', 'N/A')} m/s")
        print(f"  • Max Angular Velocity: {self.metadata.get('max_ang_vel', 'N/A')} rad/s")
        print(f"  • Max Linear Acceleration: {self.metadata.get('max_lin_acc', 'N/A')} m/s²")
        print(f"  • Max Angular Acceleration: {self.metadata.get('max_ang_acc', 'N/A')} rad/s²")
        print()
        
        # if self.metadata.get('heading_angles'):
        #     headings = self.metadata['heading_angles']
        #     print(f"Discrete Headings ({len(headings)} angles, in radians):")
        #     headings_deg = np.degrees(headings)
        #     for i, (rad, deg) in enumerate(zip(headings, headings_deg)):
        #         print(f"  [{i:2d}]  {rad:7.4f} rad  ({deg:6.2f}°)")
    
    def print_statistics(self):
        """Print statistics about the primitive set."""
        print("\n" + "="*70)
        print("PRIMITIVE STATISTICS")
        print("="*70)
        print(f"Total Number of Primitives: {len(self.primitives)}")
        print()
        
        # Analyze by start angle
        start_angles = [p.get('start_angle_index') for p in self.primitives]
        start_angle_counts = Counter(start_angles)
        # print(f"Primitives per Start Angle: {dict(sorted(start_angle_counts.items()))}")
        # print()
        
        # Analyze left/right turns
        turns = [p.get('left_turn') for p in self.primitives]
        left_turns = sum(1 for t in turns if t is True)
        right_turns = sum(1 for t in turns if t is False)
        print(f"Left Turns: {left_turns}")
        print(f"Right Turns: {right_turns}")
        print()
        
        # Trajectory properties
        radii = [p.get('trajectory_radius', 0) for p in self.primitives]
        lengths = [p.get('trajectory_length', 0) for p in self.primitives]
        arc_lengths = [p.get('arc_length', 0) for p in self.primitives]
        
        print("Trajectory Properties:")
        print(f"  Turning Radius:")
        print(f"    • Min: {min(radii):.4f} m, Max: {max(radii):.4f} m")
        print(f"  Trajectory Length:")
        print(f"    • Min: {min(lengths):.4f} m, Max: {max(lengths):.4f} m")
        print(f"    • Mean: {np.mean(lengths):.4f} m")
        print(f"  Arc Length:")
        print(f"    • Min: {min(arc_lengths):.4f} m, Max: {max(arc_lengths):.4f} m")
        print(f"    • Mean: {np.mean(arc_lengths):.4f} m")
    
    def print_primitives_summary(self, limit=None):
        """Print summary table of primitives."""
        print("\n" + "="*70)
        print("PRIMITIVES SUMMARY")
        print("="*70)
        
        limit = limit or len(self.primitives)
        print(f"Showing {min(limit, len(self.primitives))} of {len(self.primitives)} primitives:\n")
        
        # Header
        print(f"{'ID':>3} {'S':>2} {'E':>2} {'Turn':>5} {'Radius':>7} {'Length':>8} {'Arc':>8} {'Poses':>6}")
        print("-" * 70)
        
        for i, prim in enumerate(self.primitives[:limit]):
            traj_id = prim.get('trajectory_id', 'N/A')
            start_idx = prim.get('start_angle_index', 'N/A')
            end_idx = prim.get('end_angle_index', 'N/A')
            left_turn = 'L' if prim.get('left_turn') else 'R'
            radius = prim.get('trajectory_radius', 0)
            length = prim.get('trajectory_length', 0)
            arc = prim.get('arc_length', 0)
            num_poses = prim.get('poses', np.array([])).shape[0]
            
            print(f"{traj_id:3d} {start_idx:2d} {end_idx:2d} {left_turn:>5} {radius:7.3f} {length:8.5f} {arc:8.5f} {num_poses:6d}")
        
        if len(self.primitives) > limit:
            print(f"... and {len(self.primitives) - limit} more primitives")
    
    
    def plot_primitive(self, trajectory_ids=None, show_headings=False, show_curvature=False):
        """Plot one or more primitive trajectories.
        
        Args:
            trajectory_ids: int or list of ints specifying trajectory IDs to plot
            show_headings: bool, whether to show heading arrows along trajectories (default False)
            show_curvature: bool, whether to show curvature profile (default False)
        """
        # Convert single ID to list
        if isinstance(trajectory_ids, int):
            trajectory_ids = [trajectory_ids]
            
        if trajectory_ids is None:
            trajectory_ids = range(len(self.primitives))
        
        # Validate IDs
        for traj_id in trajectory_ids:
            if traj_id >= len(self.primitives):
                print(f"Error: trajectory_id {traj_id} out of range")
                return
        
        if show_curvature:
            fig, axes = plt.subplots(1, 2, figsize=(12, 6))
            ax1, ax2 = axes
        else:
            fig, ax1 = plt.subplots(figsize=(12, 10))

        
        # Use colormap for multiple trajectories
        colors = plt.cm.tab10(np.linspace(0, 1, 10))
        
        # Plot each trajectory
        for idx, trajectory_id in enumerate(trajectory_ids):
            prim = self.primitives[trajectory_id]
            poses = prim.get('poses', np.array([]))
            
            if poses.shape[0] == 0:
                print(f"Warning: No poses available for trajectory {trajectory_id}")
                continue
            
            xs = poses[:, 0]
            ys = poses[:, 1]
            color = colors[idx % 10]
            
            # Only add label to legend for first 15 entries
            label = f'ID {trajectory_id}' if idx < 15 else ''
            
            # Plot trajectory path
            ax1.plot(xs, ys, '-', linewidth=2, color=color, label=label, alpha=0.8)
            ax1.plot(xs[0], ys[0], 'o', markerfacecolor='none', markeredgecolor=color,markersize=5)
            ax1.plot(xs[-1], ys[-1], 'o', color=color, markersize=5)
            
            # Add heading arrows if requested
            if show_headings:
                # Scale arrows based on primitive trajectory length
                trajectory_length = prim.get('trajectory_length', 0.1)
                arrow_length = max(0.01, trajectory_length * 0.15)  # ~15% of trajectory length
                head_width = arrow_length * 0.4
                head_length = arrow_length * 0.25
                
                step = poses.shape[0] // 4
                for i in range(0, poses.shape[0], step):
                    theta = poses[i, 2]
                    dx = arrow_length * np.cos(theta)
                    dy = arrow_length * np.sin(theta)
                    ax1.arrow(xs[i], ys[i], dx, dy, head_width=head_width, head_length=head_length, 
                             fc=color, ec=color, alpha=0.5)
            
            # Plot curvature if requested
            if show_curvature:
                curvatures = poses[:, 3]
                segement_lengths = np.hypot(np.diff(xs, prepend=xs[0]), np.diff(ys, prepend=ys[0]))
                curve_lenght = np.cumsum(segement_lengths)
                ax2.plot(curve_lenght, curvatures, '-', linewidth=2, color=color, label=label, alpha=0.8)

                # tot_curvature = np.sum(curvatures * segement_lengths)
                # print(f"Trajectory ID {trajectory_id}: Total Curvature = {tot_curvature:.3f} rad")
                # delta_theta = poses[-1, 2] - poses[0, 2]
                # delta_theta = np.arctan2(np.sin(delta_theta), np.cos(delta_theta))
                # print(f"Trajectory ID {trajectory_id}: Change in Heading = {delta_theta:.3f} rad")
                
        ax1.set_xlabel('X (m)')
        ax1.set_ylabel('Y (m)')
        ax1.set_title(f'Trajectories: {trajectory_ids}')
        
        # Align grid to grid resolution
        xlim = ax1.get_xlim()
        xrange = xlim[1] - xlim[0]
        grid_resolution = self.metadata.get('grid_resolution', 0.05)
        divisor = grid_resolution*10
        grid_adjusted = np.round((xrange/10)/divisor) * divisor
        grid_resolution = max(grid_adjusted, grid_resolution)  # Ensure a minimum grid resolution
        ax1.xaxis.set_major_locator(MultipleLocator(grid_resolution))
        ax1.yaxis.set_major_locator(MultipleLocator(grid_resolution))
        ax1.grid(True, alpha=0.3, which='major')
        ax1.axis('equal')
        ax1.legend()
        
        # Format curvature plot
        if show_curvature:
            ax2.axhline(y=0, color='k', linestyle='--', alpha=0.3)
            ax2.set_xlabel('Arc Length (m)')
            ax2.set_ylabel('Signed Curvature (rad/m)')
            ax2.set_title('Curvature Profile')
            ax2.grid(True, alpha=0.3)
            ax2.legend()
        
        plt.tight_layout()
        plt.show()


# ============================================================================
# INTERACTIVE USAGE
# ============================================================================

# %% -----------------------------------------------
# 1) Analyze all dataset

# Load primitives
# primitives_path = r"outputs\omni_primitives\primitives.json"
# primitives_path = r"outputs\diff_primitives\primitives.json"
# primitives_path = r"outputs\diff_primitives\diff_primitives_wcurv.json"
# primitives_path = r"outputs\diff_primitives\diff_primitives_wcurv_resample.json"
primitives_path = r"outputs\diff_primitives\diff_primitives_wcurv_downsamp4.json"
primitives_path = r"outputs\diff_primitives_cc\diff_primitives_wcurv.json"
# primitives_path = r"outputs\ackermann_primitives\ackermann_primitives_wcurv.json"

analyzer = PrimitivesAnalyzer(primitives_path)

# Display information
analyzer.print_metadata()
analyzer.print_statistics()
analyzer.print_primitives_summary(limit=20)

# %% -----------------------------------------------
# 2) Analyze selected trajetory

# Few trajectories witrh heading and curvature
analyzer.plot_primitive([0, 2], show_headings=True, show_curvature=True)

# Multiple trajectories, no headings
analyzer.plot_primitive(range(0, 5),show_curvature=True)
analyzer.plot_primitive([11, 22, 225],show_curvature=True)
analyzer.plot_primitive(range(0, 100),show_curvature=True)

# Plot all trajectories
analyzer.plot_primitive()



# %%
