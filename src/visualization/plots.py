"""
Suspension Analysis Visualization Module

Creates professional engineering diagrams and plots for suspension analysis:
- Camber gain curves
- Roll center diagrams
- Load transfer distributions
- Kinematic diagrams
- Performance comparisons

Uses matplotlib with publication-quality settings.
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.patches import FancyBboxPatch, Circle, Arc, FancyArrowPatch
from typing import Dict, List, Tuple
import os


# Set professional plot style
plt.style.use('seaborn-v0_8-darkgrid')
plt.rcParams['figure.figsize'] = (12, 8)
plt.rcParams['font.size'] = 11
plt.rcParams['axes.labelsize'] = 12
plt.rcParams['axes.titlesize'] = 14
plt.rcParams['xtick.labelsize'] = 10
plt.rcParams['ytick.labelsize'] = 10
plt.rcParams['legend.fontsize'] = 10
plt.rcParams['figure.titlesize'] = 16


def plot_camber_gain_curve(travel: np.ndarray, camber: np.ndarray,
                           output_path: str = None):
    """
    Plot camber angle vs suspension travel.

    Args:
        travel: Wheel travel array (inches, 0 = ride height)
        camber: Camber angle array (degrees, negative = top tilted in)
        output_path: Where to save the plot
    """
    fig, ax = plt.subplots(figsize=(10, 7))

    # Plot data
    ax.plot(travel, camber, 'b-o', linewidth=2.5, markersize=8,
            label='Measured Camber', markerfacecolor='white', markeredgewidth=2)

    # Linear fit
    coeffs = np.polyfit(travel, camber, 1)
    fit_line = np.polyval(coeffs, travel)
    ax.plot(travel, fit_line, 'r--', linewidth=2, alpha=0.7,
            label=f'Linear Fit: {coeffs[0]:.3f} deg/inch')

    # Formatting
    ax.set_xlabel('Suspension Travel (inches)\nNegative = Droop, Positive = Bump',
                  fontsize=12, fontweight='bold')
    ax.set_ylabel('Camber Angle (degrees)\nNegative = Top In',
                  fontsize=12, fontweight='bold')
    ax.set_title('Front Suspension Camber Gain Curve\n1992 Toyota Celica GT',
                 fontsize=14, fontweight='bold')

    # Grid and legend
    ax.grid(True, alpha=0.3, linestyle='--')
    ax.legend(loc='best', framealpha=0.9)

    # Annotations
    ax.axhline(y=0, color='k', linestyle='-', linewidth=0.8, alpha=0.3)
    ax.axvline(x=0, color='k', linestyle='-', linewidth=0.8, alpha=0.3)
    ax.text(0, camber[len(camber)//2], ' Ride Height', ha='left', va='bottom',
            fontsize=9, style='italic')

    # Add zones
    ax.axvspan(-4, 0, alpha=0.1, color='blue', label='Droop Zone')
    ax.axvspan(0, 4, alpha=0.1, color='red', label='Bump Zone')

    plt.tight_layout()

    if output_path:
        plt.savefig(output_path, dpi=150, bbox_inches='tight')
        print(f"✓ Saved: {output_path}")
    else:
        plt.show()

    plt.close()


def plot_load_transfer_distribution(front_pct: float, rear_pct: float,
                                    front_weight_pct: float,
                                    output_path: str = None):
    """
    Plot load transfer distribution vs weight distribution.

    Args:
        front_pct: Front lateral load transfer percentage
        rear_pct: Rear lateral load transfer percentage
        front_weight_pct: Front static weight percentage
        output_path: Where to save the plot
    """
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 6))

    # Plot 1: Bar chart comparison
    categories = ['Front', 'Rear']
    weight_dist = [front_weight_pct, 100 - front_weight_pct]
    lt_dist = [front_pct, rear_pct]

    x = np.arange(len(categories))
    width = 0.35

    bars1 = ax1.bar(x - width/2, weight_dist, width, label='Weight Distribution',
                    color='#2E86AB', alpha=0.8, edgecolor='black')
    bars2 = ax1.bar(x + width/2, lt_dist, width, label='Load Transfer Distribution',
                    color='#A23B72', alpha=0.8, edgecolor='black')

    ax1.set_ylabel('Percentage (%)', fontsize=12, fontweight='bold')
    ax1.set_title('Weight vs Load Transfer Distribution\n@ 1.0g Lateral',
                  fontsize=13, fontweight='bold')
    ax1.set_xticks(x)
    ax1.set_xticklabels(categories, fontsize=12)
    ax1.legend(loc='upper right', framealpha=0.9)
    ax1.grid(axis='y', alpha=0.3)
    ax1.set_ylim(0, 75)

    # Add value labels on bars
    for bar in bars1 + bars2:
        height = bar.get_height()
        ax1.text(bar.get_x() + bar.get_width()/2., height,
                f'{height:.1f}%', ha='center', va='bottom', fontsize=10)

    # Plot 2: Bias analysis
    bias = front_pct - front_weight_pct
    colors = ['#00A878' if abs(bias) < 2 else '#FF6B35' if bias > 0 else '#4ECDC4']

    ax2.barh(['Balance'], [bias], color=colors[0], alpha=0.8, edgecolor='black', height=0.4)
    ax2.axvline(x=0, color='black', linestyle='-', linewidth=1.5)
    ax2.set_xlabel('Front Bias (%)\nNegative = Oversteer, Positive = Understeer',
                   fontsize=11, fontweight='bold')
    ax2.set_title('Handling Balance Analysis',
                  fontsize=13, fontweight='bold')
    ax2.grid(axis='x', alpha=0.3)
    ax2.set_xlim(-15, 15)

    # Add bias zones
    ax2.axvspan(-15, -5, alpha=0.15, color='blue')
    ax2.axvspan(-5, 5, alpha=0.15, color='green')
    ax2.axvspan(5, 15, alpha=0.15, color='red')

    ax2.text(-10, 0.2, 'Oversteer\nTendency', ha='center', va='center',
             fontsize=9, style='italic')
    ax2.text(0, 0.2, 'Neutral', ha='center', va='center',
             fontsize=9, style='italic', weight='bold')
    ax2.text(10, 0.2, 'Understeer\nTendency', ha='center', va='center',
             fontsize=9, style='italic')

    # Add value
    ax2.text(bias, -0.25, f'{bias:+.1f}%', ha='center', va='top',
             fontsize=12, weight='bold')

    plt.tight_layout()

    if output_path:
        plt.savefig(output_path, dpi=150, bbox_inches='tight')
        print(f"✓ Saved: {output_path}")
    else:
        plt.show()

    plt.close()


def plot_roll_center_diagram(ic_left_y: float, ic_left_z: float,
                             ic_right_y: float, ic_right_z: float,
                             rc_y: float, rc_z: float,
                             track_width: float,
                             output_path: str = None):
    """
    Create front-view roll center diagram.

    Args:
        ic_left_y, ic_left_z: Left instant center coordinates
        ic_right_y, ic_right_z: Right instant center coordinates
        rc_y, rc_z: Roll center coordinates
        track_width: Track width for scaling
        output_path: Where to save the diagram
    """
    fig, ax = plt.subplots(figsize=(12, 10))

    # Contact patches
    cp_left_y, cp_right_y = -track_width/2, track_width/2
    cp_z = 0

    # Plot contact patches
    ax.plot([cp_left_y, cp_right_y], [cp_z, cp_z], 'ks', markersize=12,
            label='Contact Patches')

    # Plot instant centers
    ax.plot(ic_left_y, ic_left_z, 'ro', markersize=10, label='Instant Centers')
    ax.plot(ic_right_y, ic_right_z, 'ro', markersize=10)

    # Plot swing arms (IC to contact patch)
    ax.plot([ic_left_y, cp_left_y], [ic_left_z, cp_z], 'b--', linewidth=2,
            alpha=0.6, label='Virtual Swing Arms')
    ax.plot([ic_right_y, cp_right_y], [ic_right_z, cp_z], 'b--', linewidth=2,
            alpha=0.6)

    # Plot roll center
    ax.plot(rc_y, rc_z, 'g*', markersize=20, label='Roll Center',
            markeredgecolor='darkgreen', markeredgewidth=1.5)

    # Add annotations
    ax.annotate(f'RC Height: {rc_z:.2f}"',
                xy=(rc_y, rc_z), xytext=(10, rc_z + 5),
                fontsize=11, fontweight='bold',
                bbox=dict(boxstyle='round,pad=0.5', facecolor='yellow', alpha=0.7),
                arrowprops=dict(arrowstyle='->', lw=2))

    ax.annotate('Left IC',
                xy=(ic_left_y, ic_left_z), xytext=(ic_left_y - 8, ic_left_z + 3),
                fontsize=10, ha='right',
                arrowprops=dict(arrowstyle='->', lw=1.5))

    ax.annotate('Right IC',
                xy=(ic_right_y, ic_right_z), xytext=(ic_right_y + 8, ic_right_z + 3),
                fontsize=10, ha='left',
                arrowprops=dict(arrowstyle='->', lw=1.5))

    # Formatting
    ax.set_xlabel('Lateral Position (inches)\nNegative = Left, Positive = Right',
                  fontsize=12, fontweight='bold')
    ax.set_ylabel('Height Above Ground (inches)',
                  fontsize=12, fontweight='bold')
    ax.set_title('Front Suspension Roll Center Diagram\n1992 Toyota Celica GT - Front View',
                 fontsize=14, fontweight='bold')

    ax.axhline(y=0, color='brown', linestyle='-', linewidth=3, alpha=0.5, label='Ground')
    ax.axvline(x=0, color='gray', linestyle='--', linewidth=1, alpha=0.3)

    ax.grid(True, alpha=0.3)
    ax.legend(loc='upper right', framealpha=0.9)
    ax.set_aspect('equal', adjustable='box')

    # Set reasonable axis limits
    y_range = max(abs(ic_left_y), abs(ic_right_y)) * 1.3
    ax.set_xlim(-y_range, y_range)
    ax.set_ylim(-2, max(ic_left_z, ic_right_z) * 1.2)

    plt.tight_layout()

    if output_path:
        plt.savefig(output_path, dpi=150, bbox_inches='tight')
        print(f"✓ Saved: {output_path}")
    else:
        plt.show()

    plt.close()


def create_summary_dashboard(analysis_data: Dict, output_path: str = None):
    """
    Create comprehensive summary dashboard with key metrics.

    Args:
        analysis_data: Dictionary with all analysis results
        output_path: Where to save the dashboard
    """
    fig = plt.figure(figsize=(16, 10))
    gs = fig.add_gridspec(3, 3, hspace=0.4, wspace=0.4)

    # Title
    fig.suptitle('1992 Toyota Celica GT - Suspension Analysis Dashboard',
                 fontsize=18, fontweight='bold', y=0.98)

    # Extract data
    front = analysis_data.get('front_suspension', {})
    lt_1g = analysis_data.get('load_transfer_1g', {})

    # Plot 1: Key Metrics (text summary)
    ax1 = fig.add_subplot(gs[0, :])
    ax1.axis('off')

    metrics_text = f"""
    FRONT SUSPENSION GEOMETRY - MacPherson Strut

    Roll Center Height: {front['roll_center']['height']:.2f}" | Assessment: {front['roll_center']['evaluation']['assessment']}
    Camber Gain: {front['camber_gain']['gain_deg_per_inch']:.3f} deg/inch | Assessment: {front['camber_gain']['assessment']}
    Anti-Dive: {front['anti_dive_percent']:.1f}% | Motion Ratio: {front['motion_ratio']:.3f}
    """

    ax1.text(0.5, 0.5, metrics_text, transform=ax1.transAxes,
             fontsize=11, verticalalignment='center', horizontalalignment='center',
             bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5),
             family='monospace')

    # Plot 2: Camber gain
    ax2 = fig.add_subplot(gs[1, 0])
    travel = np.array(front['camber_gain']['travel_range'])
    camber = np.array(front['camber_gain']['camber_angles'])
    ax2.plot(travel, camber, 'o-', linewidth=2)
    ax2.set_xlabel('Travel (in)')
    ax2.set_ylabel('Camber (deg)')
    ax2.set_title('Camber Gain Curve')
    ax2.grid(True, alpha=0.3)

    # Plot 3: Load transfer bar
    ax3 = fig.add_subplot(gs[1, 1])
    ax3.bar(['Front', 'Rear'],
            [lt_1g['front_percentage'], lt_1g['rear_percentage']],
            color=['#2E86AB', '#A23B72'], alpha=0.8, edgecolor='black')
    ax3.set_ylabel('Load Transfer (%)')
    ax3.set_title('LT Distribution @ 1.0g')
    ax3.set_ylim(0, 100)
    ax3.grid(axis='y', alpha=0.3)

    # Plot 4: Handling balance gauge
    ax4 = fig.add_subplot(gs[1, 2])
    balance = lt_1g['handling_balance']['balance']
    bias = lt_1g['handling_balance']['front_bias']

    if "Understeer" in balance:
        color = 'red'
    elif "Oversteer" in balance:
        color = 'blue'
    else:
        color = 'green'

    ax4.barh([0], [bias], color=color, alpha=0.7, height=0.5)
    ax4.set_xlim(-10, 10)
    ax4.set_yticks([])
    ax4.set_xlabel('Front Bias (%)')
    ax4.set_title(f'Balance: {balance}')
    ax4.axvline(x=0, color='black', linestyle='-', linewidth=2)
    ax4.grid(axis='x', alpha=0.3)

    # Plot 5: Performance summary table
    ax5 = fig.add_subplot(gs[2, :])
    ax5.axis('off')

    summary_text = f"""
    LOAD TRANSFER ANALYSIS @ 1.0g LATERAL

    Total Load Transfer: {lt_1g['total_transfer_lbs']:.0f} lbs
    Front: {lt_1g['front_transfer_lbs']:.0f} lbs ({lt_1g['front_percentage']:.1f}%)  |  Rear: {lt_1g['rear_transfer_lbs']:.0f} lbs ({lt_1g['rear_percentage']:.1f}%)

    Handling Balance: {balance}
    Tendency: {lt_1g['handling_balance']['tendency']}
    """

    ax5.text(0.5, 0.5, summary_text, transform=ax5.transAxes,
             fontsize=10, verticalalignment='center', horizontalalignment='center',
             bbox=dict(boxstyle='round', facecolor='lightblue', alpha=0.5),
             family='monospace')

    if output_path:
        plt.savefig(output_path, dpi=150, bbox_inches='tight')
        print(f"✓ Saved: {output_path}")
    else:
        plt.show()

    plt.close()


if __name__ == "__main__":
    print("Suspension Visualization Module")
    print("Run main.py to generate all plots")
