"""
Data Visualization Module.

This module provides comprehensive plotting and visualization tools
for suspension analysis data, including time series, frequency spectra,
G-G diagrams, track maps, and more.
"""

import logging
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple, Union

import numpy as np

try:
    import matplotlib.pyplot as plt
    import matplotlib.colors as mcolors
    from matplotlib.figure import Figure
    from matplotlib.gridspec import GridSpec
    HAS_MATPLOTLIB = True
except ImportError:
    HAS_MATPLOTLIB = False

try:
    import plotly.graph_objects as go
    import plotly.express as px
    from plotly.subplots import make_subplots
    HAS_PLOTLY = True
except ImportError:
    HAS_PLOTLY = False

logger = logging.getLogger(__name__)


@dataclass
class PlotConfig:
    """Configuration for plot appearance."""
    figsize: Tuple[int, int] = (12, 8)
    dpi: int = 150
    style: str = 'default'
    title_fontsize: int = 14
    label_fontsize: int = 12
    legend_fontsize: int = 10
    grid: bool = True
    tight_layout: bool = True

    # Colors for corners
    corner_colors: Dict[str, str] = None

    def __post_init__(self):
        if self.corner_colors is None:
            self.corner_colors = {
                'front_left': '#1f77b4',
                'front_right': '#ff7f0e',
                'rear_left': '#2ca02c',
                'rear_right': '#d62728'
            }


class Visualizer:
    """
    Suspension Data Visualizer.

    Provides methods for creating various plots and visualizations
    of suspension data. Supports both matplotlib (static) and
    plotly (interactive) backends.
    """

    def __init__(
        self,
        config: Optional[PlotConfig] = None,
        backend: str = 'matplotlib',
        output_dir: str = 'plots'
    ):
        """
        Initialize visualizer.

        Args:
            config: Plot configuration
            backend: Plotting backend ('matplotlib' or 'plotly')
            output_dir: Directory for saving plots
        """
        self.config = config or PlotConfig()
        self.backend = backend
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(parents=True, exist_ok=True)

        if backend == 'matplotlib' and not HAS_MATPLOTLIB:
            logger.warning("matplotlib not available, falling back to plotly")
            self.backend = 'plotly'
        if backend == 'plotly' and not HAS_PLOTLY:
            logger.warning("plotly not available, falling back to matplotlib")
            self.backend = 'matplotlib'

    def _create_figure(
        self,
        nrows: int = 1,
        ncols: int = 1,
        **kwargs
    ) -> Tuple[Any, Any]:
        """Create a new figure."""
        if self.backend == 'matplotlib':
            fig, axes = plt.subplots(
                nrows, ncols,
                figsize=self.config.figsize,
                **kwargs
            )
            return fig, axes
        else:
            fig = make_subplots(rows=nrows, cols=ncols, **kwargs)
            return fig, None

    def _save_figure(
        self,
        fig: Any,
        filename: str,
        format: str = 'png'
    ) -> str:
        """Save figure to file."""
        filepath = self.output_dir / f"{filename}.{format}"

        if self.backend == 'matplotlib':
            fig.savefig(
                filepath,
                dpi=self.config.dpi,
                bbox_inches='tight' if self.config.tight_layout else None
            )
            plt.close(fig)
        else:
            if format == 'html':
                fig.write_html(str(filepath))
            else:
                fig.write_image(str(filepath))

        return str(filepath)

    def plot_time_series(
        self,
        time: np.ndarray,
        data: Dict[str, np.ndarray],
        title: str = "Time Series",
        ylabel: str = "Value",
        save_as: Optional[str] = None
    ) -> Any:
        """
        Plot time series data.

        Args:
            time: Time array (seconds)
            data: Dictionary mapping series names to data arrays
            title: Plot title
            ylabel: Y-axis label
            save_as: Filename to save (without extension)

        Returns:
            Figure object
        """
        if self.backend == 'matplotlib':
            fig, ax = plt.subplots(figsize=self.config.figsize)

            for name, values in data.items():
                color = self.config.corner_colors.get(name)
                ax.plot(time, values, label=name, color=color)

            ax.set_xlabel('Time (s)', fontsize=self.config.label_fontsize)
            ax.set_ylabel(ylabel, fontsize=self.config.label_fontsize)
            ax.set_title(title, fontsize=self.config.title_fontsize)
            ax.legend(fontsize=self.config.legend_fontsize)
            if self.config.grid:
                ax.grid(True, alpha=0.3)

            if save_as:
                self._save_figure(fig, save_as)

            return fig

        else:
            fig = go.Figure()

            for name, values in data.items():
                color = self.config.corner_colors.get(name)
                fig.add_trace(go.Scatter(
                    x=time, y=values,
                    name=name,
                    line=dict(color=color)
                ))

            fig.update_layout(
                title=title,
                xaxis_title='Time (s)',
                yaxis_title=ylabel
            )

            if save_as:
                self._save_figure(fig, save_as, 'html')

            return fig

    def plot_suspension_travel(
        self,
        time: np.ndarray,
        pot_fl: np.ndarray,
        pot_fr: np.ndarray,
        pot_rl: np.ndarray,
        pot_rr: np.ndarray,
        title: str = "Suspension Travel",
        save_as: Optional[str] = None
    ) -> Any:
        """
        Plot suspension travel for all four corners.

        Args:
            time: Time array
            pot_*: Potentiometer data for each corner
            title: Plot title
            save_as: Filename to save

        Returns:
            Figure object
        """
        data = {
            'front_left': pot_fl,
            'front_right': pot_fr,
            'rear_left': pot_rl,
            'rear_right': pot_rr
        }

        return self.plot_time_series(
            time, data,
            title=title,
            ylabel='Travel (mm)',
            save_as=save_as
        )

    def plot_gg_diagram(
        self,
        lateral_g: np.ndarray,
        longitudinal_g: np.ndarray,
        title: str = "G-G Diagram",
        show_boundary: bool = True,
        save_as: Optional[str] = None
    ) -> Any:
        """
        Plot G-G diagram (friction circle).

        Args:
            lateral_g: Lateral acceleration
            longitudinal_g: Longitudinal acceleration
            title: Plot title
            show_boundary: Draw friction circle boundary
            save_as: Filename to save

        Returns:
            Figure object
        """
        if self.backend == 'matplotlib':
            fig, ax = plt.subplots(figsize=(10, 10))

            # Scatter plot with density coloring
            ax.scatter(lateral_g, longitudinal_g, alpha=0.3, s=1, c='blue')

            # Draw friction circle
            if show_boundary:
                max_g = max(np.max(np.abs(lateral_g)), np.max(np.abs(longitudinal_g)))
                theta = np.linspace(0, 2*np.pi, 100)
                ax.plot(max_g * np.cos(theta), max_g * np.sin(theta),
                       'r--', linewidth=2, label=f'Max: {max_g:.2f}g')

            ax.axhline(y=0, color='gray', linestyle='-', linewidth=0.5)
            ax.axvline(x=0, color='gray', linestyle='-', linewidth=0.5)

            ax.set_xlabel('Lateral G', fontsize=self.config.label_fontsize)
            ax.set_ylabel('Longitudinal G', fontsize=self.config.label_fontsize)
            ax.set_title(title, fontsize=self.config.title_fontsize)
            ax.set_aspect('equal')
            ax.legend()
            ax.grid(True, alpha=0.3)

            if save_as:
                self._save_figure(fig, save_as)

            return fig

        else:
            fig = go.Figure()

            fig.add_trace(go.Scatter(
                x=lateral_g, y=longitudinal_g,
                mode='markers',
                marker=dict(size=2, opacity=0.3),
                name='G-G'
            ))

            if show_boundary:
                max_g = max(np.max(np.abs(lateral_g)), np.max(np.abs(longitudinal_g)))
                theta = np.linspace(0, 2*np.pi, 100)
                fig.add_trace(go.Scatter(
                    x=max_g * np.cos(theta),
                    y=max_g * np.sin(theta),
                    mode='lines',
                    line=dict(dash='dash', color='red'),
                    name=f'Max: {max_g:.2f}g'
                ))

            fig.update_layout(
                title=title,
                xaxis_title='Lateral G',
                yaxis_title='Longitudinal G',
                yaxis_scaleanchor="x",
                yaxis_scaleratio=1
            )

            if save_as:
                self._save_figure(fig, save_as, 'html')

            return fig

    def plot_frequency_spectrum(
        self,
        frequencies: np.ndarray,
        magnitudes: np.ndarray,
        title: str = "Frequency Spectrum",
        natural_freq: Optional[float] = None,
        xlim: Tuple[float, float] = (0, 30),
        save_as: Optional[str] = None
    ) -> Any:
        """
        Plot frequency spectrum with optional natural frequency marker.

        Args:
            frequencies: Frequency array (Hz)
            magnitudes: Magnitude array
            title: Plot title
            natural_freq: Natural frequency to highlight
            xlim: X-axis limits
            save_as: Filename to save

        Returns:
            Figure object
        """
        if self.backend == 'matplotlib':
            fig, ax = plt.subplots(figsize=self.config.figsize)

            ax.plot(frequencies, magnitudes, 'b-', linewidth=1)

            if natural_freq is not None:
                ax.axvline(natural_freq, color='r', linestyle='--',
                          label=f'Natural freq: {natural_freq:.2f} Hz')

            ax.set_xlabel('Frequency (Hz)', fontsize=self.config.label_fontsize)
            ax.set_ylabel('Magnitude', fontsize=self.config.label_fontsize)
            ax.set_title(title, fontsize=self.config.title_fontsize)
            ax.set_xlim(xlim)
            if natural_freq is not None:
                ax.legend()
            ax.grid(True, alpha=0.3)

            if save_as:
                self._save_figure(fig, save_as)

            return fig

        else:
            fig = go.Figure()

            fig.add_trace(go.Scatter(
                x=frequencies, y=magnitudes,
                mode='lines',
                name='Spectrum'
            ))

            if natural_freq is not None:
                fig.add_vline(x=natural_freq, line_dash='dash', line_color='red',
                             annotation_text=f'fn={natural_freq:.2f}Hz')

            fig.update_layout(
                title=title,
                xaxis_title='Frequency (Hz)',
                yaxis_title='Magnitude',
                xaxis_range=xlim
            )

            if save_as:
                self._save_figure(fig, save_as, 'html')

            return fig

    def plot_damper_curve(
        self,
        velocities: np.ndarray,
        forces: np.ndarray,
        corner: str = "",
        title: str = "Damper Force-Velocity Curve",
        save_as: Optional[str] = None
    ) -> Any:
        """
        Plot damper force-velocity curve.

        Args:
            velocities: Velocity array (mm/s)
            forces: Force array (N)
            corner: Corner name
            title: Plot title
            save_as: Filename to save

        Returns:
            Figure object
        """
        if self.backend == 'matplotlib':
            fig, ax = plt.subplots(figsize=self.config.figsize)

            color = self.config.corner_colors.get(corner, 'blue')
            ax.scatter(velocities, forces, alpha=0.1, s=1, c=color)

            # Bin and plot mean values
            vel_bins = np.linspace(-400, 400, 41)
            bin_centers = (vel_bins[:-1] + vel_bins[1:]) / 2
            mean_forces = np.zeros(len(bin_centers))

            for i in range(len(bin_centers)):
                mask = (velocities >= vel_bins[i]) & (velocities < vel_bins[i+1])
                if np.sum(mask) > 0:
                    mean_forces[i] = np.mean(forces[mask])

            ax.plot(bin_centers, mean_forces, 'r-', linewidth=2, label='Mean')

            ax.axhline(y=0, color='gray', linestyle='-', linewidth=0.5)
            ax.axvline(x=0, color='gray', linestyle='-', linewidth=0.5)

            ax.set_xlabel('Velocity (mm/s)', fontsize=self.config.label_fontsize)
            ax.set_ylabel('Force (N)', fontsize=self.config.label_fontsize)
            ax.set_title(f"{title} - {corner}", fontsize=self.config.title_fontsize)
            ax.legend()
            ax.grid(True, alpha=0.3)

            if save_as:
                self._save_figure(fig, save_as)

            return fig

        else:
            fig = go.Figure()

            fig.add_trace(go.Scatter(
                x=velocities, y=forces,
                mode='markers',
                marker=dict(size=2, opacity=0.1),
                name='Measurements'
            ))

            fig.update_layout(
                title=f"{title} - {corner}",
                xaxis_title='Velocity (mm/s)',
                yaxis_title='Force (N)'
            )

            if save_as:
                self._save_figure(fig, save_as, 'html')

            return fig

    def plot_velocity_histogram(
        self,
        velocities: np.ndarray,
        bins: Optional[np.ndarray] = None,
        corner: str = "",
        title: str = "Damper Velocity Distribution",
        save_as: Optional[str] = None
    ) -> Any:
        """
        Plot damper velocity histogram.

        Args:
            velocities: Velocity array (mm/s)
            bins: Histogram bin edges
            corner: Corner name
            title: Plot title
            save_as: Filename to save

        Returns:
            Figure object
        """
        if bins is None:
            bins = np.linspace(-500, 500, 51)

        if self.backend == 'matplotlib':
            fig, ax = plt.subplots(figsize=self.config.figsize)

            color = self.config.corner_colors.get(corner, 'blue')
            ax.hist(velocities, bins=bins, color=color, alpha=0.7, edgecolor='black')

            ax.set_xlabel('Velocity (mm/s)', fontsize=self.config.label_fontsize)
            ax.set_ylabel('Count', fontsize=self.config.label_fontsize)
            ax.set_title(f"{title} - {corner}", fontsize=self.config.title_fontsize)
            ax.grid(True, alpha=0.3, axis='y')

            if save_as:
                self._save_figure(fig, save_as)

            return fig

        else:
            fig = go.Figure()

            fig.add_trace(go.Histogram(
                x=velocities,
                xbins=dict(start=bins[0], end=bins[-1], size=bins[1]-bins[0]),
                name=corner
            ))

            fig.update_layout(
                title=f"{title} - {corner}",
                xaxis_title='Velocity (mm/s)',
                yaxis_title='Count'
            )

            if save_as:
                self._save_figure(fig, save_as, 'html')

            return fig

    def plot_track_map(
        self,
        latitudes: np.ndarray,
        longitudes: np.ndarray,
        values: np.ndarray,
        parameter: str = "speed",
        title: str = "Track Map",
        save_as: Optional[str] = None
    ) -> Any:
        """
        Plot GPS track with color-coded parameter.

        Args:
            latitudes: Latitude array
            longitudes: Longitude array
            values: Parameter values for coloring
            parameter: Parameter name
            title: Plot title
            save_as: Filename to save

        Returns:
            Figure object
        """
        if self.backend == 'matplotlib':
            fig, ax = plt.subplots(figsize=(12, 10))

            scatter = ax.scatter(
                longitudes, latitudes,
                c=values,
                cmap='RdYlGn' if parameter == 'speed' else 'coolwarm',
                s=2,
                alpha=0.8
            )

            cbar = plt.colorbar(scatter, ax=ax)
            cbar.set_label(parameter)

            ax.set_xlabel('Longitude', fontsize=self.config.label_fontsize)
            ax.set_ylabel('Latitude', fontsize=self.config.label_fontsize)
            ax.set_title(title, fontsize=self.config.title_fontsize)
            ax.set_aspect('equal')

            if save_as:
                self._save_figure(fig, save_as)

            return fig

        else:
            fig = go.Figure()

            fig.add_trace(go.Scattermapbox(
                lat=latitudes,
                lon=longitudes,
                mode='markers',
                marker=dict(
                    size=5,
                    color=values,
                    colorscale='RdYlGn' if parameter == 'speed' else 'RdBu',
                    showscale=True,
                    colorbar=dict(title=parameter)
                ),
                name=parameter
            ))

            fig.update_layout(
                title=title,
                mapbox=dict(
                    style='open-street-map',
                    center=dict(lat=np.mean(latitudes), lon=np.mean(longitudes)),
                    zoom=14
                )
            )

            if save_as:
                self._save_figure(fig, save_as, 'html')

            return fig

    def plot_corner_comparison(
        self,
        data: Dict[str, float],
        title: str = "Corner Comparison",
        ylabel: str = "Value",
        save_as: Optional[str] = None
    ) -> Any:
        """
        Plot bar chart comparing four corners.

        Args:
            data: Dictionary with corner names as keys
            title: Plot title
            ylabel: Y-axis label
            save_as: Filename to save

        Returns:
            Figure object
        """
        corners = ['front_left', 'front_right', 'rear_left', 'rear_right']
        values = [data.get(c, 0) for c in corners]
        colors = [self.config.corner_colors[c] for c in corners]
        labels = ['FL', 'FR', 'RL', 'RR']

        if self.backend == 'matplotlib':
            fig, ax = plt.subplots(figsize=(8, 6))

            bars = ax.bar(labels, values, color=colors)

            ax.set_ylabel(ylabel, fontsize=self.config.label_fontsize)
            ax.set_title(title, fontsize=self.config.title_fontsize)
            ax.grid(True, alpha=0.3, axis='y')

            # Add value labels on bars
            for bar, val in zip(bars, values):
                ax.text(bar.get_x() + bar.get_width()/2, bar.get_height(),
                       f'{val:.2f}', ha='center', va='bottom')

            if save_as:
                self._save_figure(fig, save_as)

            return fig

        else:
            fig = go.Figure()

            fig.add_trace(go.Bar(
                x=labels,
                y=values,
                marker_color=colors,
                text=[f'{v:.2f}' for v in values],
                textposition='outside'
            ))

            fig.update_layout(
                title=title,
                yaxis_title=ylabel
            )

            if save_as:
                self._save_figure(fig, save_as, 'html')

            return fig

    def create_dashboard_figure(
        self,
        data: Dict[str, Any],
        save_as: Optional[str] = None
    ) -> Any:
        """
        Create multi-panel dashboard figure.

        Args:
            data: Dictionary containing all analysis data
            save_as: Filename to save

        Returns:
            Figure object
        """
        if self.backend != 'matplotlib':
            logger.warning("Dashboard figure only available with matplotlib")
            return None

        fig = plt.figure(figsize=(16, 12))
        gs = GridSpec(3, 3, figure=fig)

        # Suspension travel
        ax1 = fig.add_subplot(gs[0, 0])
        if 'pot_fl' in data:
            time = np.arange(len(data['pot_fl'])) / 100
            ax1.plot(time, data['pot_fl'], label='FL')
            ax1.plot(time, data['pot_fr'], label='FR')
            ax1.plot(time, data['pot_rl'], label='RL')
            ax1.plot(time, data['pot_rr'], label='RR')
            ax1.set_xlabel('Time (s)')
            ax1.set_ylabel('Travel (mm)')
            ax1.set_title('Suspension Travel')
            ax1.legend(fontsize=8)
            ax1.grid(True, alpha=0.3)

        # G-G diagram
        ax2 = fig.add_subplot(gs[0, 1])
        if 'lateral_g' in data and 'longitudinal_g' in data:
            ax2.scatter(data['lateral_g'], data['longitudinal_g'], s=1, alpha=0.3)
            ax2.set_xlabel('Lateral G')
            ax2.set_ylabel('Longitudinal G')
            ax2.set_title('G-G Diagram')
            ax2.set_aspect('equal')
            ax2.grid(True, alpha=0.3)

        # Speed trace
        ax3 = fig.add_subplot(gs[0, 2])
        if 'speed_mph' in data:
            time = np.arange(len(data['speed_mph'])) / 100
            ax3.plot(time, data['speed_mph'])
            ax3.set_xlabel('Time (s)')
            ax3.set_ylabel('Speed (mph)')
            ax3.set_title('Speed')
            ax3.grid(True, alpha=0.3)

        # Body roll
        ax4 = fig.add_subplot(gs[1, 0])
        if 'body_roll_deg' in data:
            time = np.arange(len(data['body_roll_deg'])) / 100
            ax4.plot(time, data['body_roll_deg'])
            ax4.set_xlabel('Time (s)')
            ax4.set_ylabel('Roll (deg)')
            ax4.set_title('Body Roll')
            ax4.grid(True, alpha=0.3)

        # Corner weights
        ax5 = fig.add_subplot(gs[1, 1])
        if 'corner_weights' in data:
            cw = data['corner_weights']
            corners = ['FL', 'FR', 'RL', 'RR']
            weights = [cw.get('front_left', 0), cw.get('front_right', 0),
                      cw.get('rear_left', 0), cw.get('rear_right', 0)]
            colors = [self.config.corner_colors[c.lower().replace(' ', '_')]
                     for c in ['front_left', 'front_right', 'rear_left', 'rear_right']]
            ax5.bar(corners, weights, color=colors)
            ax5.set_ylabel('Weight (lbs)')
            ax5.set_title('Corner Weights')
            ax5.grid(True, alpha=0.3, axis='y')

        # Temperature
        ax6 = fig.add_subplot(gs[1, 2])
        if 'temp_shock_fl' in data:
            time = np.arange(len(data['temp_shock_fl'])) / 100
            ax6.plot(time, data['temp_shock_fl'], label='FL')
            ax6.plot(time, data['temp_shock_fr'], label='FR')
            ax6.plot(time, data['temp_shock_rl'], label='RL')
            ax6.plot(time, data['temp_shock_rr'], label='RR')
            ax6.set_xlabel('Time (s)')
            ax6.set_ylabel('Temp (°F)')
            ax6.set_title('Shock Temperatures')
            ax6.legend(fontsize=8)
            ax6.grid(True, alpha=0.3)

        plt.tight_layout()

        if save_as:
            self._save_figure(fig, save_as)

        return fig
