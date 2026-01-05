#!/usr/bin/env python3
"""
Simple runner script for suspension analysis.
Handles Python path setup correctly.
"""

import sys
import os
from pathlib import Path

# Add src to Python path
project_root = Path(__file__).parent
sys.path.insert(0, str(project_root / 'src'))

# Now import and run
from main import run_complete_analysis

if __name__ == "__main__":
    config_path = str(project_root / 'config' / 'vehicle_templates' / '1992_celica_gt.json')
    output_dir = str(project_root / 'data' / 'output' / 'analysis_results')

    run_complete_analysis(config_path, output_dir)
