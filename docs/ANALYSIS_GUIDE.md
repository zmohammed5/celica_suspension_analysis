# Analysis Guide

This guide explains how to interpret the analysis outputs from the suspension DAQ system.

## Damper Analysis

### Force-Velocity Curve
The force-velocity (F-V) curve is the fundamental characterization of a damper.

**Reading the curve:**
- X-axis: Damper velocity (mm/s)
  - Positive = Compression (bump)
  - Negative = Rebound (extension)
- Y-axis: Damper force (N)
  - Positive = Resisting compression
  - Negative = Resisting extension

**What to look for:**
- **Symmetry**: Compression and rebound should be balanced
- **Linearity**: Ideal dampers are roughly linear
- **Knee point**: Where digressive valving takes effect
- **Hysteresis**: Loop indicates friction in the system

### Velocity Histogram
Shows time spent at each damper velocity.

**Interpretation:**
- Peak near zero: Most time spent at low velocities (normal)
- Spread distribution: Active road surface
- Spikes at extremes: Hitting bumps or curbs

**Tuning implications:**
- If most time is spent at low velocities, prioritize low-speed damping
- High-velocity events (jumps, big bumps) need adequate high-speed damping

### Temperature Fade
Damper performance degrades with heat.

**Indicators:**
- Force reduction percentage per °F
- Temperature at which fade begins
- Recovery rate when cooling

**Acceptable values:**
- < 0.5%/°F: Excellent
- 0.5-1%/°F: Good
- > 1%/°F: Consider fluid upgrade

## Ride Analysis

### Natural Frequency
The frequency at which the suspension naturally oscillates.

**Typical values:**
- Front: 1.0-1.3 Hz (street), 1.5-2.0 Hz (sport), 2.0-3.0 Hz (race)
- Rear: 1.2-1.5 Hz (street), 1.8-2.2 Hz (sport), 2.2-3.5 Hz (race)

**Relationship to spring rate:**
```
f = (1/2π) × √(k/m)
where k = wheel rate, m = sprung mass at corner
```

### Damping Ratio
Measure of how quickly oscillations decay.

**Interpretation:**
| Value | Description | Characteristic |
|-------|-------------|----------------|
| 0.15-0.25 | Underdamped | Floaty, multiple oscillations |
| 0.25-0.35 | Optimal | One small overshoot, quick settle |
| 0.35-0.5 | Firm | No overshoot, feels stiff |
| > 0.5 | Overdamped | Harsh, poor bump absorption |

### ISO 2631 Comfort Rating
Weighted RMS acceleration per ISO 2631-1 standard.

**Comfort scale:**
| RMS (m/s²) | Rating |
|------------|--------|
| < 0.315 | Not uncomfortable |
| 0.315-0.5 | A little uncomfortable |
| 0.5-0.8 | Fairly uncomfortable |
| 0.8-1.25 | Uncomfortable |
| 1.25-2.0 | Very uncomfortable |
| > 2.0 | Extremely uncomfortable |

### Harshness Events
High-frequency, high-amplitude impulses.

**Causes:**
- Bump stop contact
- Pothole impacts
- Curb strikes
- Excessive unsprung mass

## Handling Analysis

### Roll Gradient
Degrees of body roll per g of lateral acceleration.

**Typical values:**
- Street car: 4-6 deg/g
- Sport suspension: 2-4 deg/g
- Race car: 1-2 deg/g

**Effect on handling:**
- Lower roll gradient = more responsive turn-in
- Higher roll gradient = more forgiving at limit
- Front vs rear distribution affects balance

### Pitch Gradient
Degrees of pitch per g of longitudinal acceleration.

**Typical values:**
- Street car: 2-4 deg/g
- Sport suspension: 1-2 deg/g
- Race car: 0.5-1 deg/g

**Anti-dive/Anti-squat:**
- Anti-dive: Reduces nose dive during braking
- Anti-squat: Reduces rear squat during acceleration

### Roll Couple Distribution
Percentage of total roll stiffness at front vs rear.

**Effect on balance:**
- More front roll stiffness → More understeer
- More rear roll stiffness → More oversteer

**Typical distributions:**
- Neutral: 50% front / 50% rear
- Slight understeer: 55% front / 45% rear
- FWD compensation: 45% front / 55% rear

### Understeer Gradient
Rate of change of steering angle with lateral acceleration.

**Interpretation:**
- Positive: Understeer (need more steering as speed increases)
- Zero: Neutral
- Negative: Oversteer (need less steering, or counter-steer)

**Typical values:**
- Street car: 2-4 deg/g (slight understeer for safety)
- Balanced sport car: 0-2 deg/g
- Oversteer-prone: < 0 deg/g

### Transient Response

**Time constant:**
Time to reach 63% of final roll angle after step input.
- Fast: < 0.15s (responsive)
- Moderate: 0.15-0.25s (balanced)
- Slow: > 0.25s (lazy)

**Overshoot:**
Amount roll exceeds steady-state before settling.
- < 10%: Well-damped
- 10-20%: Acceptable
- > 20%: Needs more roll damping

## Corner Analysis

### Static Corner Weights
Weight distribution at each corner when stationary.

**Ideal distribution:**
- Equal left-right weight (50% each side)
- Front-rear per design (typically 55-60% front for FWD)

**Cross-weight (wedge):**
```
Cross% = (FL + RR) / Total × 100
```
- 50% = Balanced
- > 50% = Looser entry, tighter exit
- < 50% = Tighter entry, looser exit

### Dynamic Load Transfer
Weight movement during acceleration/cornering.

**Lateral transfer:**
```
ΔW = (W × ay × h) / t
where W = weight, ay = lateral g, h = CG height, t = track width
```

**Longitudinal transfer:**
```
ΔW = (W × ax × h) / L
where L = wheelbase
```

### Tire Load Variation
Coefficient of variation (CV) of tire load.

**Interpretation:**
- Low CV (< 20%): Consistent grip, predictable handling
- High CV (> 30%): Inconsistent grip, difficult to drive at limit

## Track Analysis

### Lap Time Breakdown
Identify where time is gained or lost.

**Analysis approach:**
1. Compare to best lap
2. Find sectors with biggest differences
3. Analyze corner speeds, braking points

### G-G Diagram
Shows how much of the tire's grip is being used.

**Reading the diagram:**
- Center: No acceleration (coasting)
- Top: Pure acceleration
- Bottom: Pure braking
- Sides: Pure cornering
- Corners: Combined (trail braking, power-on exit)

**What to look for:**
- Utilization: Are you reaching the edges?
- Shape: Oval = not using combined, Round = using combined
- Consistency: Scattered = inconsistent, Tight = consistent

### Braking Zone Analysis
Evaluate braking efficiency.

**Metrics:**
- Peak braking g
- Braking distance
- Speed reduction
- Trail braking technique

### Corner Analysis
Evaluate corner approach.

**Metrics:**
- Entry speed
- Apex speed
- Exit speed
- Minimum speed
- Maximum lateral g

**What good technique looks like:**
- Smooth speed scrub before apex
- Minimum speed at or slightly past apex
- Acceleration beginning before apex exit
