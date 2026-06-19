"""
test_tracker.py  –  smoke-test for multi_rat_tracker.py

Simulates three drones in spherical coordinates, injects Gaussian noise,
feeds measurements into the tracker, and prints a step-by-step summary.

Run:
    python test_tracker.py
"""

import numpy as np
from multi_rat_tracker import multi_rat_tracker, reset_tracker, MAX_DRONE_SPEED_MPS


# ─── Reproducible noise ───────────────────────────────────────────────────────
rng_seed = np.random.default_rng(42)

# ─── Simulation parameters ────────────────────────────────────────────────────
DT          = 0.5    # seconds per step
N_STEPS     = 30     # total time steps  (~15 seconds)
DRONE_3_APPEARS_AT_STEP = 8   # drone 3 enters mid-scenario

# ─── Sensor noise (must match tracker's R_LIDAR assumptions) ──────────────────
MEAS_NOISE_AZ_RAD   = 0.015
MEAS_NOISE_EL_RAD   = 0.015
MEAS_NOISE_RANGE_M  = 0.05

# ─── Ground-truth drone definitions ───────────────────────────────────────────
# Each drone: [az_start, el_start, range_start, az_rate, el_rate, range_rate]
#   – units:  rad, rad, m, rad/s, rad/s, m/s
DRONE_TRUE_STATES = {
    "Drone-A": np.array([0.50,  0.20, 35.0,  0.04,  0.01, -0.8]),  # closing, banking right
    "Drone-B": np.array([-0.80, 0.30, 55.0, -0.02, -0.005, 0.5]),  # receding slowly
    "Drone-C": np.array([1.20,  0.10, 20.0,  0.06,  0.02,  1.2]),  # appears at step 8
}


# ─── Helper: propagate true state one step ────────────────────────────────────
def propagate_true_state(state: np.ndarray, dt: float) -> np.ndarray:
    """Constant-velocity step in spherical coordinates (no process noise)."""
    az, el, rng, az_rate, el_rate, rng_rate = state
    return np.array([
        az  + az_rate  * dt,
        el  + el_rate  * dt,
        rng + rng_rate * dt,
        az_rate,
        el_rate,
        rng_rate,
    ])


def add_sensor_noise(true_pos: np.ndarray) -> np.ndarray:
    """Add realistic LiDAR measurement noise to a [az, el, range] vector."""
    noise = rng_seed.normal(0, [MEAS_NOISE_AZ_RAD, MEAS_NOISE_EL_RAD, MEAS_NOISE_RANGE_M])
    return true_pos + noise


# ─── Run the simulation ───────────────────────────────────────────────────────
def run_simulation():
    reset_tracker()

    # Mutable truth table
    true_states = {name: state.copy() for name, state in DRONE_TRUE_STATES.items()}

    print("=" * 70)
    print(f"  Operation Midnight Cobra – Tracker Test")
    print(f"  {N_STEPS} steps × {DT}s = {N_STEPS * DT:.1f}s scenario")
    print(f"  3 drones (Drone-C appears at step {DRONE_3_APPEARS_AT_STEP})")
    print("=" * 70)

    for step in range(N_STEPS):
        time_sec = step * DT

        # ── Build measurement list for this step ──────────────────────────────
        raw_measurements = []
        active_drones    = []

        for name, state in true_states.items():
            if name == "Drone-C" and step < DRONE_3_APPEARS_AT_STEP:
                continue  # not yet in airspace

            noisy_meas = add_sensor_noise(state[:3])

            # Sanity-check: skip if noise pushed range out of airspace bounds
            if 0 < noisy_meas[2] <= 90.0:
                raw_measurements.append(noisy_meas)
                active_drones.append(name)

        measurements = (
            np.array(raw_measurements) if raw_measurements else np.empty((0, 3))
        )

        # ── Run tracker ───────────────────────────────────────────────────────
        confirmed_ids, confirmed_positions = multi_rat_tracker(
            measurements, DT
        )

        # ── Print step summary ────────────────────────────────────────────────
        drone_label = ", ".join(active_drones) if active_drones else "none"
        print(f"\nStep {step+1:02d}  t={time_sec:.1f}s   visible drones: [{drone_label}]"
              f"   measurements: {len(raw_measurements)}")

        if len(confirmed_ids) == 0:
            print("    [no confirmed tracks yet]")
        else:
            for tid, pos in zip(confirmed_ids, confirmed_positions):
                az_deg  = np.degrees(pos[0])
                el_deg  = np.degrees(pos[1])
                rng_m   = pos[2]
                print(f"    Track {tid:02d}  az={az_deg:+7.2f}°  el={el_deg:+6.2f}°  range={rng_m:6.2f}m")

        # ── Propagate true states forward ─────────────────────────────────────
        for name in true_states:
            true_states[name] = propagate_true_state(true_states[name], DT)

    # ── Final comparison: tracker estimate vs. ground truth ───────────────────
    print("\n" + "=" * 70)
    print("  Final-step ground truth vs. tracker estimates")
    print("=" * 70)

    if len(confirmed_ids) == 0:
        print("  No confirmed tracks at end of simulation.")
    else:
        # Show true positions at the last step (already propagated one extra step above,
        # so step back one for a fair comparison)
        for name, state in true_states.items():
            if name == "Drone-C" and N_STEPS <= DRONE_3_APPEARS_AT_STEP:
                continue
            true_az_deg  = np.degrees(state[0] - state[3] * DT)
            true_el_deg  = np.degrees(state[1] - state[4] * DT)
            true_rng     = state[2] - state[5] * DT
            print(f"  {name} true   az={true_az_deg:+7.2f}°  el={true_el_deg:+6.2f}°  range={true_rng:6.2f}m")

        print()
        for tid, pos in zip(confirmed_ids, confirmed_positions):
            az_deg = np.degrees(pos[0])
            el_deg = np.degrees(pos[1])
            print(f"  Track {tid:02d} est   az={az_deg:+7.2f}°  el={el_deg:+6.2f}°  range={pos[2]:6.2f}m")

    print("\n  Done.\n")


if __name__ == "__main__":
    run_simulation()
