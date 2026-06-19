"""
multi_rat_tracker.py
Multi-target EKF tracker – state and measurements stay in spherical coordinates.

State vector  : [az, el, range, az_rate, el_rate, range_rate]
Measurements  : [az_rad, el_rad, range_m]   (LiDAR default; pass sensor_R for radar)

Association   : Global Nearest Neighbour (GNN) via Hungarian algorithm
Gating        : Mahalanobis distance, chi-squared 99 % threshold (3 DOF)
Track lifecycle: confirmed after HITS_NEEDED_TO_CONFIRM consecutive detections;
                 deleted after MISSES_BEFORE_DELETION consecutive misses.
"""

import numpy as np
from dataclasses import dataclass
from scipy.optimize import linear_sum_assignment


# ─── Physical constraints ─────────────────────────────────────────────────────
MAX_DRONE_SPEED_MPS   = 5.0   # m/s  – bounds initial velocity uncertainty
MAX_AIRSPACE_RANGE_M  = 90.0  # m    – detections beyond this are discarded

# ─── Process noise (how aggressively can the drone manoeuvre?) ────────────────
ANGULAR_ACCEL_NOISE_RADPS2 = 0.08   # rad/s²  for both az and el axes
RANGE_ACCEL_NOISE_MPS2     = 1.5    # m/s²    radial axis

# ─── LiDAR sensor noise ───────────────────────────────────────────────────────
LIDAR_AZ_NOISE_RAD    = 0.015   # rad
LIDAR_EL_NOISE_RAD    = 0.015   # rad
LIDAR_RANGE_NOISE_M   = 0.05    # m

# ─── Association gate ─────────────────────────────────────────────────────────
GATE_CHI2_99PCT_3DOF  = 11.34   # chi-squared 99 %, 3 degrees of freedom

# ─── Track lifecycle ──────────────────────────────────────────────────────────
HITS_NEEDED_TO_CONFIRM = 3
MISSES_BEFORE_DELETION = 4

# ─── Default sensor noise covariance (override with radar R when needed) ──────
R_LIDAR = np.diag([
    LIDAR_AZ_NOISE_RAD  ** 2,
    LIDAR_EL_NOISE_RAD  ** 2,
    LIDAR_RANGE_NOISE_M ** 2,
])


# ─── Track data class ─────────────────────────────────────────────────────────
@dataclass
class Track:
    track_id:   int
    state:      np.ndarray   # (6,) [az, el, range, az_rate, el_rate, range_rate]
    covariance: np.ndarray   # (6, 6)
    age:        int  = 1
    hit_count:  int  = 1
    miss_count: int  = 0
    confirmed:  bool = False


# ─── Persistent tracker state ─────────────────────────────────────────────────
_active_tracks: list[Track] = []
_next_track_id: int         = 1


def reset_tracker() -> None:
    global _active_tracks, _next_track_id
    _active_tracks = []
    _next_track_id = 1


# ─── Main entry point ─────────────────────────────────────────────────────────
def multi_rat_tracker(
    measurements: np.ndarray,            # (M, 3) [az_rad, el_rad, range_m]; M may be 0
    sample_time:  float,                 # seconds between calls
    reset:        bool       = False,
    sensor_R:     np.ndarray = R_LIDAR,  # swap in radar noise covariance here
) -> tuple[np.ndarray, np.ndarray]:
    """
    Run one tracker cycle.

    Returns
    -------
    confirmed_track_ids       : (N,)    int32   unique IDs of confirmed tracks
    confirmed_track_positions : (N, 3)  float64 [az, el, range] of each confirmed track
    """
    global _active_tracks, _next_track_id

    if reset:
        reset_tracker()

    measurements = _filter_valid_measurements(measurements)

    state_transition_matrix = _build_state_transition_matrix(sample_time)
    process_noise_covariance = _build_process_noise_covariance(sample_time)

    # Measurement matrix H: [az, el, range] are the first 3 elements of the state
    observation_matrix = np.hstack([np.eye(3), np.zeros((3, 3))])

    # ── Step 1: Predict each track forward by one time step ───────────────────
    for track in _active_tracks:
        track.state      = state_transition_matrix @ track.state
        track.covariance = (
            state_transition_matrix @ track.covariance @ state_transition_matrix.T
            + process_noise_covariance
        )

    # ── Step 2: Compute per-track innovation stats; gate measurement candidates ─
    n_tracks = len(_active_tracks)
    n_meas   = measurements.shape[0]

    gating_cost_matrix    = np.full((n_tracks, n_meas), np.inf)
    innovation_cache      = {}   # (track_idx, meas_idx) -> (3,) innovation vector
    innov_covariance_list = []   # one (3,3) S per track
    kalman_gain_list      = []   # one (6,3) K per track

    for track_idx, track in enumerate(_active_tracks):
        # Innovation covariance: S = H P H' + R
        innov_cov = observation_matrix @ track.covariance @ observation_matrix.T + sensor_R
        S_inv     = np.linalg.inv(innov_cov)                        # compute once, reuse below
        kalman_gain = track.covariance @ observation_matrix.T @ S_inv
        innov_covariance_list.append(innov_cov)
        kalman_gain_list.append(kalman_gain)

        predicted_spherical_pos = track.state[:3]

        for meas_idx in range(n_meas):
            innovation    = _wrap_angle_innovation(measurements[meas_idx] - predicted_spherical_pos)
            mahal_dist_sq = float(innovation @ S_inv @ innovation)

            if mahal_dist_sq <= GATE_CHI2_99PCT_3DOF:
                gating_cost_matrix[track_idx, meas_idx] = mahal_dist_sq
                innovation_cache[track_idx, meas_idx]   = innovation

    # ── Step 3: GNN – globally optimal assignment via Hungarian algorithm ──────
    track_to_meas_assignment = _hungarian_gnn_assignment(gating_cost_matrix)

    # ── Step 4: EKF update for each assigned track ────────────────────────────
    for track_idx, track in enumerate(_active_tracks):
        assigned_meas_idx = track_to_meas_assignment.get(track_idx)

        if assigned_meas_idx is not None:
            kalman_gain = kalman_gain_list[track_idx]
            innovation  = innovation_cache[track_idx, assigned_meas_idx]
            I_minus_KH  = np.eye(6) - kalman_gain @ observation_matrix

            track.state      = track.state + kalman_gain @ innovation
            # Joseph form: (I-KH) P (I-KH)' + K R K'  — numerically stable
            track.covariance = (
                I_minus_KH @ track.covariance @ I_minus_KH.T
                + kalman_gain @ sensor_R @ kalman_gain.T
            )
            track.miss_count  = 0
            track.hit_count  += 1
            track.age        += 1
            if track.hit_count >= HITS_NEEDED_TO_CONFIRM:
                track.confirmed = True
        else:
            track.miss_count += 1
            track.hit_count   = 0

    # ── Step 5: Delete tracks that have gone missing too many times ───────────
    _active_tracks = [t for t in _active_tracks if t.miss_count <= MISSES_BEFORE_DELETION]

    # ── Step 6: Spawn tentative tracks for unmatched measurements ─────────────
    matched_meas_indices   = {j for j in track_to_meas_assignment.values() if j is not None}
    unmatched_meas_indices = [j for j in range(n_meas) if j not in matched_meas_indices]

    for j in unmatched_meas_indices:
        az, el, rng = measurements[j]

        # Angular rate bound: fastest drone at this range
        max_angular_rate_radps = MAX_DRONE_SPEED_MPS / max(rng, 1.0)

        initial_state = np.array([az, el, rng, 0.0, 0.0, 0.0])

        initial_covariance = np.diag([
            LIDAR_AZ_NOISE_RAD  ** 2,            # az position — from sensor spec
            LIDAR_EL_NOISE_RAD  ** 2,             # el position — from sensor spec
            LIDAR_RANGE_NOISE_M ** 2,             # range      — from sensor spec
            (max_angular_rate_radps / 2) ** 2,    # az rate    — physically bounded
            (max_angular_rate_radps / 2) ** 2,    # el rate    — physically bounded
            (MAX_DRONE_SPEED_MPS   / 3) ** 2,     # range rate — 3-sigma = max speed
        ])

        _active_tracks.append(Track(
            track_id   = _next_track_id,
            state      = initial_state,
            covariance = initial_covariance,
        ))
        _next_track_id += 1

    # ── Step 7: Collect and return confirmed tracks only ──────────────────────
    confirmed_tracks = [t for t in _active_tracks if t.confirmed]

    if confirmed_tracks:
        confirmed_track_ids       = np.array([t.track_id  for t in confirmed_tracks], dtype=np.int32)
        confirmed_track_positions = np.array([t.state[:3] for t in confirmed_tracks], dtype=np.float64)
    else:
        confirmed_track_ids       = np.empty(0, dtype=np.int32)
        confirmed_track_positions = np.empty((0, 3), dtype=np.float64)

    return confirmed_track_ids, confirmed_track_positions


# ─── EKF matrix builders ──────────────────────────────────────────────────────

def _build_state_transition_matrix(sample_time: float) -> np.ndarray:
    """Constant-velocity model in spherical coordinates."""
    F      = np.eye(6)
    F[0, 3] = sample_time   # az    += az_rate    * dt
    F[1, 4] = sample_time   # el    += el_rate    * dt
    F[2, 5] = sample_time   # range += range_rate * dt
    return F


def _build_process_noise_covariance(sample_time: float) -> np.ndarray:
    """
    Discrete white-noise acceleration model.
    Gamma maps independent az, el, and range acceleration inputs
    onto the 6-element spherical state vector.
    """
    dt = sample_time

    # Each column of Gamma drives one acceleration channel into [pos, vel]
    Gamma = np.array([
        [dt**2 / 2, 0,          0         ],   # az position
        [0,          dt**2 / 2, 0         ],   # el position
        [0,          0,          dt**2 / 2],   # range position
        [dt,         0,          0         ],  # az rate
        [0,          dt,         0         ],  # el rate
        [0,          0,          dt        ],  # range rate
    ])

    accel_noise_variances = np.diag([
        ANGULAR_ACCEL_NOISE_RADPS2 ** 2,
        ANGULAR_ACCEL_NOISE_RADPS2 ** 2,
        RANGE_ACCEL_NOISE_MPS2     ** 2,
    ])

    return Gamma @ accel_noise_variances @ Gamma.T


# ─── Measurement utilities ────────────────────────────────────────────────────

def _wrap_angle_innovation(innovation: np.ndarray) -> np.ndarray:
    """Wrap az and el components of an innovation vector to (-pi, pi]."""
    wrapped    = innovation.copy()
    wrapped[0] = (wrapped[0] + np.pi) % (2 * np.pi) - np.pi   # az
    wrapped[1] = (wrapped[1] + np.pi) % (2 * np.pi) - np.pi   # el
    return wrapped


def _filter_valid_measurements(measurements: np.ndarray) -> np.ndarray:
    """Drop any detections with zero or out-of-airspace range."""
    if measurements is None or np.asarray(measurements).size == 0:
        return np.empty((0, 3))
    measurements = np.atleast_2d(measurements)
    in_range = (measurements[:, 2] > 0) & (measurements[:, 2] <= MAX_AIRSPACE_RANGE_M)
    return measurements[in_range]


# ─── GNN assignment ───────────────────────────────────────────────────────────

def _hungarian_gnn_assignment(gating_cost_matrix: np.ndarray) -> dict[int, int]:
    """
    Find the globally optimal one-to-one track-to-measurement assignment
    that minimises total gated Mahalanobis cost.

    gating_cost_matrix[i, j] = Mahalanobis distance² if the pair passed gating,
                                np.inf                 if gated out (infeasible).

    Returns a dict: track_index -> measurement_index for each valid assignment.
    """
    n_tracks, n_meas = gating_cost_matrix.shape
    if n_tracks == 0 or n_meas == 0:
        return {}

    # Replace inf sentinels with a large finite value for the solver,
    # then reject any assignments that landed on a sentinel afterwards.
    INFEASIBLE_SENTINEL = 1e9
    solver_input = np.where(np.isinf(gating_cost_matrix), INFEASIBLE_SENTINEL, gating_cost_matrix)

    row_indices, col_indices = linear_sum_assignment(solver_input)

    assignment = {}
    for track_idx, meas_idx in zip(row_indices, col_indices):
        if not np.isinf(gating_cost_matrix[track_idx, meas_idx]):
            assignment[int(track_idx)] = int(meas_idx)

    return assignment
