"""
test_suite.py
Pytest suite for Operation Midnight Cobra.

Run:
    pytest test_suite.py -v
"""

import math
import struct
import numpy as np
import pytest
from unittest.mock import MagicMock, patch


# ══════════════════════════════════════════════════════════════════════════════
#  multi_rat_tracker
# ══════════════════════════════════════════════════════════════════════════════

from multi_rat_tracker import (
    multi_rat_tracker,
    reset_tracker,
    _filter_valid_measurements,
    _wrap_angle_innovation,
    _build_state_transition_matrix,
    _build_process_noise_covariance,
    _hungarian_gnn_assignment,
    HITS_NEEDED_TO_CONFIRM,
    MISSES_BEFORE_DELETION,
    MAX_AIRSPACE_RANGE_M,
)


@pytest.fixture(autouse=True)
def fresh_tracker():
    """Reset global tracker state before every test."""
    reset_tracker()
    yield
    reset_tracker()


# ── filter ────────────────────────────────────────────────────────────────────

class TestFilterValidMeasurements:
    def test_empty_array_returns_empty(self):
        result = _filter_valid_measurements(np.empty((0, 3)))
        assert result.shape == (0, 3)

    def test_none_returns_empty(self):
        result = _filter_valid_measurements(None)
        assert result.shape == (0, 3)

    def test_zero_range_is_rejected(self):
        meas = np.array([[0.1, 0.1, 0.0]])
        assert _filter_valid_measurements(meas).shape[0] == 0

    def test_negative_range_is_rejected(self):
        meas = np.array([[0.1, 0.1, -5.0]])
        assert _filter_valid_measurements(meas).shape[0] == 0

    def test_range_beyond_airspace_is_rejected(self):
        meas = np.array([[0.1, 0.1, MAX_AIRSPACE_RANGE_M + 1.0]])
        assert _filter_valid_measurements(meas).shape[0] == 0

    def test_range_at_boundary_is_accepted(self):
        meas = np.array([[0.1, 0.1, MAX_AIRSPACE_RANGE_M]])
        assert _filter_valid_measurements(meas).shape[0] == 1

    def test_mixed_valid_and_invalid(self):
        meas = np.array([
            [0.1, 0.1, 20.0],    # valid
            [0.1, 0.1, 0.0],     # invalid — zero range
            [0.1, 0.1, 100.0],   # invalid — beyond airspace
            [0.5, 0.2, 45.0],    # valid
        ])
        result = _filter_valid_measurements(meas)
        assert result.shape[0] == 2


# ── angle wrapping ────────────────────────────────────────────────────────────

class TestWrapAngleInnovation:
    def test_small_innovation_unchanged(self):
        v = np.array([0.1, -0.2, 5.0])
        result = _wrap_angle_innovation(v)
        assert abs(result[0] - 0.1) < 1e-9
        assert abs(result[1] - (-0.2)) < 1e-9
        assert abs(result[2] - 5.0) < 1e-9  # range not wrapped

    def test_wraps_az_above_pi(self):
        v = np.array([math.pi + 0.1, 0.0, 0.0])
        result = _wrap_angle_innovation(v)
        assert result[0] < 0   # wrapped to negative side

    def test_wraps_el_below_negative_pi(self):
        v = np.array([0.0, -(math.pi + 0.1), 0.0])
        result = _wrap_angle_innovation(v)
        assert result[1] > 0

    def test_range_component_never_wrapped(self):
        v = np.array([0.0, 0.0, 200.0])
        result = _wrap_angle_innovation(v)
        assert result[2] == 200.0


# ── EKF matrices ──────────────────────────────────────────────────────────────

class TestBuildStateTransitionMatrix:
    def test_shape(self):
        F = _build_state_transition_matrix(0.1)
        assert F.shape == (6, 6)

    def test_identity_block(self):
        F = _build_state_transition_matrix(0.5)
        np.testing.assert_array_equal(F[3:, 3:], np.eye(3))

    def test_dt_entries(self):
        dt = 0.25
        F = _build_state_transition_matrix(dt)
        assert F[0, 3] == dt
        assert F[1, 4] == dt
        assert F[2, 5] == dt

    def test_zero_off_diagonal(self):
        F = _build_state_transition_matrix(0.1)
        assert F[0, 4] == 0.0
        assert F[1, 3] == 0.0


class TestBuildProcessNoiseCovariance:
    def test_shape(self):
        Q = _build_process_noise_covariance(0.1)
        assert Q.shape == (6, 6)

    def test_symmetric(self):
        Q = _build_process_noise_covariance(0.1)
        np.testing.assert_allclose(Q, Q.T, atol=1e-12)

    def test_positive_semidefinite(self):
        Q = _build_process_noise_covariance(0.1)
        eigenvalues = np.linalg.eigvalsh(Q)
        assert np.all(eigenvalues >= -1e-12)

    def test_scales_with_dt(self):
        Q_small = _build_process_noise_covariance(0.1)
        Q_large = _build_process_noise_covariance(1.0)
        # Larger dt → larger process noise
        assert np.trace(Q_large) > np.trace(Q_small)


# ── GNN assignment ────────────────────────────────────────────────────────────

class TestHungarianGnnAssignment:
    def test_empty_cost_matrix(self):
        result = _hungarian_gnn_assignment(np.empty((0, 0)))
        assert result == {}

    def test_no_tracks(self):
        cost = np.empty((0, 3))
        assert _hungarian_gnn_assignment(cost) == {}

    def test_no_measurements(self):
        cost = np.empty((2, 0))
        assert _hungarian_gnn_assignment(cost) == {}

    def test_all_gated_out(self):
        cost = np.full((2, 2), np.inf)
        result = _hungarian_gnn_assignment(cost)
        assert result == {}

    def test_single_clear_assignment(self):
        cost = np.array([[1.0, np.inf], [np.inf, 2.0]])
        result = _hungarian_gnn_assignment(cost)
        assert result[0] == 0
        assert result[1] == 1

    def test_prefers_lower_cost(self):
        cost = np.array([[0.5, 9.0], [9.0, 0.5]])
        result = _hungarian_gnn_assignment(cost)
        assert result[0] == 0
        assert result[1] == 1

    def test_one_to_one_constraint(self):
        # Two tracks competing for the same measurement
        cost = np.array([[1.0, np.inf], [0.5, np.inf]])
        result = _hungarian_gnn_assignment(cost)
        # Only one track can be assigned to meas 0
        assigned_meas = list(result.values())
        assert len(assigned_meas) == len(set(assigned_meas))


# ── end-to-end tracker behaviour ─────────────────────────────────────────────

class TestTrackerLifecycle:
    def _make_meas(self, az=0.3, el=0.1, rng=30.0):
        return np.array([[az, el, rng]])

    def test_no_confirmed_tracks_on_first_call(self):
        ids, pos = multi_rat_tracker(self._make_meas(), sample_time=0.5)
        assert len(ids) == 0

    def test_track_confirmed_after_required_hits(self):
        meas = self._make_meas()
        for _ in range(HITS_NEEDED_TO_CONFIRM):
            ids, pos = multi_rat_tracker(meas, sample_time=0.5)
        assert len(ids) == 1

    def test_confirmed_track_has_correct_shape(self):
        meas = self._make_meas()
        for _ in range(HITS_NEEDED_TO_CONFIRM):
            ids, pos = multi_rat_tracker(meas, sample_time=0.5)
        assert ids.dtype == np.int32
        assert pos.shape == (1, 3)

    def test_track_deleted_after_too_many_misses(self):
        meas = self._make_meas()
        for _ in range(HITS_NEEDED_TO_CONFIRM):
            multi_rat_tracker(meas, sample_time=0.5)
        # Now stop feeding measurements
        for _ in range(MISSES_BEFORE_DELETION + 1):
            ids, _ = multi_rat_tracker(np.empty((0, 3)), sample_time=0.5)
        assert len(ids) == 0

    def test_two_simultaneous_targets(self):
        meas = np.array([
            [0.3, 0.1, 30.0],
            [-0.5, 0.2, 55.0],
        ])
        for _ in range(HITS_NEEDED_TO_CONFIRM):
            ids, pos = multi_rat_tracker(meas, sample_time=0.5)
        assert len(ids) == 2
        assert len(set(ids.tolist())) == 2   # unique IDs

    def test_empty_measurements_no_crash(self):
        ids, pos = multi_rat_tracker(np.empty((0, 3)), sample_time=0.5)
        assert len(ids) == 0

    def test_out_of_range_measurement_ignored(self):
        meas = np.array([[0.3, 0.1, MAX_AIRSPACE_RANGE_M + 10.0]])
        for _ in range(HITS_NEEDED_TO_CONFIRM + 1):
            ids, _ = multi_rat_tracker(meas, sample_time=0.5)
        assert len(ids) == 0

    def test_reset_clears_all_tracks(self):
        meas = self._make_meas()
        for _ in range(HITS_NEEDED_TO_CONFIRM):
            multi_rat_tracker(meas, sample_time=0.5)
        ids, _ = multi_rat_tracker(np.empty((0, 3)), sample_time=0.5, reset=True)
        assert len(ids) == 0

    def test_track_ids_increment(self):
        meas_a = self._make_meas(az=0.3)
        meas_b = self._make_meas(az=-0.8, rng=50.0)
        for _ in range(HITS_NEEDED_TO_CONFIRM):
            multi_rat_tracker(meas_a, sample_time=0.5)
        for _ in range(HITS_NEEDED_TO_CONFIRM):
            ids, _ = multi_rat_tracker(meas_b, sample_time=0.5)
        assert ids[-1] > 1   # second track has a higher ID


# ══════════════════════════════════════════════════════════════════════════════
#  lidar_collect
# ══════════════════════════════════════════════════════════════════════════════

from lidar_collect import LidarMeasurement, LidarEngine, DETECTION_THRESHOLD_MM


class TestLidarMeasurement:
    def test_detected_true_within_threshold(self):
        m = LidarMeasurement(range_mm=DETECTION_THRESHOLD_MM - 1)
        assert m.detected is True

    def test_detected_false_at_threshold(self):
        m = LidarMeasurement(range_mm=DETECTION_THRESHOLD_MM)
        assert m.detected is False

    def test_detected_false_beyond_threshold(self):
        m = LidarMeasurement(range_mm=DETECTION_THRESHOLD_MM + 1000)
        assert m.detected is False

    def test_az_el_stored_correctly(self):
        m = LidarMeasurement(range_mm=5000, az_rad=1.2, el_rad=-0.3)
        assert m.az_rad == pytest.approx(1.2)
        assert m.el_rad == pytest.approx(-0.3)

    def test_repr_contains_key_fields(self):
        m = LidarMeasurement(range_mm=5000, az_rad=0.0, el_rad=0.0)
        r = repr(m)
        assert "5000" in r
        assert "detected" in r


@pytest.fixture
def mock_lidar_engine():
    with patch("lidar_collect.find_port", return_value="/dev/ttyUSB0"), \
         patch("lidar_collect.serial.Serial"):
        engine = LidarEngine()
    return engine


class TestLidarFormatting:
    def test_valid_response_parsed(self, mock_lidar_engine):
        m = mock_lidar_engine.lidar_formatting("ld,0:15.234")
        assert m is not None
        assert m.range == 15234

    def test_missing_colon_returns_none(self, mock_lidar_engine):
        assert mock_lidar_engine.lidar_formatting("garbage") is None

    def test_non_numeric_value_returns_none(self, mock_lidar_engine):
        assert mock_lidar_engine.lidar_formatting("ld,0:abc") is None

    def test_az_el_stamped_on_measurement(self, mock_lidar_engine):
        m = mock_lidar_engine.lidar_formatting("ld,0:10.0", az_rad=0.5, el_rad=0.1)
        assert m.az_rad == pytest.approx(0.5)
        assert m.el_rad == pytest.approx(0.1)

    def test_zero_range_parsed(self, mock_lidar_engine):
        m = mock_lidar_engine.lidar_formatting("ld,0:0.0")
        assert m is not None
        assert m.range == 0


class TestDrainBuffer:
    def test_drain_returns_all_items(self, mock_lidar_engine):
        m1 = LidarMeasurement(1000)
        m2 = LidarMeasurement(2000)
        mock_lidar_engine._buffer.extend([m1, m2])
        result = mock_lidar_engine.drain_buffer()
        assert len(result) == 2

    def test_drain_clears_buffer(self, mock_lidar_engine):
        mock_lidar_engine._buffer.append(LidarMeasurement(1000))
        mock_lidar_engine.drain_buffer()
        assert len(mock_lidar_engine._buffer) == 0

    def test_drain_empty_buffer_returns_empty_list(self, mock_lidar_engine):
        assert mock_lidar_engine.drain_buffer() == []


class TestDrainAsTrackerInput:
    def test_filters_out_non_detections(self, mock_lidar_engine):
        mock_lidar_engine._buffer.extend([
            LidarMeasurement(5000),                        # detected
            LidarMeasurement(DETECTION_THRESHOLD_MM + 1),  # not detected
        ])
        result = mock_lidar_engine.drain_as_tracker_input()
        assert result.shape[0] == 1

    def test_converts_mm_to_metres(self, mock_lidar_engine):
        mock_lidar_engine._buffer.append(LidarMeasurement(10_000))  # 10 m
        result = mock_lidar_engine.drain_as_tracker_input()
        assert result[0, 2] == pytest.approx(10.0)

    def test_empty_buffer_returns_empty_array(self, mock_lidar_engine):
        result = mock_lidar_engine.drain_as_tracker_input()
        assert result.shape == (0, 3)

    def test_output_shape_is_n_by_3(self, mock_lidar_engine):
        for rng in [5000, 10000, 15000]:
            mock_lidar_engine._buffer.append(LidarMeasurement(rng))
        result = mock_lidar_engine.drain_as_tracker_input()
        assert result.shape == (3, 3)


# ══════════════════════════════════════════════════════════════════════════════
#  rf_receiver
# ══════════════════════════════════════════════════════════════════════════════

from rf_receiver import rf_reading, unpack_rf_reading, PACKET_SIZE


class TestRfReading:
    def test_fields_accessible(self):
        r = rf_reading(
            rf_detection=True,
            rf_strength=-72,
            rf_position_az=0.5,
            rf_position_el=0.1,
        )
        assert r.rf_detection is True
        assert r.rf_strength == -72
        assert r.rf_position_az == pytest.approx(0.5)
        assert r.rf_position_el == pytest.approx(0.1)


class TestUnpackRfReading:
    def _make_packet(self, detected=1, strength=-72, az=0.5, el=0.1):
        return struct.pack("<Biff", detected, strength, az, el)

    def test_detected_true(self):
        r = unpack_rf_reading(self._make_packet(detected=1))
        assert r.rf_detection is True

    def test_detected_false(self):
        r = unpack_rf_reading(self._make_packet(detected=0))
        assert r.rf_detection is False

    def test_strength_roundtrip(self):
        r = unpack_rf_reading(self._make_packet(strength=-95))
        assert r.rf_strength == -95

    def test_az_roundtrip(self):
        r = unpack_rf_reading(self._make_packet(az=1.234))
        assert r.rf_position_az == pytest.approx(1.234, abs=1e-5)

    def test_el_roundtrip(self):
        r = unpack_rf_reading(self._make_packet(el=-0.456))
        assert r.rf_position_el == pytest.approx(-0.456, abs=1e-5)

    def test_packet_size_constant_matches_struct(self):
        assert len(self._make_packet()) == PACKET_SIZE


# ══════════════════════════════════════════════════════════════════════════════
#  pi_to_arduino
# ══════════════════════════════════════════════════════════════════════════════

from pi_to_arduino import pack_measurement


class TestPackMeasurement:
    def test_packet_is_13_bytes(self):
        m = LidarMeasurement(5000, az_rad=0.5, el_rad=0.2)
        assert len(pack_measurement(m)) == 13

    def test_az_roundtrip(self):
        m = LidarMeasurement(5000, az_rad=1.23, el_rad=0.0)
        az, _, _, _ = struct.unpack("<iiiB", pack_measurement(m))
        assert az == round(math.degrees(1.23) * 1000)

    def test_el_roundtrip(self):
        m = LidarMeasurement(5000, az_rad=0.0, el_rad=-0.45)
        _, el, _, _ = struct.unpack("<iiiB", pack_measurement(m))
        assert el == round(math.degrees(-0.45) * 1000)

    def test_range_encoded_in_millimetres(self):
        m = LidarMeasurement(range_mm=25_000)   # 25 m
        _, _, rng, _ = struct.unpack("<iiiB", pack_measurement(m))
        assert rng == 25_000

    def test_detected_true_encodes_as_1(self):
        m = LidarMeasurement(range_mm=5000)     # within threshold → detected
        _, _, _, det = struct.unpack("<fffB", pack_measurement(m))
        assert det == 1

    def test_detected_false_encodes_as_0(self):
        m = LidarMeasurement(range_mm=DETECTION_THRESHOLD_MM + 1)
        _, _, _, det = struct.unpack("<fffB", pack_measurement(m))
        assert det == 0
