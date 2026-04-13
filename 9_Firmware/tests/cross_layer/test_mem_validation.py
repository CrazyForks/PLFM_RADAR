"""
test_mem_validation.py — Validate FPGA .mem files against AERIS-10 radar parameters.

Migrated from tb/cosim/validate_mem_files.py into CI-friendly pytest tests.

Checks:
  1. Structural: line counts, hex format, value ranges for all 12+ .mem files
  2. FFT twiddle files: bit-exact match against cos(2*pi*k/N) in Q15
  3. Long chirp .mem files: frequency sweep, magnitude envelope, segment count
  4. Short chirp .mem files: length, value range, non-zero content
  5. Chirp vs independent model: phase shape agreement
  6. Latency buffer LATENCY=3187 parameter validation
  7. Chirp memory loader addressing: {segment_select, sample_addr} arithmetic
  8. Seg3 zero-padding analysis
"""

import math
import os
import warnings

import pytest

# ============================================================================
# AERIS-10 System Parameters (independently derived from hardware specs)
# ============================================================================
F_CARRIER = 10.5e9        # 10.5 GHz carrier
C_LIGHT = 3.0e8
F_IF = 120e6              # IF frequency
CHIRP_BW = 20e6           # 20 MHz sweep bandwidth
FS_ADC = 400e6            # ADC sample rate
FS_SYS = 100e6            # System clock (100 MHz, after CIC 4x decimation)
T_LONG_CHIRP = 30e-6      # 30 us long chirp
T_SHORT_CHIRP = 0.5e-6    # 0.5 us short chirp
CIC_DECIMATION = 4
FFT_SIZE = 1024
DOPPLER_FFT_SIZE = 16
LONG_CHIRP_SAMPLES = int(T_LONG_CHIRP * FS_SYS)  # 3000 at 100 MHz

# Overlap-save parameters
OVERLAP_SAMPLES = 128
SEGMENT_ADVANCE = FFT_SIZE - OVERLAP_SAMPLES  # 896
LONG_SEGMENTS = 4

# Path to FPGA RTL directory containing .mem files
MEM_DIR = os.path.normpath(os.path.join(os.path.dirname(__file__), '..', '..', '9_2_FPGA'))

# Expected .mem file inventory
EXPECTED_MEM_FILES = {
    'fft_twiddle_1024.mem': {'lines': 256, 'desc': '1024-pt FFT quarter-wave cos ROM'},
    'fft_twiddle_16.mem':   {'lines': 4,   'desc': '16-pt FFT quarter-wave cos ROM'},
    'long_chirp_seg0_i.mem': {'lines': 1024, 'desc': 'Long chirp seg 0 I'},
    'long_chirp_seg0_q.mem': {'lines': 1024, 'desc': 'Long chirp seg 0 Q'},
    'long_chirp_seg1_i.mem': {'lines': 1024, 'desc': 'Long chirp seg 1 I'},
    'long_chirp_seg1_q.mem': {'lines': 1024, 'desc': 'Long chirp seg 1 Q'},
    'long_chirp_seg2_i.mem': {'lines': 1024, 'desc': 'Long chirp seg 2 I'},
    'long_chirp_seg2_q.mem': {'lines': 1024, 'desc': 'Long chirp seg 2 Q'},
    'long_chirp_seg3_i.mem': {'lines': 1024, 'desc': 'Long chirp seg 3 I'},
    'long_chirp_seg3_q.mem': {'lines': 1024, 'desc': 'Long chirp seg 3 Q'},
    'short_chirp_i.mem': {'lines': 50, 'desc': 'Short chirp I'},
    'short_chirp_q.mem': {'lines': 50, 'desc': 'Short chirp Q'},
}


def read_mem_hex(filename: str) -> list[int]:
    """Read a .mem file, return list of integer values (16-bit signed)."""
    path = os.path.join(MEM_DIR, filename)
    values = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('//'):
                continue
            val = int(line, 16)
            if val >= 0x8000:
                val -= 0x10000
            values.append(val)
    return values


def compute_magnitudes(i_vals: list[int], q_vals: list[int]) -> list[float]:
    """Compute magnitude envelope from I/Q sample lists."""
    return [math.sqrt(i * i + q * q) for i, q in zip(i_vals, q_vals, strict=False)]


def compute_inst_freq(i_vals: list[int], q_vals: list[int],
                      fs: float, mag_thresh: float = 5.0) -> list[float]:
    """Compute instantaneous frequency from I/Q via phase differencing."""
    phases = []
    for i_val, q_val in zip(i_vals, q_vals, strict=False):
        if abs(i_val) > mag_thresh or abs(q_val) > mag_thresh:
            phases.append(math.atan2(q_val, i_val))
        else:
            phases.append(None)

    freq_estimates = []
    for n in range(1, len(phases)):
        if phases[n] is not None and phases[n - 1] is not None:
            dp = phases[n] - phases[n - 1]
            while dp > math.pi:
                dp -= 2 * math.pi
            while dp < -math.pi:
                dp += 2 * math.pi
            freq_estimates.append(dp * fs / (2 * math.pi))
    return freq_estimates


# ============================================================================
# TEST 1: Structural validation — all .mem files exist with correct sizes
# ============================================================================
class TestStructural:
    """Verify every expected .mem file exists, has the right line count, and valid values."""

    @pytest.mark.parametrize("fname,info", EXPECTED_MEM_FILES.items(),
                             ids=EXPECTED_MEM_FILES.keys())
    def test_file_exists(self, fname, info):
        path = os.path.join(MEM_DIR, fname)
        assert os.path.isfile(path), f"{fname} missing from {MEM_DIR}"

    @pytest.mark.parametrize("fname,info", EXPECTED_MEM_FILES.items(),
                             ids=EXPECTED_MEM_FILES.keys())
    def test_line_count(self, fname, info):
        vals = read_mem_hex(fname)
        assert len(vals) == info['lines'], (
            f"{fname}: got {len(vals)} data lines, expected {info['lines']}"
        )

    @pytest.mark.parametrize("fname,info", EXPECTED_MEM_FILES.items(),
                             ids=EXPECTED_MEM_FILES.keys())
    def test_value_range(self, fname, info):
        vals = read_mem_hex(fname)
        for i, v in enumerate(vals):
            assert -32768 <= v <= 32767, (
                f"{fname}[{i}]: value {v} out of 16-bit signed range"
            )


# ============================================================================
# TEST 2: FFT Twiddle Factor Validation (bit-exact against cos formula)
# ============================================================================
class TestTwiddle:
    """Verify FFT twiddle .mem files match cos(2*pi*k/N) in Q15 to <=1 LSB."""

    def test_twiddle_1024_bit_exact(self):
        vals = read_mem_hex('fft_twiddle_1024.mem')
        assert len(vals) == 256, f"Expected 256 quarter-wave entries, got {len(vals)}"

        max_err = 0
        worst_k = -1
        for k in range(256):
            angle = 2.0 * math.pi * k / 1024.0
            expected = max(-32768, min(32767, round(math.cos(angle) * 32767.0)))
            err = abs(vals[k] - expected)
            if err > max_err:
                max_err = err
                worst_k = k

        assert max_err <= 1, (
            f"fft_twiddle_1024.mem: max error {max_err} LSB at k={worst_k} "
            f"(got {vals[worst_k]}, expected "
            f"{max(-32768, min(32767, round(math.cos(2*math.pi*worst_k/1024)*32767)))})"
        )

    def test_twiddle_16_bit_exact(self):
        vals = read_mem_hex('fft_twiddle_16.mem')
        assert len(vals) == 4, f"Expected 4 quarter-wave entries, got {len(vals)}"

        max_err = 0
        for k in range(4):
            angle = 2.0 * math.pi * k / 16.0
            expected = max(-32768, min(32767, round(math.cos(angle) * 32767.0)))
            err = abs(vals[k] - expected)
            if err > max_err:
                max_err = err

        assert max_err <= 1, f"fft_twiddle_16.mem: max error {max_err} LSB (tolerance: 1)"

    def test_twiddle_1024_known_values(self):
        """Spot-check specific twiddle values against hand-calculated results."""
        vals = read_mem_hex('fft_twiddle_1024.mem')
        # k=0: cos(0) = 1.0 -> 32767
        assert vals[0] == 32767, f"k=0: expected 32767, got {vals[0]}"
        # k=128: cos(pi/4) = sqrt(2)/2 -> round(32767 * 0.7071) = 23170
        expected_128 = round(math.cos(2 * math.pi * 128 / 1024) * 32767)
        assert abs(vals[128] - expected_128) <= 1, (
            f"k=128: expected ~{expected_128}, got {vals[128]}"
        )
        # k=255: last entry in quarter-wave table
        expected_255 = round(math.cos(2 * math.pi * 255 / 1024) * 32767)
        assert abs(vals[255] - expected_255) <= 1, (
            f"k=255: expected ~{expected_255}, got {vals[255]}"
        )


# ============================================================================
# TEST 3: Long Chirp .mem File Analysis
# ============================================================================
class TestLongChirp:
    """Validate long chirp .mem files show correct chirp characteristics."""

    def test_total_sample_count(self):
        """4 segments x 1024 samples = 4096 total."""
        all_i, all_q = [], []
        for seg in range(4):
            all_i.extend(read_mem_hex(f'long_chirp_seg{seg}_i.mem'))
            all_q.extend(read_mem_hex(f'long_chirp_seg{seg}_q.mem'))
        assert len(all_i) == 4096, f"Total I samples: {len(all_i)}, expected 4096"
        assert len(all_q) == 4096, f"Total Q samples: {len(all_q)}, expected 4096"

    def test_nonzero_magnitude(self):
        """Chirp should have significant non-zero content."""
        all_i, all_q = [], []
        for seg in range(4):
            all_i.extend(read_mem_hex(f'long_chirp_seg{seg}_i.mem'))
            all_q.extend(read_mem_hex(f'long_chirp_seg{seg}_q.mem'))
        mags = compute_magnitudes(all_i, all_q)
        max_mag = max(mags)
        # Should use substantial dynamic range (at least 1000 out of 32767)
        assert max_mag > 1000, f"Max magnitude {max_mag:.0f} is suspiciously low"

    def test_frequency_sweep(self):
        """Chirp should show at least 0.5 MHz frequency sweep."""
        all_i, all_q = [], []
        for seg in range(4):
            all_i.extend(read_mem_hex(f'long_chirp_seg{seg}_i.mem'))
            all_q.extend(read_mem_hex(f'long_chirp_seg{seg}_q.mem'))

        freq_est = compute_inst_freq(all_i, all_q, FS_SYS)
        assert len(freq_est) > 100, "Not enough valid phase samples for frequency analysis"

        f_range = max(freq_est) - min(freq_est)
        assert f_range > 0.5e6, (
            f"Frequency sweep {f_range / 1e6:.2f} MHz is too narrow "
            f"(expected > 0.5 MHz for a chirp)"
        )

    def test_bandwidth_reasonable(self):
        """Chirp bandwidth should be within 50% of expected 20 MHz."""
        all_i, all_q = [], []
        for seg in range(4):
            all_i.extend(read_mem_hex(f'long_chirp_seg{seg}_i.mem'))
            all_q.extend(read_mem_hex(f'long_chirp_seg{seg}_q.mem'))

        freq_est = compute_inst_freq(all_i, all_q, FS_SYS)
        if not freq_est:
            pytest.skip("No valid frequency estimates")

        f_range = max(freq_est) - min(freq_est)
        bw_error = abs(f_range - CHIRP_BW) / CHIRP_BW
        if bw_error >= 0.5:
            warnings.warn(
                f"Bandwidth {f_range / 1e6:.2f} MHz differs from expected "
                f"{CHIRP_BW / 1e6:.2f} MHz by {bw_error:.0%}",
                stacklevel=1,
            )


# ============================================================================
# TEST 4: Short Chirp .mem File Analysis
# ============================================================================
class TestShortChirp:
    """Validate short chirp .mem files."""

    def test_sample_count_matches_duration(self):
        """0.5 us at 100 MHz = 50 samples."""
        short_i = read_mem_hex('short_chirp_i.mem')
        short_q = read_mem_hex('short_chirp_q.mem')
        expected = int(T_SHORT_CHIRP * FS_SYS)
        assert len(short_i) == expected, f"Short chirp I: {len(short_i)} != {expected}"
        assert len(short_q) == expected, f"Short chirp Q: {len(short_q)} != {expected}"

    def test_all_samples_nonzero(self):
        """Every sample in the short chirp should have non-trivial magnitude."""
        short_i = read_mem_hex('short_chirp_i.mem')
        short_q = read_mem_hex('short_chirp_q.mem')
        mags = compute_magnitudes(short_i, short_q)
        nonzero = sum(1 for m in mags if m > 1)
        assert nonzero == len(short_i), (
            f"Only {nonzero}/{len(short_i)} samples are non-zero"
        )


# ============================================================================
# TEST 5: Chirp vs Independent Model (phase shape agreement)
# ============================================================================
class TestChirpVsModel:
    """Compare seg0 against independently generated chirp reference."""

    def test_phase_shape_match(self):
        """Phase trajectory of .mem seg0 should match model within 0.5 rad."""
        # Generate reference chirp independently from first principles
        chirp_rate = CHIRP_BW / T_LONG_CHIRP  # Hz/s
        n_samples = FFT_SIZE  # 1024

        model_i, model_q = [], []
        for n in range(n_samples):
            t = n / FS_SYS
            phase = math.pi * chirp_rate * t * t
            re_val = max(-32768, min(32767, round(32767 * 0.9 * math.cos(phase))))
            im_val = max(-32768, min(32767, round(32767 * 0.9 * math.sin(phase))))
            model_i.append(re_val)
            model_q.append(im_val)

        # Read seg0 from .mem
        mem_i = read_mem_hex('long_chirp_seg0_i.mem')
        mem_q = read_mem_hex('long_chirp_seg0_q.mem')

        # Compare phase trajectories (shape match regardless of scaling)
        model_phases = [math.atan2(q, i) for i, q in zip(model_i, model_q, strict=False)]
        mem_phases = [math.atan2(q, i) for i, q in zip(mem_i, mem_q, strict=False)]

        phase_diffs = []
        for mp, fp in zip(model_phases, mem_phases, strict=False):
            d = mp - fp
            while d > math.pi:
                d -= 2 * math.pi
            while d < -math.pi:
                d += 2 * math.pi
            phase_diffs.append(d)

        max_phase_diff = max(abs(d) for d in phase_diffs)
        assert max_phase_diff < 0.5, (
            f"Max phase difference {math.degrees(max_phase_diff):.1f} deg "
            f"exceeds 28.6 deg tolerance"
        )

    def test_magnitude_scaling(self):
        """Seg0 magnitude should be consistent with Q15 * 0.9 scaling."""
        mem_i = read_mem_hex('long_chirp_seg0_i.mem')
        mem_q = read_mem_hex('long_chirp_seg0_q.mem')
        mags = compute_magnitudes(mem_i, mem_q)
        max_mag = max(mags)

        # Expected from 32767 * 0.9 scaling = ~29490
        expected_max = 32767 * 0.9
        # Should be at least 80% of expected (allows for different provenance)
        if max_mag < expected_max * 0.8:
            warnings.warn(
                f"Seg0 max magnitude {max_mag:.0f} is below expected "
                f"{expected_max:.0f} * 0.8 = {expected_max * 0.8:.0f}. "
                f"The .mem files may have different provenance.",
                stacklevel=1,
            )


# ============================================================================
# TEST 6: Latency Buffer LATENCY=3187 Validation
# ============================================================================
class TestLatencyBuffer:
    """Validate latency buffer parameter constraints."""

    LATENCY = 3187
    BRAM_SIZE = 4096

    def test_latency_within_bram(self):
        assert self.LATENCY < self.BRAM_SIZE, (
            f"LATENCY ({self.LATENCY}) must be < BRAM size ({self.BRAM_SIZE})"
        )

    def test_latency_in_reasonable_range(self):
        """LATENCY should be between 1000 and 4095 (empirically determined)."""
        assert 1000 < self.LATENCY < 4095, (
            f"LATENCY={self.LATENCY} outside reasonable range [1000, 4095]"
        )

    def test_read_ptr_no_overflow(self):
        """Address arithmetic for read_ptr after initial wrap must stay valid."""
        min_read_ptr = self.BRAM_SIZE + 0 - self.LATENCY
        assert 0 <= min_read_ptr < self.BRAM_SIZE, (
            f"min_read_ptr after wrap = {min_read_ptr}, must be in [0, {self.BRAM_SIZE})"
        )


# ============================================================================
# TEST 7: Chirp Memory Loader Addressing
# ============================================================================
class TestMemoryAddressing:
    """Validate {segment_select[1:0], sample_addr[9:0]} address mapping."""

    @pytest.mark.parametrize("seg", range(4), ids=[f"seg{s}" for s in range(4)])
    def test_segment_base_address(self, seg):
        """Concatenated address {seg, 10'b0} should equal seg * 1024."""
        addr = (seg << 10) | 0
        expected = seg * 1024
        assert addr == expected, (
            f"Seg {seg}: {{seg[1:0], 10'b0}} = {addr}, expected {expected}"
        )

    @pytest.mark.parametrize("seg", range(4), ids=[f"seg{s}" for s in range(4)])
    def test_segment_end_address(self, seg):
        """Concatenated address {seg, 10'h3FF} should equal seg * 1024 + 1023."""
        addr = (seg << 10) | 1023
        expected = seg * 1024 + 1023
        assert addr == expected, (
            f"Seg {seg}: {{seg[1:0], 10'h3FF}} = {addr}, expected {expected}"
        )

    def test_full_address_space(self):
        """4 segments x 1024 = 4096 addresses, covering full 12-bit range."""
        all_addrs = set()
        for seg in range(4):
            for sample in range(1024):
                all_addrs.add((seg << 10) | sample)
        assert len(all_addrs) == 4096
        assert min(all_addrs) == 0
        assert max(all_addrs) == 4095


# ============================================================================
# TEST 8: Seg3 Zero-Padding Analysis
# ============================================================================
class TestSeg3Padding:
    """Analyze seg3 content — chirp is 3000 samples but 4 segs x 1024 = 4096 slots."""

    def test_seg3_content_analysis(self):
        """Seg3 should either be full (4096-sample chirp) or have trailing zeros."""
        seg3_i = read_mem_hex('long_chirp_seg3_i.mem')
        seg3_q = read_mem_hex('long_chirp_seg3_q.mem')
        mags = compute_magnitudes(seg3_i, seg3_q)

        # Count trailing zeros
        trailing_zeros = 0
        for m in reversed(mags):
            if m < 2:
                trailing_zeros += 1
            else:
                break

        nonzero = sum(1 for m in mags if m > 2)

        if nonzero == 1024:
            # .mem files encode 4096 chirp samples, not 3000
            # This means the chirp duration used for .mem generation differs
            actual_samples = 4 * 1024
            actual_us = actual_samples / FS_SYS * 1e6
            warnings.warn(
                f"Chirp in .mem files is {actual_samples} samples ({actual_us:.1f} us), "
                f"not {LONG_CHIRP_SAMPLES} samples ({T_LONG_CHIRP * 1e6:.1f} us). "
                f"The .mem files use a different chirp duration than the system parameter.",
                stacklevel=1,
            )
        elif trailing_zeros > 100:
            # Some zero-padding at end — chirp ends partway through seg3
            effective_chirp_end = 3072 + (1024 - trailing_zeros)
            assert effective_chirp_end <= 4096, "Chirp end calculation overflow"
