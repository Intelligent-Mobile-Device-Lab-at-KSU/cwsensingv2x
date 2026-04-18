# C-V2X ISAC System-Level Simulation

MATLAB simulation framework for **C-V2X Mode 4 sidelink** with integrated
**Continuous-Wave (CW) sensing** (ISAC — Integrated Sensing and Communication).

Implements 3GPP Release 14 sensing-based Semi-Persistent Scheduling (SPS,
TS 36.213 §14.1.1.6) with full SAE J2735 multi-packet-type traffic, and extends
it with six CW sensing modes that place narrowband tones at the 7.5 kHz midpoints
between OFDM subcarriers. Includes a baseband synthesis and micro-Doppler
extraction pipeline for evaluating CW sensing performance.

---

## Repository structure

```
common/                     Shared utilities
  idx_helpers.m             Index conversions, L_subCH computation
  cw_phy_params.m           CW PHY parameters (bin spacing, FFT sizes)
  write_complex_binary.m    Write interleaved float32 I/Q .dat files

v2_L2_multisubCH/           Baseline: Mode 4 SPS with multi-subchannel data
  L2_sim_SPS.m              Main simulation script
  sps_reselect.m            3GPP TS 36.213 §14.1.1.6 resource selection
  L2_output_interface.m     Output struct builder
  L2_plot_results.m         Visualization
  L2_interactive_grid.m     Interactive resource grid viewer
  README.md

v3_L2_CW/                   Extension: CW sensing + ISAC pipeline
  L2_CW_sim.m               Main simulation (data SPS + CW modes)
  sps_reselect.m            Plain SPS reselection (modes 1b/2b)
  sps_reselect_cw.m         CW-aware SPS reselection (modes 1a/1c/2a/2c)
  cw_assign.m               CW element assignment (all modes)
  cw_sensing_reselect.m     SPS-like bin reselection for CW
  L2_output_interface.m     Output struct builder
  L2_plot_results.m         Visualization
  L2_interactive_grid.m     Interactive resource grid viewer
  generate_cw_baseband.m    Synthesize per-vehicle CW baseband .dat
  read_complex_binary.m     Read interleaved float32 I/Q .dat files
  mDopPx_speedUp_allPRBs.m  Micro-Doppler extraction and chirp comparison
  run_baseband_mdop.m       Baseband + micro-Doppler orchestrator
  run_full_pipeline.m       End-to-end driver (run this)
  README.md
```

---

## Quick start

### Baseline SPS simulation (v2)
```matlab
cd v2_L2_multisubCH
addpath('../common')
L2_sim_SPS
```

### Full CW ISAC pipeline (v3)
```matlab
cd v3_L2_CW
run_full_pipeline
```

`run_full_pipeline` runs the MAC-layer simulation, synthesizes the CW baseband
signal, extracts per-subframe Doppler estimates, and overlays them on the
ground-truth linear chirp. RMSE is printed to the console.

---

## CW sensing modes

| Mode | Tones | Placement       | Reselection          |
|------|-------|-----------------|----------------------|
| 1a   | 1     | Global          | Fixed (entire sim)   |
| 1b   | 1     | PRB-coupled     | Hops with data SPS   |
| 1c   | 1     | Global          | SPS-like dwell       |
| 2a   | 2–8   | Global          | Fixed (entire sim)   |
| 2b   | 2–8   | PRB-coupled     | Hops with data SPS   |
| 2c   | 2–8   | Global          | SPS-like dwell       |

Configure in `v3_L2_CW/L2_CW_sim.m` Section 1:
```matlab
cw_mode = '2b';          % sensing mode
N_cw    = 'random';      % tones per vehicle (Mode 2); ignored for Mode 1
```

---

## Key parameters (v3)

| Parameter | Default | Description |
|-----------|---------|-------------|
| `cw_mode` | `'2b'` | CW sensing mode |
| `channel_bw_MHz` | `10` | 10 or 20 MHz |
| `Nslch` | `10` | Sidelink subchannels |
| `Nrbs` | `5` | PRBs per subchannel |
| `simseconds` | `100` | Simulation duration (s) |
| `Vnum` | `30` | Number of OBUs |
| `RSUnum` | `1` | Number of RSUs |
| `cw_dwell_range` | `[5,15]` | Mode 1c/2c dwell counter range (subframes) |
| `record_cw_rh_always` | `true` | Record bin-level CW history for all modes |

RRI is derived per (vehicle, packet type): **100 ms** for 10 Hz messages
(BSM, SPAT) and **1000 ms** for 1 Hz messages (SRM, MAP, etc.).

---

## Requirements

- MATLAB R2019b or later
- Signal Processing Toolbox (`physconst`, `hann`, `blackman`)
- No other toolboxes required

---

## Output

After `run_full_pipeline`:
- **Figures 1–N** from `L2_CW_sim`: CBR comparison, data grid, CW subchannel
  grid, bin-level CW grid, interactive resource viewer
- **`cw_baseband_v<id>_mode<X>_1000kHz_<T>s.dat`** — synthesized baseband file
- **Micro-Doppler figures**: spectrogram, detected vs ground-truth chirp overlay,
  raw peaks, TX energy profile, Doppler error curve
- **Console**: `Doppler RMSE vs linear chirp: XX.X Hz`
