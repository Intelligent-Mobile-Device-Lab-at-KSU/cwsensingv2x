# v3: L2 + CW Sensing Simulation — Integrated Sensing Modes

Extends v2 with CW (Continuous Wave) sensing transmissions. Vehicles transmit CW tones at 5.9 GHz during data subframes, placed at frequencies between OFDM data subcarriers (7.5 kHz midpoints via 2xM FFT). Six sensing modes control CW element assignment.

## CW Sensing Modes

| Mode | Elements | Scope | Persistence | Frequency Hops? |
|------|----------|-------|-------------|-----------------|
| 1a | 1 | Global (full BW) | Fixed entire sim | No |
| 1b | 1 | Within allocated PRBs | Changes with data SPS | Yes |
| 1c | 1 | Global, SPS-like | SPS-counter dwell | Yes |
| 2a | 2-8 | Global (full BW) | Fixed entire sim | No |
| 2b | 2-8 | Within allocated PRBs | Changes with data SPS | Yes |
| 2c | 2-8 | Global, SPS-like | SPS-counter dwell | Yes |

## PHY Parameters
- Data: 15 kHz subcarrier spacing, M=1024 (10 MHz) or M=2048 (20 MHz)
- Sensing: 2xM FFT -> 7.5 kHz spacing, sensing bins at midpoints
- 10 MHz: 600 data subcarriers, 599 sensing bins (~60 per subchannel)
- 20 MHz: 1200 data subcarriers, 1199 sensing bins (~60 per subchannel)

## CW Impact on SPS
- Modes 1a/1c/2a/2c: CW in random subchannels increases CBR slightly
- Modes 1b/2b: CW within data allocation, zero CBR impact
- CW does NOT corrupt data at PHY (experimentally validated)

## Configuration
```matlab
cw_mode = '1a';           % Sensing mode: '1a','1b','1c','2a','2b','2c'
N_cw = 1;                 % Number of CW elements (Mode 2: 2-8, or 'random')
channel_bw_MHz = 10;      % Channel bandwidth: 10 or 20
```

## Running the Full Pipeline

### Option A — Full end-to-end (recommended)
Runs the MAC simulation, generates the CW baseband, and compares micro-Doppler against the ground-truth linear chirp in one shot.

```matlab
cd('C:\Users\bkihei\Documents\cwdistributed\v3_L2_CW')
run_full_pipeline
```

**What you get:**
- All standard L2_CW_sim figures (CBR, data grid, CW grid, interactive viewer)
- A `.dat` baseband file written to `v3_L2_CW/` (e.g. `cw_baseband_v1_mode2b_1000kHz_100s.dat`)
- 5 micro-Doppler figures:
  1. 2-D Doppler spectrogram
  2. Detected Doppler vs ground-truth linear chirp (with overlay)
  3. Raw detected peak frequencies
  4. Per-subframe TX energy (burst detection)
  5. Doppler error vs chirp
- Console output: `Doppler RMSE vs linear chirp: XX.X Hz`

**To configure** before running, open `L2_CW_sim.m` and edit Section 1 (lines 56–66):
```matlab
cw_mode = '2b';          % '1a','1b','1c','2a','2b','2c'
simseconds = 100;        % simulation duration
record_cw_rh_always = true;  % must stay true for baseband generation
```

To change which vehicle is analyzed or the Doppler sweep range, edit the bottom of `run_full_pipeline.m`:
```matlab
pipeline_vehicle_id = 1;          % vehicle to analyze

[mdop_results, bb_info] = run_baseband_mdop( ...
    L2out, CWout, pipeline_vehicle_id, ...
    'Fs_out',       1e6, ...           % 1 MHz low-rate de-hopped
    'DopplerRange', [-1000, 1000], ... % ground-truth chirp range (Hz)
    'BurstMode',    true);             % per-ms RMSE comparison
```

### Option B — MAC sim only
```matlab
addpath('../common');
L2_CW_sim;
```

### Option C — Baseband + micro-Doppler on existing sim output
If `L2out` and `CWout` are already in the workspace:
```matlab
[results, info] = run_baseband_mdop(L2out, CWout, 1);
```

---

## Files
- `L2_CW_sim.m` — Main simulation (configurable sensing mode)
- `run_full_pipeline.m` — End-to-end driver (MAC sim → baseband → micro-Doppler)
- `run_baseband_mdop.m` — Baseband generation + micro-Doppler orchestrator
- `generate_cw_baseband.m` — Synthesize per-vehicle CW baseband `.dat` file
- `mDopPx_speedUp_allPRBs.m` — Micro-Doppler extraction and chirp comparison
- `sps_reselect_cw.m` — CW-aware SPS reselection
- `sps_reselect.m` — Plain SPS Mode 4 reselection
- `cw_assign.m` — CW sensing element assignment (all modes)
- `cw_sensing_reselect.m` — SPS-like reselection for CW bins
- `L2_plot_results.m` — Extended plots (data + CW)
- `L2_output_interface.m` — Output struct builder
- `L2_interactive_grid.m` — Interactive resource grid viewer
