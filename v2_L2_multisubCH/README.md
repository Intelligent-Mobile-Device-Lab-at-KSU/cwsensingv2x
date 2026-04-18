# v2: L2 SPS Simulation — Multi-Subchannel (L_subCH per message type)

Full L2 fidelity simulation where each transmission occupies L_subCH contiguous subchannels based on SAE message size and MCS. This is the pure L2 simulation (no CW sensing).

## Parameters
- Nslch = 10, Nrbs = 5, PRB = 50 (10 MHz), MCS = 7
- L_subCH computed per message type (e.g., BSM = 3, MAP = 8)
- 256 OBUs + 2 RSUs, 100 seconds
- Scheduling table optimization for fast simulation

## Key Features
- 3GPP Rel 14 PC5 Mode 4 SPS with all bug fixes
- Multi-subchannel resource selection (contiguous L_subCH)
- SPS-like reselection with vectorized RSSI scoring
- Interactive resource grid viewer (L2_interactive_grid.m)
- 8 L2 performance plots

## Run
```matlab
addpath('../common');
L2_sim_SPS;
```

## Output
- `L2_SPS_output.mat` — L2out and CWout structs
- `vehicle_tx_schedule.csv` — Per-event TX log
