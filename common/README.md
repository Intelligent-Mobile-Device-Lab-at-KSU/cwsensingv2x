# common/ — Shared Utilities

Shared MATLAB utility functions used by all simulation versions.

## Files

- `idx_helpers.m` — Static class with index conversion methods (linear <-> subchannel/subframe), L_subCH computation, and reselection counter drawing.
- `cw_phy_params.m` — CW sensing element PHY parameter computation (sensing bin counts, bin-to-subchannel mapping). Supports 10 MHz and 20 MHz.

## Usage

Add to MATLAB path from any version folder:
```matlab
addpath('../common');
```
