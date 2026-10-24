# SwarmNS-3: UAV swarm mesh networking in ns-3

Preliminary simulation study for **SwarmNet-IND**, an indigenous SDR-based open protocol stack for UAV swarm mesh networking (SRM Institute of Science and Technology, Kattankulathur).

The model runs on ns-3.49 and compares standard mesh routing (OLSR, AODV) with the proposed mechanisms:

- **N2, trajectory-aware routing:** UAVs share position and velocity in 40-byte beacons every 100 ms; a relay is used only if it is predicted to stay in range 0.5 s ahead, so routes are switched before links break.
- **N1, C2-protecting MAC:** a 3 ms window every 20 ms reserved for command-and-control (C2) messages, stale C2 (older than 40 ms) discarded, and a separate relay chain of short links (70% of radio range) for C2.

## Contents

| Path | What it is |
|---|---|
| `scratch/swarm-baseline.cc` | The simulation program (copy into `ns-3-dev/scratch/`) |
| `scripts/run-*.sh` | Parameter sweeps; each run uses its own folder and seed |
| `results/<sweep>-<date>/all*.csv` | Raw results, one row per run |

## Running

```
cp scratch/swarm-baseline.cc ~/ns-3-dev/scratch/
cd ~/ns-3-dev && ./ns3 build swarm-baseline
./ns3 run "swarm-baseline --routing=N2 --n2Info=real --nUav=50 --run=1"
```

Run `./ns3 run "swarm-baseline --help"` for all options. Each sweep script prints where its results are written.

## Simulation set-up

IEEE 802.11a ad-hoc radio (12 Mbps) at 5.18 GHz and 16 dBm, Friis propagation, −82 dBm detection threshold (range about 365 m); 3D Gauss-Markov mobility at 30–120 km/h; 1000-byte video packets; 100 s runs with traffic from 20 s; five seeds per configuration.

## Key results (mean of 5 runs)

| Video delivered within 100 ms | 20 UAVs | 50 UAVs | 100 UAVs |
|---|---|---|---|
| OLSR, 1 km | 65% | 75% | 77% |
| N2, 1 km | 95% | 99% | 99% |
| OLSR, 1.5 km | 30% | 45% | 54% |
| N2, 1.5 km | 59% | 87% | 99% |

| C2 delivered within 40 ms (50 UAVs, 1 km) | Light video | Heavy video (5 × 600 kbps) | Heavy video (10 × 300 kbps) |
|---|---|---|---|
| Priority queue only | 97.7% | 76.6% | 66.1% |
| N1 window | 98.8% | 86.6% | 76.3% |
| N1 + short-link C2 chain | 99.2% | 92.3% | 83.5% |

Approaches tested and rejected (results included): altitude-banded TDMA, slotted access near the ground station, fixed per-UAV C2 slots, and duplicating every C2 message over two paths.

## Team

Dr. K. Priyadarsini (PI), Dr. Revathy Venkataraman, Dr. S. Karthik, Dr. P. Rajasekaran, SRM Institute of Science and Technology.

