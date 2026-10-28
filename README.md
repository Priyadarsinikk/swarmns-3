# SwarmNS-3: UAV swarm mesh networking in ns-3

Simulation study for **SwarmNet-IND**, an indigenous SDR-based open protocol stack for UAV swarm mesh networking (SRM Institute of Science and Technology, Kattankulathur).

The model runs on ns-3.49 and compares standard mesh routing (OLSR, AODV) and a predictive-routing baseline against the proposed mechanisms:

- **N2, trajectory-aware routing:** UAVs share position and velocity in 40-byte beacons every 100 ms. A relay is used only if it is predicted to stay in range 0.5 s ahead, the next hop is bound at the moment of transmission rather than at the last routing update, and data is held rather than transmitted when no relay qualifies. C2 uses a separate chain of short links (70% of radio range).
- **N1, C2-protecting MAC:** a 3 ms window every 20 ms in which only command-and-control (C2) frames may be sent, with C2 older than 40 ms discarded.
- **PRED, a predictive baseline:** link expiration time from relative position and velocity, in the manner of Su, Lee and Gerla, written into the routing table and left there until the next update. It has the prediction but none of the three mechanisms above, so each one can be measured separately. It is not a reimplementation of any published protocol and should not be cited as one.

## Contents

| Path | What it is |
|---|---|
| `scratch/swarm-baseline.cc` | The simulation program (copy into `ns-3-dev/scratch/`) |
| `scripts/run-ablation.sh` | The ablation, sweeps and two-way C2 runs |
| `scripts/run-*.sh` | Earlier parameter sweeps; each run uses its own folder and seed |
| `results/<sweep>-<date>/all*.csv` | Raw results, one row per run |

## Running

```
cp scratch/swarm-baseline.cc ~/ns-3-dev/scratch/
cp scripts/run-ablation.sh   ~/ns-3-dev/
cd ~/ns-3-dev && ./ns3 build swarm-baseline
./ns3 run "swarm-baseline --routing=N2 --n2Info=real --nUav=50 --run=1"
```

`bash run-ablation.sh` runs everything below over five seeds (355 runs, about 15 minutes on 35 cores). `bash run-ablation.sh quick R` runs the regression block alone. Run `./ns3 run "swarm-baseline --help"` for all options.

Key options added since the first release:

| Option | Meaning |
|---|---|
| `--routing=PRED` | the predictive baseline |
| `--predHold=1` | PRED plus holding data when no relay qualifies |
| `--letThreshS` | PRED look-ahead threshold, seconds (default 1.0) |
| `--baseQueue=1` | give OLSR or AODV N2's queue discipline with their own routing |
| `--c2Downlink=1` | also send C2 from the ground station to each C2 drone |
| `--c2DlPos` | `tracked` (destination position from its own uplink reports) or `ideal` |
| `--safeFrac` | N2 relay range margin as a fraction of radio range (default 0.85) |

## Simulation set-up

IEEE 802.11a ad-hoc radio (12 Mbps) at 5.18 GHz and 16 dBm, Friis propagation, −82 dBm detection threshold (range about 365 m); 3D Gauss-Markov mobility at 30–120 km/h; 1000-byte video packets; 100 s runs with traffic from 20 s; five seeds per configuration. C2 traffic follows the 3GPP TS 22.125 direct-steering profile: ten UAVs, 120-byte messages every 40 ms, 40 ms deadline.

The 802.11a radio is a stand-in until the PHY shim is calibrated. Absolute numbers will change on SDR hardware; the comparisons between schemes are what the study relies on.

**Reproducibility.** Runs are deterministic given the seed. The regression block was reproduced to two decimal places on two different machines (an HP Z4 G5 workstation and a separate ns-3.49 build), so a third party should obtain the same numbers.

## Key results (mean of 5 seeds)

### Video delivered within 100 ms, OLSR vs N2

| | 20 UAVs | 50 UAVs | 100 UAVs |
|---|---|---|---|
| OLSR, 1 km | 65% | 75% | 77% |
| N2, 1 km | 95% | 99% | 99% |
| OLSR, 1.5 km | 30% | 45% | 54% |
| N2, 1.5 km | 59% | 87% | 99% |

### C2 delivered within 40 ms (50 UAVs, 1 km, uplink only)

| | Light video | Heavy (5 × 600 kbps) | Heavy (10 × 300 kbps) |
|---|---|---|---|
| Priority queue only | 97.7% | 76.6% | 66.1% |
| N1 window | 98.8% | 86.6% | 76.3% |
| N1 + short-link C2 chain | 99.2% | 92.3% | 83.5% |

### Ablation: which mechanism earns which part of the gain

Video delivered within 100 ms. Each row adds one mechanism to the row above.

| Routing | 1.5 km, 20 | 1.5 km, 50 | 1 km, 50 | 1 km, 50, heavy video |
|---|---|---|---|---|
| OLSR | 30.7% | 44.5% | 69.2% | 52.3% |
| OLSR + N2's queue discipline | 30.7% | 44.5% | 69.2% | 52.3% |
| PRED (prediction only) | 51.3% | 77.5% | 92.7% | 49.7% |
| PRED + hold | 58.3% | 80.9% | 92.7% | 49.6% |
| + late binding (N2) | 57.7% | 81.8% | 95.8% | 52.5% |
| + short-link C2 chain (full N2) | 57.5% | 81.0% | 95.3% | 52.5% |

C2 within 40 ms over the same runs, OLSR to full N2: 35.8 → 61.1%, 47.6 → 88.9%, 74.6 → 99.2%, 58.7 → 92.3%.

Three things this shows:

1. **The queue discipline contributes nothing on its own.** Giving OLSR N2's three-class queue, deadline discard and window hooks, while leaving OLSR's routing in place, produced identical results to two decimal places in all four conditions.
2. **Prediction accounts for most of the video gain** (77% in the sparsest case, 90% in the dense one). Prediction is prior art. Holding adds up to 7 points where the swarm is thin and nothing where it is dense; late binding adds about 3 points where it is dense.
3. **Under heavy video load the routing barely affects video** (52.3% OLSR against 52.5% full N2) but strongly affects C2 (58.7% against 92.3%). At high load the channel is the limit.

### Two-way C2 (50 UAVs, 1 km)

Ten UAVs report every 40 ms while the ground station sends each of them a command on the same period.

| Video load | UAV to ground | Ground to UAV | Video within 100 ms |
|---|---|---|---|
| Light (5 × 300 kbps) | 97.4% | 96.5% | 81.5% |
| Heavy (5 × 600 kbps) | 87.5% | 79.3% | 36.4% |
| Heavy (10 × 300 kbps) | 74.9% | 54.0% | 31.0% |

The outward direction is the harder one, and it is where N2's mechanisms matter most: the predictive baseline delivers 68.7% and 20.2% of outward commands at the two heavy loads, against 79.3% and 54.0% for the full stack.

Routing toward a moving destination needs its position. The command carries the destination's last reported GPS fix, taken from its own uplink report. Giving every relay the destination's exact current position instead (`--c2DlPos=ideal`, not implementable) gains about one point: 80.3% and 55.2%. The outward path therefore depends on the inward one.

### Parameter sweeps

N2 relay range margin, video within 100 ms:

| Margin | 1.5 km, 20 | 1.5 km, 50 | 1 km, 50 | 1 km, heavy |
|---|---|---|---|---|
| 0.80 | 57.7% | 82.6% | 96.5% | 54.3% |
| 0.85 (used throughout) | 57.7% | 81.8% | 95.8% | 52.5% |
| 0.90 | 58.3% | 81.4% | 94.7% | 50.7% |
| 0.95 | 58.8% | 81.7% | 94.1% | 50.6% |
| 1.00 | 58.8% | 81.0% | 92.5% | 49.6% |

The best value is 0.80 in three conditions and 1.00 in the sparsest, with no consistent relationship to swarm density. The spread is at most 4.6 points and 0.85 is within 1.8 points of the best everywhere, so results do not depend on a finely tuned margin.

PRED look-ahead threshold: sweeping 0.5 s to 3 s changes the result by 0.2 points over 1.5 km and 0.8 points over 1 km, so the baseline is not handicapped by that choice.

N1 window length with two-way C2 at 5 × 600 kbps: widening from 3 ms to 6 ms raises outward C2 from 79.3% to 84.2% and lowers video from 36.4% to 30.0%.

## Which sweep block produces which table

`run-ablation.sh` takes an optional block list as its second argument.

| Block | Produces |
|---|---|
| R | the regression check against the published C2 figures |
| L | the ablation table |
| S | the range margin sweep |
| T | the look-ahead sweep |
| B | the two-way C2 table |
| W | the window length sweep |

## Approaches tested and rejected

Results are included for all of these: altitude-banded TDMA, slotted access near the ground station, fixed per-UAV C2 slots, duplicating every C2 message over two paths, a software priority queue without MAC support, and a relay range margin varied with local neighbour density.

## Team

Dr. K. Priyadarsini (PI), Dr. Revathy Venkataraman, Dr. S. Karthik, Dr. P. Rajasekaran, SRM Institute of Science and Technology.
