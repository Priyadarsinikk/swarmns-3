#!/bin/bash
# SwarmNet-IND -- run the baseline sweep in parallel on the workstation.
#
# What it runs:  OLSR and AODV  x  20, 50, 100 UAVs  x  random runs 1..5  = 30 simulations
# Each simulation runs in its own folder (so they never overwrite each other's CSV),
# several at once (one per CPU core, leaving one core free).
#
# Usage (from anywhere):
#   bash run-sweep.sh
# Results land in ~/ns-3-dev/results/<date-time>/all.csv

set -e
cd ~/ns-3-dev

echo "Building swarm-baseline once before starting..."
./ns3 build swarm-baseline

OUT="$PWD/results/$(date +%Y%m%d-%H%M)"
mkdir -p "$OUT"
JOBS=$(( $(nproc) - 1 ))
[ "$JOBS" -lt 1 ] && JOBS=1
echo "Running on $JOBS cores. Results folder: $OUT"
export OUT

# Job list: OLSR first (fast), AODV last (slow), small swarms before large ones
{
  for routing in OLSR AODV; do
    for n in 20 50 100; do
      for run in 1 2 3 4 5; do
        echo "$routing $n $run"
      done
    done
  done
} | xargs -P "$JOBS" -L 1 bash -c '
    routing=$0; n=$1; run=$2
    dir="$OUT/$routing-n$n-run$run"
    mkdir -p "$dir"
    start=$(date +%s)
    ./ns3 run "swarm-baseline --routing=$routing --nUav=$n --run=$run" \
        --no-build --cwd="$dir" > "$dir/log.txt" 2>&1 \
      && echo "done   $routing n=$n run=$run  ($(( $(date +%s) - start )) s)" \
      || echo "FAILED $routing n=$n run=$run  (see $dir/log.txt)"
  '

# Merge every run into one CSV with a header row
echo "routing,nUav,nCam,run,areaSize,expected,leftSource,received,e2eDeliveryPct,networkPdrPct,meanDelayMs,p95DelayMs,p99DelayMs,maxDelayMs" > "$OUT/all.csv"
cat "$OUT"/*/swarm-baseline.csv >> "$OUT/all.csv" 2>/dev/null || true

echo
echo "All done. Combined results: $OUT/all.csv"
echo "Rows (one per finished run): $(( $(wc -l < "$OUT/all.csv") - 1 ))"
