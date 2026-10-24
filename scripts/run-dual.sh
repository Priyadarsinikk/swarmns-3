#!/bin/bash
# SwarmNet-IND -- C2 over two paths (version 11).
# All: 50 UAVs, 1 km, N2 with realistic beacons, final N1 (3 ms window every 20 ms + 40 ms C2 expiry),
#      10 C2 drones, 120 B every 40 ms.
#   video load: light (5 x 300 kbps) and heavy (10 x 300 kbps)
#   copies: 1 (one path) or 2 (two paths, ground keeps the first copy)
#   Usage:  bash run-dual.sh quick   -> run 3 only (4 runs, a few minutes)
#           bash run-dual.sh         -> runs 1..5   (20 runs)
# Results: ~/ns-3-dev/results/dual-<date-time>/all-with-tags.csv
set -e
cd ~/ns-3-dev
echo "Building swarm-baseline once before starting..."
./ns3 build swarm-baseline
RUNS="1 2 3 4 5"; [ "$1" = "quick" ] && RUNS="3"
OUT="$PWD/results/dual-$(date +%Y%m%d-%H%M)"
mkdir -p "$OUT"
JOBS=$(( $(nproc) - 1 )); [ "$JOBS" -lt 1 ] && JOBS=1
echo "Running on $JOBS cores. Results folder: $OUT"
export OUT
{
  for load in "5 300kbps" "10 300kbps"; do
    for run in $RUNS; do
      for copies in 1 2; do echo "$load $copies $run"; done
    done
  done
} | xargs -P "$JOBS" -L 1 bash -c '
    cams=$0; rate=$1; copies=$2; run=$3
    tag="c$cams-r$rate-copies$copies-run$run"
    dir="$OUT/$tag"; mkdir -p "$dir"; start=$(date +%s)
    args="--routing=N2 --n2Info=real --beaconLoss=0.1 --gpsErrM=5 --areaSize=1000 --nUav=50 --nCam=$cams --flowRate=$rate --run=$run --arpPrefill=1 --deadlineMs=100 --nC2=10 --c2Prio=1 --n1c2=1 --n1SfMs=20 --c2WinMs=3 --c2Copies=$copies"
    timeout 3600 ./ns3 run "swarm-baseline $args" --no-build --cwd="$dir" > "$dir/log.txt" 2>&1 \
      && echo "done   $tag  ($(( $(date +%s) - start )) s)" \
      || echo "FAILED $tag  (see $dir/log.txt)"
  '
for d in "$OUT"/*/; do
  [ -f "$d/swarm-baseline.csv" ] && echo "$(basename "$d"),$(cat "$d/swarm-baseline.csv")"
done > "$OUT/all-with-tags.csv"
echo
echo "All done. Combined results: $OUT/all-with-tags.csv"
echo
echo "tag | C2 delivered % | C2 within 40 ms % | C2 mean ms | C2 max ms | video within 100 ms % | duplicates"
awk -F, '{printf "%s | %.2f | %.2f | %.1f | %.0f | %.1f | %s\n",$1,$55,$56,$57,$59,$44,$61}' "$OUT/all-with-tags.csv"
