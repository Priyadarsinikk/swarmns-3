#!/bin/bash
# SwarmNet-IND -- C2 loss diagnosis + short-link C2 relay chain (version 11b).
# All: 50 UAVs, 1 km, N2 with realistic beacons, final N1 (3 ms window every 20 ms + 40 ms C2 expiry),
#      10 C2 drones, 120 B every 40 ms.
#   video load: light (5 x 300 kbps), heavy (5 x 600 kbps and 10 x 300 kbps)
#   C2 route: same relays as video, or own chain of short links (70% or 80% of radio range)
#   Usage:  bash run-c2route.sh quick   -> run 3 only (9 runs)
#           bash run-c2route.sh         -> runs 1..5   (45 runs)
# Results: ~/ns-3-dev/results/c2route-<date-time>/all-with-tags.csv
set -e
cd ~/ns-3-dev
echo "Building swarm-baseline once before starting..."
./ns3 build swarm-baseline
RUNS="1 2 3 4 5"; [ "$1" = "quick" ] && RUNS="3"
OUT="$PWD/results/c2route-$(date +%Y%m%d-%H%M)"
mkdir -p "$OUT"
JOBS=$(( $(nproc) - 1 )); [ "$JOBS" -lt 1 ] && JOBS=1
echo "Running on $JOBS cores. Results folder: $OUT"
export OUT
{
  for load in "5 300kbps" "5 600kbps" "10 300kbps"; do
    for run in $RUNS; do
      for v in "0 0.7" "1 0.7" "1 0.8"; do echo "$load $v $run"; done
    done
  done
} | xargs -P "$JOBS" -L 1 bash -c '
    cams=$0; rate=$1; route=$2; frac=$3; run=$4
    tag="c$cams-r$rate-route$route-f$frac-run$run"
    dir="$OUT/$tag"; mkdir -p "$dir"; start=$(date +%s)
    args="--routing=N2 --n2Info=real --beaconLoss=0.1 --gpsErrM=5 --areaSize=1000 --nUav=50 --nCam=$cams --flowRate=$rate --run=$run --arpPrefill=1 --deadlineMs=100 --nC2=10 --c2Prio=1 --n1c2=1 --n1SfMs=20 --c2WinMs=3 --c2Copies=1 --c2Route=$route --c2SafeFrac=$frac"
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
echo "tag | C2 delivered % | within 40 ms % | mean ms | max ms | video within 100 ms % | no route at src | held too old | WiFi gave up in range | out of range | queue expired"
awk -F, '{printf "%s | %.2f | %.2f | %.1f | %.0f | %.1f | %s | %s | %s | %s | %s\n",$1,$55,$56,$57,$59,$44,$64,$65,$66,$67,$68}' "$OUT/all-with-tags.csv"
