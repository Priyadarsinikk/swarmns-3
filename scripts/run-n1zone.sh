#!/bin/bash
# SwarmNet-IND -- N1 v2 (funnel-zone TDMA) test under the loads where collisions hurt N2.
# All: 50 UAVs, 1 km, N2 with realistic beacons, ARP pre-filled, 100 ms deadline.
#
#   loads: 5 x 600 kbps, 10 x 300 kbps (3 Mbps, collision-limited), 5 x 1 Mbps (5 Mbps, capacity-limited)
#   N2 alone  vs  N2 + N1 v2 with (superframe, zone share) = (20 ms, 0.5), (20 ms, 0.6), (30 ms, 0.6)
#   x runs 1..5   = 3 loads x 4 schemes x 5 = 60 runs
# Results: ~/ns-3-dev/results/n1zone-<date-time>/all-with-tags.csv

set -e
cd ~/ns-3-dev

echo "Building swarm-baseline once before starting..."
./ns3 build swarm-baseline

OUT="$PWD/results/n1zone-$(date +%Y%m%d-%H%M)"
mkdir -p "$OUT"
JOBS=$(( $(nproc) - 1 ))
[ "$JOBS" -lt 1 ] && JOBS=1
echo "Running on $JOBS cores. Results folder: $OUT"
export OUT

{
  for load in "5 600kbps" "10 300kbps" "5 1Mbps"; do
    for run in 1 2 3 4 5; do
      echo "$load 0 20 0.5 $run"
      echo "$load 1 20 0.5 $run"
      echo "$load 1 20 0.6 $run"
      echo "$load 1 30 0.6 $run"
    done
  done
} | xargs -P "$JOBS" -L 1 bash -c '
    cams=$0; rate=$1; zone=$2; sf=$3; af=$4; run=$5
    tag="c$cams-r$rate-n1zone$zone-sf$sf-a$af-run$run"
    dir="$OUT/$tag"
    mkdir -p "$dir"
    start=$(date +%s)
    args="--routing=N2 --n2Info=real --beaconLoss=0.1 --gpsErrM=5 --areaSize=1000 --nUav=50 --nCam=$cams --flowRate=$rate --run=$run --arpPrefill=1 --deadlineMs=100 --n1zone=$zone --n1SfMs=$sf --n1AFrac=$af"
    timeout 3600 ./ns3 run "swarm-baseline $args" --no-build --cwd="$dir" > "$dir/log.txt" 2>&1 \
      && echo "done   $tag  ($(( $(date +%s) - start )) s)" \
      || echo "FAILED $tag  (see $dir/log.txt)"
  '

for d in "$OUT"/*/; do
  [ -f "$d/swarm-baseline.csv" ] && echo "$(basename "$d"),$(cat "$d/swarm-baseline.csv")"
done > "$OUT/all-with-tags.csv"

echo
echo "All done. Combined results: $OUT/all-with-tags.csv"
echo "Rows (one per finished run): $(wc -l < "$OUT/all-with-tags.csv")"
