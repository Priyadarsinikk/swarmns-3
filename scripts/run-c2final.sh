#!/bin/bash
# SwarmNet-IND -- N1 final test: v3 shared window + 40 ms C2 discard: can command-and-control (C2) messages meet the 3GPP target
# (TS 22.125 direct steering: 84-140 B every 40 ms, delivered within 40 ms, 99.9 %)
# while video loads the swarm?
#
# All: 50 UAVs, 1 km, N2 with realistic beacons, ARP pre-filled, 100 ms video deadline,
#      10 C2 drones sending 120 B every 40 ms.
#   video load: 5 x 300 kbps (light), 5 x 600 kbps and 10 x 300 kbps (heavy, ~3 Mbps)
#   scheme:   C = N1 v3 shared window 3 ms every 20 ms (+ 40 ms C2 deadline)
#   x runs 1..5  = 3 x 1 x 5 = 15 runs (scheme B was already measured; it is unchanged)
# Results: ~/ns-3-dev/results/c2final-<date-time>/all-with-tags.csv

set -e
cd ~/ns-3-dev

echo "Building swarm-baseline once before starting..."
./ns3 build swarm-baseline

OUT="$PWD/results/c2final-$(date +%Y%m%d-%H%M)"
mkdir -p "$OUT"
JOBS=$(( $(nproc) - 1 ))
[ "$JOBS" -lt 1 ] && JOBS=1
echo "Running on $JOBS cores. Results folder: $OUT"
export OUT

{
  for load in "5 300kbps" "5 600kbps" "10 300kbps"; do
    for run in 1 2 3 4 5; do
      echo "$load C 1 1 20 3 $run"
    done
  done
} | xargs -P "$JOBS" -L 1 bash -c '
    cams=$0; rate=$1; scheme=$2; prio=$3; win=$4; sf=$5; wms=$6; run=$7
    tag="c$cams-r$rate-$scheme-run$run"
    dir="$OUT/$tag"
    mkdir -p "$dir"
    start=$(date +%s)
    args="--routing=N2 --n2Info=real --beaconLoss=0.1 --gpsErrM=5 --areaSize=1000 --nUav=50 --nCam=$cams --flowRate=$rate --run=$run --arpPrefill=1 --deadlineMs=100 --nC2=10 --c2Prio=$prio --n1c2=$win --n1SfMs=$sf --c2WinMs=$wms"
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
