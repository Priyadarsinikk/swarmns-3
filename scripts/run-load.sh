#!/bin/bash
# SwarmNet-IND -- heavy-traffic (funnel) test.
# Question: as more drones stream more video to the ground, does the shared channel near the
# ground station become the bottleneck (collisions, queueing)? If yes, that is N1's job.
#
#   OLSR and N2 (realistic beacons) x 50 UAVs, 1 km
#   camera drones 5 or 10  x  rate 300 kbps, 600 kbps, 1 Mbps  (offered load 1.5 to 10 Mbps)
#   x runs 1..5   = 60 runs.  100 ms deadline and ARP pre-filled for all.
# Results: ~/ns-3-dev/results/load-<date-time>/all.csv

set -e
cd ~/ns-3-dev

echo "Building swarm-baseline once before starting..."
./ns3 build swarm-baseline

OUT="$PWD/results/load-$(date +%Y%m%d-%H%M)"
mkdir -p "$OUT"
JOBS=$(( $(nproc) - 1 ))
[ "$JOBS" -lt 1 ] && JOBS=1
echo "Running on $JOBS cores. Results folder: $OUT"
export OUT

{
  for cams in 5 10; do
    for rate in 300kbps 600kbps 1Mbps; do
      for run in 1 2 3 4 5; do
        echo "OLSR $cams $rate $run"
        echo "N2 $cams $rate $run"
      done
    done
  done
} | xargs -P "$JOBS" -L 1 bash -c '
    routing=$0; cams=$1; rate=$2; run=$3
    tag="$routing-c$cams-r$rate-run$run"
    dir="$OUT/$tag"
    mkdir -p "$dir"
    start=$(date +%s)
    args="--routing=$routing --areaSize=1000 --nUav=50 --nCam=$cams --flowRate=$rate --run=$run --arpPrefill=1 --deadlineMs=100"
    [ "$routing" = "N2" ] && args="$args --n2Info=real --beaconLoss=0.1 --gpsErrM=5"
    timeout 3600 ./ns3 run "swarm-baseline $args" --no-build --cwd="$dir" > "$dir/log.txt" 2>&1 \
      && echo "done   $tag  ($(( $(date +%s) - start )) s)" \
      || echo "FAILED $tag  (see $dir/log.txt)"
  '

echo "routing,nUav,nCam,run,areaSize,arpPrefill,expected,leftSource,received,e2eDeliveryPct,networkPdrPct,meanDelayMs,p95DelayMs,p99DelayMs,maxDelayMs,noRouteSrc,ipNoRoute,ipTtl,ipQueue,ipRouteErr,ipOther,macRetryNear,macRetryFar,macQueueFull,macExpired,arpQueueDrop,arpTimeoutDrop,mac,bands,cols,slotMs,drop0to300,drop300to500,drop500to650,dropOver650,n2Info,beaconLoss,gpsErrM,deadlineMs,n2Hold,connPct,ofPossiblePct,withinDeadlinePct,deadlineDrops" > "$OUT/all.csv"
cat "$OUT"/*/swarm-baseline.csv >> "$OUT/all.csv" 2>/dev/null || true
# the CSV has no flow-rate column, so record it from the folder names alongside
for d in "$OUT"/*/; do
  [ -f "$d/swarm-baseline.csv" ] && echo "$(basename "$d"),$(cat "$d/swarm-baseline.csv")"
done > "$OUT/all-with-tags.csv"

echo
echo "All done. Combined results: $OUT/all-with-tags.csv"
echo "Rows (one per finished run): $(wc -l < "$OUT/all-with-tags.csv")"
