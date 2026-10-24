#!/bin/bash
# SwarmNet-IND -- final preliminary-results sweep (version 6 of the simulation).
# All runs: ARP pre-filled, 100 ms deadline (applied to every scheme), same flights per run number.
#
#   OLSR                                              x 1 km, 1.5 km x 20, 50, 100 UAVs x runs 1..5
#   N2, ideal information                             x ...
#   N2, realistic information (10% beacon loss, 5 m GPS error, beacons up to 100 ms old)  x ...
#   N2, harsh information (30% loss, 10 m GPS error)  x 1 km, 1.5 km x 50 UAVs x runs 1..5
# = 100 runs. Results: ~/ns-3-dev/results/final-<date-time>/all.csv

set -e
cd ~/ns-3-dev

echo "Building swarm-baseline once before starting..."
./ns3 build swarm-baseline

OUT="$PWD/results/final-$(date +%Y%m%d-%H%M)"
mkdir -p "$OUT"
JOBS=$(( $(nproc) - 1 ))
[ "$JOBS" -lt 1 ] && JOBS=1
echo "Running on $JOBS cores. Results folder: $OUT"
export OUT

{
  for area in 1000 1500; do
    for n in 20 50 100; do
      for run in 1 2 3 4 5; do
        echo "OLSR ideal 0 0 $area $n $run"
        echo "N2 ideal 0 0 $area $n $run"
        echo "N2 real 0.1 5 $area $n $run"
      done
    done
    for run in 1 2 3 4 5; do
      echo "N2 real 0.3 10 $area 50 $run"
    done
  done
} | xargs -P "$JOBS" -L 1 bash -c '
    routing=$0; info=$1; loss=$2; gps=$3; area=$4; n=$5; run=$6
    tag="$routing-$info-l$loss-g$gps-a$area-n$n-run$run"
    dir="$OUT/$tag"
    mkdir -p "$dir"
    start=$(date +%s)
    args="--routing=$routing --areaSize=$area --nUav=$n --run=$run --arpPrefill=1 --deadlineMs=100"
    [ "$routing" = "N2" ] && args="$args --n2Info=$info --beaconLoss=$loss --gpsErrM=$gps"
    timeout 3600 ./ns3 run "swarm-baseline $args" --no-build --cwd="$dir" > "$dir/log.txt" 2>&1 \
      && echo "done   $tag  ($(( $(date +%s) - start )) s)" \
      || echo "FAILED $tag  (see $dir/log.txt)"
  '

echo "routing,nUav,nCam,run,areaSize,arpPrefill,expected,leftSource,received,e2eDeliveryPct,networkPdrPct,meanDelayMs,p95DelayMs,p99DelayMs,maxDelayMs,noRouteSrc,ipNoRoute,ipTtl,ipQueue,ipRouteErr,ipOther,macRetryNear,macRetryFar,macQueueFull,macExpired,arpQueueDrop,arpTimeoutDrop,mac,bands,cols,slotMs,drop0to300,drop300to500,drop500to650,dropOver650,n2Info,beaconLoss,gpsErrM,deadlineMs,n2Hold,connPct,ofPossiblePct,withinDeadlinePct,deadlineDrops" > "$OUT/all.csv"
cat "$OUT"/*/swarm-baseline.csv >> "$OUT/all.csv" 2>/dev/null || true

echo
echo "All done. Combined results: $OUT/all.csv"
echo "Rows (one per finished run): $(( $(wc -l < "$OUT/all.csv") - 1 ))"
