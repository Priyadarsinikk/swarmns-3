#!/bin/bash
# SwarmNet-IND -- N1 comparison sweep.
# Fair comparison: BOTH sides use ARP pre-filled neighbour tables (the stronger baseline).
#
#   CSMA baseline                 x 20, 50, 100 UAVs x runs 1..5
#   N1, 3 bands,           2 ms   x 20, 50, 100 UAVs x runs 1..5
#   N1, 3 bands,           3 ms   x ...
#   N1, 3 bands x 2 cols,  2 ms   x ...
#   N1, 3 bands x 2 cols,  3 ms   x ...
# = 75 runs. Results: ~/ns-3-dev/results/n1-<date-time>/all.csv

set -e
cd ~/ns-3-dev

echo "Building swarm-baseline once before starting..."
./ns3 build swarm-baseline

OUT="$PWD/results/n1-$(date +%Y%m%d-%H%M)"
mkdir -p "$OUT"
JOBS=$(( $(nproc) - 1 ))
[ "$JOBS" -lt 1 ] && JOBS=1
echo "Running on $JOBS cores. Results folder: $OUT"
export OUT

{
  for n in 20 50 100; do
    for run in 1 2 3 4 5; do
      echo "CSMA 0 0 0 $n $run"
      for cols in 1 2; do
        for slot in 2 3; do
          echo "N1 3 $cols $slot $n $run"
        done
      done
    done
  done
} | xargs -P "$JOBS" -L 1 bash -c '
    mac=$0; bands=$1; cols=$2; slot=$3; n=$4; run=$5
    tag="$mac-b$bands-c$cols-s$slot-n$n-run$run"
    dir="$OUT/$tag"
    mkdir -p "$dir"
    start=$(date +%s)
    args="--routing=OLSR --nUav=$n --run=$run --arpPrefill=1 --mac=$mac"
    [ "$mac" = "N1" ] && args="$args --bands=$bands --cols=$cols --slotMs=$slot"
    timeout 3600 ./ns3 run "swarm-baseline $args" --no-build --cwd="$dir" > "$dir/log.txt" 2>&1 \
      && echo "done   $tag  ($(( $(date +%s) - start )) s)" \
      || echo "FAILED $tag  (see $dir/log.txt)"
  '

echo "routing,nUav,nCam,run,areaSize,arpPrefill,expected,leftSource,received,e2eDeliveryPct,networkPdrPct,meanDelayMs,p95DelayMs,p99DelayMs,maxDelayMs,noRouteSrc,ipNoRoute,ipTtl,ipQueue,ipRouteErr,ipOther,macRetryNear,macRetryFar,macQueueFull,macExpired,arpQueueDrop,arpTimeoutDrop,mac,bands,cols,slotMs,drop0to300,drop300to500,drop500to650,dropOver650" > "$OUT/all.csv"
cat "$OUT"/*/swarm-baseline.csv >> "$OUT/all.csv" 2>/dev/null || true

echo
echo "All done. Combined results: $OUT/all.csv"
echo "Rows (one per finished run): $(( $(wc -l < "$OUT/all.csv") - 1 ))"
