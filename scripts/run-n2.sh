#!/bin/bash
# SwarmNet-IND -- N2 comparison sweep: OLSR (baseline) vs N2 (trajectory-aware make-before-break).
# Fair comparison: both use ARP pre-filled neighbour tables and the same CSMA radio.
#
#   OLSR, N2  x  area 1000 m and 1500 m  x  20, 50, 100 UAVs  x  runs 1..5   = 60 runs
# Results: ~/ns-3-dev/results/n2-<date-time>/all.csv

set -e
cd ~/ns-3-dev

echo "Building swarm-baseline once before starting..."
./ns3 build swarm-baseline

OUT="$PWD/results/n2-$(date +%Y%m%d-%H%M)"
mkdir -p "$OUT"
JOBS=$(( $(nproc) - 1 ))
[ "$JOBS" -lt 1 ] && JOBS=1
echo "Running on $JOBS cores. Results folder: $OUT"
export OUT

{
  for area in 1000 1500; do
    for n in 20 50 100; do
      for run in 1 2 3 4 5; do
        for routing in OLSR N2; do
          echo "$routing $area $n $run"
        done
      done
    done
  done
} | xargs -P "$JOBS" -L 1 bash -c '
    routing=$0; area=$1; n=$2; run=$3
    tag="$routing-a$area-n$n-run$run"
    dir="$OUT/$tag"
    mkdir -p "$dir"
    start=$(date +%s)
    timeout 3600 ./ns3 run "swarm-baseline --routing=$routing --areaSize=$area --nUav=$n --run=$run --arpPrefill=1" \
        --no-build --cwd="$dir" > "$dir/log.txt" 2>&1 \
      && echo "done   $tag  ($(( $(date +%s) - start )) s)" \
      || echo "FAILED $tag  (see $dir/log.txt)"
  '

echo "routing,nUav,nCam,run,areaSize,arpPrefill,expected,leftSource,received,e2eDeliveryPct,networkPdrPct,meanDelayMs,p95DelayMs,p99DelayMs,maxDelayMs,noRouteSrc,ipNoRoute,ipTtl,ipQueue,ipRouteErr,ipOther,macRetryNear,macRetryFar,macQueueFull,macExpired,arpQueueDrop,arpTimeoutDrop,mac,bands,cols,slotMs,drop0to300,drop300to500,drop500to650,dropOver650" > "$OUT/all.csv"
cat "$OUT"/*/swarm-baseline.csv >> "$OUT/all.csv" 2>/dev/null || true

echo
echo "All done. Combined results: $OUT/all.csv"
echo "Rows (one per finished run): $(( $(wc -l < "$OUT/all.csv") - 1 ))"
