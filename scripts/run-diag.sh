#!/bin/bash
# SwarmNet-IND -- diagnosis sweep: WHY does the baseline lose packets, and does ARP cause the 2-3 s delays?
#
# Runs (all in parallel, one per core):
#   OLSR  x  20, 50, 100 UAVs  x  runs 1..5  x  ARP normal / ARP pre-filled   = 30 runs
#   AODV  x  100 UAVs          x  runs 1..5  x  ARP normal                    =  5 runs
# Results: ~/ns-3-dev/results/diag-<date-time>/all.csv

set -e
cd ~/ns-3-dev

echo "Building swarm-baseline once before starting..."
./ns3 build swarm-baseline

OUT="$PWD/results/diag-$(date +%Y%m%d-%H%M)"
mkdir -p "$OUT"
JOBS=$(( $(nproc) - 1 ))
[ "$JOBS" -lt 1 ] && JOBS=1
echo "Running on $JOBS cores. Results folder: $OUT"
export OUT

{
  for n in 20 50 100; do
    for run in 1 2 3 4 5; do
      for arp in 0 1; do
        echo "OLSR $n $run $arp"
      done
    done
  done
  for run in 1 2 3 4 5; do
    echo "AODV 100 $run 0"
  done
} | xargs -P "$JOBS" -L 1 bash -c '
    routing=$0; n=$1; run=$2; arp=$3
    dir="$OUT/$routing-n$n-run$run-arp$arp"
    mkdir -p "$dir"
    start=$(date +%s)
    timeout 3600 ./ns3 run "swarm-baseline --routing=$routing --nUav=$n --run=$run --arpPrefill=$arp" \
        --no-build --cwd="$dir" > "$dir/log.txt" 2>&1 \
      && echo "done   $routing n=$n run=$run arp=$arp  ($(( $(date +%s) - start )) s)" \
      || echo "FAILED $routing n=$n run=$run arp=$arp  (see $dir/log.txt)"
  '

echo "routing,nUav,nCam,run,areaSize,arpPrefill,expected,leftSource,received,e2eDeliveryPct,networkPdrPct,meanDelayMs,p95DelayMs,p99DelayMs,maxDelayMs,noRouteSrc,ipNoRoute,ipTtl,ipQueue,ipRouteErr,ipOther,macRetryNear,macRetryFar,macQueueFull,macExpired,arpQueueDrop,arpTimeoutDrop" > "$OUT/all.csv"
cat "$OUT"/*/swarm-baseline.csv >> "$OUT/all.csv" 2>/dev/null || true

echo
echo "All done. Combined results: $OUT/all.csv"
echo "Rows (one per finished run): $(( $(wc -l < "$OUT/all.csv") - 1 ))"
