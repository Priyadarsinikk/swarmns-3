#!/bin/bash
# SwarmNet-IND -- the full ablation: which mechanism earns which part of the gain.
#
# Blocks:
#   R  regression          downlink off, published configuration. Must reproduce.
#   L  ablation ladder     OLSR -> OLSR+N2 queue -> PRED -> PRED+hold -> N2 -> N2 full
#   S  safe-fraction sweep N2's link margin, 0.80 to 1.00, in sparse and dense conditions
#   T  LET-threshold sweep the baseline's look-ahead, 0.5 to 3 s (shows it is not handicapped)
#   B  two-way C2          uplink and downlink commands together
#   W  C2 window sweep     3 / 4 / 5 ms with two-way traffic
#   J  jamming            in-band jammers, N2 against OLSR, jammed fraction measured
#
#   bash run-ablation.sh quick        seed 3 only
#   bash run-ablation.sh quick LS     seed 3, ladder and safe-fraction only
#   bash run-ablation.sh             seeds 1-5, all blocks
# Results: ~/ns-3-dev/results/ablation-<date-time>/all-with-tags.csv
set -e
cd ~/ns-3-dev
echo "Building swarm-baseline once before starting..."
./ns3 build swarm-baseline

RUNS="1 2 3 4 5"; [ "$1" = "quick" ] && RUNS="3"
BLOCKS="${2:-RLSTBWJ}"
OUT="$PWD/results/ablation-$(date +%Y%m%d-%H%M)"
mkdir -p "$OUT"
JOBS=$(( $(nproc) - 1 )); [ "$JOBS" -lt 1 ] && JOBS=1
echo "Running on $JOBS cores. Results folder: $OUT"
export OUT

COMMON="--n2Info=real --beaconLoss=0.1 --gpsErrM=5 --arpPrefill=1 --deadlineMs=100 --nC2=10 --c2Prio=1 --n1c2=1 --n1SfMs=20 --c2Copies=1"
export COMMON

{
for run in $RUNS; do
  # The four conditions the proposal reports on
  SPARSE20="--areaSize=1500 --nUav=20 --nCam=5 --flowRate=300kbps"
  SPARSE50="--areaSize=1500 --nUav=50 --nCam=5 --flowRate=300kbps"
  DENSE50="--areaSize=1000 --nUav=50 --nCam=5 --flowRate=300kbps"
  HEAVY50="--areaSize=1000 --nUav=50 --nCam=5 --flowRate=600kbps"
  HEAVYB="--areaSize=1000 --nUav=50 --nCam=10 --flowRate=300kbps"

  case "$BLOCKS" in *R*)
    for L in "light $DENSE50" "heavyA $HEAVY50" "heavyB $HEAVYB"; do
      set -- $L; n=$1; shift
      echo "R-$n-route0-run$run|--routing=N2 $* --run=$run --c2Route=0 --c2Downlink=0"
      echo "R-$n-route1-run$run|--routing=N2 $* --run=$run --c2Route=1 --c2SafeFrac=0.7 --c2Downlink=0"
    done
  ;; esac

  case "$BLOCKS" in *L*)
    for L in "sparse20 $SPARSE20" "sparse50 $SPARSE50" "dense50 $DENSE50" "heavy50 $HEAVY50"; do
      set -- $L; n=$1; shift
      echo "L-$n-1-olsr-run$run|--routing=OLSR $* --run=$run"
      echo "L-$n-2-olsrq-run$run|--routing=OLSR $* --run=$run --baseQueue=1"
      echo "L-$n-3-pred-run$run|--routing=PRED $* --run=$run --letThreshS=1.0"
      echo "L-$n-4-predhold-run$run|--routing=PRED $* --run=$run --letThreshS=1.0 --predHold=1"
      echo "L-$n-5-n2-run$run|--routing=N2 $* --run=$run --c2Route=0"
      echo "L-$n-6-n2full-run$run|--routing=N2 $* --run=$run --c2Route=1 --c2SafeFrac=0.7"
    done
  ;; esac

  case "$BLOCKS" in *S*)
    for sf in 0.80 0.85 0.90 0.95 1.00; do
      echo "S-sparse20-safe$sf-run$run|--routing=N2 $SPARSE20 --run=$run --c2Route=0 --safeFrac=$sf"
      echo "S-sparse50-safe$sf-run$run|--routing=N2 $SPARSE50 --run=$run --c2Route=0 --safeFrac=$sf"
      echo "S-dense50-safe$sf-run$run|--routing=N2 $DENSE50 --run=$run --c2Route=0 --safeFrac=$sf"
      echo "S-heavy50-safe$sf-run$run|--routing=N2 $HEAVY50 --run=$run --c2Route=0 --safeFrac=$sf"
    done
  ;; esac

  case "$BLOCKS" in *T*)
    for t in 0.5 1.0 2.0 3.0; do
      echo "T-sparse20-let$t-run$run|--routing=PRED $SPARSE20 --run=$run --letThreshS=$t --predHold=1"
      echo "T-dense50-let$t-run$run|--routing=PRED $DENSE50 --run=$run --letThreshS=$t --predHold=1"
    done
  ;; esac

  case "$BLOCKS" in *B*)
    for L in "light $DENSE50" "heavyA $HEAVY50" "heavyB $HEAVYB"; do
      set -- $L; n=$1; shift
      echo "B-$n-bidir-n2full-run$run|--routing=N2 $* --run=$run --c2Route=1 --c2SafeFrac=0.7 --c2Downlink=1 --c2DlPos=tracked"
      echo "B-$n-bidir-pred-run$run|--routing=PRED $* --run=$run --letThreshS=1.0 --predHold=1 --c2Downlink=1 --c2DlPos=tracked"
      echo "B-$n-bidir-ideal-run$run|--routing=N2 $* --run=$run --c2Route=1 --c2SafeFrac=0.7 --c2Downlink=1 --c2DlPos=ideal"
    done
  ;; esac

  case "$BLOCKS" in *J*)
    # Jamming. Power sets the denial radius: at 16 dBm the jammer is heard as far as a UAV
    # (365 m), at 12 dBm about 230 m, at 8 dBm about 145 m. Counts and powers are swept so
    # that the jammed fraction, which the model measures rather than assumes, spans the
    # 20% figure that milestone M4 commits to.
    for jp in 8 12 16; do
      echo "J-heavy50-n2-1jam-p$jp-run$run|--routing=N2 $HEAVY50 --run=$run --c2Route=1 --c2SafeFrac=0.7 --nJam=1 --jamPowerDbm=$jp"
      echo "J-heavy50-olsr-1jam-p$jp-run$run|--routing=OLSR $HEAVY50 --run=$run --nJam=1 --jamPowerDbm=$jp"
    done
    for nj in 2 3; do
      echo "J-heavy50-n2-${nj}jam-p12-run$run|--routing=N2 $HEAVY50 --run=$run --c2Route=1 --c2SafeFrac=0.7 --nJam=$nj --jamPowerDbm=12"
      echo "J-heavy50-olsr-${nj}jam-p12-run$run|--routing=OLSR $HEAVY50 --run=$run --nJam=$nj --jamPowerDbm=12"
    done
    echo "J-dense50-n2-2jam-p12-run$run|--routing=N2 $DENSE50 --run=$run --c2Route=1 --c2SafeFrac=0.7 --nJam=2 --jamPowerDbm=12"
    echo "J-dense50-olsr-2jam-p12-run$run|--routing=OLSR $DENSE50 --run=$run --nJam=2 --jamPowerDbm=12"
    echo "J-dense50-n2-0jam-run$run|--routing=N2 $DENSE50 --run=$run --c2Route=1 --c2SafeFrac=0.7 --nJam=0"
    echo "J-dense50-olsr-0jam-run$run|--routing=OLSR $DENSE50 --run=$run --nJam=0"
  ;; esac

  case "$BLOCKS" in *W*)
    for w in 3 4 5 6; do
      echo "W-heavyA-win${w}ms-run$run|--routing=N2 $HEAVY50 --run=$run --c2Route=1 --c2SafeFrac=0.7 --c2Downlink=1 --c2DlPos=tracked --c2WinMs=$w"
    done
  ;; esac
done
} | xargs -P "$JOBS" -I{} bash -c '
    line="{}"; tag="${line%%|*}"; args="${line#*|}"
    dir="$OUT/$tag"; mkdir -p "$dir"; start=$(date +%s)
    timeout 5400 ./ns3 run "swarm-baseline $COMMON $args" --no-build --cwd="$dir" > "$dir/log.txt" 2>&1 \
      && echo "done   $tag  ($(( $(date +%s) - start )) s)" \
      || echo "FAILED $tag  (see $dir/log.txt)"
  '

for d in "$OUT"/*/; do
  [ -f "$d/swarm-baseline.csv" ] && echo "$(basename "$d"),$(cat "$d/swarm-baseline.csv")"
done > "$OUT/all-with-tags.csv"

echo
echo "Combined results: $OUT/all-with-tags.csv"
echo
echo "tag | video within 100 ms % | uplink C2 within 40 ms % | downlink C2 within 40 ms % | connected % | jammed %"
awk -F, '{printf "%s | %.2f | %.2f | %.2f | %.1f | %.1f\n", $1,$44,$56,$74,$42,$90}' "$OUT/all-with-tags.csv" | sort
echo
echo "Means over seeds (strip the run suffix):"
awk -F, '{k=$1; sub(/-run[0-9]+$/,"",k); v[k]+=$44; c[k]+=$56; d[k]+=$74; j[k]+=$90; n[k]++}
         END{for(k in n) printf "%-40s video=%6.2f  c2_up=%6.2f  c2_dn=%6.2f  jammed=%5.1f  (n=%d)\n", k, v[k]/n[k], c[k]/n[k], d[k]/n[k], j[k]/n[k], n[k]}' \
  "$OUT/all-with-tags.csv" | sort
