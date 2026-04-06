#!/bin/bash
# Run all experiment batches for the adaptive area monitoring paper.
#
# Experiments:
#   E1: Baselines comparison (adaptive vs static vs lawnmower vs random)
#   E2: Scalability (3 vs 5 drones)
#   E3: Target density (3, 5, 10 targets)
#   E4: Response time analysis
#   E5: Ablation study (binary_voronoi, all_converge)
#
# Usage:
#   ./run_all_experiments.sh              # run everything
#   ./run_all_experiments.sh --exp e1     # run only E1
#   ./run_all_experiments.sh --exp e5     # run only E5 (ablations)
#   ./run_all_experiments.sh --trials 5   # override trial count

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
EXPERIMENT=""
NUM_TRIALS=10
DURATION=180

while [[ $# -gt 0 ]]; do
    case $1 in
        --exp)    EXPERIMENT=$2; shift 2 ;;
        --trials) NUM_TRIALS=$2; shift 2 ;;
        --duration) DURATION=$2; shift 2 ;;
        *) echo "Unknown: $1"; exit 1 ;;
    esac
done

run_batch() {
    local method=$1
    local drones=$2
    local targets=$3
    local trials=$4
    local duration=$5

    echo ""
    echo "════════════════════════════════════════════════════════"
    echo "  BATCH: method=$method drones=$drones targets=$targets"
    echo "  Trials: $trials x ${duration}s"
    echo "════════════════════════════════════════════════════════"

    for t in $(seq 1 $trials); do
        echo ""
        echo "─── Trial $t / $trials ───"
        bash "$SCRIPT_DIR/run_experiment.sh" \
            --method "$method" \
            --drones "$drones" \
            --targets "$targets" \
            --duration "$duration" \
            --trial "$t" \
            --seed "$((42 + t))"

        echo "Cooling down 10s between trials..."
        sleep 10
    done
}

# ── E1: Baselines comparison ────────────────────────────────────────────────
if [ -z "$EXPERIMENT" ] || [ "$EXPERIMENT" = "e1" ]; then
    echo "╔══════════════════════════════════════════════════╗"
    echo "║  E1: Baselines Comparison (4 methods x $NUM_TRIALS trials) ║"
    echo "╚══════════════════════════════════════════════════╝"
    for method in adaptive static lawnmower random; do
        run_batch "$method" 3 5 "$NUM_TRIALS" "$DURATION"
    done
fi

# ── E2: Scalability ─────────────────────────────────────────────────────────
if [ -z "$EXPERIMENT" ] || [ "$EXPERIMENT" = "e2" ]; then
    echo "╔══════════════════════════════════════════════════╗"
    echo "║  E2: Scalability (3 vs 5 drones, $NUM_TRIALS trials each) ║"
    echo "╚══════════════════════════════════════════════════╝"
    # 3-drone data comes from E1 adaptive runs
    run_batch "adaptive" 5 5 "$NUM_TRIALS" "$DURATION"
fi

# ── E3: Target density ──────────────────────────────────────────────────────
if [ -z "$EXPERIMENT" ] || [ "$EXPERIMENT" = "e3" ]; then
    echo "╔══════════════════════════════════════════════════╗"
    echo "║  E3: Target Density (3, 10 targets, $NUM_TRIALS trials)   ║"
    echo "╚══════════════════════════════════════════════════╝"
    # 5-target data comes from E1 adaptive runs
    run_batch "adaptive" 3 3 "$NUM_TRIALS" "$DURATION"
    run_batch "adaptive" 3 10 "$NUM_TRIALS" "$DURATION"
fi

# ── E4: Response time ───────────────────────────────────────────────────────
if [ -z "$EXPERIMENT" ] || [ "$EXPERIMENT" = "e4" ]; then
    echo "╔══════════════════════════════════════════════════╗"
    echo "║  E4: Response Time ($NUM_TRIALS trials)                    ║"
    echo "╚══════════════════════════════════════════════════╝"
    run_batch "adaptive" 3 5 "$NUM_TRIALS" "$DURATION"
fi

# ── E5: Ablation study ────────────────────────────────────────────────────
if [ -z "$EXPERIMENT" ] || [ "$EXPERIMENT" = "e5" ]; then
    echo "╔══════════════════════════════════════════════════╗"
    echo "║  E5: Ablation Study ($NUM_TRIALS trials each)             ║"
    echo "║  binary_voronoi (no conf weighting)              ║"
    echo "║  all_converge (no split-and-reform)              ║"
    echo "╚══════════════════════════════════════════════════╝"
    # Full adaptive data reused from E1
    run_batch "binary_voronoi" 3 5 "$NUM_TRIALS" "$DURATION"
    run_batch "all_converge" 3 5 "$NUM_TRIALS" "$DURATION"
fi

echo ""
echo "══════════════════════════════════════════════════════════════"
echo "  ALL EXPERIMENTS COMPLETE"
echo "  Results: data/logs/"
echo "══════════════════════════════════════════════════════════════"
