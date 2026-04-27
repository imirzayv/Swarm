#!/bin/bash
# Run the ISTRAS'26 experiment battery.
#
# Two experiment series, distinguished by how targets appear in the world:
#
#   Static series (S1, S2)  — every target spawns at t=0, no schedule. The
#                             swarm sees the whole field from the start;
#                             this isolates coverage efficiency from event
#                             response. See docs/istras_experiments_static.md.
#
#   Dynamic series (D1–D3)  — targets spawn on a timeline over the mission.
#                             Measures event response latency and mid-mission
#                             reconfiguration. See docs/istras_experiments_dynamic.md.
#
# Definitions:
#   S1 — static head-to-head at 3 drones : adaptive, lawnmower, static | 8 targets | 200 s
#   S2 — static scaling to 5 drones      : adaptive, lawnmower         | 8 targets | 200 s
#   D1 — dynamic head-to-head at 3 drones: adaptive, lawnmower, static | sched 25,85,145    | 200 s
#   D2 — dynamic scaling to 5 drones     : adaptive, lawnmower         | sched 25,85,145    | 200 s
#   D3 — dense-event stress at 3 drones  : adaptive, lawnmower         | sched 20,45,75,105,140,175 | 200 s
#
# Geometry for every trial: 400 m × 400 m area, altitude 60 m (camera
# footprint radius ≈ 71 m), class sequence person→vehicle→fire rotated.
#
# Usage:
#   ./run_all_experiments.sh                            # full battery (S1 S2 D1 D2 D3)
#   ./run_all_experiments.sh --experiments S1 S2        # static series only
#   ./run_all_experiments.sh --experiments D1 D2 D3     # dynamic series only
#   ./run_all_experiments.sh --experiments S1 --trials 3
#   ./run_all_experiments.sh --resume                   # skip trials with metrics_summary.json
#   ./run_all_experiments.sh --dry-run

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"

# ── Defaults ─────────────────────────────────────────────────────────────────
EXPERIMENTS=()
TRIALS_OVERRIDE=""
RESUME=false
DRY_RUN=false
OUTPUT_ROOT="$PROJECT_DIR/data/logs/istras"
ALTITUDE=60
AREA_SIZE=400

# ── Per-experiment configuration ─────────────────────────────────────────────
# CFG_SCHEDULE=""         → static (all targets at t=0, count from CFG_TARGETS)
# CFG_SCHEDULE="a,b,c"    → dynamic (comma-separated spawn times in seconds)
declare -A CFG_METHODS=(
    [S1]="adaptive lawnmower static"
    [S2]="adaptive lawnmower"
    [D1]="adaptive lawnmower static"
    [D2]="adaptive lawnmower"
    [D3]="adaptive lawnmower"
)
declare -A CFG_DRONES=(    [S1]=3 [S2]=5 [D1]=3 [D2]=5 [D3]=3 )
declare -A CFG_SCHEDULE=(
    [S1]=""
    [S2]=""
    [D1]="25,85,145"
    [D2]="25,85,145"
    [D3]="20,45,75,105,140,175"
)
declare -A CFG_CLASSES=(
    [S1]="person,vehicle,fire"
    [S2]="person,vehicle,fire"
    [D1]="person,vehicle,fire"
    [D2]="person,vehicle,fire"
    [D3]="person,vehicle,fire,person,vehicle,fire"
)
declare -A CFG_TARGETS=( [S1]=8 [S2]=8 [D1]=3 [D2]=3 [D3]=6 )
declare -A CFG_DURATION=( [S1]=200 [S2]=200 [D1]=200 [D2]=200 [D3]=200 )
declare -A CFG_TRIALS=(   [S1]=5 [S2]=5 [D1]=5 [D2]=5 [D3]=3 )

# Backward-compatible alias table: legacy E1/E2/E3 names map to the new
# dynamic experiments, so existing wrappers keep working.
declare -A LEGACY_ALIAS=( [E1]=D1 [E2]=D2 [E3]=D3 )

# ── Argument parser ─────────────────────────────────────────────────────────
while [[ $# -gt 0 ]]; do
    case $1 in
        --experiments)
            shift
            while [[ $# -gt 0 ]] && [[ ! "$1" =~ ^-- ]]; do
                # Normalize: uppercase, and map E* → D* if needed.
                name=$(echo "$1" | tr '[:lower:]' '[:upper:]')
                if [ -n "${LEGACY_ALIAS[$name]:-}" ]; then
                    name="${LEGACY_ALIAS[$name]}"
                fi
                EXPERIMENTS+=("$name")
                shift
            done
            ;;
        --trials)      TRIALS_OVERRIDE=$2; shift 2 ;;
        --resume)      RESUME=true; shift ;;
        --dry-run)     DRY_RUN=true; shift ;;
        --output-root) OUTPUT_ROOT=$2; shift 2 ;;
        --altitude)    ALTITUDE=$2; shift 2 ;;
        --area-size)   AREA_SIZE=$2; shift 2 ;;
        --exp)
            name=$(echo "$2" | tr '[:lower:]' '[:upper:]')
            if [ -n "${LEGACY_ALIAS[$name]:-}" ]; then
                name="${LEGACY_ALIAS[$name]}"
            fi
            EXPERIMENTS+=("$name"); shift 2 ;;
        *) echo "Unknown option: $1"; exit 1 ;;
    esac
done

if [ ${#EXPERIMENTS[@]} -eq 0 ]; then
    EXPERIMENTS=(S1 S2 D1 D2 D3)
fi

# ── Validate ────────────────────────────────────────────────────────────────
for exp in "${EXPERIMENTS[@]}"; do
    if [ -z "${CFG_METHODS[$exp]:-}" ]; then
        echo "ERROR: Unknown experiment '$exp'. Valid: S1 S2 D1 D2 D3"
        exit 1
    fi
done

# ── Helpers ─────────────────────────────────────────────────────────────────
trial_dir_for() {
    local exp=$1 method=$2 drones=$3 num_targets=$4 trial=$5
    printf "%s/%s/%s_d%d_t%d_trial%02d" \
        "$OUTPUT_ROOT" "$exp" "$method" "$drones" "$num_targets" "$trial"
}

run_trial() {
    local exp=$1 method=$2 drones=$3 schedule=$4 classes=$5
    local num_targets=$6 duration=$7 trial=$8

    local outdir; outdir=$(trial_dir_for "$exp" "$method" "$drones" "$num_targets" "$trial")
    local seed=$((42 + trial))

    if [ "$RESUME" = true ] && [ -f "$outdir/metrics_summary.json" ]; then
        echo "  [skip ] $exp/$method trial $trial — metrics already present"
        return 0
    fi

    echo ""
    echo "──────────────────────────────────────────────────────────────"
    echo "  [$exp] method=$method drones=$drones trial=$trial seed=$seed"
    if [ -n "$schedule" ]; then
        echo "  schedule=$schedule classes=$classes duration=${duration}s"
    else
        echo "  mode=static targets=$num_targets classes=$classes duration=${duration}s"
    fi
    echo "  output=$outdir"
    echo "──────────────────────────────────────────────────────────────"

    if [ "$DRY_RUN" = true ]; then
        echo "  (dry-run — would call run_experiment.sh)"
        return 0
    fi

    # Static mode: pass --targets instead of --schedule.
    local sched_args=()
    if [ -n "$schedule" ]; then
        sched_args=(--schedule "$schedule")
    else
        sched_args=(--targets "$num_targets")
    fi

    bash "$SCRIPT_DIR/run_experiment.sh" \
        --method "$method" \
        --drones "$drones" \
        "${sched_args[@]}" \
        --classes "$classes" \
        --duration "$duration" \
        --trial "$trial" \
        --seed "$seed" \
        --altitude "$ALTITUDE" \
        --area-size "$AREA_SIZE" \
        --output-base "$OUTPUT_ROOT/$exp"

    echo "Cooling down 10s between trials..."
    sleep 10
}

# ── Main loop ────────────────────────────────────────────────────────────────
echo "══════════════════════════════════════════════════════════════"
echo "  ISTRAS'26 experiment battery"
echo "  Experiments: ${EXPERIMENTS[*]}"
echo "  Geometry:    ${AREA_SIZE} m × ${AREA_SIZE} m  @  altitude ${ALTITUDE} m"
echo "  Resume:      $RESUME    Dry-run: $DRY_RUN"
echo "  Output root: $OUTPUT_ROOT"
echo "══════════════════════════════════════════════════════════════"

for exp in "${EXPERIMENTS[@]}"; do
    methods=${CFG_METHODS[$exp]}
    drones=${CFG_DRONES[$exp]}
    schedule=${CFG_SCHEDULE[$exp]}
    classes=${CFG_CLASSES[$exp]}
    num_targets=${CFG_TARGETS[$exp]}
    duration=${CFG_DURATION[$exp]}
    n_trials=${TRIALS_OVERRIDE:-${CFG_TRIALS[$exp]}}

    if [ -n "$schedule" ]; then
        # In dynamic mode, the schedule's length is the number of targets.
        num_targets=$(awk -F',' -v s="$schedule" 'BEGIN{n=split(s,a,","); print n}')
    fi

    echo ""
    echo "╔══════════════════════════════════════════════════════════════╗"
    printf "║  %-60s║\n" "$exp — methods: $methods (drones=$drones, $n_trials trials)"
    echo "╚══════════════════════════════════════════════════════════════╝"

    for method in $methods; do
        for t in $(seq 1 "$n_trials"); do
            run_trial "$exp" "$method" "$drones" "$schedule" "$classes" \
                "$num_targets" "$duration" "$t"
        done
    done
done

echo ""
echo "══════════════════════════════════════════════════════════════"
echo "  ALL EXPERIMENTS COMPLETE"
echo "  Results: $OUTPUT_ROOT"
echo "══════════════════════════════════════════════════════════════"
