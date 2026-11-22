#!/usr/bin/env bash
# Run an experiment matrix: every run in a fresh simulation (one Docker container each), resumable.
#
#   scripts/run_matrix.sh <matrix.yaml | pilot | m1 | m2>
#
# The runs come from `rapid_nav_tools.experiments` (repeats outer, cells shuffled with the seed).
# Finished runs are listed in experiments/<name>/index.csv; a restart skips the ones with exit 0 and
# a bundle. A failed run is retried once; a second failure is recorded and the runner moves on.
# Stop it with Ctrl+C or `touch experiments/<name>/STOP`: it ends after the current run.
# A run that exceeds three times its expected wall time (3.5 min / rtf) is stopped and counts as a
# failure (exit code "timeout"). It stops cleanly when / has less than MIN_FREE_GIB (default 3) free.
set -uo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
MIN_FREE_GIB="${MIN_FREE_GIB:-3}"
MIN_FREE_KB="$(awk -v g="$MIN_FREE_GIB" 'BEGIN { printf "%d", g * 1024 * 1024 }')"
POLL_S=5

if [ "$#" -ne 1 ]; then
    echo "usage: scripts/run_matrix.sh <matrix.yaml | pilot | m1 | m2>" >&2
    exit 2
fi
matrix="$1"
if [ ! -f "$matrix" ]; then
    matrix="$REPO/src/rapid_nav_tools/config/experiments/$1.yaml"
fi
[ -f "$matrix" ] || { echo "no matrix file: $1" >&2; exit 2; }

helper() {
    PYTHONPATH="$REPO/src/rapid_nav_tools" uv run --quiet --with numpy --with pyyaml \
        python -m rapid_nav_tools.experiments "$@"
}

docker info > /dev/null 2>&1 || { echo "Docker is not running (docker desktop start)" >&2; exit 1; }

name="$(helper name "$matrix")" || exit 2
dir="$REPO/experiments/$name"
mkdir -p "$dir"
cp "$matrix" "$dir/matrix.yaml"
index="$dir/index.csv"
todo="$dir/.pending.tsv"
helper pending "$matrix" "$dir" > "$todo" || exit 2
total="$(helper expand "$matrix" | wc -l | tr -d ' ')"
left="$(wc -l < "$todo" | tr -d ' ')"
echo "matrix $name: $total runs, $left to do, results in experiments/$name/"

interrupted=0
trap 'interrupted=1; echo "stop requested: finishing the current run"' INT TERM
rm -f "$dir/STOP"
marker="$dir/.marker"
retries=0

# One attempt of one run. Sets: exit_code, wall_s, bundle (name inside $dir, may be empty).
attempt() {
    local scenario="$1" controller="$2" hazards="$3" rtf="$4" track="$5" n="$6"
    local tag="" prefix cname log start limit pid rc waited
    local -a cmd
    if [ "$track" = "1" ]; then
        if [ "$controller" != "dwb" ]; then
            echo "track_check runs use dwb, not $controller" >&2
            exit_code="usage"; wall_s=0; bundle=""
            return
        fi
        cmd=(scripts/dev.sh track-smoke "$scenario" --rtf "$rtf")
        prefix="track_check_${scenario}_"
    else
        cmd=(scripts/dev.sh nav-smoke "$scenario" "$controller" "$hazards" --rtf "$rtf")
        [ "$hazards" = "off" ] || tag="_$hazards"
        prefix="nav_run_${scenario}_${controller}${tag}_"
    fi
    cname="rapidnav-exp-${name}-$$-${n}"
    log="$dir/.run.log"
    : > "$marker"
    sleep 1
    start="$(date +%s)"
    # the expected wall time is 3.5 min at rtf 1; the limit is three times that, scaled by 1/rtf
    limit="$(awk -v r="$rtf" 'BEGIN { printf "%d", 3 * 3.5 * 60 / r }')"
    ( trap '' INT; cd "$REPO" && CONTAINER_NAME="$cname" exec "${cmd[@]}" ) > "$log" 2>&1 < /dev/null &
    pid=$!
    exit_code=""
    while kill -0 "$pid" 2> /dev/null; do
        sleep "$POLL_S"
        waited=$(( $(date +%s) - start ))
        if [ "$waited" -gt "$limit" ]; then
            echo "  timeout after ${waited} s: stopping container $cname"
            docker stop -t 10 "$cname" > /dev/null 2>&1
            exit_code="timeout"
            break
        fi
    done
    wait "$pid" 2> /dev/null
    rc=$?
    [ -n "$exit_code" ] || exit_code="$rc"
    wall_s=$(( $(date +%s) - start ))
    bundle=""
    found="$(find "$REPO" -maxdepth 1 -type d -name "${prefix}*" -newer "$marker" | sort | tail -n 1)"
    if [ -n "$found" ]; then
        bundle="$(basename "$found")"
        mv "$found" "$dir/$bundle"
        cp "$log" "$dir/$bundle/runner.log"
    else
        cp "$log" "$dir/failed_${scenario}_${controller}_${hazards}_r${repeat}_a${n}.log"
    fi
}

done_runs=0
while IFS=$'\t' read -r scenario controller hazards repeat rtf track <&3; do
    if [ "$interrupted" -eq 1 ] || [ -f "$dir/STOP" ]; then
        echo "stopped before $scenario $controller $hazards repeat $repeat"
        break
    fi
    free_kb="$(df -k / | awk 'NR==2 { print $4 }')"
    if [ "$free_kb" -lt "$MIN_FREE_KB" ]; then
        echo "less than $MIN_FREE_GIB GiB free on /: stopping cleanly" >&2
        break
    fi
    for n in 1 2; do
        echo "$(date +%H:%M:%S) run $scenario $controller $hazards repeat $repeat (attempt $n)"
        attempt "$scenario" "$controller" "$hazards" "$rtf" "$track" "$n"
        valid=""
        if [ -n "$bundle" ]; then
            valid="$(helper valid "$dir/$bundle" 2> /dev/null || echo 0)"
        fi
        helper append "$index" "$scenario" "$controller" "$hazards" "$repeat" "$bundle" \
            "$exit_code" "$wall_s" "$valid"
        echo "  exit $exit_code, ${wall_s} s, bundle ${bundle:-none}, valid ${valid:-n/a}"
        if [ "$exit_code" = "0" ] && [ -n "$bundle" ]; then
            break
        fi
        retries=$((retries + 1))
        if [ "$interrupted" -eq 1 ] || [ -f "$dir/STOP" ]; then
            break
        fi
    done
    done_runs=$((done_runs + 1))
done 3< "$todo"

rm -f "$todo" "$marker" "$dir/.run.log"
echo "this session: $done_runs runs started, $retries failed attempts"
if [ "$retries" -gt 3 ]; then
    echo "NOTE: more than 3 attempts failed; tell the planner (see index.csv)"
fi
echo "resume with: scripts/dev.sh experiment $1"
