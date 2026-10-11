#!/usr/bin/env bash
# ==============================================================================
# SAM Scaled Discovery Service Controller (Full 267.8k Images)
#
# Commands:
#   ./scripts/manage_sam_discovery.sh start   - Start background daemon
#   ./scripts/manage_sam_discovery.sh pause   - Freeze process immediately (SIGSTOP)
#   ./scripts/manage_sam_discovery.sh resume  - Resume frozen process (SIGCONT)
#   ./scripts/manage_sam_discovery.sh stop    - Graceful shutdown & flush DB (SIGINT)
#   ./scripts/manage_sam_discovery.sh status  - Check progress, DB stats & GPU
#   ./scripts/manage_sam_discovery.sh follow  - Tail live progress log
#   ./scripts/manage_sam_discovery.sh run     - Run interactively in terminal
# ==============================================================================

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

PID_FILE="${PROJECT_ROOT}/artifacts/pseudo_labels/discovery.pid"
LOG_FILE="${PROJECT_ROOT}/artifacts/pseudo_labels/discovery.log"
DB_FILE="${DB_FILE:-${PROJECT_ROOT}/artifacts/pseudo_labels/sam_discovery_enriched.sqlite}"
TOTAL_IMAGES="${TOTAL_IMAGES:-14000}"

mkdir -p "${PROJECT_ROOT}/artifacts/pseudo_labels"

is_pid_running() {
    local pid="$1"
    if kill -0 "${pid}" 2>/dev/null; then
        return 0
    else
        return 1
    fi
}

get_process_state() {
    local pid="$1"
    if [ -f "/proc/${pid}/status" ]; then
        grep "State:" "/proc/${pid}/status" | awk '{print $2}'
    else
        echo "DEAD"
    fi
}

get_db_count() {
    if [ -f "${DB_FILE}" ]; then
        sqlite3 "${DB_FILE}" "SELECT count(*) FROM images;" 2>/dev/null || echo "0"
    else
        echo "0"
    fi
}

get_cohort_completed_count() {
    if [ -f "${DB_FILE}" ] && [ "${TOTAL_IMAGES}" -le 14000 ]; then
        uv run python -c "
import sqlite3
from student_detector.human_gt_data import DEFAULT_ENRICHED_GT_INDEX_PATH, get_stage2_cohort_ids
cohort_ids = get_stage2_cohort_ids(DEFAULT_ENRICHED_GT_INDEX_PATH, target_count=14000)
conn = sqlite3.connect('file:${DB_FILE}?mode=ro', uri=True)
db_ids = set(r[0] for r in conn.execute('SELECT image_id FROM images').fetchall())
print(len(cohort_ids.intersection(db_ids)))
" 2>/dev/null || get_db_count
    else
        get_db_count
    fi
}

get_db_positive_count() {
    if [ -f "${DB_FILE}" ]; then
        sqlite3 "${DB_FILE}" "SELECT count(*) FROM annotations WHERE is_positive=1;" 2>/dev/null || echo "0"
    else
        echo "0"
    fi
}

get_db_novel_count() {
    if [ -f "${DB_FILE}" ]; then
        sqlite3 "${DB_FILE}" "SELECT count(*) FROM annotations WHERE is_discovered=1 AND is_positive=1;" 2>/dev/null || echo "0"
    else
        echo "0"
    fi
}

cmd_start() {
    if [ -f "${PID_FILE}" ]; then
        local existing_pid
        existing_pid=$(cat "${PID_FILE}")
        if is_pid_running "${existing_pid}"; then
            local state
            state=$(get_process_state "${existing_pid}")
            if [ "${state}" = "T" ]; then
                echo "Process (PID ${existing_pid}) is currently PAUSED. Resuming it now..."
                kill -CONT "${existing_pid}"
                echo "Resumed process ${existing_pid}."
                return 0
            else
                echo "SAM discovery is ALREADY running (PID ${existing_pid}, state: ${state})."
                echo "To view live output: ${0} follow"
                return 0
            fi
        else
            echo "Stale PID file found (${existing_pid}), cleaning up..."
            rm -f "${PID_FILE}"
        fi
    fi

    echo "=================================================================="
    echo "Starting SAM Scaled Discovery Daemon across ${TOTAL_IMAGES} images..."
    echo "Output DB: ${DB_FILE}"
    echo "Log file:  ${LOG_FILE}"
    echo "=================================================================="

    cd "${PROJECT_ROOT}"
    local EXTRA_ARGS="${1:-}"
    if [ -z "${EXTRA_ARGS}" ]; then
        if [ "${TOTAL_IMAGES}" -le 14000 ]; then
            EXTRA_ARGS="--index-db /home/sashamikoff/git/skillup/data/visual-inference-datasets/output/indexes/quad_train_enriched.sqlite --num-images ${TOTAL_IMAGES} --output-db ${DB_FILE}"
        else
            EXTRA_ARGS="--all-images --output-db ${DB_FILE}"
        fi
    fi
    setsid .venv/bin/python3 -u scripts/run_sam_pilot_discovery.py \
        ${EXTRA_ARGS} \
        --save-interval 25 \
        --chunk-size 250 \
        >> "${LOG_FILE}" 2>&1 &

    local new_pid=$!
    echo "${new_pid}" > "${PID_FILE}"

    sleep 2
    if is_pid_running "${new_pid}"; then
        echo "Daemon successfully launched with PID: ${new_pid}"
        echo "Use '${0} status' to monitor progress or '${0} follow' to stream logs."
    else
        echo "ERROR: Process failed to start. Last log entries:"
        tail -n 25 "${LOG_FILE}"
        rm -f "${PID_FILE}"
        exit 1
    fi
}

cmd_pause() {
    if [ ! -f "${PID_FILE}" ]; then
        echo "No running discovery process found (PID file missing)."
        return 1
    fi

    local pid
    pid=$(cat "${PID_FILE}")
    if ! is_pid_running "${pid}"; then
        echo "Process ${pid} is not running."
        rm -f "${PID_FILE}"
        return 1
    fi

    local state
    state=$(get_process_state "${pid}")
    if [ "${state}" = "T" ]; then
        echo "Process ${pid} is ALREADY paused."
        return 0
    fi

    echo "Freezing process ${pid} (sending SIGSTOP)..."
    kill -STOP "${pid}"
    echo "Process ${pid} is now PAUSED. GPU and CPU compute are frozen."
    echo "Use '${0} resume' to unfreeze execution instantly."
}

cmd_resume() {
    if [ ! -f "${PID_FILE}" ]; then
        echo "No PID file found. To start from last saved SQLite checkpoint, run: ${0} start"
        return 1
    fi

    local pid
    pid=$(cat "${PID_FILE}")
    if ! is_pid_running "${pid}"; then
        echo "Process ${pid} is not active. Starting new instance from SQLite checkpoint..."
        rm -f "${PID_FILE}"
        cmd_start
        return 0
    fi

    echo "Unfreezing process ${pid} (sending SIGCONT)..."
    kill -CONT "${pid}"
    echo "Process ${pid} is now ACTIVE and running."
}

cmd_stop() {
    if [ ! -f "${PID_FILE}" ]; then
        echo "No discovery process running."
        return 0
    fi

    local pid
    pid=$(cat "${PID_FILE}")
    if ! is_pid_running "${pid}"; then
        echo "Process ${pid} was not running. Cleaning PID file."
        rm -f "${PID_FILE}"
        return 0
    fi

    # If it was paused, resume it first so it can process the signal
    local state
    state=$(get_process_state "${pid}")
    if [ "${state}" = "T" ]; then
        kill -CONT "${pid}" 2>/dev/null || true
    fi

    echo "Sending graceful shutdown signal (SIGINT) to PID ${pid}..."
    echo "The daemon will finish the current image and flush pending transactions to SQLite..."
    kill -INT "${pid}" 2>/dev/null || true

    local count=0
    while is_pid_running "${pid}"; do
        sleep 1
        count=$((count + 1))
        if [ ${count} -ge 20 ]; then
            echo "Process taking long to flush, sending SIGTERM..."
            kill -TERM "${pid}" 2>/dev/null || true
            sleep 2
            break
        fi
    done

    rm -f "${PID_FILE}"
    echo "Daemon stopped cleanly. Database transactions are synced."
    echo "Current DB status: $(get_db_count)/${TOTAL_IMAGES} images processed."
}

cmd_status() {
    echo "=================================================================="
    echo "SAM SCALED DISCOVERY SERVICE STATUS"
    echo "=================================================================="

    local is_active=false
    local proc_state="STOPPED"
    local pid="NONE"

    if [ -f "${PID_FILE}" ]; then
        pid=$(cat "${PID_FILE}")
        if is_pid_running "${pid}"; then
            is_active=true
            local state
            state=$(get_process_state "${pid}")
            if [ "${state}" = "T" ]; then
                proc_state="PAUSED (Frozen via SIGSTOP)"
            else
                proc_state="RUNNING (Active background daemon)"
            fi
        fi
    fi

    echo "Process State:    ${proc_state} (PID: ${pid})"

    local cohort_completed
    cohort_completed=$(get_cohort_completed_count)
    local db_imgs
    db_imgs=$(get_db_count)
    local db_pos
    db_pos=$(get_db_positive_count)
    local db_novel
    db_novel=$(get_db_novel_count)

    local pct="0.0"
    if [ "${TOTAL_IMAGES}" -gt 0 ] && [ "${cohort_completed}" -gt 0 ]; then
        pct=$(awk "BEGIN {printf \"%.2f\", (${cohort_completed} / ${TOTAL_IMAGES}) * 100}")
    fi

    echo "Database:         ${DB_FILE}"
    echo "Cohort Completed: ${cohort_completed} / ${TOTAL_IMAGES} (${pct}%)"
    echo "Total DB Images:  ${db_imgs}"
    echo "Positive Objects: ${db_pos}"
    echo "Novel SAM Discov: ${db_novel}"

    if command -v nvidia-smi >/dev/null 2>&1; then
        echo "------------------------------------------------------------------"
        echo "GPU Utilization:"
        nvidia-smi --query-gpu=name,utilization.gpu,memory.used,memory.total,temperature.gpu --format=csv,noheader || true
    fi

    echo "------------------------------------------------------------------"
    echo "Latest Log Output (${LOG_FILE}):"
    if [ -f "${LOG_FILE}" ]; then
        tail -n 8 "${LOG_FILE}"
    else
        echo "(Log file does not exist yet)"
    fi
    echo "=================================================================="
}

cmd_follow() {
    if [ ! -f "${LOG_FILE}" ]; then
        echo "Log file does not exist yet. Run '${0} start' first."
        exit 1
    fi
    echo "Streaming live log (${LOG_FILE}) - Press Ctrl+C to exit viewing..."
    exec tail -n 25 -f "${LOG_FILE}"
}

cmd_run() {
    local EXTRA_ARGS="${1:-}"
    if [ -z "${EXTRA_ARGS}" ]; then
        if [ "${TOTAL_IMAGES}" -le 14000 ]; then
            EXTRA_ARGS="--index-db /home/sashamikoff/git/skillup/data/visual-inference-datasets/output/indexes/quad_train_enriched.sqlite --num-images ${TOTAL_IMAGES} --output-db ${DB_FILE}"
        else
            EXTRA_ARGS="--all-images --output-db ${DB_FILE}"
        fi
    fi
    echo "Starting SAM Discovery interactively in foreground..."
    cd "${PROJECT_ROOT}"
    exec uv run python scripts/run_sam_pilot_discovery.py \
        ${EXTRA_ARGS} \
        --save-interval 25 \
        --chunk-size 250
}

case "${1:-status}" in
    start)
        cmd_start
        ;;
    pause)
        cmd_pause
        ;;
    resume)
        cmd_resume
        ;;
    stop)
        cmd_stop
        ;;
    status)
        cmd_status
        ;;
    follow)
        cmd_follow
        ;;
    run)
        cmd_run
        ;;
    *)
        echo "Usage: $0 {start|pause|resume|stop|status|follow|run}"
        exit 1
        ;;
esac
