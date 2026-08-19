#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"

HOST="${HOST:-127.0.0.1}"
PORT="${PORT:-8765}"
export MPLCONFIGDIR="${MPLCONFIGDIR:-/tmp/coverage-planner-matplotlib}"

mkdir -p "${MPLCONFIGDIR}"
cd "${PROJECT_ROOT}"

echo "Coverage Planner Web: http://${HOST}:${PORT}"

if command -v uv >/dev/null 2>&1; then
    exec uv run uvicorn coverage_planner.web:app --host "${HOST}" --port "${PORT}"
fi

if [[ -x "${PROJECT_ROOT}/.venv/bin/uvicorn" ]]; then
    exec "${PROJECT_ROOT}/.venv/bin/uvicorn" \
        coverage_planner.web:app --host "${HOST}" --port "${PORT}"
fi

echo "Error: uv and .venv/bin/uvicorn are both unavailable." >&2
exit 1
