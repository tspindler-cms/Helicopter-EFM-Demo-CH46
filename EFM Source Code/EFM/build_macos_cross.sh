#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TOOLCHAIN_BIN="${CXX:-x86_64-w64-mingw32-g++}"

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  cat <<'EOF'
Usage: ./build_macos_cross.sh [make args...]

Cross-builds the CH46D Windows DLL from macOS using MinGW-w64.

Examples:
  ./build_macos_cross.sh
  ./build_macos_cross.sh clean all
  CXX=x86_64-w64-mingw32-g++ ./build_macos_cross.sh -j4

Prerequisite (macOS):
  brew install mingw-w64
EOF
  exit 0
fi

if [[ "$(uname -s)" != "Darwin" ]]; then
  echo "warning: This helper is intended for macOS (Darwin). Continuing anyway." >&2
fi

if ! command -v make >/dev/null 2>&1; then
  echo "error: make not found in PATH." >&2
  exit 1
fi

if ! command -v "${TOOLCHAIN_BIN}" >/dev/null 2>&1; then
  cat >&2 <<EOF
error: ${TOOLCHAIN_BIN} not found in PATH.
Install MinGW-w64 first:
  brew install mingw-w64
EOF
  exit 1
fi

echo "Using toolchain: ${TOOLCHAIN_BIN}"
make -C "${SCRIPT_DIR}" CXX="${TOOLCHAIN_BIN}" "${@:-all}"
