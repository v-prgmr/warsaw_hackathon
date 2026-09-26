#!/usr/bin/env bash
# Run a Windows executable with the isolated Wine runtime/prefix in this repo.
set -euo pipefail

repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
wine_dir="$repo_dir/ar_glasses_test/wine"
runtime_dir="$wine_dir/runtime"
wine_loader="$runtime_dir/usr/lib/wine/wine64"

if [ ! -x "$wine_loader" ]; then
  echo "Wine is not prepared. Run scripts/setup_spectacles_wine.sh first." >&2
  exit 1
fi

if [ "$#" -eq 0 ]; then
  echo "Usage: $0 <Windows-installer-or-program.exe> [arguments...]" >&2
  exit 2
fi

export WINEPREFIX="$wine_dir/prefix"
export WINEARCH=win64
export WINELOADER="$wine_loader"
export WINESERVER="$runtime_dir/usr/lib/wine/wineserver64"
export LD_LIBRARY_PATH="$runtime_dir/usr/lib/x86_64-linux-gnu:$runtime_dir/usr/lib/x86_64-linux-gnu/wine:${LD_LIBRARY_PATH:-}"

# This sandbox cannot write /run/user/$UID; keep Wine's socket private here.
wine_run_dir="$wine_dir/session"
mkdir -p "$wine_run_dir"
chmod 700 "$wine_run_dir"
export XDG_RUNTIME_DIR="$wine_run_dir"

exec "$wine_loader" "$@"
