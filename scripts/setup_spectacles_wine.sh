#!/usr/bin/env bash
# Rootless, experimental Wine 64-bit runtime for Spectacles (2024) Lens Studio.
set -euo pipefail

repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
wine_dir="$repo_dir/ar_glasses_test/wine"
download_dir="$wine_dir/downloads"
runtime_dir="$wine_dir/runtime"

mkdir -p "$download_dir" "$runtime_dir" "$wine_dir/prefix"

if [ ! -x "$runtime_dir/usr/lib/wine/wine64" ]; then
  for package_name in wine64 libwine libz-mingw-w64 libcapi20-3t64; do
    if ! compgen -G "$download_dir/${package_name}_*.deb" >/dev/null; then
      (cd "$download_dir" && apt download "$package_name")
    fi
  done
  for package in "$download_dir"/*.deb; do
    dpkg-deb -x "$package" "$runtime_dir"
  done
fi

if [ ! -x "$runtime_dir/usr/lib/wine/wine64" ]; then
  echo "Wine loader not found after package extraction." >&2
  exit 1
fi

# Ubuntu's wineserver wrapper contains an absolute /usr/lib/wine path. In a
# rootless extraction, point the private copy directly at its 64-bit binary.
if [ ! -L "$runtime_dir/usr/lib/wine/wineserver" ]; then
  mv "$runtime_dir/usr/lib/wine/wineserver" "$runtime_dir/usr/lib/wine/wineserver.ubuntu"
  ln -s wineserver64 "$runtime_dir/usr/lib/wine/wineserver"
fi

# Debian normally exposes this MinGW DLL through system Wine search paths.
# A private extraction needs it alongside the 64-bit Wine PE DLLs.
if [ ! -e "$runtime_dir/usr/lib/x86_64-linux-gnu/wine/x86_64-windows/zlib1.dll" ]; then
  cp "$runtime_dir/usr/x86_64-w64-mingw32/lib/zlib1.dll" \
    "$runtime_dir/usr/lib/x86_64-linux-gnu/wine/x86_64-windows/zlib1.dll"
fi

echo "Rootless Wine runtime prepared at $runtime_dir"
echo "Use scripts/run_spectacles_wine.sh --version to test it."
echo "Download the Windows Lens Studio 5.15.4 installer yourself from https://ar.snap.com/download/v5-15-4"
