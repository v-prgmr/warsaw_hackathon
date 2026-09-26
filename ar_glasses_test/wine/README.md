# Experimental Lens Studio on Ubuntu with Wine

This is an **unsupported experiment**, not a validated Spectacles deployment
path. Snap supports Lens Studio 5.15.4 for Spectacles (2024) on Windows 11 and
macOS 12+, not Ubuntu/Wine. The rootless Wine runtime and Wine prefix live in
the ignored `downloads/`, `runtime/`, and `prefix/` directories here. Nothing
is installed system-wide or committed to Git.

From the repo root:

```bash
scripts/setup_spectacles_wine.sh
scripts/run_spectacles_wine.sh --version
```

To try Lens Studio, **you** must obtain the Windows installer from Snap's
[Lens Studio 5.15.4 download page](https://ar.snap.com/download/v5-15-4),
including accepting its license terms. Then run:

```bash
scripts/run_spectacles_wine.sh /absolute/path/to/your/LensStudioInstaller.exe
```

The filename above is a placeholder; use the actual downloaded path. Installer
success, editor startup, Snap login, Spectacles discovery, and wireless Lens
transfer are separate tests. None should be assumed from `wine --version`.

For wireless deployment, Snap requires the glasses and Ubuntu laptop on the
same Wi-Fi/hotspot with working internet and no client isolation. Lens Studio
must use the same Snap account paired to the glasses. Keep the glasses awake
in Lens Explorer. A successful transfer puts the Lens in **Draft**; it does not
require Browser Lens or `localhost`. See Snap's
[connection guide](https://developers.snap.com/spectacles/get-started/start-building/connecting-lens-studio-to-spectacles).
