# Actual Spectacles bounding-box display test (no Lens Studio)

This is a small WebXR AR page for Spectacles (2024) **Browser Lens**. It draws
one cyan 3D wireframe box 1.5 m in front of your head when AR starts, then
keeps the box in the local tracking world as you move. A pinch/tap mapped to
WebXR `select` repositions it. It is **not** an object detector, camera feed,
AprilTag localizer, or G1 overlay. No pose or image data leaves the glasses.

Snap documents [Browser Lens and WebXR](https://developers.snap.com/spectacles/about-spectacles-features/webxr)
on Spectacles. The actual device behavior of this page has **not** been tested
yet; report what you see so we can adjust it.

## Use it on the glasses

The USB-C connection is not a display cable. The most direct route to try on
this laptop is Android Debug Bridge (ADB) USB port forwarding. I downloaded
Google's platform-tools to `/tmp/spectacles-platform-tools/platform-tools/`;
the sandbox could not inspect the USB device node, so this **must be confirmed
in your normal Ubuntu terminal**. Do not assume Spectacles allow ADB just
because a debug USB interface is visible.

From the repo root, start the local web server and leave it running:

```bash
python3 -m http.server 8787 --bind 127.0.0.1 \
  --directory ar_glasses_test/webxr_bbox
```

In another normal Ubuntu terminal, check whether the device authorizes ADB:

```bash
/tmp/spectacles-platform-tools/platform-tools/adb devices -l
```

If it says `device`, run:

```bash
/tmp/spectacles-platform-tools/platform-tools/adb reverse tcp:8787 tcp:8787
```

Then on the glasses, open **Lens Explorer → Browser Lens**, enter
`http://localhost:8787`, and select **Start AR bounding box**. Browser engines
normally treat `localhost` as a secure context for WebXR, but this exact
Spectacles Browser Lens/ADB combination is unverified. Look at an open area:
the box should appear about 1.5 m ahead and remain in place when you move
your head. Keep the server and USB connection running during the test.

On a second pair, `http://127.0.0.1:8787/index.html` showed a white screen.
The plain-HTTP server logged TLS handshake data instead of a page request at
that time, suggesting Browser Lens upgraded that URL to HTTPS. Try the
explicit `http://localhost:8787/` URL above and check server logs; do not
infer that a white page means the WebXR code ran. A trusted HTTPS host may be
needed if Browser Lens upgrades both loopback forms.

**Do not enter `http://localhost:8787` into Lens Explorer's search box.** That
search finds Lens *names*, so it will say “No lenses found” for a URL. Clear
search, locate and open the **Browser** Lens tile (browse All Lenses if
needed), then enter the URL in the Browser's own address bar. The wearer has
since confirmed the **Browser Lens tile itself is missing**. Until it appears,
this WebXR test cannot run on the glasses. Check whether other Lens Explorer
tiles load, that the glasses are connected to Wi-Fi, and the Snap OS version
and update availability in the Spectacles (2024) companion app. Snap's
[update guide](https://support.spectacles.com/hc/en-us/articles/30214953982740-Updating)
says to open the app, select the Spectacles icon, and choose Software Update.
Do not assume the page is broken or that USB forwarding installs the Browser.

On this laptop, the udev rule has been installed and `adb devices -l` now
reports `Snap_matador` as `device`. The local server and `adb reverse` were
started successfully on 2026-09-26. The on-glasses Browser Lens display has
**not** yet been confirmed; re-run the server and reverse command if either
process or the USB connection is interrupted.

If ADB says `unauthorized`, look for an authorization prompt on the glasses.
If it says `no permissions`, check `id -nG` in your normal terminal. If
`plugdev` is already listed, install the narrow
[Spectacles USB rule](../usb/51-spectacles-adb.rules) with:

```bash
sudo install -m 0644 ar_glasses_test/usb/51-spectacles-adb.rules \
  /etc/udev/rules.d/51-spectacles-adb.rules
sudo udevadm control --reload-rules
```

Unplug/replug the USB-C cable, then rerun `adb devices -l`. If `plugdev` is
*not* in `id -nG`, your login has not activated that group; add it only if
needed and log out/in, per [Android's Ubuntu ADB guide](https://developer.android.com/studio/run/device).
This rule changes host USB permissions only. Do not run ADB as root or use
`chmod 666` on the device. If ADB lists no device after reconnect, this USB
route is unavailable until device-side debug access is enabled. Do not use
`adb install`, `adb shell`, or modify the glasses
system for this display test. If the Browser Lens refuses the localhost URL,
the next route is hosting these two static files at a publicly trusted HTTPS
URL. The current Wi-Fi network blocks both ngrok and Cloudflare Quick Tunnel,
so neither provides a usable URL from here.

If Browser Lens is missing, this no-editor route is unavailable on the current
glasses software. If the page says no `immersive-ar` support or entering AR
fails, note the exact text and your Snap OS version. If the box appears but
moves with your head, local tracking/anchoring needs correction. A box that
stays in the room confirms only display and local tracking; positioning it on
a real object or in the laptop/RTAB map requires registration and camera data.
Read-only `adb shell` attempts to inspect the device returned `error: closed`
on this pair, although `adb devices` and port reverse work; do not rely on
ADB to inspect or install missing Lenses.
