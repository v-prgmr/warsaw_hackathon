# Spectacles visual smoke test

This folder is a **display-only experiment** for Snap AR glasses. It does not
connect to ROS, RTAB-Map, the G1, or a robot controller. The first goal is simply
to see custom text and a 3D object on the glasses and confirm world tracking.

## Important platform limitation

The Ubuntu laptop detects a USB device named `Spectacles` (`05c6:6769`), but
that USB connection is not an arbitrary video display. The custom Lens route
uses Snap's **Lens Studio** to build and send content to the device. Snap also
documents a built-in **Browser Lens** that can open hosted WebXR pages without
Lens Studio for simpler AR experiments. Lens Studio's supported desktop
systems are Windows and macOS, not Linux.
The USB ID alone does not establish the generation; the team has confirmed
these are **Spectacles (2024)**.

- For **Spectacles (2024)**, Snap currently directs developers to Lens Studio
  **5.15.4**, not the latest SPECS version.
- For **SPECS 27**, use the compatible version listed on Snap's download page.
- Check the Snap OS version in the Spectacles companion app; Snap's download
  page lists the minimum version compatible with Lens Studio 5.15+.

Official references: [Lens Studio download and compatibility](https://ar.snap.com/download),
[Spectacles Lens setup](https://developers.snap.com/spectacles/get-started/start-building/spectacles-lens-setup),
[wired connection](https://developers.snap.com/spectacles/get-started/start-building/connecting-lens-studio-to-spectacles),
[WebXR Browser Lens](https://developers.snap.com/spectacles/about-spectacles-features/webxr).

Without an editor, you can immediately open **Browser Lens** in the glasses'
Lens Explorer and visit a [Snap-listed WebXR demo](https://developers.snap.com/spectacles/about-spectacles-features/webxr).
This is a display/local-tracking smoke test, not our custom `HELLO G1` Lens or
the AprilTag-to-laptop localization test. The documented WebXR feature list
does not establish raw camera access for our host AprilTag detector.

## Test A: show our own text

On a supported Windows or macOS computer:

1. Install the Lens Studio version appropriate for the glasses. Power on and
   pair the glasses using their companion app.
2. Create a new **Spectacles Starter/Base** Lens project in Lens Studio. Save it
   under this folder as `lens_project/` if possible, so the project can be shared.
3. Add a `Text3D` object using `Scene Hierarchy > +`. Put it about 80 cm in front
   of the camera and make it comfortably readable. Parent it to the **Camera**
   for this first, head-following test.
4. Import [`hello_g1.js`](hello_g1.js) as a script asset and attach it to a scene
   object. In the script Inspector, assign the `Text3D` component to `label`.
5. Preview the Lens. The text should read `HELLO G1`. Change that string in the
   script and preview again to confirm that our own code controls the content.

Lens Studio supports [Text3D on Spectacles](https://developers.snap.com/lens-studio/features/text/3d-text).

## Test B: keep an object in the world

1. Add a **Device Tracking** component to the Camera and select **World** mode.
2. Add a small, bright 3D cube (or another primitive) at the scene root, **not**
   as a child of the Camera. Place it roughly 1–2 m in front of the starting
   viewpoint. Lens Studio's World-tracking units are centimetres.
3. Preview and then wear the glasses. The text should follow your head; the
   cube should stay approximately fixed in the room as you move around it.

This tests only the glasses' *local* world. It does **not** align their world to
RTAB-Map's `map` frame. See [Device Tracking](https://developers.snap.com/lens-studio/features/ar-tracking/world/tracking-modes).

## Send to the glasses

In the companion app, enable **Developer Settings → Lens Development → Enable
Wired Connectivity**. Connect the powered-on glasses to the Windows/macOS
computer by USB-C, set the Lens project to **Made for Spectacles**, and use
Lens Studio's **Preview Lens / Send to Spectacles** action. The Lens should
appear in the glasses' **Draft** section. USB detection on Ubuntu is not by
itself proof that Lens Studio can deploy to them.

## Acceptance checklist

- [ ] Model and matching Lens Studio version recorded.
- [ ] Lens Studio preview shows `HELLO G1` and the cube.
- [ ] Both appear in the glasses' display.
- [ ] Text follows head motion; cube stays in the local world.
- [ ] Changing `hello_g1.js` and resending updates the displayed text.

After these pass, a separate experiment can receive the G1's live pose/POIs
and register the glasses' local tracking frame to RTAB-Map `map`. Keep that
integration out of this smoke test so failures are easy to isolate.

The ROS-side [localization prototype](../docs/spectacles_localization_architecture.md)
now provides that separate registration experiment and an offline mock; it
still needs a real Lens/camera/pose bridge before physical testing.
For a G1-free, laptop-frame test with exact Ubuntu commands and a printable
AprilTag, see [laptop anchor](laptop_anchor/README.md).
