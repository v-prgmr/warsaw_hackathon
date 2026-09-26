#!/usr/bin/env python3
"""Generate a print-scale AprilTag 36h11 ID 0 SVG in AprilRobotics orientation."""

from pathlib import Path

import cv2


BLACK_EDGE_MM = 160
QUIET_MARGIN_MM = 10
TAG_ID = 0


def main():
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    modules = int(dictionary.markerSize) + 2  # 6x6 data plus one black border
    pixels = modules * 100
    if hasattr(cv2.aruco, "generateImageMarker"):
        marker = cv2.aruco.generateImageMarker(dictionary, TAG_ID, pixels)
    else:
        marker = cv2.aruco.drawMarker(dictionary, TAG_ID, pixels)
    # OpenCV's generated ID 0 is 180 degrees from AprilRobotics' canonical
    # tag36_11_00000.png. Both detect as ID 0, but matching its orientation
    # avoids a silent 180-degree tag-frame assumption when measuring the mount.
    marker = cv2.rotate(marker, cv2.ROTATE_180)
    module_mm = BLACK_EDGE_MM / modules
    sheet_mm = BLACK_EDGE_MM + 2 * QUIET_MARGIN_MM
    parts = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{sheet_mm}mm" '
        f'height="{sheet_mm}mm" viewBox="0 0 {sheet_mm} {sheet_mm}">',
        '<desc>AprilRobotics-orientation AprilTag 36h11 ID 0; black square edge 160 mm at 100% print scale.</desc>',
        f'<rect width="{sheet_mm}" height="{sheet_mm}" fill="white"/>',
    ]
    for row in range(modules):
        for col in range(modules):
            if marker[row * 100 + 50, col * 100 + 50] < 128:
                x = QUIET_MARGIN_MM + col * module_mm
                y = QUIET_MARGIN_MM + row * module_mm
                parts.append(
                    f'<rect x="{x:g}" y="{y:g}" width="{module_mm:g}" '
                    f'height="{module_mm:g}" fill="black"/>'
                )
    parts.append("</svg>")
    target = Path(__file__).with_name("apriltag_36h11_id0_160mm.svg")
    target.write_text("\n".join(parts) + "\n", encoding="utf-8")
    print(target)


if __name__ == "__main__":
    main()
