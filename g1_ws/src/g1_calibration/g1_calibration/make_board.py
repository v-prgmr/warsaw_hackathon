"""Write a printable chessboard matching calibration.yaml.

    ros2 run g1_calibration make_board --out board.png [--config calibration.yaml] [--dpi 300]

Print at 100 % scale ("actual size", no "fit to page"), glue it FLAT onto a rigid board (foam
board / plywood), then measure the squares and put the measured size in calibration.yaml.
"""
import argparse

import cv2

from .board import Board
from .calibrate_extrinsics import default_config
from .dataset import load_config


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--out", default="board.png")
    ap.add_argument("--config", help="calibration.yaml (default: the installed one)")
    ap.add_argument("--dpi", type=float, default=300.0)
    args = ap.parse_args(argv)
    board = Board.from_config(load_config(args.config or default_config())["board"])
    img, _ = board.render(args.dpi / 0.0254)
    # Label in the bottom margin (outside the pattern) so the board can be identified later.
    label = (f"{board.cols}x{board.rows} inner corners, square {board.square * 1000:.1f} mm. "
             "Origin: top-left inner corner; dark square top-left.")
    if board.border >= 0.01:
        scale = max(0.4, img.shape[1] / 3000)
        cv2.putText(img, label, (int(0.02 * img.shape[1]), img.shape[0] - int(0.3 * board.border
                    * args.dpi / 0.0254)), cv2.FONT_HERSHEY_SIMPLEX, scale, 128, 2)
    cv2.imwrite(args.out, img)
    x0, x1, y0, y1 = board.extent()
    print(f"{args.out}: {img.shape[1]}x{img.shape[0]} px at {args.dpi:.0f} dpi = "
          f"{(x1 - x0) * 100:.1f} x {(y1 - y0) * 100:.1f} cm. {label}")


if __name__ == "__main__":
    main()
