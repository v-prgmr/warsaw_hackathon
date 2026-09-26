# g1_semantic_map handoff (2026-09-26)

This new, independent ROS 2 Humble package projects rectified OAK-D RGB and
COCO instance masks onto **RTAB-Map's existing LiDAR `/cloud_map`**. It publishes
`/g1_semantic_map/cloud` in the input cloud frame with XYZ, packed RGB, COCO
`label`, and `confidence`; the optional latched `/g1_semantic_map/class_names`
contains the ID-to-name dictionary. It never publishes TF or changes RTAB-Map.
See `README.md` for exact frame convention, launch arguments and limitations.

Current verification: five ROS-independent unit tests pass. In the existing
`g1-humble:latest` Docker image, `colcon build` passes and the synthetic ROS
scene plus fixture-only oracle detector verifies 3,072 visible colored points,
872 labeled points (432 chair, 440 dining table), and 100 hidden points left
gray/unlabeled. This is **not** a test of real COCO inference or OAK-D hardware.

Pending before real use: verify OAK-D RGB/depth/CameraInfo topics; publish the
chessboard-calibrated OAK optical TF; confirm RGB rectification, aligned depth,
timestamp synchronization, and overlay in RViz; install compatible PyTorch and
torchvision; benchmark Mask R-CNN latency and class accuracy on real frames.
No OAK-D was visible on the local USB bus during this work. Do not hardcode
unverified topic/frame names into the mapping stack.

Git state caveat: the workspace `.git` directory is read-only to this agent.
The intended `main -> feature/oakd-semantic-color-map` branch must be created
by a human before these currently untracked files are committed. Once on that
branch, add a short status pointer to the repository-root `AGENTS.md`.
