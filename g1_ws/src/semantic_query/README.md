# semantic_query — search for an object by name and mark it (Module 5, M4)

Type **"red cup"** in the AR glasses (or publish it on `/semantic_query/query`): the node searches
the chest OAK-D view with **Grounding DINO + SAM2**, lifts the object into the RTAB-Map `map`
frame with the aligned depth, and marks it with a **3D bounding box + label** in the glasses. The
POI is also published for the Leo handoff (AGENTS.md §12).

```
query ("red cup", "search for a red cup", "where is my bottle?")
   -> search: new OAK-D frames for up to search_timeout_s (20 s)
      -> Grounding DINO (2D box) -> SAM2 (mask) -> aligned depth -> 3D points (optical frame)
      -> TF oak_rgb_camera_optical_frame -> map -> gravity-aligned 3D box
   -> /ar_glasses/markers (CUBE + label, ns "semantic_query")    drawn by g1_ar_bridge
   -> /ar_glasses/reply   ("Searching for red cup…", "Found red cup (0.71), 1.8 m …")
   -> /semantic_query/poi (JSON: id, label, query, xyz, frame_id, confidence, box, stamp)
```

## Interfaces
| Dir | Topic | Type | Notes |
|-----|-------|------|-------|
| in  | `rgb_topic` / `depth_topic` / `info_topic` | Image / Image / CameraInfo | `/oak/rgb/image_raw`, `/oak/stereo/image_raw` (RGB-aligned, 16UC1 mm), `/oak/rgb/camera_info` — in domain 0 via `g1_sensors oak_domain_relay` |
| in  | `/semantic_query/query`, `/ar_glasses/user_command` | `std_msgs/String` | `query_topics`; "stop", "clear" also work |
| out | `/ar_glasses/markers` | `visualization_msgs/MarkerArray` | `ns="semantic_query"`: CUBE (3D box, `text` = label) per query; `ns="semantic_status"`: "searching: …" above the robot |
| out | `/ar_glasses/reply` | `std_msgs/String` | shown in the glasses' assistant panel (`g1_ar_bridge`) |
| out | `/semantic_query/poi` | `std_msgs/String` (JSON) | `{id,label,query,xyz,frame_id:"map",confidence,box{center,size,yaw},points,mask,stamp}` — Leo handoff |
| out | `/semantic_query/image` | `sensor_msgs/Image` | last searched frame with the 2D box and SAM2 mask (RViz) |

## Run
```bash
# with the robot and the glasses (GPU container g1-search, image g1-semantic):
docker build -t g1-semantic -f docker/Dockerfile.semantic .     # once, with internet
bash scripts/start_ar_glasses.sh --tag-size 0.16 --orin --search
# then, in the glasses (menu -> Agent mode), say: "Robot, find the red cup"

# by hand, in a g1-semantic container (docker run --gpus all ... g1-semantic):
ros2 launch semantic_query semantic_query.launch.py
ros2 topic pub --once /semantic_query/query std_msgs/msg/String "{data: 'red cup'}"
ros2 topic echo /ar_glasses/reply

# offline, no GPU (mock detector + synthetic OAK feed + identity TF)
ros2 run semantic_query fake_oak_pub &
ros2 run tf2_ros static_transform_publisher --frame-id map --child-frame-id oak_rgb_camera_optical_frame &
ros2 launch semantic_query semantic_query.launch.py backend:=mock
ros2 topic pub --once /semantic_query/query std_msgs/msg/String "{data: 'red bottle'}"
ros2 topic echo /semantic_query/poi
```

## Detector backends (`backend` param)
- `grounding_dino_sam2` (default) — Grounding DINO (`IDEA-Research/grounding-dino-tiny`) and SAM2
  (`facebook/sam2.1-hiera-small`), both through Hugging Face `transformers` (4.56, `Sam2Model`;
  the `facebookresearch/sam2` package is a fallback). `device: auto` uses the GPU when present.
  Measured on an RTX 5070 Laptop: models load in ~13 s (cached), ~1 s per frame. On failure with
  `allow_mock:=true` the node logs and falls back to the mock, so the graph still runs.
- `mock` — centred box, no model; for offline pipeline tests.

The GPU image `docker/Dockerfile.semantic` (FROM `g1-humble`): PyTorch 2.8 CUDA 12.8 (RTX 50xx),
`transformers==4.56.2`, **Pillow ≥ 10** (Ubuntu 22.04's 9.0.1 breaks the SAM2 processor),
`numpy<2` (ROS Humble's cv_bridge). Weights are cached in `bags/hf_cache` (`HF_HOME`).
Inference runs in its own container and a worker thread, never in a robot-command process
(AGENTS.md §25.3).

## Config (`config/semantic_query.yaml`)
Topics and `optical_frame` (verified 2026-09-27), `depth_scale` (16UC1 mm → `0.001`), detector
thresholds (`box_threshold`, `text_threshold`, `min_confidence` — tune on real scenes: on
synthetic noise Grounding DINO still returns ~0.8), search (`query_topics`, `search_timeout_s`,
`search_period_s`), `robot_frame`.

## Notes
- The 3D box (`box3d.py`): mask pixels → depth within a robust band around the median (drops
  background pixels at the mask edge) → points → `map`; yaw from the main horizontal direction,
  extent from 5–95 % percentiles. One view only sees the front surface, so the box is extended
  away from the camera to about the object's width. Without SAM2 the central half of the 2D box
  is used.
- TF is looked up at the image stamp, falling back to the latest transform (OAK and robot clocks
  can differ).
- Read-only towards the robot: it never moves. Keyframes are best taken while the G1 stands still.
- Tests: `test/test_search_core.py` (commands, 3D box on a rendered cup), `test/test_search_ros.py`
  (ROS end to end with the mock detector).
