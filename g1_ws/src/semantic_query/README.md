# semantic_query — NL object detection → 3D POI (Module 5)

On-demand: send a text query, get a metric **3D Point of Interest** in the `map` frame for the AR
glasses and the Leo handoff. Detects in the OAK-D RGB view (Grounding DINO + SAM2), backprojects
with the aligned depth, transforms into `map`.

```
/semantic_query/query (String "red bottle")
   -> Grounding DINO (2D box) -> SAM2 (mask) -> backproject(aligned depth, K)
   -> TF oak_rgb_camera_optical_frame -> map
   -> /ar_glasses/markers (MarkerArray, ns="semantic_query")  +  /semantic_query/poi (JSON)
```

## Interfaces
| Dir | Topic | Type | Notes |
|-----|-------|------|-------|
| in  | `rgb_topic` / `depth_topic` / `info_topic` | Image / Image / CameraInfo | OAK-D; aligned depth; **verify names on hardware** |
| in  | `/semantic_query/query` | `std_msgs/String` | the NL query, e.g. `red bottle` |
| out | `/ar_glasses/markers` | `visualization_msgs/MarkerArray` | `ns="semantic_query"`: SPHERE + TEXT at the POI |
| out | `/semantic_query/poi` | `std_msgs/String` (JSON) | `{id,label,xyz,frame_id:"map",confidence,stamp}` — Leo handoff |

Composes with `g1_ar_bridge` (same `/ar_glasses/markers`, distinct `ns`).

## Run
```bash
# real (CUDA host with weights): OAK-D + g1_sensors tf_chain must be up
ros2 launch semantic_query semantic_query.launch.py
ros2 topic pub --once /semantic_query/query std_msgs/msg/String "{data: 'red bottle'}"

# offline, no GPU (mock detector + synthetic OAK feed + identity TF)
ros2 run semantic_query fake_oak_pub &
ros2 run tf2_ros static_transform_publisher --frame-id map --child-frame-id oak_rgb_camera_optical_frame &
ros2 launch semantic_query semantic_query.launch.py backend:=mock
ros2 topic pub --once /semantic_query/query std_msgs/msg/String "{data: 'red bottle'}"
ros2 topic echo /semantic_query/poi
```

## Detector backends (`backend` param)
- `grounding_dino_sam2` (default) — Grounding DINO via HF `transformers`
  (`IDEA-Research/grounding-dino-tiny`) + optional SAM2 (`facebook/sam2-hiera-small`). Needs
  `torch`, `transformers`, `sam2`, CUDA. Heavy imports are lazy; on failure with `allow_mock:=true`
  the node logs and falls back to the mock, so the graph still runs.
- `mock` — centred box, no model; for offline pipeline tests.

Install on the GPU host: `pip install torch transformers` and SAM2 from `facebookresearch/sam2`;
cache weights offline. Set `device`, `box_threshold`, `text_threshold`, `min_confidence` in
`config/semantic_query.yaml`.

## Config to verify on hardware
`config/semantic_query.yaml`: OAK-D topic names (depthai defaults assumed), `optical_frame`
(`oak_rgb_camera_optical_frame` from `g1_sensors/tf_chain`), and `depth_scale` (16UC1 mm → `0.001`).

## Notes
- SAM2 mask only tightens the depth sample; box-centre fallback if SAM2 is unavailable.
- POI is a JSON String (not `vision_msgs`, which isn't installed) matching the AGENTS.md §12 struct;
  easy to upgrade to `vision_msgs/Detection3DArray` later.
- Depth median is robust to minority nearer/farther pixels in the region.
