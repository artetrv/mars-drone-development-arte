# tag_hover_two_tags Package Summary

## Purpose

ROS 2 package for two-tag AprilTag relative pose measurement. It estimates the vibrating tag pose in the reference tag frame and logs CSV output for vibration analysis.

## Package Structure

```
harmonic_ws/src/tag_hover_two_tags/
├── package.xml
├── setup.py
├── setup.cfg
├── README.md
├── QUICK_REFERENCE.md
├── PACKAGE_SUMMARY.md
├── resource/
│   └── tag_hover_two_tags
├── tag_hover_two_tags/
│   ├── __init__.py
│   ├── tag_oscillator.py
│   ├── tag_pose_selector.py
│   ├── relative_vibration_pose.py
│   ├── apriltag_tf_broadcaster.py
│   └── apriltag_pnp_broadcaster.py
├── tag_hover_controller/
│   └── hover_yaw_search.py
├── tag_hover_sim/
│   ├── hover_yaw_search.py
│   ├── hover_yaw_search_v1.py
│   ├── hover_yaw_search_v2.py
│   └── hover_yaw_search_sensor_lock.py
├── config/
│   └── apriltag_params.yaml
└── launch/
    ├── sim_vision_stack.launch.py
    └── sim_lockon_backbone.launch.py
```

## Key Nodes

- `tag_pose_selector` - extracts a specific tag pose from `/detections` and publishes `PoseStamped`.
- `relative_vibration_pose` - time-syncs ref/vib poses, computes $T_{vib}^{ref}$, publishes `/relative_vibration_pose`, and logs CSV.
- `tag_oscillator` - sinusoidal joint command publisher for the vibrating tag.
- `apriltag_tf_broadcaster` - optional TF broadcaster for detections.
- `apriltag_pnp_broadcaster` - optional PnP TF broadcaster using camera intrinsics.
- `tag_hover_controller/hover_yaw_search.py` - standalone yaw-search controller (not registered as a console script).

## Launch Files

- `sim_vision_stack.launch.py` - Gazebo world + camera bridge + apriltag detector + tag pose selectors.
- `sim_lockon_backbone.launch.py` - relative pose estimator + CSV logging.

## Notes

- Default world file: `worlds/apriltag_two_tags.sdf`.
- Replace the second tag model texture with a unique AprilTag ID for correct separation.
