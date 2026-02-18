# Two-Tag Progress Log

## 2026-02-16
**Summary:**
- Added `hover_yaw_search` controller to `setup.py` entry points (now callable as console script).
- Created `hover_controller.launch.py` for easy controller bringup with configurable parameters.
- Documented 3 launch patterns in QUICK_REFERENCE.md:
  1. **2-terminal quick start** (measurement only)
  2. **5-terminal full flight stack** (SITL + MAVROS + controller)
  3. **7-9 terminal debug split** (component-by-component inspection)
- Updated TWO_TAG_NOTES.md with controller details and launch flow.
- Controller is now fully integrated into the package.

**Status:** Ready for build and end-to-end testing.

**Next Todos:**
- [ ] Build package: `colcon build --packages-select tag_hover_two_tags`
- [ ] Test controller registration: `ros2 run tag_hover_two_tags hover_yaw_search --help`
- [ ] Pick a launch pattern (2-terminal, 5-terminal, or debug) and run end-to-end sim.
- [ ] Verify CSV logging and relative pose output.
- [ ] Document any issues found and tune controller parameters as needed.

---

## 2026-02-15
**Summary:**
- Working on a new controller under `src/tag_hover_two_tags/tag_hover_controller/hover_yaw_search.py`.
- Two-tag pipeline already includes tag pose selectors, relative pose fusion, and CSV logging.

**Notes:**
- The controller file was not registered in `setup.py` console scripts yet.

**Status:** Initial index and documentation created.
