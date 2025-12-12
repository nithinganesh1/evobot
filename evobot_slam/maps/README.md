evobot_slam maps directory

Save Cartographer output files here so they can be committed and reused.

Recommended files to keep in this folder:
- my_map.pbstream        # Cartographer state (full submaps)
- my_map.pgm             # Occupancy grid image produced by map_saver
- my_map.yaml            # YAML file describing the occupancy grid

How to save (high level):
1. Save Cartographer pbstream (Cartographer write_state service).
2. Make sure an occupancy_grid node publishes `/map` (Cartographer or occupancy_grid_node).
3. Use `map_saver` (ROS1) or `map_saver_cli` (ROS2 nav2) to write the `.pgm` + `.yaml`.
4. Move the produced files into this `maps/` folder and commit.

See project README or ask the maintainer for help with exact commands for your ROS distribution.