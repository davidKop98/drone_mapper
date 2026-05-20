# Drone Mapper

## 1. Contributors

David Kopilevitch 207230731
Amit Nisenbaum    209907773
## 2. Project description

A simulated drone explores a 3D building, scans walls with a mock lidar, and builds an occupancy map.

Given a ground-truth `map_input.txt`, the drone (with no prior knowledge of the map) autonomously decides its commands via a DFS exploration algorithm:

- **Scan**: sphere-scan the surroundings with a configurable lidar (FOV circles, beam range, spacing).
- **Move**: rotate, advance, or elevate to an unvisited Empty neighbor cell, with body-aware collision checking.
- **Backtrack**: when no forward target exists, replay the inverse of previous moves.

The drone marks cells it confirms via lidar (`Empty` / `Occupied`) into a `BuildingMap`, which is serialized to `map_output.txt` at the end. A score is computed by comparing the drone's discovered map against the ground truth.

## 3. Building the project

**Requirements:**
- C++20 (gcc 11.4+ or compatible)
- CMake 3.21+
- vcpkg (with `VCPKG_ROOT` environment variable set to the vcpkg install path)

**Build:**

```bash
cmake --preset vcpkg
cmake --build build
```

The first `cmake` call uses the `vcpkg` preset (defined in `CMakePresets.json`) to fetch and configure the sole direct dependency, `mp-units`, via vcpkg manifest mode. (`fmt` and `gsl-lite` are pulled in transitively by mp-units and are not used directly by our code.) The build directory is `build/`.

The executable is produced at `build/drone_mapper`.

Strict compile flags (`-Wall -Wextra -Werror -pedantic`) are enabled — the build will fail on any warning.

## 4. Running the project

```bash
./build/drone_mapper [<path>]
```

- `<path>` is a directory containing the three required input files (see Section 5).
- If `<path>` is omitted, the current working directory (`.`) is used.

**Output to stdout:**
- `Score: <N>/100`
- `Commands executed: <N>`
- `Status: FINISHED` (drone exploration completed cleanly) or `FAILED` (drone collided with a wall — simulator stopped early)

Output files (`map_output.txt`, and optionally `input_errors.txt`) are written to `<path>`.

## 5. Input file format

All three files must live in the directory passed as `<path>`. Key/value config files use `key: value` lines; lines starting with `#` and blank lines are ignored.

IMPORTANT NOTE: It is much easier to understand the input file format by looking at an example. 
but formally:
### `drone_config.txt`

| Key | Units | Description |
|-----|-------|-------------|
| `min_pass_width` | cm | Drone body width |
| `min_pass_length` | cm | Drone body length (along heading axis) |
| `min_pass_height` | cm | Drone body height |
| `max_rotate` | degrees | Max rotation per single `Rotate` command (chunked if larger) |
| `max_advance` | cm | Max distance per single `Advance` command |
| `max_elevate` | cm | Max distance per single `Elevate` command |
| `lidar_beam_min` | cm | Lidar minimum detection range; hits closer than this return 0-distance |
| `lidar_beam_max` | cm | Lidar maximum range; beyond this counted as "no hit" |
| `lidar_spacing` | cm | Spacing between beam circles (controls FOV cone tightness) |
| `lidar_fov_circles` | count | Number of concentric beam circles per scan (0 = no beams) |

### `mission_config.txt`

| Key | Units | Description |
|-----|-------|-------------|
| `start_x`, `start_y`, `start_z` | cm | Drone starting center position |
| `start_heading` | degrees | Starting horizontal heading (0° = +X, 90° = +Y) |
| `min_x`, `max_x`, `min_y`, `max_y`, `min_z`, `max_z` | cm | Mission boundary box (cells outside this are OutOfBounds) |
| `xy_resolution` | integer | Decimal places of XY cell precision; cell size = 10^(-xy_resolution) cm |
| `z_resolution` | integer | Same, for Z axis |

### `map_input.txt`

Layered grid format. Header is key/value lines (no units, all integers except `origin_*`):

| Key | Description |
|-----|-------------|
| `resolution` | Decimal places of cell precision (matches `xy_resolution`) |
| `size_x`, `size_y`, `size_z` | Grid dimensions in cells |
| `origin_x`, `origin_y`, `origin_z` | World coordinate (in cm) of cell (0, 0, 0)'s lower corner |

Then comes the grid data, **z-layer by z-layer, row by row**:

- Each layer has `size_y` rows, each row is `size_x` characters.
- Each character is `0` (Empty) or `1` (Occupied / wall).
- Layers are stacked from `z=0` (bottom) to `z=size_z - 1` (top).
- `# layer z=N` comment lines are optional and ignored.

Cells outside the grid are treated as Empty/OutOfBounds by the lidar.

## 6. Output file format

### `map_output.txt`

Same layered grid format as `map_input.txt`, with two differences:

1. Cell values are space-separated (since they can be negative): `0`, `1`, `-1`, or `-2`.
2. Cell meanings:

| Value | Meaning |
|-------|---------|
| `0`  | Empty — confirmed free by lidar |
| `1`  | Occupied — confirmed wall by lidar hit |
| `-1` | Unmapped — never observed by lidar (treated as wall in scoring) |
| `-2` | OutOfBounds — outside mission boundaries |

Header lines (`resolution`, `size_x`, etc., `origin_x`, etc.) match what the input format expects, so the output can be fed back as input if needed.

### `input_errors.txt`

Created **only if** the config parser encountered missing keys, unparseable values, or out-of-range values. Each line describes one such issue. If no errors occurred, this file is not written.

## 7. Test folders

Three demonstration scenarios sit at the project root, each with `drone_config.txt`, `mission_config.txt`, `map_input.txt`, and `original_output/map_output.txt`.

### [`test1_basic/`](test1_basic/) — Score: 100/100

Open 10×10×4 single room. 1×1×1 drone. Sanity-check that the drone fully explores and maps a simple building without obstacles.

### [`test2_diagonal_corridor/`](test2_diagonal_corridor/) — Score: 56.12/100

14×14 map with two 5×5 rooms (top-left and bottom-right) connected by a stair-stepped corridor. The corridor is designed so each step's "due-east" cell is a wall, **forcing the drone to use SE diagonal moves** to traverse it. Demonstrates the planner's ability to choose diagonal moves when cardinal moves are blocked.

The <100 score is from buried-wall cells the lidar can't line-of-sight-hit.

### [`test3_complex/`](test3_complex/) — Score: 99.10/100

16×12×11 multi-level building with a 2×2×3 drone (max_rotate=45°). **5 rooms total**:

- 2 rooms on the lower deck connected by a 3-cell-wide doorway
- 2 rooms on the upper deck connected by a 3-cell-wide doorway
- **A sealed 3×3×4 pocket** on the lower deck that has no doorway — fully inaccessible

The two decks are connected by a 3×3 stairwell hole between z=1-4 and z=6-9. The drone navigates all 4 accessible rooms via doorways and the stairwell. The 0.9% score deduction is the 36 cells of the sealed pocket, which stay `Unmapped` because no lidar beam can see inside the surrounding walls. Demonstrates: multi-room exploration, multi-level navigation, large-drone passage through tight doorways, and correct handling of unreachable regions.
