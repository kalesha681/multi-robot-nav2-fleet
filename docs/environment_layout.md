# Warehouse environment survey and handoff reference

This document is the source-of-truth inventory for the Gazebo world at
`src/amr_sim/worlds/small_warehouse.sdf`.  It is intentionally checked into
the repository so another engineer or AI can resume environment work without
having to rediscover the coordinate layout.

The companion [top-down warehouse plan](warehouse_plan.svg) gives a
full-footprint coordinate overview of the wall, shelves, clutter, robot
starts, and floor markings.  The marking coordinates below are extracted from
the GroundB visual mesh and its referenced texture, rather than estimated from
a GUI screenshot.

## Coordinate convention

- World: `default`; navigation frame: `world`.
- Pose format: `x y z roll pitch yaw`, in metres and radians.
- Positive `x` points toward the shelf bank at `x = 4.73156`.
- All warehouse models below are included at their stated world pose.  They
  are static environment assets unless a later world edit changes that.
- The ground plane is centred at the origin, has no explicit pose (identity),
  and measures `100 m × 100 m`.

## Structural assets

| Entity name | Asset / role | World pose (`x y z r p y`) | Placement notes |
| --- | --- | --- | --- |
| `aws_robomaker_warehouse_RoofB_01_001` | Warehouse roof | `0.0 0.0 0 0 0 0` | Structural visual/collision asset. |
| `aws_robomaker_warehouse_WallB_01_001` | Warehouse wall shell | `0.0 0.0 0 0 0 0` | Defines the main enclosed warehouse footprint; do not place terrain near an exterior wall without visual inspection. |
| `aws_robomaker_warehouse_GroundB_01_001` | Warehouse ground asset | `0.0 0.0 -0.090092 0 0 0` | Sits over the explicit ground plane. |
| `aws_robomaker_warehouse_Lamp_01_005` | Warehouse lamp model | `0 0 -4 0 0 0` | Decorative world model; distinct from the active light entities below. |
| `ground_plane` | Global collision/visual plane | `0 0 0 0 0 0` | `100 m × 100 m`; not a warehouse-aisle guarantee. |

## Storage racks

| Entity name | Asset type | World pose (`x y z r p y`) | Placement notes |
| --- | --- | --- | --- |
| `aws_robomaker_warehouse_ShelfF_01_001` | Shelf F | `-5.795143 -0.956635 0 0 0 0` | Isolated western rack. |
| `aws_robomaker_warehouse_ShelfE_01_001` | Shelf E | `4.73156 0.57943 0 0 0 0` | Eastern shelf bank. |
| `aws_robomaker_warehouse_ShelfE_01_002` | Shelf E | `4.73156 -4.827049 0 0 0 0` | Eastern shelf bank. |
| `aws_robomaker_warehouse_ShelfE_01_003` | Shelf E | `4.73156 -8.6651 0 0 0 0` | Eastern shelf bank. |
| `aws_robomaker_warehouse_ShelfD_01_001` | Shelf D | `4.73156 -1.242668 0 0 0 0` | Eastern shelf bank. |
| `aws_robomaker_warehouse_ShelfD_01_002` | Shelf D | `4.73156 -3.038551 0 0 0 0` | Eastern shelf bank. |
| `aws_robomaker_warehouse_ShelfD_01_003` | Shelf D | `4.73156 -6.750542 0 0 0 0` | Eastern shelf bank. |

## Loose warehouse objects and clutter

| Entity name | Asset type | World pose (`x y z r p y`) |
| --- | --- | --- |
| `aws_robomaker_warehouse_Bucket_01_020` | Bucket | `0.433449 9.631706 0 0 0 -1.563161` |
| `aws_robomaker_warehouse_Bucket_01_022` | Bucket | `0.433449 8.59 0 0 0 -1.563161` |
| `aws_robomaker_warehouse_ClutteringA_01_016` | Clutter A | `5.708138 8.616844 -0.017477 0 0 0` |
| `aws_robomaker_warehouse_ClutteringA_01_017` | Clutter A | `3.408638 8.616844 -0.017477 0 0 0` |
| `aws_robomaker_warehouse_ClutteringA_01_018` | Clutter A | `-1.491287 5.222435 -0.017477 0 0 -1.583185` |
| `aws_robomaker_warehouse_ClutteringC_01_027` | Clutter C | `3.324959 3.822449 -0.012064 0 0 1.563871` |
| `aws_robomaker_warehouse_ClutteringC_01_028` | Clutter C | `5.54171 3.816475 -0.015663 0 0 -1.583191` |
| `aws_robomaker_warehouse_ClutteringC_01_029` | Clutter C | `5.384239 6.137154 0 0 0 3.150000` |
| `aws_robomaker_warehouse_ClutteringC_01_030` | Clutter C | `3.236 6.137154 0 0 0 3.150000` |
| `aws_robomaker_warehouse_ClutteringC_01_031` | Clutter C | `-1.573677 2.301994 -0.015663 0 0 -3.133191` |
| `aws_robomaker_warehouse_ClutteringC_01_032` | Clutter C | `-1.2196 9.407 -0.015663 0 0 1.563871` |
| `aws_robomaker_warehouse_ClutteringD_01_005` | Clutter D | `-1.634682 -7.811813 -0.319559 0 0 0` |
| `aws_robomaker_warehouse_TrashCanC_01_002` | Trash can | `-1.592441 7.715420 0 0 0 0` |

## Lighting

| Entity name | Type | World pose (`x y z r p y`) |
| --- | --- | --- |
| `Warehouse_CeilingLight_003` | Point light | `0 0 9 0 0 0` |
| `sun` | Directional light | `0 0 10 0 0 0` |

## Fleet starting positions

These are not world objects; they are launch-time robot poses and are recorded
here because terrain placement must preserve their approach routes.

| Robot | Spawn pose (`x y yaw`) | Source |
| --- | --- | --- |
| AMR-1 (`bcr_bot_amr1`) | `0.0 0.0 0.0` | `nav2_fleet.launch.py` |
| AMR-2 (`bcr_bot_amr2`) | `4.0 0.0 0.0` | `nav2_fleet.launch.py` |

## Ramp placement rules

1. Do not use the exterior region outside `WallB` merely because the global
   ground plane exists there.
2. Do not overlap any listed rack, clutter item, spawn point, or the current
   concurrent-mission corridors.
3. A candidate must form a useful alternate path between two reachable areas;
   an isolated ramp is not acceptable evidence for slope-aware planning.
4. Before committing a model, inspect the candidate in Gazebo from top and
   side views, then drive AMR-1 over it using a manual velocity command.
5. Record the final ramp bounding box, slope angle, surface friction, and
   route-test start/goal coordinates in this document.

## Terrain validation notes

- A raised platform changes the 2D LiDAR scan plane relative to floor-level
  objects and nearby shelving.  During Nav2 testing, unexpected marking or
  clearing near platform edges is a known diagnostic to investigate first.
- The map-fusion grid currently begins near the warehouse's western/southern
  extent.  A terrain addition near or beyond those bounds can resize the
  published merged map.  That is expected, but must be recorded in validation
  logs rather than mistaken for a fusion regression.
- Slope goals will use a dedicated future mission-manager mode,
  `--slope-demo`, rather than altering the existing normal or selective-map
  demonstrations.  It will contain an upper-storage goal and a ground-level
  through-goal with a flat detour.

## Current terrain status

### Marked green traffic lanes

Green floor markings are a no-build traffic corridor.  They were measured by
sampling the material `Material #946569` texture at each GroundB mesh
triangle's wrapped UV coordinate, then transforming the mesh centimetres to
world metres.  This is an asset-level measurement, not an inferred screenshot
measurement.  The marked network lies within the following thin strips
(coordinates rounded to centimetres):

| Marked strip | World extent (`x`, `y` metres) |
| --- | --- |
| West outer vertical | `x=-6.90..-6.82`, `y=-10.47..10.47` |
| West lower horizontal | `x=-6.90..-4.15`, `y=-9.70..-9.61` |
| Inner-west vertical | `x=-4.24..-4.15`, `y=-9.84..8.76` |
| Inner-west upper horizontal | `x=-6.86..-4.15`, `y=8.76..8.84` |
| Centre-west vertical | `x=-3.38..-3.30`, `y=-10.47..10.47` |
| Centre-south horizontal | `x=-3.30..1.32`, `y=-9.93..-9.84` |
| Centre-east vertical | `x=1.32..1.41`, `y=-10.47..10.47` |
| East vertical, south segment | `x=2.16..2.25`, `y=-10.44..1.78` |
| East vertical, north segment | `x=2.16..2.25`, `y=2.58..10.47` |
| East branch horizontal, south | `x=2.25..7.01`, `y=1.68..1.78` |
| East branch horizontal, north | `x=2.25..7.01`, `y=2.58..2.67` |

The apparent gaps are deliberate breaks in the painted network, not missing
data.  The plan SVG renders the exact measured mesh triangles, so it preserves
corner joins that are not represented by the rounded table.

### Hazard-striped storage stalls

The black/yellow marked stall boundary is also texture-derived.  Its active
area is `x=-2.55..0.45 m`, `y=-8.82..7.07 m`; it is a no-build zone until a
future task explicitly relocates the represented storage.  The present SVG
does not simplify it into a false set of rectangles; the source texture is the
authoritative detailed pattern.

No custom ramp is currently present.  The latest candidate was removed because
the visual review showed that it occupied a primary aisle.  The next placement
must be selected only after a full-footprint top-down survey identifies a
genuine side bay outside marked lanes, storage stalls, shelves, and normal
mission corridors.
