# Coverage method

The planner is a deterministic, geometry-first pipeline. It accepts one bounded search polygon,
zero or more hard occupied polygons, a home point, a fixed flight altitude, and a fixed-nadir
camera model.

## Pipeline

1. **Construct searchable ground.** The requested search polygon is validated and occupied
   footprints are subtracted. Overlapping occupied polygons are unioned by GEOS/Shapely.
2. **Project the camera footprint.** Horizontal and vertical field of view, height above the
   target plane, target envelope, image-edge margin, and side overlap determine the usable
   footprint and lane spacing.
3. **Generate coverage lanes.** `global_scanline` rotates a regular scanline lattice into the
   selected direction and clips each line against searchable ground. `bcd` first performs a
   boustrophedon-style cell decomposition and generates lanes per cell.
4. **Choose scan direction.** When no direction is supplied, orthogonal candidates derived from
   polygon edges are compared. The deterministic objective favors fewer disconnected segments
   and shorter connectors.
5. **Order and orient lanes.** Each lane can be flown in either direction. The route optimizer
   uses obstacle-aware transition costs and chooses lane order and orientation using greedy,
   local-search, exact, or automatic selection.
6. **Route around occupied areas.** Occupied polygons are buffered by the configured horizontal
   clearance. A visibility graph is built from buffer vertices, then deterministic `heapq`
   Dijkstra computes collision-free connectors. No NetworkX dependency is used.
7. **Validate continuous coverage.** The flown segments are sampled geometrically and their
   visible camera footprints are unioned. Each clipped ground patch must meet the configured
   coverage ratio.
8. **Complete gaps.** The local-insertion strategy adds observations for remaining uncovered
   ground while preserving primary lane order. The full-greedy strategy can re-optimize all
   observations. Coverage is evaluated again before the result is accepted.
9. **Return sparse navigation geometry.** The ROS adapter publishes the home point, coverage-lane
   endpoints, connector/avoidance vertices, and optional home return. Dense points used only for
   continuous-coverage evaluation are deliberately not published.

## Coordinate and obstacle assumptions

All geometry is planar local ENU in metres. Camera heading is measured clockwise from North in
the core. Occupied areas are hard 2D obstacles: they are excluded from ground coverage and may
not be overflown at any altitude. The startup schema does not infer obstacle heights, so only the
footprints occlude detectable ground.

The resulting `nav_msgs/Path` is navigation geometry, not a complete flight-controller command
protocol. It intentionally carries no speed, acceptance-radius, trigger, or replanning metadata.
