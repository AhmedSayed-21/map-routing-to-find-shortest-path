# README — Route Planner (C++)

## Overview

This repository contains a C++ program that reads a map (nodes and roads) and a set of queries (start point, end point, walking radius), then computes the fastest route combining walking and vehicle travel. The program uses a KD-tree to quickly find points within a given distance and Dijkstra’s algorithm to find the shortest travel time between network points.

## Features

* Build a KD-tree from map nodes for fast spatial queries.
* Dijkstra with multiple sources and multiple targets.
* Walking time calculation based on a constant walking speed (5 km/h) and converted to minutes.
* Outputs the route as a sequence of node IDs, along with distance and time summaries.
* Execution time measurement (ms).

## Requirements

* C++17 compiler (e.g., `g++`).
* No external libraries required.

## Compilation

```bash
g++ -O2 -std=c++17 -o route_planner main.cpp
```

(Replace `main.cpp` with the name of your file if different.)

## Input File Formats

### MAP file

Format:

```
N
id1 x1 y1
id2 x2 y2
...
idN xN yN
M
u1 v1 len1 speed1
u2 v2 len2 speed2
...
uM vM lenM speedM
```

* `N`: number of points.
* Each line `id x y` contains the node ID (integer) and coordinates (`x`, `y`) in kilometers.
* `M`: number of edges (roads).
* Each edge line `u v len speed` contains the IDs of two nodes (`u`, `v`), the road length in km, and speed in km/h.

> Note: Vehicle travel time is calculated as `(len / speed) * 60.0` to get minutes.

### QUERIES file

Format:

```
Q
sx1 sy1 dx1 dy1 Rm1
sx2 sy2 dx2 dy2 Rm2
...
```

* `Q`: number of queries.
* Each query: `sx sy` (start coordinates), `dx dy` (destination coordinates), `Rm` (walking radius in meters).
* The program converts `Rm` to km (`R = Rm / 1000.0`) and searches for all nodes within `R` km.

## Output Format

For each query, the output file contains:

1. A line with the sequence of node IDs forming the route.
2. A line with the total travel time in minutes: `XX.XX mins`.
3. A line with the total distance in km: `XX.XX km`.
4. A line with the walking distance in km.
5. A line with the vehicle distance in km.

After all queries, the program outputs execution times in milliseconds for the query processing stage and the total runtime.

## Assumptions & Speeds

* Walking speed is fixed at 5 km/h (`walkInv = 60 / 5 = 12` minutes per km).
* Node coordinates are assumed to be in kilometers.

## Performance & Complexity

* KD-tree build: \~`O(N log N)`.
* Per query: KD-tree search for start/end nodes + Dijkstra search (`O((V+E) log V)`).

## Example

**MAP.txt**

```
4
1 0.0 0.0
2 0.0 1.0
3 1.0 1.0
4 1.0 0.0
4
1 2 1.0 40.0
2 3 1.0 40.0
3 4 1.0 40.0
4 1 1.0 40.0
```

**QUERIES.txt**

```
1
0.1 0.1 0.9 0.9 200
```

Here, `Rm = 200` meters → `R = 0.2` km.

## Tips

* Ensure node IDs in the MAP file are unique and match the edges.
* If coordinates are in meters, convert them to km or adjust the code.
* Walking speed can be changed by modifying `walkInv` in the code.

## Future Improvements

* Support for asymmetric edges or time-dependent weights.
* Memory optimization for large datasets.
* JSON or GeoJSON output for map integration.
