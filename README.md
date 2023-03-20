# AStar
This is my implementation of the AStar algorithm which is used as an efficent way to find the "shortest" between two points avoiding all obstacles 

## Example Usage:
```
AStar astar = AStar(1, 2, 100, 250, map);
std::vector<std::pair<int, int>> path = astar.FindPath(false, AStar::Heuristic::Manhattan);
```
