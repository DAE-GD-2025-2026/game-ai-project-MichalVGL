# Game AI Project

A project made following the course Game AI Programming (Algorithms 2).

## Extra Assignment | Fallback Path

 Assignment from Week05-Pathfinding.

 Both Astar and BFS accept a boolean allowing them to return a path that does not reach the target node.
 They will instead return a path to a node closest to the original target that is reachable.
 It does this by finding the closest node in its records after the algorithm and reconstructing the path from that node instead.

 The ImGui can switch between both of these algorithms and you can see the results by surrounding the target tile with blockers ('3' on keyboard).