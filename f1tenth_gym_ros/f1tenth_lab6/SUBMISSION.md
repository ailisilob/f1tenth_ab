# Lab 6
why i deserve extra credit?
I believe I have fully implemented the RRT* algorithm, strictly following Algorithm 6 from the provided paper.
my implementation goes beyond the standard RRT exploration by introducing continuous path optimization through the below mechanisms:

1. the algorithm identifies a set of neighboring nodes within a search_radius and explicitly tracks the cumulative path cost (distance from the root) for each node.

2. when a new node is sampled, it evaluates all neighbors to select the parent that yields the minimum overall cost, rather than defaulting to the geometrically nearest node.

3. the algo revisits existing nodes within the neighborhood to check if routing them through the newly added node would decrease their current cost. If so, it dynamically "rewires" the tree by updating their parent pointers and edges.

4. another difference is that I don’t terminate the algorithm as soon as a valid path is discovered. I let it run for the full iteration budget, continuously refining the tree. I also keep track of all nodes that reach the goal region and finally return the lowest-cost path among them. This makes the final trajectory noticeably more optimal.

5. since RRT* is more computationally expensive, I also spent time making sure it runs efficiently in practice. I optimized the occupancy grid updates using vectorized operations and scipy’s binary dilation, and used Bresenham’s line algorithm for fast collision checking.

SHOW ME THE 10 POINTS
THANK YOU heartfully

## Video Link
### Simulation:  
without obstable
https://youtu.be/xhln8RQn_wU
with obstacle
https://youtu.be/WF1HsL8iIlI
### Real Car: 
without rviz
https://youtube.com/shorts/EXUcnMTxbDg
https://youtube.com/shorts/b9ytiGJ5GNQ
with rviz
https://youtube.com/shorts/t9u3Y09ziWU

sorry for too many videos