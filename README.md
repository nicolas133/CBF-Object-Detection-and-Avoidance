# CBF-Object-Detection-and-Avoidance

This research proposal aims to explore the use of a control barrier function (CBF) safety filter to correct the output of traditional PID controllers. Additionally, it will investigate the integration of this safety filter with optimization-based approaches, particularly focusing on control Lyapunov functions (CLFs). The ultimate goal of this thesis is to achieve autonomous navigation for an Ackerman steering vehicle, enabling it to autonomously navigate from point A to point B in a partially known environment. In this scenario, the vehicle may have access to a general map but lacks information about all potential obstacles, necessitating real-time detection and avoidance of new obstacles. This project is primarily concerned with the challenges of obstacle detection and avoidance in autonomous navigation rather than the complexities of path planning. The focus will be on addressing these challenges at the low-level controller stage rather than on the high-level pathfinding plans generated by heuristic and shortest-path algorithms like A* or Dijkstra’s. The project will build on previous research in which an offline CBF safety filter was used to correct the PID controller output for both steering angle and velocity.
