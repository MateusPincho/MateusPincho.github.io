---
layout: post
title: An Introduction to Path Planning Algorithms
date: 2025-10-02 09:33:00
description: tutorial of the principal path planning algorithms
tags: path-planning
categories: navigation mobile-robots
featured: true
toc:
  sidebar: true
---


In the field of mobile robotics, the ability of a robot to autonomously navigate from one point to another is fundamental. Behind every movement and obstacle avoidance lies a complex architecture of perception, localization, and, crucially, path planning algorithms. The task of a global path planner is to determine an optimal, collision-free route from a starting position to a final goal, using a map of the environment that is known a priori or built in real-time.

The global path planner establishes the macro-trajectory the robot should follow, leaving it to local planners (and controllers) to handle the fine details and react to dynamic obstacles along the way. The complexity and efficiency of these algorithms vary, depending on the representation of the environment, the kinematic constraints of the robot and the need for optimality or completeness.

## The Grid-Based Approach: Discretizing the Configuration Space

Most global path planning algorithms for mobile robots begin with a discretized representation of the environment. The most common method is the grid map, where the robot's operating space is divided into a mesh of uniform cells. Each cell can be labeled as "free," "occupied" or "unknown."

Formally, we can define the environment as a graph $G=(V,E)$, where $V$ is the set of vertices (the cells of the grid) and $E$ is the set of edges (the allowed transitions between adjacent cells). The objective is to find a path $P=(v_0​,v_1​,\cdots,v_n​)$ such that $v_0​$ is the starting position, $v_n$​ is the goal, and each edge $(vi​,vi+1​) \in E$ represents a valid, collision-free transition.

For mobile robots, it's important to consider the robot's physical size. This is usually incorporated into the grid map through a process called **obstacle inflation**. Each obstacle is inflated by a safety margin equal to the robot's radius (or an approximation of its footprint), ensuring that the center of the robot can follow the planned path without collision.

## The A* Algorithm

The A* algorithm is an extension of Dijkstra's algorithm that incorporates a heuristic function to guide the search of the more efficiently path toward the goal. It is optimal (finds the lowest-cost path) and complete (finds a path if one exists) under certain conditions.

A* evaluates each cell $n$ in the grid using a total cost function $f(n)$ defined by:

$$
f(n) = g(n) + h(n)
$$

Where $g(n)$ is the cost of the cheapest known path from the starting cell to cell $n$ and $h(n)$ is the estimated heuristic cost from cell $n$ to the goal cell. This function must be admissible (never overestimates the true cost) and ideally consistent (satisfies the triangle inequality) to guarantee optimality.

The algorithm works by maintaining two lists of cells: an open list that contains cells that have been discovered but not yet fully explored (their neighbors have not yet been evaluated), and a closed list, which contains cells that have already been explored.

### Steps of the A* Planner

1. Initialization:
- Add the start cell to the Open List.
- Set $g(start)=0$ and $f(start)=h(start)$. For all other cells $n$, set $g(n)=\infty$.

2. Main Loop: While the Open List is not empty:

- Select the cell $n$ in the Open List with the lowest $f(n)$ value.
- Remove $n$ from the Open List and add it to the Closed List.
- If $n$ is the goal cell, the path has been found. Reconstruct it from the predecessors.
- **For each neighbor $v$ of $n$**:
  - If v is in the Closed List, ignore it.
  - Calculate the tentative path cost $g_{temp}​(v)=g(n)+cost$( from n to v ).
  - If $g_{temp}​(v) < g(v)$ (or if $v$ is not in the Open List):
    - Set $g(v)=g_{temp}​(v)$.
    - Set the predecessor of $v$ to be $n$.
    - Calculate $f(v)=g(v)+h(v)$.
    - If $v$ is not in the Open List, add it.

### Choosing the heuristic $h(n)$

The choice of the heuristic is crucial for A*'s performance. Common heuristics for 2D grid maps include: 

1. The Manhattan Distance

$$
h(n)=∣x_n​−x_{goal}​∣+∣y_n​−y_{goal}​∣
$$

Ideal for orthogonal movements (4 directions).

2. The Euclidian Distance

$$
h(n)=\sqrt{(x_n​−x_{goal}​​)^2+(y_n​−y_{goal}​)^2}
$$

Best for movements in 8 directions (diagonals allowed).

## Introducing Kinematic Constraints with Hybrid A*

