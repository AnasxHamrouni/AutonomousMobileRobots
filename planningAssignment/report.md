# Planning Assignment Report



## Quantitative Results


### Metrics Summary


## Quantitative Comparison Table

### BUG0 Summary
| Scenario | Runs | Success Rate | Avg Time (s) | Avg Path (m) | Avg Wall Follow Entries | Avg Recovery Events |
|---|---:|---:|---:|---:|---:|---:|
| convex_01 | 1 | 100.0% | 22.20 | 3.65 | 0.00 | 0.00 |
| convex_02 | 1 | 100.0% | 19.80 | 3.26 | 0.00 | 0.00 |
| convex_03 | 1 | 100.0% | 7.30 | 0.68 | 0.00 | 0.00 |

### BUG1 Summary
| Scenario | Runs | Success Rate | Avg Time (s) | Avg Path (m) | Avg Wall Follow Entries | Avg Recovery Events |
|---|---:|---:|---:|---:|---:|---:|
| convex_01 | 2 | 50.0% | 17.00 | 2.22 | 0.00 | 0.00 |
| convex_02 | 2 | 50.0% | 49.30 | 5.11 | 1.50 | 3.00 |

### A* Summary
| Scenario | Runs | Success Rate | Avg Planning (ms) | Avg Path (m) | Avg Expanded Nodes |
|---|---:|---:|---:|---:|---:|
| convex_01 | 4 | 100.0% | 124.45 | 5.35 | 1789.8 |
| convex_02 | 1 | 100.0% | 65.91 | 3.55 | 812.0 |
| convex_03 | 1 | 100.0% | 103.35 | 4.77 | 1075.0 |

---

## A* Solution Screenshots

inside metrics folder, convex-*.jpg

---

## Discussion & Assignment Questions

### 1. Bug 0: Performance and Limitations
- **Strengths:** Simple, fast, and effective for convex obstacles. Minimal state and logic.
- **Limitations:** Can get trapped by concave obstacles ( U-shapes) since it leaves the wall as soon as the goal direction is clear, even if this leads into a trap.
- **Observed:** In all convex scenarios, Bug 0 succeeded quickly with short paths and no wall-follow/corner recovery events. In more complex environments, it may fail or loop indefinitely.

### 2. Bug 1: Performance and Limitations
- **Strengths:** Better than Bug 0. By circumnavigating the obstacle and tracking the closest point, it can escape some traps that Bug 0 cannot.
- **Limitations:** Less efficient as it may take longer paths and more time due to full circumnavigation. Still not guaranteed to solve all environments (if the closest point is unreachable).
- **Observed:** Success rate was lower in some scenarios, with longer paths and more recovery/circumnavigation events. Sometimes failed to complete the task if the closest point was not accessible.

### 3. A* Global Planner: Performance and Limitations
- **Strengths:** Finds optimal (shortest) paths if the map is accurate and complete. Always succeeds if a path exists in the map. Handles complex environments well.
- **Limitations:** Requires a complete and accurate map. Computationally more expensive. Sensitive to map errors and dynamic obstacles.
- **Observed:** Success rate was 100% in all tested scenarios. Planning time and expanded nodes were reasonable. Path length was sometimes longer than Bug 0 due to map discretization and inflation.

### 4. Comparison and Recommendations
- **Bug 0** is best for simple, convex environments.
- **Bug 1** is more robust for unknown or complex environments but may be less efficient.
- **A*** is ideal when a reliable map is available and optimality is important.

### 5. Assignment Questions
- **When does Bug 0 fail?**
  - When the robot leaves the wall too early and enters a concave trap ( U-shaped obstacle).
- **How does Bug 1 improve on Bug 0?**
  - By requiring a full circumnavigation and tracking the closest point, Bug 1 can escape some traps that Bug 0 cannot.
- **Why is A* optimal?**
  - It explores all possible paths in the grid and selects the shortest one based on cost and heuristic, given a static map.
