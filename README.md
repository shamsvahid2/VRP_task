# Vehicle Routing Problem Solution

This project offers a comprehensive solution to a variant of the Vehicle Routing Problem (VRP) incorporating specific operational constraints. The solution is tailored to handle missions from a hub to various locations and vice versa, with vehicles of differing capabilities and costs.

## Implementation

The implementation follows a structured approach to solving the VRP:

1. **Distance Calculation**: Utilizes the Haversine formula to compute the distance matrix among all nodes, providing a realistic measure of distances on the Earth's surface.
2. **OR-Tools Solver**: Leverages Google's OR-Tools to model and solve the VRP, incorporating:
   - A modified `cost_callback` to include both distance and delivery costs.
   - Capacity constraints for vehicles, ensuring no vehicle exceeds its maximum capacity.
   - Demands for each node, represented as positive (for hub to location missions) and negative (for location to hub missions) values.

## Installation and Usage

To use this code, follow these steps:

1. Install the required Python packages:
    ```bash
    pip install -r requirements.txt
2. Run the main script:
    ```bash
    python main.py

## Code Documentation

The code is fully documented with comprehensive docstrings and comments to ensure clarity and ease of understanding for developers and users alike. Each function within the script is accompanied by a docstring that explains:

- The purpose of the function.
- The parameters it takes.
- The value it returns.

This documentation approach provides insights into the functionality of each component and the rationale behind specific implementation choices, facilitating easier maintenance and future enhancements.

## Results

The effectiveness of various solution strategies was evaluated to identify the most efficient approach for solving the Vehicle Routing Problem with the given constraints. The testing process involved experimenting with different `FirstSolutionStrategy` and `LocalSearchMetaheuristic` options provided by OR-Tools. The results are summarized in the tables below:

### First Solution Strategy Results

| Strategy                       | Total Cost of All Routes |
|--------------------------------|-------------------------:|
| **PARALLEL_CHEAPEST_INSERTION**|                  1188.487|
| AUTOMATIC                     |                  1360.596|
| PATH_CHEAPEST_ARC             |                  1360.596|
| PATH_MOST_CONSTRAINED_ARC     |                  1357.707|

The `PARALLEL_CHEAPEST_INSERTION` strategy emerged as the most effective initial solution strategy, providing the lowest total cost among the options tested.

### Local Search Metaheuristic Results

| Metaheuristic         | Total Cost of All Routes |
|-----------------------|-------------------------:|
| AUTOMATIC             |                  1188.487|
| GREEDY_DESCENT        |                  1188.487|
| GUIDED_LOCAL_SEARCH   |                  1188.487|
| **SIMULATED_ANNEALING**|                 1187.005|

Further refinement using the `SIMULATED_ANNEALING` local search metaheuristic resulted in the best overall solution, achieving an approximate total cost of ~1187.  
