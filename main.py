import json
import math
#OR tools
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp


def load_json_data(file_path):
    """
    Load data from a JSON file.

    Args:
    file_path (str): The path to the JSON file.

    Returns:
    dict: The data loaded from the JSON file.
    """
    with open(file_path, 'r') as file:
        data = json.load(file)
    return data

def haversine(lat1, lon1, lat2, lon2):
    """
    Calculate the Haversine distance between two points on the earth.

    Args:
    lat1, lon1: Latitude and longitude of the first point in decimal degrees.
    lat2, lon2: Latitude and longitude of the second point in decimal degrees.

    Returns:
    float: Distance between the two points in kilometers.
    """
    # Convert latitude and longitude from degrees to radians
    lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lon1, lat2, lon2])

    # Haversine formula
    dlat = lat2 - lat1
    dlon = lon2 - lon1
    a = math.sin(dlat / 2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2)**2
    c = 2 * math.asin(math.sqrt(a))

    # Radius of earth in kilometers. Use 3956 for miles
    r = 6371

    # Calculate the result
    return c * r

def haversine_all_locations(locations):
    """
    Creates a distance matrix for all locations using the Haversine formula.

    Args:
    locations (list): A list of tuples where each tuple contains the latitude and longitude of a location.

    Returns:
    list: A 2D list representing the distance matrix.
    """
    distances = []
    for loc_a in locations:
        row = []
        for loc_b in locations:
            distance = haversine(loc_a[1],loc_a[0],loc_b[1],loc_b[0])
            row.append(distance)
        distances.append(row)
    return distances

def create_data_model(json_data):
    """
    Prepares the data for the Vehicle Routing Problem.

    Args:
    json_data (dict): Data loaded from JSON describing vehicles, missions, and the warehouse.

    Returns:
    dict: A dictionary containing all necessary data for the VRP.
    """
    data = {}
    # Locations include the warehouse and missions
    locations = [(json_data['warehouse']['latitude'], json_data['warehouse']['longitude'])] + [
        (mission['latitude'], mission['longitude']) for mission in json_data['missions']]
    data['distance_matrix'] = haversine_all_locations(locations)
    data['num_vehicles'] = sum(vehicle['number of vehicles'] for vehicle in json_data['Vehicles'].values())
    data['depot'] = 0
    # Vehicle capacities and costs
    data['vehicle_capacities'] = []
    data['vehicle_costs'] = {'distance_cost': [], 'delivery_cost': []}
    for vehicle_type, attributes in json_data['Vehicles'].items():
        data['vehicle_capacities'] += [attributes['capacity']] * attributes['number of vehicles']
        data['vehicle_costs']['distance_cost'] += [attributes['cost per kilometers']] * attributes['number of vehicles']
        data['vehicle_costs']['delivery_cost'] += [attributes['cost per delivery']] * attributes['number of vehicles']
    #missions
    data['demands'] = [0]
    for mission in json_data['missions']:
        #if return is true
        if mission['return']:
            data['demands'].append(-1)
        #from hub missions
        else:
            data['demands'].append(+1)

    return data

def print_solution(data, manager, routing, solution):
    """
    Prints the solution of the routing problem.

    Args:
    data (dict): The data used for the VRP.
    manager (pywrapcp.RoutingIndexManager): Index manager created for the routing.
    routing (pywrapcp.RoutingModel): The routing model.
    solution (pywrapcp.Assignment): The solution to the routing problem.
    """
    total_distance = 0
    total_cost = 0
    for vehicle_id in range(data['num_vehicles']):
        index = routing.Start(vehicle_id)
        plan_output = 'Route for vehicle {}:\n'.format(vehicle_id)
        route_distance = 0
        route_cost = 0
        while not routing.IsEnd(index):
            node_index = manager.IndexToNode(index)
            next_node_index = manager.IndexToNode(solution.Value(routing.NextVar(index)))
            route_distance += data['distance_matrix'][node_index][next_node_index]
            distance_cost = data['distance_matrix'][node_index][next_node_index] * data['vehicle_costs']['distance_cost'][vehicle_id]
            delivery_action = data['demands'][next_node_index] != 0
            delivery_cost = data['vehicle_costs']['delivery_cost'][vehicle_id] if delivery_action else 0
            segment_cost = distance_cost + delivery_cost
            route_cost += segment_cost
            plan_output += ' {0} ->'.format(node_index)
            index = solution.Value(routing.NextVar(index))
        plan_output += ' {0}\n'.format(manager.IndexToNode(index))
        plan_output += 'Distance of the route: {}km\n'.format(route_distance)
        plan_output += 'Cost of the route: {}\n'.format(route_cost)
        print(plan_output)
        total_distance += route_distance
        total_cost += route_cost
    print('Total distance of all routes: {}km'.format(total_distance))
    print('Total cost of all routes: {}'.format(total_cost))
def main():

    """
    Main function to solve the Vehicle Routing Problem.
    """
        # Instantiate the data problem.
    file_path = 'Sample_data.json'
    json_data = load_json_data(file_path)

    # Instantiate the data problem.
    data = create_data_model(json_data)

    # Create the routing index manager.
    manager = pywrapcp.RoutingIndexManager(len(data['distance_matrix']), data['num_vehicles'], data['depot'])

    # Create Routing Model.
    routing = pywrapcp.RoutingModel(manager)


    #cost callbacks for different vehicles
    for vehicle in range(data['num_vehicles']):
        # Distance callback
        def cost_callback(from_index, to_index):
            cost = 0
            cost += data['vehicle_costs']['distance_cost'][vehicle] * data['distance_matrix'][manager.IndexToNode(from_index)][manager.IndexToNode(to_index)]
            if manager.IndexToNode(to_index) != data['depot'] and manager.IndexToNode(from_index) != data['depot']:
                cost += data['vehicle_costs']['delivery_cost'][vehicle]
            return int(round(cost))

        transit_callback_index = routing.RegisterTransitCallback(cost_callback)
        routing.SetArcCostEvaluatorOfVehicle(transit_callback_index,vehicle=vehicle)


    # Add Capacity constraint.
    def demand_callback(from_index):
        """Returns the demand of the node."""
        # Convert from routing variable Index to demands NodeIndex.
        from_node = manager.IndexToNode(from_index)
        return data['demands'][from_node]

    demand_callback_index = routing.RegisterUnaryTransitCallback(demand_callback)
    routing.AddDimensionWithVehicleCapacity(
        demand_callback_index,
        0,  # null capacity slack
        data['vehicle_capacities'],  # vehicle maximum capacities
        True,  # start cumul to zero
        'Capacity')

    # Setting first solution heuristic.
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PARALLEL_CHEAPEST_INSERTION)

    #Specify the local search metaheuristic to refine the solution.
    search_parameters.local_search_metaheuristic = (
        routing_enums_pb2.LocalSearchMetaheuristic.SIMULATED_ANNEALING)
    search_parameters.time_limit.FromSeconds(30)  # Set a time limit for the search.


    # Solve the problem.
    solution = routing.SolveWithParameters(search_parameters)

    # Print solution on console.
    if solution:
        print_solution(data, manager, routing, solution)




    

if __name__ == '__main__':
    main()
