import heapq
from math import sqrt
from xml.etree.ElementTree import tostring

class Node:
    """ Class representing a node in a graph. """

    # Counter for default id
    id_counter = 0

    def __init__(self, id=None, data=None):
        """ Initialize a node.

        Parameters
        ----------
        hashable id : The unique identifier of this node.
        Any data : The data to store in this node. Defaults to None.
        """

        if (id is None):
            self.id = Node.id_counter
            Node.id_counter += 1
        else:
            self.id = id

        # Dictionary of nodes this node is connected to. 
        self.connections = {}
        self.data = data

    def add_connection(self, node, distance, bidirectional=False):
        """ Add a connection to another node.

        Parameters
        ----------
        Node node : The node to add a connection to. 
        float distance : The length of the connection.
        boolean bidirectional : Whether or not the connection is bidirectional. If set to
            True, add a connection to this node into the connected node. Default is False.

        Throws
        ------
        ValueError : If the id of the given node conflicts with an existing connection, 
            and the existing connection is not the same object.
        """ 
        
        # Check for id conflict
        if node.id in self.connections:
            if self.connections[node.id].node is node:
                # If trying to add a node that already has a connection, do nothing
                return
            else:
                # Otherwise, trying to add conflicting node
                raise ValueError(
                    f"Encountered ID conflict when trying to add node {node.id}")
        
        # If bidirectional, add this node to the target's connections.
        if bidirectional == True:
            try:
                node.add_connection(self, distance, bidirectional=False)
            except ValueError:
                raise ValueError(
                    f"Tried to create bidirectional connection to node {node.id} but \
                    encountered ID conflict in target.")
        
        self.connections[node.id] = Connection(node, distance)

    def __repr__(self) -> str:
        # return f"ID: {self.id}, connections: {('(' + connection.node.id + ', ' + connection.length + ')') for connection in self.connections.values()}"
        return f"ID: {self.id}, connections: {', '.join(f'({connection.node.id}, {round(connection.length, 2)})' for connection in self.connections.values())}"

class Connection:
    """ Class representing a connection between two nodes in a graph. """

    def __init__(self, node, length):
        """ Initialize a connection to a target node. For bidirectional connections, 
        make a Connection in each direction.

        Parameters
        ----------
        Node node : The node to connect to.
        float length : The length of the connection.
        """
        self.node = node
        self.length = length


def remove_from_priority_queue(queue, item):
    """ Go through a priority queue and remove every instance of an item, ensuring the 
    structure of the heap is maintained.

    Parameters
    ----------
    list queue : The priority queue managed using the heapq library
    any item : The item to remove from the queue. All instances of the item will removed.

    Returns
    -------
    list : The updated queue
    """
    
    result = []
    for entry in queue:
        if entry[1] != item:
            result.append(entry)

    return heapq.heapify(result)


def a_star(nodes, start_id, end_id, dijkstra=False):
    """ Use A* to find the shortest path between two nodes in a graph. 

    Parameters
    ----------
    dict nodes : A dictionary of all nodes in the graph, with their IDs as keys and their
        heuristic values as data.
    hashable start_id : The id of the start node.
    hashable end_id : The id of the end node.
    boolean dijkstra : If true, ignore the heuristic and use Dijkstra's algorithm instead.

    Returns
    -------
    list : The list of IDs of nodes comprising the shortest path.
    float : The length of the shortest path
    """


    visited = {}
    distances = {}
    parents = {}
    distances[start_id] = 0
    parents[start_id] = None

    # Create a priority queue using a min-heap, storing (priority, node_id) pairs.
    Q = []
    heapq.heappush(Q, (0, start_id))

    current_node = None

    # Only used for printing verbose output
    iteration = 0

    # Continue looping until the priority queue is empty (all nodes have been visited)
    while Q:
        iteration += 1
        # print(f"Iteration {iteration}:")
        # print("===================================")
        # Process the highest priority node
        current_node = nodes[heapq.heappop(Q)[1]]
        # print(f"Exploring node {current_node.id}, distance {distances[current_node.id]} from start.")

        # Visiting final node, reconstruct and return path and length
        if current_node.id == end_id:
            # print("End node reached, constructing path from parents")
            path = []
            while current_node is not None:
                path.insert(0, current_node)
                current_node = parents[current_node.id]
            # print(f"Shortest path from {start_id} to {end_id}: {' '.join(path)}, total length {distances[end_id]}")
            return path, distances[end_id]

        # Iterate over connections and evaulate each
        for connection in current_node.connections.values():
            # Ignore visited nodes
            if connection.node.id not in visited:
                distance_through_current_node = distances[current_node.id] + connection.length
                heuristic = 0
                if not dijkstra:
                    heuristic = connection.node.data
                # print(f"Heuristic weight: {heuristic}")
                
                # Node to add or update distance found
                if connection.node.id not in distances or distances[connection.node.id] > distance_through_current_node:
                    # if connection.node.id not in distances:
                    #     print(f"Found path to node {connection.node.id} from start with distance {distance_through_current_node}, added to Q")
                    # else:
                        # print(f"Found shorter path to node {connection.node.id} from start. Reducing weight from {distances[connection.node.id] + heuristic} to {[distance_through_current_node + heuristic]}, updated in Q")
                    # Update distances and parents and add to priority queue
                    distances[connection.node.id] = distance_through_current_node
                    parents[connection.node.id] = nodes[current_node.id]
                    heapq.heappush(Q, (distance_through_current_node + heuristic, connection.node.id))
        
        # Mark the current node as visited
        visited[current_node.id] = True

        # visited_string = ' '.join(visited.keys())
        # if (current_node.id == start_id):
        #     current_parent = "."
        # else:
        #     current_parent = parents[visited_string[-1]].id
        #
        # parent_strings = ["."]
        # for node in visited.keys():
        #     if parents[node] is None:
        #         continue
        #     parent_strings.append(parents[node].id)

        # print(f"Expanded:\t{visited_string}({parent_strings[-1]},{distances[current_node.id]})")
        # print(f"Q:\t\t{Q}")

        # print("\n")
