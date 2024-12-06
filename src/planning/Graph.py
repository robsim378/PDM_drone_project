import heapq
from src.planning.Node import Node

class Graph:
    """ Class representing a graph. """

    def __init__(self, nodes=None):
        """ Initialize a Graph.

        Parameters
        ----------
        Node[] nodes : (Optional) A list of nodes to initialize the graph with.
        """

        # Nodes are stored in a dictionary where the keys are the node's ID
        self.nodes = {}
        if nodes is not None:
            for node in nodes:
                self.add_node(node)


    def add_node(self, node):
        """ Add a node to the graph

        Parameters
        ----------
        Node node : The node to add to the graph

        Throws
        ------
        ValueError : If the id of the given node conflicts with an existing node in the
            graph, and it is not the same object.
        """

        # Check for ID conflict
        if node.id in self.nodes.keys():
            if self.nodes[node.id] is node:
                # If the node is already in the graph, do nothing
                return
            else:
                raise ValueError(f"Encountered ID conflict when trying to add node {node.id} to graph.")

        self.nodes[node.id] = node

    def remove_node(self, node_id):
        """ Remove a node from the graph

        Parameters
        ----------
        hashable ID : The ID of the node to remove from the graph

        Throws
        ------
        ValueError : If there is no node in the graph with this ID.
        """
        try:
            del self.nodes[node_id]
        except:
            raise ValueError(f"Tried to remove node {node_id} from graph, but it was not found.")


    def a_star_heuristic(self, node_id, goal_id):
        """ Get the value of the heuristic used for A* for a given node. Uses the data 
        field of the node, using 0 if it is not convertible to float.

        Parameters
        ----------
        hashable node_id : The ID of the node to get the heuristic value of
        hashable goal_id : The ID of the goal node. Unused in this implementation, 
            but useful for subclasses.

        Returns
        -------
        float : The heuristic value of the node

        Throws
        ------
        ValueError : If the node does not exist
        """

        if node_id not in self.nodes.keys():
            raise ValueError(f"Tried to get heuristic value of node {node_id}, but it is not in the graph.")
        if goal_id not in self.nodes.keys():
            raise ValueError(f"Tried to get heuristic value of node {node_id}, but goal node {goal_id} is not in the graph.")

        try:
            # If not convertible to float, this will raise an exception
            return float(self.nodes[node_id].data)
        except:
            return 0

    def a_star(self, start_id, end_id, dijkstra=False):
        """ Use A* to find the shortest path between two nodes in a graph. 

        Parameters
        ----------
        hashable start_id : The id of the start node.
        hashable end_id : The id of the end node.
        boolean dijkstra : If true, ignore the heuristic and use Dijkstra's algorithm instead.

        Returns
        -------
        (list, float) : A tuple containing:
            - The list of IDs of nodes comprising the shortest path
            - The length of the shortest path
        None : If no path exists

        Throws
        ------
        ValueError : If the start or end nodes are not in the graph
        """

        # Check that the start and end nodes are in the graph
        if start_id not in self.nodes.keys():
            raise ValueError(f"Tried to find path from node {start_id}, but it is not in the graph.")
        if end_id not in self.nodes.keys():
            raise ValueError(f"Tried to find path to node {end_id}, but it is not in the graph.")

        visited = {}
        distances = {}
        parents = {}
        distances[start_id] = 0
        parents[start_id] = None

        # Create a priority queue using a min-heap, storing (priority, node_id) pairs.
        Q = []
        heapq.heappush(Q, (0, start_id))

        current_node = None

        # Continue looping until the priority queue is empty (all nodes have been visited)
        while Q:
            # Process the highest priority node
            current_node = self.nodes[heapq.heappop(Q)[1]]

            # Visiting final node, reconstruct and return path and length
            if current_node.id == end_id:
                # print("End node reached, constructing path from parents")
                path = []
                while current_node is not None:
                    path.insert(0, current_node)
                    current_node = parents[current_node.id]
                return path, distances[end_id]

            # Iterate over connections and evaulate each
            for connection in current_node.connections.values():
                # Ignore visited nodes
                if connection.node.id not in visited:
                    distance_through_current_node = distances[current_node.id] + connection.length
                    heuristic = 0
                    if not dijkstra:
                        heuristic = self.a_star_heuristic(connection.node.id, end_id)
                    
                    # Node to add or update distance found
                    if connection.node.id not in distances or distances[connection.node.id] > distance_through_current_node:
                        distances[connection.node.id] = distance_through_current_node
                        parents[connection.node.id] = self.nodes[current_node.id]
                        heapq.heappush(Q, (distance_through_current_node + heuristic, connection.node.id))
            
            # Mark the current node as visited
            visited[current_node.id] = True

        # Q is empty and path not found, no path exists.
        return None
