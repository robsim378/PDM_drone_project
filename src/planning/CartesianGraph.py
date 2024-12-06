import numpy as np
from src.planning.Graph import Graph
from src.planning.Node import Node

class CartesianGraph(Graph):
    """ Class representing a graph where nodes are points in Cartesian space. """

    def __init__(self, nodes=None):
        """ Initialize a CartesianGraph.

        Parameters
        ----------
        Node[] nodes : (Optional) A list of nodes to initialize the graph with. Data 
            field of all nodes must be a tuple of floats representing (x, y, z) coordinates
        """
        super().__init__(nodes)

    def add_node(self, node):
        """ Add a node to the graph

        Parameters
        ----------
        Node node : The node to add to the graph. Data field must be a tuple of floats
            representing (x, y, z) coordinates.

        Throws
        ------
        ValueError : If the id of the given node conflicts with an existing node in the
            graph, and it is not the same object.
        ValueError : If the data field of the node is not a valid set of coordinates
        """
        if isinstance(node.data, tuple) and len(node.data) == 3:
            try:
                float(node.data[0])
                float(node.data[1])
                float(node.data[2])
            except:
                raise ValueError("Tried to add node to CartesianGraph without valid Cartesian coordinates as data.")

        super().add_node(node)


    def a_star_heuristic(self, node_id, goal_id):
        """ Get the value of the heuristic used for A* for a given node. Uses the 
        straight line distance from the node to the goal node.

        Parameters
        ----------
        hashable node_id : The ID of the node to get the heuristic value of
        hashable goal_id : The ID of the goal node.

        Returns
        -------
        float : The heuristic value of the node

        Throws
        ------
        ValueError : If the node or goal node does not exist.
        """

        if node_id not in self.nodes.keys():
            raise ValueError(f"Tried to get heuristic value of node {node_id}, but it is not in the graph.")
        if goal_id not in self.nodes.keys():
            raise ValueError(f"Tried to get heuristic value of node {node_id}, but goal node {goal_id} is not in the graph.")

        return self.distance_between_nodes(node_id, goal_id)


    def distance_between_nodes(self, node_a_id, node_b_id):
        """ Compute the straight-line distance between two nodes. 

        Parameters
        ----------
        hashable node_a_id : The ID of one of the nodes
        hashable node_b_id : The ID of the other node

        Returns
        -------
        float : The straight-line distance between the two nodes.
        """

        # The float() cast is probably unneccessary, but my linter was complaining about 
        # a type mismatch and I wanted to get rid of the error
        return float(np.linalg.norm(
            np.array(self.nodes[node_a_id].data) - np.array(self.nodes[node_b_id].data)
        ))

    def get_closest_neighbour(self, node_id):
        """ Gets the ID of the closest neighbour of the target node.
        
        Parameters
        ----------
        hashable node_id : The ID of the node to find the closest neighbour of

        Returns
        -------
        hashable : The ID of the closest neighbour
        None : If there is only one node in the graph

        Throws
        ------
        ValueError : If the node does not exist.
        """

        if node_id not in self.nodes.keys():
            raise ValueError(f"Tried to find the closest neighbour of node {node_id}, but it is not in the graph.")

        # TODO: This just uses a naive brute-force solution by checking the distance to
        # every node in the graph. This will scale poorly (O(N)) compared to alternative
        # methods. Look into using a KD-tree, which is O(log(N)) for this.

        closest = None
        distance = np.inf

        for node in self.nodes.values():
            if node.id == node_id:
                continue
            current_distance = self.distance_between_nodes(node_id, node.id)
            if current_distance < distance:
                closest = node.id
                distance = current_distance

        return closest

    def get_neighbours(self, node_id, distance):
        """ Gets the IDs of all the nodes within a certain distance of the target node.

        Parameters
        ----------
        hashable node_id : The ID of the node to find the neighbours of

        Returns
        -------
        hashable[] : The IDs of the neighbours

        Throws
        ------
        ValueError : If the node does not exist.
        """

        if node_id not in self.nodes.keys():
            raise ValueError(f"Tried to find the closest neighbour of node {node_id}, but it is not in the graph.")

        # TODO: As with get_closest_neighbour, this is a naive and inefficient solution.
        # The use of a KD-tree would improve it quite a bit

        neighbours = []

        for node in self.nodes.values():
            if node.id == node_id:
                continue
            if self.distance_between_nodes(node_id, node.id) < distance:
                neighbours.append(node.id)

        return neighbours

    def get_node_from_coordinates(self, coordinates):
        """ Gets the ID of the closest node to the given coordinates.
        
        Parameters
        ----------
        tuple(float, float, float) coordinates : The (x, y, z) coordinates to find the closest node to

        Returns
        -------
        hashable : The ID of the closest node to the given coordinates

        Throws
        ------
        ValueError : If the coordinates given are not valid.
        """

        # TODO: As with all the functions relating to cartesian space, this could be greatly
        # improved using a KD tree.


        # Check if there is already a node with these exact coordinates in the graph
        for node in self.nodes.values():
            if node.data == coordinates:
                return node.id
    
        # If not, create a temporary node at the desired position and find the 
        # closest neighbour, then remove the node.
        temp_node = Node(data=coordinates)
        self.add_node(temp_node)
        
        try:
            closest = self.get_closest_neighbour(temp_node.id)
            print(f"Closest node to {coordinates}: {self.nodes[closest].data}")
        except:
            # Ensure the temporary node is removed even in case of an error, then reraise the error
            self.remove_node(temp_node.id)
            raise

        self.remove_node(temp_node.id)
        return closest
        

