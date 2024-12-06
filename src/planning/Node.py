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
                    f"Encountered ID conflict when trying to add connection from node {self.id} to node {node.id}")
        
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
