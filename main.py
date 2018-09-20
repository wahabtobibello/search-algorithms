from collections import deque


class Node:
    def __init__(self, value):
        self.value = value
        self.adjacent_nodes = set()

    def add_adjacent_node(self, node):
        self.adjacent_nodes.add(node)

    @staticmethod
    def connect_nodes(node1, node2):
        node1.add_adjacent_node(node2)
        node2.add_adjacent_node(node1)


def depth_first_search(startNode, goal):
    visited, open_list = set(), [startNode]
    while len(open_list) > 0:
        current_node = open_list.pop()
        if current_node.value == goal:
            return current_node
        open_list.extend(current_node.adjacent_nodes - visited)
        visited.add(current_node)
    return False


def depth_limited_search(startNode, goal, max_depth):
    visited, open_list = set(), [(startNode, 0)]
    while len(open_list) > 0:
        current_node, depth = open_list.pop()
        if current_node.value == goal:
            return current_node
        if depth + 1 <= max_depth:
            open_list.extend([(node, depth + 1)
                              for node in current_node.adjacent_nodes - visited])
        visited.add(current_node)
    return False


def iterative_deepening_search(startNode, goal, max_depth):
    for i in range(max_depth + 1):
        output = depth_limited_search(startNode, goal, i)
        if output:
            return output


def breadth_first_search(startNode, goal):
    visited, open_list = set(), deque([startNode])
    while len(open_list) > 0:
        print(list(node.value for node in open_list))
        current_node = open_list.popleft()
        if current_node.value == goal:
            return current_node
        open_list.extend(current_node.adjacent_nodes - visited)
        visited.add(current_node)
    return False


def bidirectional_search(startNode, goalNode):
    pass


def uniform_cost_search():
    pass


def initial_graph():
    A = Node('A')
    B = Node('B')
    C = Node('C')
    D = Node('D')
    E = Node('E')
    F = Node('F')
    G = Node('G')
    H = Node('H')
    I = Node('I')
    J = Node('J')
    K = Node('K')
    Node.connect_nodes(A, B)
    Node.connect_nodes(A, C)
    Node.connect_nodes(B, D)
    Node.connect_nodes(B, E)
    Node.connect_nodes(C, F)
    Node.connect_nodes(C, G)
    Node.connect_nodes(D, H)
    Node.connect_nodes(D, I)
    Node.connect_nodes(E, J)
    Node.connect_nodes(E, K)
    return A


# root = initial_graph()

# output = depth_first_search(root, 'G')
# output = depth_limited_search(root, 'G', 2)
# output = iterative_deepening_search(root, 'G', 2)
# output = breadth_first_search(root, 'G')

# print(output.value) if output else print(output)
