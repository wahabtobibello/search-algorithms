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


class TreeNode(Node):
    def __init__(self, value, parent=None):
        super().__init__(value)
        self.parent = parent


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
        current_node = open_list.popleft()
        if current_node.value == goal:
            return current_node
        open_list.extend(current_node.adjacent_nodes - visited)
        visited.add(current_node)
    return False


def bidirectional_search(startNode, goalNode):
    visited_for_root_search = set()
    visited_for_goal_search = set()
    open_list_for_root_search = deque([startNode])
    open_list_for_goal_search = deque([goalNode])
    startNode.parent = None
    goalNode.parent = None

    while True:
        current_node_for_root_search = open_list_for_root_search.popleft()
        current_node_for_goal_search = open_list_for_goal_search.popleft()

        open_list_for_root_search.extend(
            current_node_for_root_search.adjacent_nodes - visited_for_root_search)
        visited_for_root_search.add(current_node_for_root_search)
        open_list_for_goal_search.extend(
            current_node_for_goal_search.adjacent_nodes - visited_for_goal_search)
        visited_for_goal_search.add(current_node_for_goal_search)

        if visited_for_root_search & visited_for_goal_search:
            # return the path from root to goal
            return True
    return False


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


def initial_graph_for_bidirectional_search():
    A = TreeNode('A')
    B = TreeNode('B')
    C = TreeNode('C')
    D = TreeNode('D')
    E = TreeNode('E')
    F = TreeNode('F')
    G = TreeNode('G')
    H = TreeNode('H')
    I = TreeNode('I')
    J = TreeNode('J')
    K = TreeNode('K')
    L = TreeNode('L')
    M = TreeNode('M')
    N = TreeNode('N')
    Node.connect_nodes(A, B)
    Node.connect_nodes(A, C)
    Node.connect_nodes(B, D)
    Node.connect_nodes(B, E)
    Node.connect_nodes(C, F)
    Node.connect_nodes(D, G)
    Node.connect_nodes(E, H)
    Node.connect_nodes(F, H)
    Node.connect_nodes(G, I)
    Node.connect_nodes(H, J)
    Node.connect_nodes(H, K)
    Node.connect_nodes(I, L)
    Node.connect_nodes(J, L)
    Node.connect_nodes(K, M)
    Node.connect_nodes(L, N)
    Node.connect_nodes(M, N)
    return A, N


if __name__ == '__main__':
    # root = initial_graph()
    # output = depth_first_search(root, 'G')
    # output = depth_limited_search(root, 'G', 2)
    # output = iterative_deepening_search(root, 'G', 2)
    # output = breadth_first_search(root, 'G')
    # print(output.value) if output else print(output)

    root, goalNode = initial_graph_for_bidirectional_search()
    output_path = bidirectional_search(root, goalNode)
    print(output_path)
