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
        current_node = open_list.popleft()
        if current_node.value == goal:
            return current_node
        open_list.extend(current_node.adjacent_nodes - visited)
        visited.add(current_node)
    return False


def bidirectional_search(startNode, goalNode):
    visited_for_root_search = set()
    visited_for_goal_search = set()
    open_list_for_root_search = deque([(startNode, [startNode])])
    open_list_for_goal_search = deque([(goalNode, [goalNode])])

    while len(open_list_for_root_search) > 0 and len(open_list_for_goal_search) > 0:
        print

        current_node_for_root_search, path_for_root_search = open_list_for_root_search.popleft()
        current_node_for_goal_search, path_for_goal_search = open_list_for_goal_search.popleft()

        node_children_for_root_search = current_node_for_root_search.adjacent_nodes - \
            visited_for_root_search
        open_list_for_root_search.extend(
            [(child, path_for_root_search+[child]) for child in node_children_for_root_search])
        visited_for_root_search.add(current_node_for_root_search)

        node_children_for_goal_search = current_node_for_goal_search.adjacent_nodes - \
            visited_for_goal_search
        open_list_for_goal_search.extend(
            [(child, path_for_goal_search+[child]) for child in node_children_for_goal_search])
        visited_for_goal_search.add(current_node_for_goal_search)

        if visited_for_root_search & visited_for_goal_search:
            # return the path from root to goal
            return path_for_root_search + path_for_goal_search[-2::-1]
    return False


def uniform_cost_search(startNode, edge_costs, goal):
    visited, open_list = set(), [(startNode, [startNode],  0)]
    while len(open_list) > 0:
        current_node, path, cost = open_list.pop()
        if current_node.value == goal:
            return path, cost

        for node in current_node.adjacent_nodes - visited:
            node_path = path[:]
            node_path.append(node)
            node_cost = edge_costs[(current_node, node)]
            open_list.append((node, node_path, cost + node_cost))

        open_list.sort(key=lambda key: key[2], reverse=True)
        visited.add(current_node)
    return False


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
    L = Node('L')
    M = Node('M')
    N = Node('N')
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


def initial_graph_for_uniform_cost_search():
    A = Node('A')
    B = Node('B')
    C = Node('C')
    D = Node('D')
    E = Node('E')

    Node.connect_nodes(A, B)
    Node.connect_nodes(A, C)
    Node.connect_nodes(A, D)
    Node.connect_nodes(B, E)
    Node.connect_nodes(C, E)
    Node.connect_nodes(D, E)

    edge_costs = {}
    edge_costs[(A, B)] = 5
    edge_costs[(A, C)] = 1
    edge_costs[(A, D)] = 2
    edge_costs[(B, E)] = 1
    edge_costs[(C, E)] = 7
    edge_costs[(D, E)] = 5
    return A, edge_costs


if __name__ == '__main__':
    root = initial_graph()
    # output = depth_first_search(root, 'G')
    # output = depth_limited_search(root, 'G', 2)
    # output = iterative_deepening_search(root, 'G', 2)
    # output = breadth_first_search(root, 'G')
    # print(output.value) if output else print(output)

    # root, goalNode = initial_graph_for_bidirectional_search()
    # output_path = bidirectional_search(root, goalNode)
    # print([node.value for node in output_path])

    # root, edge_costs = initial_graph_for_uniform_cost_search()
    # output_path, cost = uniform_cost_search(root, edge_costs, 'E')
    # print([node.value for node in output_path], cost)
