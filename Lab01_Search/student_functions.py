import numpy as np
from collections import deque
import heapq


def BFS(matrix, start, end):
    """
    BFS algorithm:
    Parameters:
    ---------------------------
    matrix: np array 
        The graph's adjacency matrix
    start: integer
        starting node
    end: integer
        ending node
    
    Returns
    ---------------------
    visited
        The dictionary contains visited nodes, each key is a visited node,
        each value is the adjacent node visited before it.
    path: list
        Founded path
    """
    # TODO:
    queue = deque([start])
    visited = {start: None}

    while queue:
        current = queue.popleft()
        if current == end or start == end: #nếu đến đích hoặc điểm bắt đầu trùng với điểm kết thúc thì dừng
            break
        for neighbor in range(len(matrix[current])): #duyệt các nút kề
            if matrix[current][neighbor] != 0 and neighbor not in visited:
                queue.append(neighbor)
                visited[neighbor] = current
    #Lấy đường đi
    path = []
    if end in visited:
        step = end
        while step is not None:
            path.insert(0, step)
            step = visited[step]

    return visited, path


def DFS(matrix, start, end):
    """
    DFS algorithm
     Parameters:
    ---------------------------
    matrix: np array 
        The graph's adjacency matrix
    start: integer 
        starting node
    end: integer
        ending node
    
    Returns
    ---------------------
    visited 
        The dictionary contains visited nodes: each key is a visited node, 
        each value is the key's adjacent node which is visited before key.
    path: list
        Founded path
    """

    # TODO: 
    stack = [start]
    visited = {start: None}

    while stack:
        current = stack.pop()
        if current == end or start == end: #nếu đến đích hoặc điểm bắt đầu trùng với điểm kết thúc thì dừng
            break
        #Duyệt các nút kề
        for neighbor in range(len(matrix[current])):
            if matrix[current][neighbor] != 0 and neighbor not in visited:
                stack.append(neighbor)
                visited[neighbor] = current
    #Lấy đường đi
    path = []
    if end in visited:
        step = end
        while step is not None:
            path.insert(0, step)
            step = visited[step]

    return visited, path


def UCS(matrix, start, end):
    """
    Uniform Cost Search algorithm
     Parameters:visited
    ---------------------------
    matrix: np array
        The graph's adjacency matrix
    start: integer
        starting node
    end: integer
        ending node
    
    Returns
    ---------------------
    visited
        The dictionary contains visited nodes: each key is a visited node, 
        each value is the key's adjacent node which is visited before key.
    path: list
        Founded path
    """
    # TODO:  
    heap = [(0, start)] #chi phí và node trong hàng đợi
    dic_cost = {start: 0} #Dictionary chứa các node và chi phí
    visited = {start: None}

    while heap:

        current_cost, current_node = heapq.heappop(heap)
        #Nếu đến đích thì dừng và lấy đường đi cho đến khi None
        if current_node == end:
            path = []
            while current_node is not None:
                path.append(current_node)
                current_node = visited[current_node]
            path.reverse() #đảo ngược để có đường đi đúng
            return visited, path

        #Duyệt các nút kề
        for neighbor, cost in enumerate(matrix[current_node]):
            if cost > 0:
                new_cost = dic_cost[current_node] + cost #Tổng chi phí cộng dồn
                if neighbor not in dic_cost or new_cost < dic_cost[neighbor]:
                    dic_cost[neighbor] = new_cost
                    visited[neighbor] = current_node
                    heapq.heappush(heap, (new_cost, neighbor))
    return visited, []


def GBFS(matrix, start, end):
    """
    Greedy Best First Search algorithm
    heuristic : edge weights
     Parameters:
    ---------------------------
    matrix: np array
        The graph's adjacency matrix
    start: integer
        starting node
    end: integer
        ending node

    Returns
    ---------------------
    visited
        The dictionary contains visited nodes: each key is a visited node,
        each value is the key's adjacent node which is visited before key.
    path: list
        Founded path
    """
    # TODO:
    heap = [(0, start)]
    visited = {start: None}

    while heap:

        current_cost, current_node = heapq.heappop(heap)
        #Nếu đến đích thì dừng và lấy đường đi
        if current_node == end:
            path = []
            while current_node is not None:
                path.append(current_node)
                current_node = visited[current_node]
            path.reverse()
            return visited, path
        #Duyệt các nút kề
        for neighbor, cost in enumerate(matrix[current_node]):
            if cost > 0 and neighbor not in visited:
                visited[neighbor] = current_node
                heapq.heappush(heap, (cost, neighbor))
    return visited, []


#Hàm heuristic tính khoảng cách Euclid từ nút hiện tại đến đích
def heuristic(pos, node, end):
    x1, y1 = pos[node]
    x2, y2 = pos[end]
    return np.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)


def Astar(matrix, start, end, pos):
    """
    A* Search algorithm
    heuristic: eclid distance based positions parameter
     Parameters:
    ---------------------------
    matrix: np array UCS
        The graph's adjacency matrix
    start: integer
        starting node
    end: integer
        ending node
    pos: dictionary. keys are nodes, values are positions
        positions of graph nodes
    Returns
    ---------------------
    visited
        The dictionary contains visited nodes: each key is a visited node,
        each value is the key's adjacent node which is visited before key.
    path: list
        Founded path
    """
    # TODO:
    heap = [(0, start)]
    visited = {start: None}
    dic_cost = {start: 0} #Chứa các node và chi phí tương ứng (Không bao gồm heuristic)
    while heap:

        current_cost, current_node = heapq.heappop(heap)
        #Nếu đến đích thì dừng và lấy đường đi
        if current_node == end:
            path = []
            while current_node is not None:
                path.append(current_node)
                current_node = visited[current_node]
            path.reverse()
            return visited, path
        #Duyệt các node kề
        for neighbor, cost in enumerate(matrix[current_node]):
            if cost > 0:
                new_cost = dic_cost[current_node] + cost
                #Nếu chưa thăm hoặc có đường đi tốt hơn thì duyệt
                if neighbor not in dic_cost or new_cost < dic_cost[neighbor]:
                    dic_cost[neighbor] = new_cost
                    sum_cost = new_cost + heuristic(pos, neighbor, end) #
                    visited[neighbor] = current_node
                    heapq.heappush(heap, (sum_cost, neighbor))
    #Trả về nếu không có đường đi
    return visited, []
