from collections import deque

# Algoritmo de Ford-Fulkerson
def ford_fulkerson(graph, source, sink):
    # Função para encontrar um caminho aumentante usando BFS
    def bfs(residual_graph, parent):
        visited = set()
        queue = deque([source])
        visited.add(source)

        while queue:
            u = queue.popleft()

            for v in residual_graph[u]:
                if v not in visited and residual_graph[u][v] > 0:  # Se houver capacidade residual
                    queue.append(v)
                    visited.add(v)
                    parent[v] = u
                    if v == sink:
                        return True
        return False

    # Construir o grafo residual
    residual_graph = {u: {} for u in graph}
    for u in graph:
        for v in graph[u]:
            residual_graph[u][v] = graph[u][v]
            if v not in residual_graph:
                residual_graph[v] = {}
            if u not in residual_graph[v]:
                residual_graph[v][u] = 0

    parent = {}  # Array para armazenar o caminho aumentante
    max_flow = 0  # Inicializa o fluxo máximo

    # Enquanto existir um caminho aumentante no grafo residual
    while bfs(residual_graph, parent):
        # Encontrar a capacidade mínima de aresta no caminho aumentante encontrado
        path_flow = float('Inf')
        s = sink
        while s != source:
            path_flow = min(path_flow, residual_graph[parent[s]][s])
            s = parent[s]

        # Atualizar as capacidades residuais das arestas e arestas reversas ao longo do caminho
        v = sink
        while v != source:
            u = parent[v]
            residual_graph[u][v] -= path_flow
            residual_graph[v][u] += path_flow
            v = u

        max_flow += path_flow

    return max_flow

# Algoritmo de Sucessivos Caminhos Mínimos (ASCM)
def ascm(graph, demand):
    # Função para encontrar o caminho de custo mínimo usando o algoritmo de Dijkstra
    def dijkstra(residual_graph, source, sink, potentials):
        import heapq

        dist = {node: float('Inf') for node in residual_graph}
        dist[source] = 0
        parent = {node: None for node in residual_graph}
        priority_queue = [(0, source)]

        while priority_queue:
            d, u = heapq.heappop(priority_queue)

            if d > dist[u]:
                continue

            for v, (capacity, cost) in residual_graph[u].items():
                if capacity > 0:
                    new_dist = dist[u] + cost + potentials[u] - potentials[v]
                    if new_dist < dist[v]:
                        dist[v] = new_dist
                        parent[v] = u
                        heapq.heappush(priority_queue, (new_dist, v))

        return parent if dist[sink] != float('Inf') else None, dist

    # Construir o grafo residual
    residual_graph = {u: {} for u in graph}
    for u in graph:
        for v in graph[u]:
            capacity, cost = graph[u][v]
            residual_graph[u][v] = [capacity, cost]
            if v not in residual_graph:
                residual_graph[v] = {}
            if u not in residual_graph[v]:
                residual_graph[v][u] = [0, -cost]

    # Inicializar variáveis
    source = 'S'
    sink = 'T'
    potentials = {node: 0 for node in residual_graph}
    total_cost = 0
    total_flow = 0

    while True:
        # Encontrar o caminho de custo mínimo no grafo residual
        parent, dist = dijkstra(residual_graph, source, sink, potentials)

        if not parent:
            break

        # Atualizar os potenciais
        for node in potentials:
            if dist[node] < float('Inf'):
                potentials[node] += dist[node]

        # Encontrar a capacidade mínima no caminho encontrado
        path_flow = float('Inf')
        v = sink
        while v != source:
            u = parent[v]
            path_flow = min(path_flow, residual_graph[u][v][0])
            v = u

        # Atualizar as capacidades residuais e custos
        v = sink
        while v != source:
            u = parent[v]
            residual_graph[u][v][0] -= path_flow
            residual_graph[v][u][0] += path_flow
            total_cost += path_flow * residual_graph[u][v][1]
            v = u

        total_flow += path_flow

        # Verificar se a demanda foi atendida
        if total_flow >= demand[sink]:
            break

    return total_flow, total_cost

def main():
    # Grafo para Ford-Fulkerson
    graph_PAGE17 = { 
        "S": {"1" : 10},
        "1": {"2" : 7, "3": 3},
        "2" : {"4" : 3},
        "3" : {"2": 3, "4": 3},
        "4" : {"T": 7}
    }
    # Grafo para Sucessivos Caminhos Mínimos (ASCM)
    graph_PAGE29 = {
        "S": {"1" : (1, 2)},
        "1": {"2" : (2, 1), "3": (3, 1), "4": (5, 2)},
        "2" : {"4" : (2, 1)},
        "3" : {"4": (3, 1)},
        "4" : {"T": (1, 2)}
    }
    
    demand = { 
        "S": -2, # Fornece 2 de fluxo
        "T": 2   # Recebe 2 de fluxo
    }
    
    max_flow = ford_fulkerson(graph_PAGE17, "S", "T")
    print("\nFord-Fulkerson Maximum Flow: ", max_flow)

    flow, cost = ascm(graph_PAGE29, demand)
    print("\nASCM Flow:", flow," Cost: ", cost)
    
if __name__ == "__main__":
    main()
