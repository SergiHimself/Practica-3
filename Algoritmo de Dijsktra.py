import networkx as nx
import matplotlib.pyplot as plt
from heapq import heappop, heappush

def dijkstra(graph, start):
    # Inicializar todas las distancias con infinito, excepto el nodo de inicio que tiene distancia 0
    distances = {node: float('inf') for node in graph}
    distances[start] = 0
    
    # Utilizar una cola de prioridad (heap) para seleccionar el siguiente nodo con la distancia más corta
    heap = [(0, start)]
    
    while heap:
        # Obtener el nodo actual y la distancia acumulada hasta ese nodo
        current_distance, current_node = heappop(heap)
        
        # Si ya se encontró una distancia más corta para este nodo, continuar con el siguiente
        if current_distance > distances[current_node]:
            continue
        
        # Explorar los vecinos del nodo actual
        for neighbor, weight in graph[current_node].items():
            # Calcular la distancia acumulada hasta el vecino a través del nodo actual
            distance = current_distance + weight
            
            # Si se encontró un camino más corto hacia el vecino, actualizar la distancia
            if distance < distances[neighbor]:
                distances[neighbor] = distance
                # Agregar el vecino a la cola de prioridad con la nueva distancia
                heappush(heap, (distance, neighbor))
    
    return distances

# Ejemplo de grafo
graph = {
    'A': {'B': 4, 'C': 2},
    'B': {'A': 4, 'D': 5},
    'C': {'A': 2, 'D': 1},
    'D': {'B': 5, 'C': 1, 'E': 3},
    'E': {'D': 3}
}

start_node = 'A'
distances = dijkstra(graph, start_node)

# Graficar el grafo
G = nx.Graph(graph)
pos = nx.spring_layout(G)

# Etiquetas de los nodos (incluyendo las distancias calculadas por Dijkstra)
labels = {node: f"{node} ({distance})" for node, distance in distances.items()}

# Colorear el nodo de inicio en rojo y los demás en azul
node_colors = ['red' if node == start_node else 'blue' for node in G.nodes()]

# Dibujar el grafo
nx.draw_networkx_nodes(G, pos, node_color=node_colors)
nx.draw_networkx_edges(G, pos)
nx.draw_networkx_labels(G, pos, labels)

# Mostrar el grafo
plt.axis('off')
plt.show()
