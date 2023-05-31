import java.util.*;

/**
 * Implementation of Breadth-First Search algorithm for a weighted graph.
 *
 * @param <V> the type of vertex in the graph
 */
public class BFS<V> implements Search<V> {
    private WeightedGraph<V> graph;

    /**
     * Constructs a BFS object with the specified weighted graph.
     *
     * @param graph the weighted graph to perform BFS on
     */
    public BFS(WeightedGraph<V> graph) {
        this.graph = graph;
    }

    /**
     * Performs breadth-first search starting from the specified start vertex.
     *
     * @param startVertex the start vertex for the BFS
     * @return a list of vertices visited during BFS traversal
     */
    public List<Vertex<V>> breadthFirstSearch(Vertex<V> startVertex) {
        List<Vertex<V>> visited = new ArrayList<>();
        Queue<Vertex<V>> queue = new LinkedList<>();

        visited.add(startVertex);
        queue.add(startVertex);

        while (!queue.isEmpty()) {
            Vertex<V> currentVertex = queue.poll();
            List<WeightedGraph<V>.Edge<V>> edges = graph.getEdge(currentVertex);

            for (WeightedGraph<V>.Edge<V> edge : edges) {
                Vertex<V> neighborVertex = edge.getDestination();
                if (!visited.contains(neighborVertex)) {
                    visited.add(neighborVertex);
                    queue.add(neighborVertex);
                }
            }
        }

        return visited;
    }

    /**
     * Prints the vertices visited during BFS traversal starting from the specified start vertex.
     *
     * @param startVertex the start vertex for the BFS
     */
    public void printBFS(Vertex<V> startVertex) {
        Set<Vertex<V>> visited = new HashSet<>();
        Queue<Vertex<V>> queue = new LinkedList<>();

        visited.add(startVertex);
        queue.offer(startVertex);

        while (!queue.isEmpty()) {
            Vertex<V> currentVertex = queue.poll();
            System.out.print(currentVertex.getData() + " ");

            List<WeightedGraph<V>.Edge<V>> edges = graph.getEdge(currentVertex);
            for (WeightedGraph<V>.Edge<V> edge : edges) {
                Vertex<V> adjacentVertex = edge.getDestination();
                if (!visited.contains(adjacentVertex)) {
                    visited.add(adjacentVertex);
                    queue.offer(adjacentVertex);
                }
            }
        }

        System.out.println();
    }

    /**
     * Finds a path from the source vertex to the destination vertex using BFS.
     *
     * @param source      the source vertex
     * @param destination the destination vertex
     * @return a list of vertices representing the path from the source to the destination, or an empty list if no path is found
     */
    @Override
    public List<V> findPath(Vertex<V> source, Vertex<V> destination) {
        List<Vertex<V>> path = new ArrayList<>();
        Queue<Vertex<V>> queue = new LinkedList<>();
        Map<Vertex<V>, Vertex<V>> parentMap = new HashMap<>();

        queue.offer(source);
        parentMap.put(source, null);

        while (!queue.isEmpty()) {
            Vertex<V> currentVertex = queue.poll();
            if (currentVertex.equals(destination)) {
                // Destination vertex found, reconstruct the path
                path.add(destination);
                Vertex<V> parent = parentMap.get(destination);
                while (parent != null) {
                    path.add(0, parent);
                    parent = parentMap.get(parent);
                }
                break;
            }

            List<WeightedGraph<V>.Edge<V>> edges = graph.getEdge(currentVertex);
            for (WeightedGraph<V>.Edge<V> edge : edges) {
                Vertex<V> neighborVertex = edge.getDestination();
                if (!parentMap.containsKey(neighborVertex)) {
                    queue.offer(neighborVertex);
                    parentMap.put(neighborVertex, currentVertex);
                }
            }
        }

        return (List<V>) path;
    }
}
