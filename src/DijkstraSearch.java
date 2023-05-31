import java.util.*;

public class DijkstraSearch<V> implements Search<V> {

    private WeightedGraph<V> graph;
    public void printDijkstra(Vertex<V> startVertex) {
        Map<Vertex<V>, Double> distances = dijkstraSearch(startVertex);

        System.out.println("Dijkstra's Algorithm Results:");
        for (Vertex<V> vertex : distances.keySet()) {
            Double distance = distances.get(vertex);
            String distanceString = (distance == Double.POSITIVE_INFINITY) ? "Infinity" : String.valueOf(distance);
            System.out.println("Vertex: " + vertex.getData() + ", Distance: " + distanceString);
        }
    }


    public DijkstraSearch(WeightedGraph<V> graph) {
        this.graph = graph;
    }

    public Map<Vertex<V>, Double> dijkstraSearch(Vertex<V> startVertex) {
        Map<Vertex<V>, Double> distances = new HashMap<>();
        PriorityQueue<DijkstraNode<V>> priorityQueue = new PriorityQueue<>();

        for (Vertex<V> vertex : graph.getVertices()) {
            distances.put(vertex, Double.POSITIVE_INFINITY);
        }

        distances.put(startVertex, 0.0);
        priorityQueue.offer(new DijkstraNode<>(startVertex, 0.0));

        while (!priorityQueue.isEmpty()) {
            DijkstraNode<V> currentNode = priorityQueue.poll();
            Vertex<V> currentVertex = currentNode.getVertex();
            double currentDistance = currentNode.getDistance();

            for (WeightedGraph<V>.Edge<V> edge : graph.getEdge(currentVertex)) {
                Vertex<V> neighborVertex = edge.getDestination();
                double edgeWeight = edge.getWeight();
                double distanceThroughCurrent = currentDistance + edgeWeight;

                if (distanceThroughCurrent < distances.get(neighborVertex)) {
                    distances.put(neighborVertex, distanceThroughCurrent);
                    priorityQueue.offer(new DijkstraNode<>(neighborVertex, distanceThroughCurrent));
                }
            }
        }

        return distances;
    }

    public List<V> findPath(Vertex<V> source, Vertex<V> destination) {
        Map<Vertex<V>, Double> distances = dijkstraSearch(source);
        Map<Vertex<V>, Vertex<V>> parentMap = new HashMap<>();
        PriorityQueue<DijkstraNode<V>> priorityQueue = new PriorityQueue<>();

        for (Vertex<V> vertex : graph.getVertices()) {
            parentMap.put(vertex, null);
        }

        parentMap.put(source, null);
        priorityQueue.offer(new DijkstraNode<>(source, 0.0));

        while (!priorityQueue.isEmpty()) {
            DijkstraNode<V> currentNode = priorityQueue.poll();
            Vertex<V> currentVertex = currentNode.getVertex();
            double currentDistance = currentNode.getDistance();

            if (currentVertex.equals(destination)) {
                break;
            }

            for (WeightedGraph<V>.Edge<V> edge : graph.getEdge(currentVertex)) {
                Vertex<V> neighborVertex = edge.getDestination();
                double edgeWeight = edge.getWeight();
                double distanceThroughCurrent = currentDistance + edgeWeight;

                if (distanceThroughCurrent < distances.get(neighborVertex)) {
                    distances.put(neighborVertex, distanceThroughCurrent);
                    parentMap.put(neighborVertex, currentVertex);
                    priorityQueue.offer(new DijkstraNode<>(neighborVertex, distanceThroughCurrent));
                }
            }
        }

        List<V> path = new ArrayList<>();
        Vertex<V> currentVertex = destination;

        while (currentVertex != null) {
            path.add(0, currentVertex.getData());
            currentVertex = parentMap.get(currentVertex);
        }

        return path;
    }

    private static class DijkstraNode<V> implements Comparable<DijkstraNode<V>> {

        private Vertex<V> vertex;
        private double distance;

        public DijkstraNode(Vertex<V> vertex, double distance) {
            this.vertex = vertex;
            this.distance = distance;
        }

        public Vertex<V> getVertex() {
            return vertex;
        }

        public double getDistance() {
            return distance;
        }

        @Override
        public int compareTo(DijkstraNode<V> other) {
            return Double.compare(distance, other.distance);
        }
    }
}
