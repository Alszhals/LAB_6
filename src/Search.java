import java.util.List;

/**
 * Represents a search algorithm for finding a path in a graph.
 *
 * @param <V> the type of data stored in the vertices
 */
public interface Search<V> {
    /**
     * Finds a path from the source vertex to the destination vertex in a graph.
     *
     * @param source      the source vertex
     * @param destination the destination vertex
     * @return a list of data representing the path from the source to the destination
     */
    List<V> findPath(Vertex<V> source, Vertex<V> destination);
}
