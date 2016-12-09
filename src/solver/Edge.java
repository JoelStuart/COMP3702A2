package solver;

/**
 * Created by Stuey on 14/09/2016.
 */
/**
 * This class implements an element of an adjacency linked-list in the adjacency
 * lists graph representation.
 */
public class Edge {
    /**
     * The target vertex.
     */
    public int target;

    /**
     * The weight of the corresponding edge
     */
    public double weight;

    /**
     * Class constructor.
     */
    public Edge(int target, double weight) {
        this.target = target;
        this.weight = weight;
    }

    public String toString() {
        return "(" + target + "; " + (float) weight + ")";
    }
}