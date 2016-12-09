package solver;

import problem.ArmConfig;

import java.util.*;

/**
 * Created by Stuey on 14/09/2016.
 */
public class StateGraph{



    //private ArrayList<ArmConfig> vertices;
    HashMap<ArmConfig, HashSet<ArmConfig>> vertices = new HashMap<>();
    //Use Map(Config -> HashSet<Configs>)


    public  StateGraph() {
        this.vertices = new HashMap<>();
    }

    public void addVertex(ArmConfig c) {
        vertices.putIfAbsent(c, new HashSet<ArmConfig>());
    }

    public Set<ArmConfig> getVertices(){
        return vertices.keySet();
    }

    public void addEdge(ArmConfig c1, ArmConfig c2){
        vertices.get(c1).add(c2);
        vertices.get(c2).add(c1);
    }

    public void removeEdge (ArmConfig c1, ArmConfig c2){
        vertices.get(c1).remove(c2);
        vertices.get(c2).remove(c1);
    }

    public void removeVertex (ArmConfig c1){
        vertices.remove(c1);
    }

    public HashSet<ArmConfig> getEdges(ArmConfig c) {
        return vertices.get(c);
    }

    public int edgeCount(ArmConfig c1){
        return vertices.get(c1).size();
    }

    public int size (){
        return vertices.size();
    }

}
