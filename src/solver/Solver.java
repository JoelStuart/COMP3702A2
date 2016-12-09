package solver;

import problem.ArmConfig;
import problem.ProblemSpec;
import tester.Tester;

import java.awt.geom.Point2D;
import java.io.*;
import java.util.*;


/**
 * Created by Stuey on 8/09/2016.
 */
public class Solver {

    public String outFile;
    public ProblemSpec ps = new ProblemSpec();
    public static final double DEFAULT_MAX_ERROR = 1e-5;
    public static final double STEP_SIZE = 0.001;

    public Tester tester = new Tester(DEFAULT_MAX_ERROR);
    public static final double ANGLE_STEP = 0.001;
    StateGraph G;
    int X = 2200;
    ArmConfig goalState;
    ArmConfig initState;
    List<Double> initAngles;
    List<Double> goalAngles;
    List<Double> initLengths;
    List<Double> goalLengths;
    double maxDistFound = 0;
    boolean solutionFound = false;
    boolean hasGripper = false;
    HashMap<ArmConfig, Double> distFromSource;
    HashMap<ArmConfig, ArmConfig> prev;
    int jointCount = 0;
    boolean pathFlag = false;

    public Solver(String[] args){
        //Read in arguments

        String inFile = args[0];
        //Output filename
        outFile = args[1];

        long startTime = System.currentTimeMillis();

        try {
            ps.loadProblem(inFile);
        } catch (IOException e1) {
            System.out.println("FAILED: Invalid problem file");
            System.out.println(e1.getMessage());
            System.exit(1);
        }
        goalState = ps.getGoalState();
        initState = ps.getInitialState();
        System.out.print("Init state is : " + initState +  " & Goal state is: " + goalState + "\n");
        hasGripper = goalState.hasGripper();
        jointCount = goalState.getJointCount();
        initAngles = initState.getJointAngles();
        goalAngles = goalState.getJointAngles();
        if (hasGripper){
            initLengths = initState.getGripperLengths();
            goalLengths = goalState.getGripperLengths();
        }




        buildGraph();
        long endTime = System.currentTimeMillis();
        System.out.println("Total time: " + (endTime - startTime) + "ms\n");
        Runtime rt = Runtime.getRuntime();
        long usedMB = (rt.totalMemory() - rt.freeMemory()) / 1024 / 1024;

        System.out.println("Memory usage: " + usedMB + "MB\n");

    }

    public void buildGraph() {

        G =  new StateGraph();
        G.addVertex(initState);
        G.addVertex(goalState);
        boolean sw = false;
        while (!solutionFound) {
            Set<ArmConfig> emptySet = new HashSet<ArmConfig>();
            Set<ArmConfig> edgeSet = new HashSet<ArmConfig>();
            for (int i = 0; i < X; i++) {
                ArmConfig s = sample();
                if (s.equals(initState)) {
                    continue;
                }
                G.addVertex(s);
                testPaths(s, emptySet);

            }
            int del = 0;
            if (G.size() > 14000 && !sw){
                sw = !sw;
                for (ArmConfig v : G.getVertices()) {

                    int edges = G.edgeCount(v);
                    if (edges < 1) {
                        emptySet.add(v);
                        del +=1;
                    } /*else if (pathFlag  && edges < 2 && !distFromSource.containsKey(v)){
                        edgeSet.add(v);
                        emptySet.add(v);
                        del +=1;
                    }*/
                }
            } else {
                sw = !sw;
            }
            for (ArmConfig v : edgeSet){
                Set<ArmConfig> e = G.getEdges(v);
                Set<ArmConfig> toRemove = new HashSet<ArmConfig>();
                for (ArmConfig c : e){
                    toRemove.add(c);
                }
                for (ArmConfig c : toRemove){
                    G.removeEdge(v, c);
                }
            }
            for (ArmConfig v : emptySet){
                G.removeVertex(v);
            }
            /*if (del > 0){
                System.out.print("Graph vertices pruned by = " + del + "!\n");
            }*/
            aSearch(initState, goalState);

        }

    }

    //test collision free direct path between s and all configs
    //if yes, add edge between nodes
    public void testPaths(ArmConfig c, Set<ArmConfig> emptySet){
        Set<ArmConfig> configSet =  G.getVertices();

        for(ArmConfig v : configSet){
            if (!c.equals(v)){
                if (naiveTestValidPath(c, v)){
                    G.addEdge(c,v);
                }
            }
        }
    }

    public boolean naiveTestValidPath(ArmConfig source, ArmConfig dest){
        if (ps.getNumObstacles() < 1){
            return true;
        }
        int depth = 3;
        //System.out.print(source.getBaseCenter().distance(dest.getBaseCenter()));
        if (source.getBaseCenter().distance(dest.getBaseCenter()) > 0.05){
            return false;
        }
        if (jointCount > 0){
            List<Double> anglesSource = source.getJointAngles();
            List<Double> anglesDest = dest.getJointAngles();

            for (int i = 0; i < jointCount; i++){
                double dif = Math.abs(anglesSource.get(i)-anglesDest.get(i));
                if (dif > 0.35){
                    return false;
                }
            }
        }
        if (hasGripper){
            if (source.maxGripperDiff(dest) > 0.025){
                return false;
            }
        }

        recursiveValidPath(source, dest, depth);
        return true;
    }

    public boolean recursiveValidPath(ArmConfig prev, ArmConfig next, int depth){
        if (depth > 0){
            double x1 = prev.getBaseCenter().getX();
            double x2 = next.getBaseCenter().getX();
            double y1 = prev.getBaseCenter().getY();
            double y2 = next.getBaseCenter().getY();
            Point2D middlePoint = new Point2D.Double((x1+x2)/2, (y1+y2)/2);
            List<Double> angles = new ArrayList<Double>();
            List<Double> angles2 = new ArrayList<Double>();
            List<Double> angles3 = new ArrayList<Double>();

            if (jointCount > 0) {
                List<Double> prevAngles = prev.getJointAngles();
                List<Double> nextAngles = next.getJointAngles();
                for (int i = 0; i < jointCount; i++){
                    double middle = (prevAngles.get(i) + nextAngles.get(i))/2;
                    angles.add(middle);
                    angles2.add(angleBounded(middle+0.08));
                    angles3.add(angleBounded(middle-0.08));
                    //System.out.print(prevAngles + " - " + nextAngles + " - " + middle + " \n");
                }
            }

            List<Double> lengths = new ArrayList<Double>();

            if (hasGripper) {
                List<Double> prevLengths = prev.getGripperLengths();
                List<Double> nextLengths = next.getGripperLengths();
                for (int i = 0; i < 4; i++){
                    double middle = (prevLengths.get(i) + nextLengths.get(i))/2;
                    lengths.add(middle);
                    //System.out.print(prevAngles + " - " + nextAngles + " - " + middle + " \n");
                }
            }

            ArmConfig intermediate, intermediate2, intermediate3;
            if (hasGripper){
                intermediate = new ArmConfig(middlePoint, angles, lengths);
                intermediate2 = new ArmConfig(middlePoint, angles2, lengths);
                intermediate3 = new ArmConfig(middlePoint, angles3, lengths);
                if (!tester.validState(intermediate2) || tester.hasCollision(intermediate2, ps.getObstacles())) {
                    return false;
                }
                if (!tester.validState(intermediate3) || tester.hasCollision(intermediate3, ps.getObstacles())) {
                    return false;
                }

            } else {
                intermediate = new ArmConfig(middlePoint, angles);
                if ( jointCount > 0){
                    intermediate2 = new ArmConfig(middlePoint, angles2);
                    intermediate3 = new ArmConfig(middlePoint, angles3);
                    if (!tester.validState(intermediate2) || tester.hasCollision(intermediate2, ps.getObstacles())) {
                        return false;
                    }
                    if (!tester.validState(intermediate3) || tester.hasCollision(intermediate3, ps.getObstacles())) {
                        return false;
                    }
                }

            }
            if (!tester.validState(intermediate) || tester.hasCollision(intermediate, ps.getObstacles())) {
                return false;
            }

            recursiveValidPath(intermediate, next, depth - 1);
            recursiveValidPath(prev, intermediate, depth - 1);
        }
        return true;
    }

    public ArmConfig sample() {
        boolean valid = false;
        while (!valid){
            ArmConfig s = sampleUnchecked();
            if (tester.validState(s) && !tester.hasCollision(s, ps.getObstacles())){
                return s;
            } else {
                ArmConfig r = resampleUnchecked(s);
                if (tester.validState(r) && !tester.hasCollision(r, ps.getObstacles())) {
                    return r;
                } else {
                    valid = false;
                }

            }
        }
        //angles between -150 - 150 -> radians
        return null;
    }


    // sample joint angles and grippers [u1, u2, l1, l2]
    // joints are varaible lenght
    // redefine cost function
    // redfine step function

    private double angleBounded(double angle){
        if (angle > 0){
            return Math.min(angle, tester.MAX_JOINT_ANGLE);
        } else {
            return Math.max(angle, tester.MIN_JOINT_ANGLE);
        }
    }

    public ArmConfig sampleUnchecked(){
        double x = Math.round(0.004 + Math.random()*992.0)/1000.0;
        double y = Math.round(0.004 + Math.random()*992.0)/1000.0;
        Point2D base = new Point2D.Double(x,y);
        ArrayList<Double> empty = new ArrayList<Double>();
        /*

        empty.add(0.2);
        empty.add(0.2);
        empty.add(0.2);*/
        // sample joint angles and grippers [u1, u2, l1, l2]
        for (int i = 0; i < jointCount; i++) {
            //double newJoint = (averageAngle(initState) + 1.3*(Math.random()*2-1));
            double newJoint = rollAngle(i);
            empty.add(newJoint);
        }
        ArmConfig s;
        if (hasGripper){
            List<Double> gripperLengths = new ArrayList<Double>();
            for (int i = 0; i < 4; i++) {
                double newLength = rollLength(i);
                gripperLengths.add(newLength);
            }
            s = new ArmConfig(base, empty, gripperLengths);


        } else {
            s = new ArmConfig(base, empty);
        }
        //System.out.print(s);
        return s;
    }

    private double rollLength(int i){
        double newLength;
        if (Math.random() > 0.2) {
            newLength = tester.MIN_GRIPPER_LENGTH;

        } else {
            double roll = Math.random();
            if (roll > 0.7){
                newLength = initLengths.get(i);
            } else if (roll < 0.3) {
                newLength = goalLengths.get(i);
            } else {
                newLength = Math.round(1000.0*(tester.MIN_GRIPPER_LENGTH + (tester.MAX_GRIPPER_LENGTH-tester.MIN_GRIPPER_LENGTH)*0.95*Math.random()))/1000.0;
            }
        }
        return newLength;
    }

    private double rollAngle(int i){
        double newJoint;
        if (Math.random() > 0.35) {
            newJoint = (initAngles.get(i) + (0.35+(i*0.01))*(Math.random()*2-1));
        } else {
            double roll = Math.random();
            /*if (roll > 0.95){
                newJoint = (initAngles.get(i) + (1.0+(i*0.05))*(Math.random()*2-1));
            } else*/
            if (roll > 0.85) {
                newJoint = (goalAngles.get(i) + (0.35+(i*0.01))*(Math.random()*2-1));
            } else if (roll > 0.847) {
                newJoint = (tester.MAX_JOINT_ANGLE - 2*(Math.random()));
            } else if (roll > 0.845) {
                newJoint = (tester.MIN_JOINT_ANGLE + 2*(Math.random()));
            } else if ( roll > 0.4){

                newJoint = (tester.MAX_JOINT_ANGLE*.5*(Math.random()*2-1));
            } else{

                newJoint = (tester.MAX_JOINT_ANGLE*(Math.random()*2-1));
            }
        }

        newJoint = Math.round(1000.0*newJoint)/1000.0;
        newJoint = angleBounded(newJoint);
        return newJoint;
    }

    public ArmConfig resampleUnchecked(ArmConfig c){
        double D = 0.05;
        double samX = c.getBaseCenter().getX();
        double samY = c.getBaseCenter().getY();
        //For min, maximum of lower bound or value, opposite for max - tricky!
        double minX = Math.max(0.004, samX - D);
        double maxX = Math.min(0.996, samX+D);
        double minY = Math.max(0.004, samY-D);
        double maxY = Math.min(0.996, samY+D);

        double x = Math.round((minX + (maxX-minX)*Math.random()) * 1000.0)/1000.0;
        double y = Math.round((minY + (maxY-minY)*Math.random()) * 1000.0)/1000.0;
        Point2D base = new Point2D.Double(x,y);
        /*ArrayList<Double> empty = new ArrayList<Double>();
        empty.add(0.2);
        empty.add(0.2);
        empty.add(0.2);*/
        ArrayList<Double> empty = new ArrayList<Double>();
        for (int i = 0; i < jointCount; i++) {
            double newJoint = rollAngle(i);
            empty.add(newJoint);
        }
        ArmConfig s;
        if (hasGripper){
            List<Double> gripperLengths = new ArrayList<Double>();
            for (int i = 0; i < 4; i++) {
                double newLength = rollLength(i);
                gripperLengths.add(newLength);
            }
            s = new ArmConfig(base, empty, gripperLengths);


        } else {
            s = new ArmConfig(base, empty);
        }
        return s;
    }

    private boolean aSearch(ArmConfig start, ArmConfig goal) {
        Set<ArmConfig> explored = new HashSet<ArmConfig>();

        Comparator<ArmConfig> myComparator = new Comparator<ArmConfig>() {
            @Override
            public int compare(ArmConfig o1, ArmConfig o2) {
                    return Double.compare(distFromSource.get(o1), distFromSource.get(o2));
            }
        };
        PriorityQueue<ArmConfig> pq = new PriorityQueue<ArmConfig>(myComparator);

        //System.out.print("Sampling and Searching\n");
        distFromSource = new HashMap<ArmConfig, Double>();
        prev =  new HashMap<ArmConfig, ArmConfig>();
        //HashSet<Edge> s = G.getEdges(root);
        G.getEdges(start);
        distFromSource.put(start, 0.0);
        pq.add(start);
        pathFlag = false;
        do {
            //For current node
            ArmConfig current = pq.poll();
            //System.out.print("Size is " + pq.size() + "\n");
            explored.add(current);
            double cDist = distFromSource.get(current);
            if (cDist > maxDistFound){
                maxDistFound = cDist;
            }
            if (current.equals(goal)) {
                //System.out.print("Path found!\n");
                pathFlag = true;
                prev.put(goal, prev.get(current));
                distFromSource.put(goal, cDist);
                List<ArmConfig> path = getPath(goal);
                List<ArmConfig> badConfigs = (validateSteps(path));
                if (!(badConfigs == null)){
                    for (ArmConfig b : badConfigs) {
                        G.removeEdge(prev.get(b), b);
                        distFromSource.remove(b);
                        prev.remove(b);
                    }

                    prev.remove(goal);
                    distFromSource.remove(goal);
                    //System.out.print("Graph size: " + G.size() + " Known edges size: " + distFromSource.size() + "\n");
                    continue;
                }
                //printPath(path);
                solutionFound = true;
                return true;
            }


            Set<ArmConfig> edges = G.getEdges(current);
            if (edges != null){


                //Set<Edge> edges = G.getEdges(current);
                for (ArmConfig n : edges) {
                    //
                    //System.out.print(current.getBaseCenter().distance(n.getBaseCenter()) + "-"+ .32*distAngles(current, n) + "-"+ .05*n.getBaseCenter().distance(initState.getBaseCenter()) + "-"+ 0.1/(2*n.getBaseCenter().distance(goal.getBaseCenter()))+ "\n") ;
                    double cost = cDist + current.getBaseCenter().distance(n.getBaseCenter()) + .65*distAngles(current, n) + .02*n.getBaseCenter().distance(initState.getBaseCenter()) - 0.03/(2*n.getBaseCenter().distance(goal.getBaseCenter()));
                    if (distFromSource.getOrDefault(n, Double.MAX_VALUE) > (cost)) {
                        pq.remove(n);
                    }
                    if (!explored.contains(n) && !pq.contains(n)) {
                        prev.put(n, current);
                        distFromSource.put(n, cost);
                        pq.add(n);
                    }
                }
            }
        } while (pq.size() != 0);

        return false;
        //Expand n with lowest cost
    }

    private double distAngles(ArmConfig a, ArmConfig b){
        List<Double> aList = a.getJointAngles();
        List<Double> bList = b.getJointAngles();
        double biggest = 0;
        for (int i = 0; i < jointCount; i++){
            double dif = Math.abs(aList.get(i) - bList.get(i));
            if (dif > biggest){
                biggest = dif;
            }
        }
        //System.out.print(a + " - " + b + " - " + biggest + "\n");
        return biggest;
    }

    private double averageAngle(ArmConfig config){
        List<Double> angles = config.getJointAngles();
        double sum = 0;
        for (double a : angles){
            sum += a;
        }
        sum = sum / jointCount;
        return sum;
    }

    private List<ArmConfig> validateSteps(List<ArmConfig> path){
        ArmConfig previous = null;
        boolean badFound = false;
        //Clear and/or create output file
        //clearFile();
        List<ArmConfig> steps = new ArrayList<ArmConfig>();
        List<ArmConfig> badSteps = new ArrayList<ArmConfig>();
        int dodgy = 0;
        for (ArmConfig current : path) {

            if (current.equals(initState)){
                previous = initState;
                continue;
            }
            while (!previous.equals(current)){
                //ArmConfig old = new ArmConfig(previous);
                ArmConfig old = previous;
                previous = nextStep(previous, current);

                if (!tester.isValidStep(old, previous) || tester.hasCollision(previous, ps.getObstacles())){
                    previous = altStep(old, current);
                    if (!tester.isValidStep(old, previous) || tester.hasCollision(previous, ps.getObstacles())) {
                        badFound = true;
                        badSteps.add(current);
                        dodgy += 1;
                        break;
                        //currentBad = true;
                    }
                }
                if (!badFound){
                    steps.add(previous);
                }
            }
        }
        /*if (dodgy >0){
            System.out.print("Dodgy steps: " + dodgy + ". Finding new path.\n");
        }*/
        if (badFound){
            return badSteps;
        }
        appendSteps(steps);
        return null;

    }


    private ArmConfig nextStep(ArmConfig from, ArmConfig toward){
        Point2D bFrom = from.getBaseCenter();
        Point2D bToward = toward.getBaseCenter();
        double xToward = bToward.getX();
        double yToward = bToward.getY();
        double xFrom = bFrom.getX();
        double yFrom = bFrom.getY();
        double delX = xToward-xFrom;
        double delY = yToward-yFrom;
        double x = xFrom;
        double y = yFrom;
        boolean stepDone = true;
        if (delX > 0) {
            x = xFrom + STEP_SIZE;
        } else if (delX < 0) {
            x = xFrom - STEP_SIZE;
        } else {
            stepDone = false;
        }

        if (!stepDone){
            if (delY > 0) {
                y = yFrom + STEP_SIZE;
            } else if (delY < 0) {
                y = yFrom - STEP_SIZE;
            }
        }
        x = Math.round(x*1000.0)/1000.0;
        y = Math.round(y*1000.0)/1000.0;
        Point2D base = new Point2D.Double(x,y);

        //Angles
        List<Double> nextAngles = stepAngles(from, toward);

        //Gripper lengths
        List<Double> nextLengths = stepLengths(from, toward);
        ArmConfig next;
        if (hasGripper){
            next = new ArmConfig(base, nextAngles, nextLengths);
        } else {
            next = new ArmConfig(base, nextAngles);
        }
        return next;

    }

    private ArmConfig altStep(ArmConfig from, ArmConfig toward){
        Point2D bFrom = from.getBaseCenter();
        Point2D bToward = toward.getBaseCenter();
        double xToward = bToward.getX();
        double yToward = bToward.getY();
        double xFrom = bFrom.getX();
        double yFrom = bFrom.getY();
        double delX = xToward-xFrom;
        double delY = yToward-yFrom;
        double x = xFrom;
        double y = yFrom;
        boolean stepDone = true;
        if (delY > 0) {
            y = yFrom + STEP_SIZE;
        } else if (delY < 0) {
            y = yFrom - STEP_SIZE;
        } else {
            stepDone = false;
        }

        if (!stepDone){
            if (delX > 0) {
                x = xFrom + STEP_SIZE;
            } else if (delX < 0) {
                x = xFrom - STEP_SIZE;
            }
        }
        x = Math.round(x*1000.0)/1000.0;
        y = Math.round(y*1000.0)/1000.0;
        Point2D base = new Point2D.Double(x,y);

        //Angles
        List<Double> nextAngles = stepAngles(from, toward);

        //Gripper lengths
        List<Double> nextLengths = stepLengths(from, toward);
        ArmConfig next;
        if (hasGripper){
            next = new ArmConfig(base, nextAngles, nextLengths);
        } else {
            next = new ArmConfig(base, nextAngles);
        }
        return next;
    }

    private List<Double> stepAngles(ArmConfig from, ArmConfig toward){
        List<Double> fromAngles = from.getJointAngles();
        List<Double> toAngles = toward.getJointAngles();
        List<Double> nextAngles = new ArrayList<Double>();
        for (int i = 0; i < jointCount; i++){
            double aFrom = fromAngles.get(i);
            double aTo = toAngles.get(i);
            double del = aTo - aFrom;
            double step;
            if (del > 0) {
                step = aFrom + ANGLE_STEP;
            } else if (del < 0) {
                step = aFrom - ANGLE_STEP;
            } else {
                step = aFrom;
            }
            step = Math.round(step*1000.0)/1000.0;
            nextAngles.add(step);
        }
        return nextAngles;
    }

    private List<Double> stepLengths(ArmConfig from, ArmConfig toward){
        List<Double> nextLengths = new ArrayList<Double>();
        if (!hasGripper){
            return nextLengths;
        }
        List<Double> fromLengths = from.getGripperLengths();
        List<Double> toLengths = toward.getGripperLengths();

        for (int i = 0; i < 4; i++){
            double aFrom = fromLengths.get(i);
            double aTo = toLengths.get(i);
            double del = aTo - aFrom;
            double step;
            if (del > 0) {
                step = aFrom + tester.MAX_GRIPPER_STEP;
            } else if (del < 0) {
                step = aFrom - tester.MAX_GRIPPER_STEP;
            } else {
                step = aFrom;
            }
            step = Math.round(step*1000.0)/1000.0;
            nextLengths.add(step);
        }
        return nextLengths;
    }

    private List<ArmConfig> getPath(ArmConfig goal){
        List<ArmConfig> path = new ArrayList<ArmConfig>();
        ArmConfig v = goal;
        while (v != null) {
            path.add(v);
            v = prev.get(v);
        }
        int n = path.size() - 1;
        Collections.reverse(path);
        return path;
    }

    private void printPath(List<ArmConfig> path) {
        int n = path.size() - 1;
        Collections.reverse(path);
        StringBuilder sb = new StringBuilder("");
        sb.append(n);
        sb.append("\n");
        sb.append(initState);
        sb.append("\n");
        for (ArmConfig x : path) {
            if (x.equals(initState)){
                continue;
            }
            sb.append(x.toString() + "\n");
        }
        //sb.append(" " + l.getDist());
        //sb.append("\n");
        System.out.print(sb.toString());
    }


    private void appendSteps(List<ArmConfig> steps ) {
        try {
            clearFile();
            File file = new File(outFile);

            FileWriter fw = new FileWriter(file.getAbsoluteFile(), true);
            BufferedWriter bw = new BufferedWriter(fw);
            System.out.print("Number of steps:" + steps.size() +"\n");
            bw.write(steps.size() + "\n");
            bw.write(initState.toString() + "\n");
            for (ArmConfig x : steps) {
                bw.write(x.toString() + "\n");
            }

            bw.close();

        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    public static void main(String[] args) {
        if (args.length != 2) {
            System.out.print("Invalid arguments\n");
            System.exit(1);
        }
        Solver solver = new Solver(args);

    }

    private void clearFile() {
        try {
            File file = new File(outFile);
            PrintWriter writer = new PrintWriter(file);
            writer.print("");
            writer.close();

        } catch (IOException e) {
            e.printStackTrace();
        }
    }
}
