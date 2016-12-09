package solver;

import problem.ArmConfig;
import tester.Tester;

import java.util.ArrayList;
import java.util.List;

/**
 * Created by Stuey on 21/09/2016.
 */
public class UnitTester {
    public static final double DEFAULT_MAX_ERROR = 1e-5;
    public static final double STEP_SIZE = 0.001;

    public Tester tester = new Tester(DEFAULT_MAX_ERROR);

    public UnitTester(){
            /*System.out.print("Hello\n");
            ArmConfig a = new ArmConfig("0.358 0.38 0.453 0.918 0.477 0.529 0.031 0.036 0.043 0.033");
            ArmConfig b = new ArmConfig("0.359 0.38 0.454 0.918 0.478 0.53 0.031 0.036 0.043 0.032");
            System.out.print(tester.isValidStep(a,b));*/
        TimeTest(5);
    }

    public void TimeTest(int testNum){
        String[] args = {"9_obst.txt" ,"9_out"};
        long t1 = 0;
        List<Long> l1 = new ArrayList<Long>();
        for (int i = 0; i < testNum; i++){
            long startTime = System.currentTimeMillis();
            Solver s = new Solver(args);
            long endTime = System.currentTimeMillis();
            l1.add(endTime - startTime);
            t1 += (endTime - startTime);
            System.gc();
        }

        String[] args2 = {"4_joints.txt" ,"solver_out"};
        long t2 = 0;
        List<Long> l2 = new ArrayList<Long>();
        for (int i = 0; i < testNum; i++){
            long startTime = System.currentTimeMillis();
            Solver s = new Solver(args2);
            long endTime = System.currentTimeMillis();
            l2.add(endTime - startTime);
            t2 += (endTime - startTime);
            System.gc();
        }
        String[] args3 = {"gripper_4_joints.txt" ,"grip_out"};
        long t3 = 0;
        List<Long> l3 = new ArrayList<Long>();
        for (int i = 0; i < testNum; i++){
            long startTime = System.currentTimeMillis();
            Solver s = new Solver(args3);
            long endTime = System.currentTimeMillis();
            l3.add(endTime - startTime);
            t3 += (endTime - startTime);
            System.gc();
        }

        String[] args4 = {"joint_3-obs_15.txt" ,"j3o15.out"};
        long t4 = 0;
        List<Long> l4 = new ArrayList<Long>();
        for (int i = 0; i < 1; i++){
            long startTime = System.currentTimeMillis();
            Solver s = new Solver(args4);
            long endTime = System.currentTimeMillis();
            l4.add(endTime - startTime);
            t4 += (endTime - startTime);
            System.gc();
        }
        String[] args5 = {"joint_6-obs_3.txt" ,"j6o3.out"};
        long t5 = 0;
        long startTime = System.currentTimeMillis();
        Solver s = new Solver(args5);
        long endTime = System.currentTimeMillis();
        t5 += (endTime - startTime);
        System.gc();

        System.out.print("Test 1: " + args[0] + "," + args[1] + " | time:" + (t1) + " ms\n ");
        System.out.print("Test 2: " + args2[0] + "," + args2[1] + " | time:" + (t2) + " ms\n ");
        System.out.print("Test 3: " + args3[0] + "," + args3[1] + " | time:" + (t3) + " ms\n ");
        System.out.print("Test 4: " + args4[0] + "," + args4[1] + " | time:" + (t4) + " ms\n ");
        System.out.print("Test 5: " + args5[0] + "," + args5[1] + " | time:" + (t5) + " ms\n ");

        System.out.print("Test 1: " + args[0] + "," + args[1] + " | times:" + (l1) + " ms\n ");
        System.out.print("Test 2: " + args2[0] + "," + args2[1] +" | times:" + (l2) + " ms\n ");
        System.out.print("Test 3: " + args3[0] + "," + args3[1] + " | times:" + (l3) + " ms\n ");
        System.out.print("Test 4: " + args4[0] + "," + args4[1] + " | time:" + (l4) + " ms\n ");


    }

    public static void main(String[] args) {
        UnitTester ut = new UnitTester();


    }
}
