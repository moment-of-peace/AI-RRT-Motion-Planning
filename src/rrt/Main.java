package rrt;
/**
 * motion planning using bidirectional rapidly exploring random tree
 */

import java.awt.geom.Point2D;
import java.awt.geom.Rectangle2D;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.HashSet;
import java.util.List;
import java.util.Random;

public class Main {
    /** The maximum distance any ASV can travel between two states */
    private static final double MAX_STEP = 0.001;
    /** The maximum allowable boom length */
    private static final double MAX_BOOM_LENGTH = 0.05;
    /** The default value for maximum error */
    //private static final double DEFAULT_MAX_ERROR = (1e-5);
    
    private static final double PI = Math.PI;
        
    private static int clockwise;
    private static int total = 0;
    private static int recurrent = 0;
    //private static int recurrentflag = 6;
    public static void main(String[] args) throws IOException {
        // load problem from a file
        String srcFile = args[0];
        String outputName = args[1];
        Test tester = new Test(srcFile);
        int obsNum = tester.ps.obstacles.size();
        if (obsNum > 2) {
            for (Obstacle o: tester.ps.obstacles) {
                o.rect = Test.grow(o.rect, 2e-5);
            }
        }
        
        int asvCount = tester.ps.getASVCount();
        int dimensions = asvCount + 1; // dimension degree of c space
        
        // HashSets used to store found configurations in cspace from init and goal sides
        HashSet<Config> fromInit = new HashSet<Config>();
        HashSet<Config> fromGoal = new HashSet<Config>();
        
        // get initial and goal coordinates in c space
        Config initConfig = asvConfigToCfg(tester.ps.getInitialState(),tester);
        Config goalConfig = asvConfigToCfg(tester.ps.getGoalState(),tester);
        for (double d: initConfig.coords) {
            System.out.println(d);
        }
        System.out.println("");
        for (double d: goalConfig.coords) {
            System.out.println(d);
        }
        System.out.println("");
        // clockwise or anti-clockwise
        if (initConfig.coords.length > 3 && initConfig.coords[3] < 0) {
            clockwise = -1;
        } else {
            clockwise = 1;
        }
        
        // add initial and goal into hashsets
        fromInit.add(initConfig);
        fromGoal.add(goalConfig);
        
        double[] angleRange = getAngleRange(initConfig, goalConfig);
        
        // extend tree from both initial and goal point
        Config initNext = initConfig;
        Config goalNext = goalConfig;
        
        Config sample, nearest1, nearest2;
        int turn = 0;

        FileWriter fw1 = new FileWriter("sample1.txt");
        FileWriter fw2 = new FileWriter("sample2.txt");
        FileWriter fw0 = new FileWriter("sample0.txt");
        fw0.write(998 + " " + 10 + "\n");
        //fw1.write("1000 15\n");
        //fw2.write("1000 15\n");
        //FileWriter fwr = new FileWriter("sample-raw.txt");
        
        while (!initNext.equals(goalNext)) {
            total++;
            if (obsNum != 2 || turn < 3) {
                sample = getRandomPoint(dimensions, angleRange, tester);
                ASVConfig asv = cfgToASVConfig(sample);
                while(!cSpaceCollisionCheck1(asv, tester)) {
                    sample = getRandomPoint(dimensions, angleRange, tester);
                    asv = cfgToASVConfig(sample);
                }//System.out.println("got random");
            } else if (turn < 6){
                sample = getRandomPoint2(dimensions, angleRange, tester);
                ASVConfig asv = cfgToASVConfig(sample);
                while(!cSpaceCollisionCheck1(asv, tester)) {
                    sample = getRandomPoint2(dimensions, angleRange, tester);
                    asv = cfgToASVConfig(sample);
                }//System.out.println("got random");
            } else {
                sample = getRandomPoint3(dimensions, angleRange, tester);
                ASVConfig asv = cfgToASVConfig(sample);
                while(!cSpaceCollisionCheck1(asv, tester)) {
                    sample = getRandomPoint3(dimensions, angleRange, tester);
                    asv = cfgToASVConfig(sample);
                }
            }
            
            if (total < 1000) {
                printPosition(cfgToASVConfig(sample), fw0);
                //fw0.flush();
                /*for (double d: sample.coords) {
                    fwr.write(d + " ");
                }
                fwr.write("\n");*/
            }
            // find nearest configurations from both sides
            if (turn%3 == 1) {
                nearest1 = findNearest(fromInit, sample, tester);
                initNext = findNext2(sample, nearest1, tester, fromInit,1,null);
                nearest2 = findNearest(fromGoal, initNext, tester);//System.out.println("found nearest");
                goalNext = findNext2(initNext, nearest2, tester, fromGoal,2,null);//System.out.println("found next");
                fromInit.add(initNext);
                fromGoal.add(goalNext);
            } else if (turn%3 == 2) {
                nearest2 = findNearest(fromGoal, sample, tester);//System.out.println("found nearest");
                goalNext = findNext2(sample, nearest2, tester, fromGoal,2,null);//System.out.println("found next");
                nearest1 = findNearest(fromInit, goalNext, tester);
                initNext = findNext2(goalNext, nearest1, tester, fromInit,1,null);
                fromInit.add(initNext);
                fromGoal.add(goalNext);
            } else {
                //start = System.currentTimeMillis();
                nearest1 = findNearest(fromInit, sample, tester);
                nearest2 = findNearest(fromGoal, sample, tester);//System.out.println("found nearest");
                //time1 = time1 + System.currentTimeMillis() - start;
                // get the next configurations for both sides
                //start = System.currentTimeMillis();
                initNext = findNext2(sample, nearest1, tester, fromInit,1,null);
                goalNext = findNext2(sample, nearest2, tester, fromGoal,2,null);//System.out.println("found next");
                fromInit.add(initNext);
                fromGoal.add(goalNext);
                
            }
            
            /*double[] position = initNext.coords;
            double[] position2 = sample.coords;
            if (position[0] > 0.334 && position[0] < 0.665 && position[1] > 0.485 && position[1] < 0.515) {
                System.out.println("sample: " + position2[0] + " " + position2[1]);
                System.out.println("extend: " + position[0] + " " + position[1]);
            }*/
            
            //time2 = time2 + System.currentTimeMillis() - start;
            //if (initNext != nearest1 && goalNext != nearest2) total++;
            if (total%500 == 0) {
                turn++;
                if (turn == 8) turn = 0;
                System.out.println("samples: " + total);
            }
            //if (total%10 == 0) System.out.println("samples: " + total);
            
            if (total == 1000) {
                
                fw1.write(fromInit.size()-1+" " + 10 + "\n");
                fw2.write(fromGoal.size()-1+" " + 10 + "\n");
                for (Config c: fromInit) {
                    printPosition(cfgToASVConfig(c), fw1);
                }
                for (Config c: fromGoal) {
                    printPosition(cfgToASVConfig(c), fw2);
                }
                fw1.close();
                fw2.close();
                fw0.close();
            }
        }
        fw0.close();
        fw1.close();
        fw2.close();
        System.out.println("finished, total samples: " + total);
        //record the whole path between initial and goal
        FileWriter fw = new FileWriter(outputName);
        ArrayList<ASVConfig> solution = new ArrayList<ASVConfig>();
        solution.add(tester.ps.getInitialState());
        solution.addAll(getSol1(initNext,tester));
        solution.addAll(getSol2(goalNext,tester));
        solution.add(tester.ps.getGoalState());
        
        // compute the cost
        tester.ps.setPath(solution);
        fw.write(solution.size()-1+" "+tester.ps.solutionCost+"\n");
        // write path to the output file
        for (ASVConfig asv:solution){
            printPosition(asv,fw);
        }
        fw.close();
        System.out.println("output file generated\n");
        
    }
    
    private static ArrayList<ASVConfig> getSol1(Config initNext, Test tester) 
            throws IOException {
        // TODO Auto-generated method stub
        ArrayList<Config> solution = new ArrayList<Config>();
        ArrayList<ASVConfig> sol1 = new ArrayList<ASVConfig>();
        solution.add(initNext);
        Config pred = initNext.predecessor;
        
        while (pred != null) {
            solution.add(pred);
            pred = pred.predecessor;
        }
        for (int i = solution.size()-1; i > 0; i--) {
            Config start = solution.get(i);
            Config result = solution.get(i-1);
            /*double step = maxDistance(start, result);
            while(step > MAX_STEP) {
                while(step > MAX_STEP){
                    result = cutDist(step, start, result);
                    step = maxDistance(start, result);
                }
                sol1.add(cfgToASVConfig(result));
                start=result;
                result=solution.get(i-1);
                step = maxDistance(start, result);
            }*/
            //sol1.add(cfgToASVConfig(start));
            //findNext2(result, start, tester, null, 1, sol1);
            sol1.add(cfgToASVConfig(result));
        }
        return sol1;
     }

    private static ArrayList<ASVConfig> getSol2(Config goalNext, Test tester) 
            throws IOException {
        // TODO Auto-generated method stub
        ArrayList<Config> solution = new ArrayList<Config>();
        ArrayList<ASVConfig> sol2 = new ArrayList<ASVConfig>();
        solution.add(goalNext);
        Config pred = goalNext.predecessor;
        while (pred != null) {
            solution.add(pred);
            pred = pred.predecessor;
        }
        Config config;
        for (int i = solution.size()-1; i > 0; i--) {
            config = solution.get(i);
            Config start = config;
            Config result = solution.get(i-1);
            /*double step = maxDistance(start, result);
            while(step > MAX_STEP) {
                while(step > MAX_STEP){
                    result = cutDist(step, start, result);
                    step = maxDistance(start, result);
                }
                sol2.add(cfgToASVConfig(result));
                start=result;
                result=solution.get(i-1);
                step = maxDistance(start, result);
            }*/
            //sol2.add(cfgToASVConfig(start));
            //findNext2(result, start, tester, null, 1, sol2);
            sol2.add(cfgToASVConfig(result));
        }
        ArrayList<ASVConfig> reverse = new ArrayList<ASVConfig>();
        int len = sol2.size();
        for (int i = 0; i < len; i++) {
            reverse.add(sol2.get(len-1-i));
        }
        return reverse;
    }
    
    private static void printPosition(ASVConfig config, FileWriter fw) throws IOException{
        List<Point2D> s2  = config.getASVPositions();
        String sep = "";
        for (Point2D p:s2){
            fw.write(sep+p.getX()+" "+p.getY());
            sep=" ";
        }
        fw.write("\n");
    }
    /**
     * 
     * @param initConfig
     * @param goalConfig
     * @return
     */
    private static double[] getAngleRange(Config initConfig, Config goalConfig) {
        double[] initCoords = initConfig.coords;
        double[] goalCoords = goalConfig.coords;
        double[] angleRange = new double[initCoords.length-2];
        angleRange[0] = 2*PI;
        
        for (int i = 1; i < angleRange.length; i++) {
            angleRange[i] = (Math.abs(initCoords[i+2]) < Math.abs(goalCoords[i+2])? 
                    initCoords[i+2]:goalCoords[i+2]);
        }
        return angleRange;
    }

    /**
     * convert a state from workspace to c space
     */
    private static Config asvConfigToCfg(ASVConfig initialState, Test tester) {
        // TODO Auto-generated method stub
        List<Point2D> positions = initialState.asvPositions;
        //length
        double [] pts = new double [initialState.getASVCount()+1];
        Point2D p0= positions.get(0);
        pts[0]=p0.getX();
        pts[1]=p0.getY();
        double prevAngle=0;
        for (int i=1;i<positions.size();i++){
            Point2D p1 = positions.get(i);
            double currentAngle = Math.atan2(p1.getY() - p0.getY(),
                    p1.getX() - p0.getX());
            
            pts[i+1]=tester.normaliseAngle(PI+prevAngle-currentAngle);
            
            prevAngle=currentAngle;
            p0=p1;
        }
        Config cfg = new Config(pts);
        return cfg;
    }

    /**
     * Get random C-state with 2 start coords and n-1 angle;
     * @param dimensions = point.number+2
     * @param angleRange 
     * @return array of C-state
     */
    private static Config getRandomPoint(int dimensions, double[] angleRange, Test tester) {
        boolean flag = true;
        int times = 0;
        
        while (true) {
            double[] position = new double[(dimensions-1)*2];
            double pre = 0;
            flag = true;
            times++;
            
            // start position
            getStartPosition(position, tester);
            int i;
            for(i = 1; i < dimensions-1; i++) {
                pre = getNextPoint(pre, angleRange[i-1], position, i, tester);
                if (pre == 10) {
                    flag = false;
                    break;
                }
            }
            if (flag == true) {
                return asvConfigToCfg(new ASVConfig(position), tester);
            }
            if (times%50 == 0) {
                System.out.println("rand: " + times + " " + i + " "+ position[0] + " " + position[1]);
            }
        }
    }
    private static Config getRandomPoint2(int dimensions, double[] angleRange, Test tester) {
        boolean flag = true;
        int times = 0;
        
        double[] range = angleRange.clone();
        while (true) {
            double[] position = new double[(dimensions-1)*2];
            double pre = 0;
            flag = true;
            times++;
            
            getSinCor(position,tester);
            int i;
            for(i = 1; i < dimensions-1; i++) {
                pre = getNextPoint(pre, range[i-1], position, i, tester);
                if (pre == 10) {
                    flag = false;
                    break;
                }
            }
            if (flag == true) {
                return asvConfigToCfg(new ASVConfig(position), tester);
            }
            if (times%50 == 0) {
                System.out.println("rand: " + times + " " + i + " "+ position[0] + " " + position[1]);
            }
        }
    }
    private static Config getRandomPoint3(int dimensions, double[] angleRange, Test tester) {
        boolean flag = true;
        int times = 0;
        
        double[] range = angleRange.clone();
        for (int j = 1; j < range.length; j++) {
            range[j] = -angleRange[range.length-j];
        }
        while (true) {
            double[] position = new double[(dimensions-1)*2];
            double pre = 0;
            flag = true;
            times++;
            
            getSinCor(position,tester);
            int i;
            for(i = 1; i < dimensions-1; i++) {
                pre = getNextPoint(pre, range[i-1], position, i, tester, -clockwise);
                if (pre == 10) {
                    flag = false;
                    //System.out.println("fail: " + position[0] + " " + position[1]);
                    break;
                }
            }
            if (flag == true) {
                reversePosition(position);
                return asvConfigToCfg(new ASVConfig(position), tester);
            }
            if (times%50 == 0) {
                System.out.println("rand: " + times + " " + i + " "+ position[0] + " " + position[1]);
            }
        }
    }
    
    private static void getStartPosition(double[] position, Test tester) {
        Random randP = new Random();
        Point2D p = new Point2D.Double(randP.nextDouble(), randP.nextDouble());
        List<Obstacle> obs = tester.ps.obstacles;
        while (containPoint(obs, p)) {
            p = new Point2D.Double(randP.nextDouble(), randP.nextDouble());
        }
        position[0] = p.getX();
        position[1] = p.getY();
    }

    private static boolean containPoint(List<Obstacle> obs, Point2D p) {
        for (Obstacle o: obs) {
            if (o.rect.contains(p)) {
                return true;
            }
        }
        return false;
    }

    /**
     * generate next random asv position
     * @param d
     * @param position
     * @param i
     * @return
     */
    private static double getNextPoint(double pre, double limit, double[] position, int i, Test tester) {
        return getNextPoint(pre, limit, position, i, tester, clockwise);
    }
    
    private static double getNextPoint(double pre, double limit, double[] position, int i, Test tester, int clock) {
        Random rand = new Random();
        int times = 0;
        i = i-1;
        
        if (i == 0) {
            while (true && times < 5000) {
                times++;
                double angle = rand.nextDouble() * 2 * PI;
                position[2*i+2] = position[2*i] + MAX_BOOM_LENGTH * Math.cos(angle);
                position[2*i+3] = position[2*i+1] + MAX_BOOM_LENGTH * Math.sin(angle);
                ASVConfig asv = new ASVConfig(cutArray(position, 2*i+3));
                if (cSpaceCollisionCheck2(asv, tester)) {
                    return angle;
                }
            }
        } else {
            while (true && times < 5000) {
                times++;
                limit = limit*clock;//*0.75;
                double angle = (rand.nextDouble()*(PI-limit)+limit)*clock;
                angle = tester.normaliseAngle(PI + pre - angle);
                position[2*i+2] = position[2*i] + MAX_BOOM_LENGTH * Math.cos(angle);
                position[2*i+3] = position[2*i+1] + MAX_BOOM_LENGTH * Math.sin(angle);
                ASVConfig asv = new ASVConfig(cutArray(position, 2*i+3));
                if (cSpaceCollisionCheck2(asv, tester)) {
                    return angle;
                }
            }
        }
        return 10;
    }

    /**
     * cut an array
     */
    private static double[] cutArray(double[] array, int i) {
        double[] result = new double[i+1];
        for (int j = 0; j < i+1; j++) {
            result[j] = array[j];
        }
        return result;
    }

    /**
     * return array of coords in work space where i%2=>y,i+1%2=>x
     * @param pts array of C-state
     * @return array of coords in work space
     */
    private static ASVConfig cfgToASVConfig(Config cfg) {
        return new ASVConfig(cfgToArray(cfg));
    }
    
    private static double[] cfgToArray(Config cfg) {
        double[] pts = cfg.coords;
        
        double [] cfgArray= new double[2*(pts.length-1)];
        double currentX=pts[0];
        double currentY=pts[1];
        double prevAngle=0;
        cfgArray[0]=pts[0];
        cfgArray[1]=pts[1];
        int j=1;
        
        for (int i=2; i<pts.length;i++){
            //transfer to angle fit coords, need test
            double theta=PI+prevAngle-pts[i];
            currentX = currentX+MAX_BOOM_LENGTH*Math.cos(theta);
            currentY = currentY+MAX_BOOM_LENGTH*Math.sin(theta);
            cfgArray[2*j]=currentX;
            cfgArray[2*j+1]=currentY;
            j++;
            //update current theta
            prevAngle=theta;
        }
        return cfgArray;
    }
    /*
    private static void cSpaceCollisionCheck(ASVConfig cfg, Test test, int[] sampleResult){
        boolean flag = true;
        if(!test.hasEnoughArea(cfg)){
            //need other test
            sampleResult[1]++;
            flag = false;
        } 
        if (!test.isConvex(cfg)) {
            sampleResult[2]++;
            flag = false;
        } 
        if (!test.fitsBounds(cfg)) {
            sampleResult[3]++;
            flag = false;
        } 
        if (!test.hasCollision(cfg, test.ps.obstacles)) {
            sampleResult[4]++;
            flag = false;
        }
        if (flag) {
            sampleResult[0]++;
        }
    }
    */
    private static boolean cSpaceCollisionCheck(ASVConfig cfg, Test test) {
        if(test.hasEnoughArea(cfg) && test.isConvex(cfg) && test.fitsBounds(cfg) 
                && !test.hasCollision(cfg, test.ps.obstacles)) {
            return true;
        } else {
            return false;
        }
}
    private static boolean cSpaceCollisionCheck1(ASVConfig cfg, Test test) {
        if(test.hasEnoughArea(cfg) && test.isConvex(cfg)) {
            return true;
        } else {
            return false;
        }
    }
    private static boolean cSpaceCollisionCheck2(ASVConfig cfg, Test test) {
        if(test.fitsBounds(cfg) && !test.hasCollision(cfg, test.ps.obstacles)) {
            return true;
        } else {
            return false;
        }
    }
    
    /**
     * retrieve a configuration which is nearest to the sampled configuration
     * @param allConfig: all found configuration
     * @param target: the sampled configuration
     * @return
     */
    private static Config findNearest(HashSet<Config> allConfig, Config sample, Test tester) {
        Config result = null;
        double dist = Double.NEGATIVE_INFINITY;
        double newDist;
        
        for (Config c: allConfig) {
            newDist = cSpaceDist2(c.coords, sample.coords, tester);
            //newDist = cSpaceDist2(c, sample, tester);
            if (newDist > dist) {
                dist = newDist;
                result = c;
            }
        }
        return result;
    }

    /**
     * compute the distance between two arrays
     * @require the size of two input arrays should be the same
     */
    private static double cSpaceDist(double[] array1, double[] array2, Test tester) {
        double dist = 0;
        double diff = 0;
        int n = 1 + array1.length/2;
        for (int i = 0; i < 2; i++) {
            dist += (array1[i] - array2[i]) * (array1[i] - array2[i]);
        }
        return diff - dist;
    }
    private static double cSpaceDist2(double[] array1, double[] array2, Test tester) {
        double dist = 0;
        for (int i = 0; i < 2; i++) {
            dist += (array1[i] - array2[i]) * (array1[i] - array2[i]);
            if (total%2 == 1) {
                double angleDiff = Math.abs(tester.normaliseAngle(array1[2] - array2[2]));
                if (angleDiff > 1) {
                    dist *= 10000;
                }
            }
        }
        return - dist;
    }
    
    /**
     * find the next vertex to extend the tree towards to the sample
     * @param sample: the sampled configuration
     * @param near: nearest configuration to the sample
     * @return: expanded configuration towards the sample from nearest
     */
    private static Config findNext(Config sample, Config near, Test tester, HashSet<Config> cfgSet) {
        Config start = near;
        Config end = sample;
        Config previous = near;
        Config next;
        ASVConfig asv;
        int num = 1;
         
        // extend towards the sample as far as possible
        while (true) {
            double step = maxDistance(start, end);
            while (step > MAX_STEP) {
                // scale down, if the next configuration exceeds the step limitation
                end = cutDist(step, start, end);
                step = maxDistance(start, end);
            }
            // if the next configuration touches collision space, break loop
            asv = cfgToASVConfig(end);
            if (!cSpaceCollisionCheck(asv, tester)) {
                double[] coords = end.coords.clone();
                for (int j = 3; j < coords.length; j++) {
                    coords[j] *= clockwise;
                    if (coords[j] + PI/180 < PI) {
                        coords[j] += PI/180;
                    }
                    coords[j] *= clockwise;
                }
                end = new Config(coords);
                asv = cfgToASVConfig(end);
                if (!cSpaceCollisionCheck(asv, tester)) {
                    for (int j = 3; j < coords.length; j++) {
                        coords[j] *= clockwise;
                        if (coords[j] - PI/90 > 0) {
                            coords[j] -= PI/90;
                        }
                        coords[j] *= clockwise;
                    }
                    end = new Config(coords);
                    asv = cfgToASVConfig(end);
                    if (cSpaceCollisionCheck(asv, tester)) {
                        step = maxDistance(start, end);
                        while (step > MAX_STEP) {
                            end = cutDist(step, start, end);
                            step = maxDistance(start, end);
                        }
                        if (cSpaceCollisionCheck(cfgToASVConfig(end), tester)) {
                            end.predecessor = previous;
                            
                            cfgSet.add(end);
                            if (validDistance(end, sample)) {
                                next = new Config(sample.coords, previous);
                                cfgSet.add(next);
                                return next;
                            }
                            return end;
                        }
                    }
                } else {
                    step = maxDistance(start, end);
                    while (step > MAX_STEP) {
                        end = cutDist(step, start, end);
                        step = maxDistance(start, end);
                    }
                    if (cSpaceCollisionCheck(cfgToASVConfig(end), tester)) {
                        end.predecessor = previous;
                        
                        cfgSet.add(end);
                        if (validDistance(end, sample)) {
                            next = new Config(sample.coords, previous);
                            cfgSet.add(next);
                            return next;
                        }
                        return end;
                    }
                }
                return start; // if start is near
            } else if (validDistance(end, sample)) {
                next = new Config(sample.coords, previous);
                cfgSet.add(next);
                return next;
            } else {
                num++;
                if (num%100 == 0) {
                    next = new Config(end.coords, previous);
                    cfgSet.add(next);
                    previous = next;
                }
                start = end;
                end = sample;
            }
        }
    }
    // version 2 of findNext
    private static Config findNext2(Config end, Config start, Test tester, HashSet<Config> cfgSet, int d, ArrayList<ASVConfig> asvs) throws IOException {
        /*recurrent++;
        if (recurrent > 5) {
            recurrent = 0;
            return start;
        }*/
        //System.out.println("findnext s");
        int num = 0;
        int same = 0;
        //Config previous = start;
        //Config x;
        //Config x_temp = start;
        //Config y = start;
        Config y_temp = start;
        while (true) {
            //System.out.println("findnext while");
            num++;
            /*if (y_temp.coords[0] > 0.48 && y_temp.coords[0] < 0.664 && d == 1) {
                num++;
            }*/
            Config x = stepMove(y_temp, end, tester, cfgSet, 0,d,asvs);
            //printPosition(cfgToASVConfig(x), fw);
            Config y = stepMove(x, end, tester, cfgSet, 1,d,asvs);
            //printPosition(cfgToASVConfig(y), fw);
            if (asvs != null) {
                asvs.add(cfgToASVConfig(x));
                asvs.add(cfgToASVConfig(y));
            }
            if (x.equals(y_temp) && y.equals(x)) {
                Config result = new Config(y.coords, start);
                //cfgSet.add(result);
                //System.out.println("findnext e1");
                return result;
            } else if (y.equals(end)) {
                Config result = new Config(y.coords, start);
                //cfgSet.add(result);
                //System.out.println("findnext e2");
                return result;
            }
            /* record intermediate c-space states every 100 times to reduce the time
            spent on finding the nearest state */
            /*if (num%100 == 0) {
                Config result = new Config(y.coords, previous);
                cfgSet.add(result);
                previous = result;
            }*/
            //x_temp = x;
            if (y.isSame(y_temp)) {
                same++;
            }
            if (same == 10) {
                return new Config(y.coords, start);
            }
            y_temp = y;
        }
    }
    
    /**
     * move horizontally or vertically within one step size
     * @param direction: 0 and 1 means move horizontally and vertically respectively
     * @throws IOException 
     */
    private static Config stepMove(Config start, Config goal, Test tester, HashSet<Config> cfgSet, int direction, int d, ArrayList<ASVConfig> asvs) throws IOException {
        //System.out.println("move s " + direction);
         Config end = new Config(goal.coords);
        //double step = maxDistance(start, end, direction);
        double step = maxDistance(start, end);
        // cut the distance until the step size is valid
        while (step > MAX_STEP) {
            //System.out.println("move while s");
            end = cutDist2(step, start, end, direction);
            //step = maxDistance(start, end, direction);
            step = maxDistance(start, end);
        }
        //System.out.println("move while e");
        ASVConfig asv = cfgToASVConfig(end);
        if (cSpaceCollisionCheck(asv, tester)) {
            //System.out.println("move e1");
            return end;
        } else {
            Config trans = increaseAngle(end, tester);
            if (trans != null) {
                //System.out.println("move findnext s " + direction);
                //fw.flush();
                //start.predecessor = previous;
                //cfgSet.add(start);
                //printPosition(cfgToASVConfig(start), fw);
                //printPosition(cfgToASVConfig(trans), fw);
                recurrent++;
                Config result;
                if (recurrent < 2) {
                    result = findNext2(trans, start, tester, cfgSet,d,asvs);
                } else {
                    result = start;
                }
                //previous.coords = result.coords;
                //previous.predecessor = result.predecessor;
                //fw.flush();
                //System.out.println("move findnext e2 " + direction);
                recurrent--;
                return result;
            }
        }
        //System.out.println("move e3");
        return start;
    }

    private static Config increaseAngle(Config cfg, Test tester) {
        double[] coords = cfg.coords.clone();
        // increase the angles by pi/270 at each step
        for (int i = 0; i < 10; i++) {
            coords[2] *= clockwise;
            coords[2] -= PI/180;
            coords[2] *= clockwise;
            for (int j = 3; j < coords.length; j++) {
                coords[j] *= clockwise;
                if (coords[j] + PI/270 < PI) {
                    coords[j] += PI/270;
                }
                coords[j] *= clockwise;
            }
            cfg = new Config(coords);
            if (cSpaceCollisionCheck(cfgToASVConfig(cfg), tester)) {
                return cfg;
            }
        }
        /*for (int j = coords.length-1; j > 2; j--) {
            for (int i = 0; i < 10; i++) {
                coords[j] *= clockwise;
                if (coords[j] + PI/270 < PI) {
                    coords[j] += PI/270;
                }
                coords[j] *= clockwise;
                cfg = new Config(coords);
                if (cSpaceCollisionCheck(cfgToASVConfig(cfg), tester)) {
                    return cfg;
                }
            }
            
        }*/
        // if no valid c-space state is found, return null
        return null;
    }

    /**
     * scale down the distance by a fixed factor
     * @param start
     * @param end
     * @return
     */
    private static Config cutDist(double step, Config start, Config end) {
        double[] coords1 = start.coords;
        double[] coords2 = end.coords;
        double[] result = new double[coords1.length];
        double scalar = step/MAX_STEP;
        if (scalar < 1.4) scalar = 1.4;
        // scale down
        for (int i = 0; i < coords1.length; i++) {
            result[i] = coords1[i] + (coords2[i] - coords1[i])/scalar;
        }
        return new Config(result);
    }
    private static Config cutDist2(double step, Config start, Config end, int direction) {
        double[] coords1 = start.coords;
        double[] coords2 = end.coords;
        double[] result = new double[coords1.length];
        double scalar = step/MAX_STEP;
        if (scalar < 1.4) scalar = 1.4;
        
        result[direction] = coords1[direction] + (coords2[direction] - coords1[direction])/scalar;
        result[1-direction] = coords1[1-direction];
        for (int i = 2; i < coords1.length; i++) {
            result[i] = coords1[i] + (coords2[i] - coords1[i])/scalar;
        }
        return new Config(result);
    }

    /**
     * test whether two configurations meet the step size restriction
     */
    private static boolean validDistance(Config start, Config end) {
        // TODO Auto-generated method stub  
        ASVConfig asv1 = cfgToASVConfig(start);
        ASVConfig asv2 = cfgToASVConfig(end);
        return asv1.maxDistance(asv2) <= MAX_STEP - (1e-5);
    }
    
    private static double maxDistance(Config start, Config end) {
        ASVConfig asv1 = cfgToASVConfig(start);
        ASVConfig asv2 = cfgToASVConfig(end);
        return asv1.maxDistance(asv2);
    }
    private static double maxDistance(Config start, Config end, int d) {
        ASVConfig asv1 = cfgToASVConfig(start);
        ASVConfig asv2 = cfgToASVConfig(end);
        return asv1.maxDistance(asv2, d);
    }
    
    private static void reversePosition(double[] position) {
        // TODO Auto-generated method stub
        double[] temp = position.clone();
        int l = position.length/2;
        int m;
        for (int i = 0; i < l; i++) {
            m = l - 1 - i;
            position[2*i] = temp[2*m];
            position[2*i+1] = temp[2*m+1];
        }
    }

    private static List<Obstacle> SampleSpace(List<Obstacle> obs){
        List<Double> x = new ArrayList<Double>();
        List<Double> y = new ArrayList<Double>();
        List<Double> w = new ArrayList<Double>();
        List<Double> h = new ArrayList<Double>();
        
        List<Obstacle> rects = new ArrayList<Obstacle>();
        for (int i=0; i<obs.size();i++){
            x.add(obs.get(i).getRect().getX());
            y.add(obs.get(i).getRect().getY());
            w.add(obs.get(i).getRect().getWidth());
            h.add(obs.get(i).getRect().getHeight());
        }
        for (int i=0, j=1; i<obs.size()-1;i++, j++){
            //can change to scale
            double left = x.get(i);
            double right = x.get(j);
            if(left==right){
                Obstacle rect = new Obstacle
                        (x.get(i), y.get(i)+h.get(i), w.get(i), Math.abs(y.get(j)-y.get(i)-h.get(i)));
                rects.add(rect);
            }
        }
        return rects;
    }
    
    private static List<Obstacle> SampleSpaceV2(List<Obstacle> obs){
        List<Obstacle> obsClone = new ArrayList<Obstacle>(obs);
        List<List<Obstacle>> groupRects = new ArrayList<List<Obstacle>>();
        int k = 0;
        for(int i=0;i<obsClone.size();i++){
        	Obstacle current = obs.get(i);
        	obsClone.remove(i);
        	groupRects.get(k).add(current);
        	for (int j=0;j<obsClone.size();j++){
        		Obstacle temp = obs.get(j);
        		if(current.getRect().getX()==temp.getRect().getX()){
        			groupRects.get(k).add(temp);
        			obsClone.remove(j);
        		}
        	}
        	k++;
        }
        
        List<Obstacle> sampleSpace = new ArrayList<Obstacle>();
        for(int l=0;l<groupRects.size();l++){
        	List<Obstacle> currentGroup = groupRects.get(l);
        	if(groupRects.size()<2) groupRects.remove(l);
        	else {
        		Collections.sort(groupRects.get(l),obstacleComparator);
        		
        		//corridor with axis
        		Rectangle2D startOb = currentGroup.get(0).getRect();
        		Rectangle2D endOb = currentGroup.get(groupRects.size()-1).getRect();
        		
        		if(startOb.getMinY()!=0){
        			Obstacle rect = new Obstacle
        					(startOb.getX(), 0,startOb.getWidth(),startOb.getMinY());
        			sampleSpace.add(rect);
        		}
        		
        		if(endOb.getMaxY()!=0){
        			Obstacle rect = new Obstacle
        					(endOb.getX(), 1,endOb.getWidth(),endOb.getMaxY());
        			sampleSpace.add(rect);
        		}
        		//end of it
        		
        		for(int m=0, n=0;m<currentGroup.size()-1;m++, n++){
        			Rectangle2D r1 = currentGroup.get(m).getRect();
        			Rectangle2D r2 = currentGroup.get(n).getRect();
        			Obstacle rect = new Obstacle
        					(r1.getX(), r1.getMaxY(),r1.getWidth(),r2.getMinY()-r1.getMaxY());
        			sampleSpace.add(rect);
        		}
        	}
        }
        return sampleSpace;        
    }
    
    static Comparator<Obstacle> obstacleComparator = new Comparator<Obstacle>(){
		@Override
		public int compare(Obstacle o1, Obstacle o2) {
			return Double.compare(o1.getRect().getMaxY(), o2.getRect().getMinY());
		}
    };
    
    private static void getSinCor(double[] position, Test tester) {
        Random randP = new Random();
        Point2D p = new Point2D.Double(randP.nextDouble(), randP.nextDouble());
        List<Obstacle> obs = tester.ps.obstacles;
        List<Obstacle> rects = SampleSpace(obs);
        while (!containPoint(rects, p)) {
            p = new Point2D.Double(randP.nextDouble(), randP.nextDouble());
        }
        
        position[0] = p.getX();
        position[1] = p.getY();
    }
}

