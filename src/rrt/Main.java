package rrt;
/**
 * motion planning using bidirectional rapidly exploring random tree
 */

import java.awt.geom.Point2D;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Random;

public class Main {
    /** The maximum distance any ASV can travel between two states */
    private static final double MAX_STEP = 0.001;
    /** The maximum allowable boom length */
    private static final double MAX_BOOM_LENGTH = 0.05;
    
    private static final double PI = Math.PI;
    /** the orientation */
    private static int clockwise;
    /** number of total samples */
    private static int total = 0;
    
    private static int recurrent = 0;

    public static void main(String[] args) throws IOException {
        // load problem from a file
        String srcFile = args[0];
        String outputName = args[1];
        Test tester = new Test(srcFile);
        int obsNum = tester.ps.obstacles.size();
        // grow obstacles to avoid collisions
        if (obsNum > 2) {
            for (Obstacle o: tester.ps.obstacles) {
                o.rect = Test.grow(o.rect, 2e-5);
            }
        }
        
        int asvCount = tester.ps.getASVCount();
        int dimensions = asvCount + 1; // dimension degree of c space
        
        // HashSets used to store found configurations in c-space from initial and goal sides
        HashSet<Config> fromInit = new HashSet<Config>();
        HashSet<Config> fromGoal = new HashSet<Config>();
        
        // get initial and goal states in c space
        Config initConfig = asvConfigToCfg(tester.ps.getInitialState(),tester);
        Config goalConfig = asvConfigToCfg(tester.ps.getGoalState(),tester);
        
        // check the orientation of initial and goal ASVs
        if (initConfig.coords.length > 3 && initConfig.coords[3] < 0) {
            clockwise = -1;
        } else {
            clockwise = 1;
        }
        
        // add initial and goal into hashsets
        fromInit.add(initConfig);
        fromGoal.add(goalConfig);
        // used to limit the angle range during sampling
        double[] angleRange = getAngleRange(initConfig, goalConfig);
        
        // extend tree from both initial and goal point
        Config initNext = initConfig;
        Config goalNext = goalConfig;
        Config sample, nearest1, nearest2;
        
        // used to switch different strategies
        int turn = 0;
        // extend the trees from both initial and goal sides
        while (!initNext.equals(goalNext)) {
            total++;
            if (obsNum != 2 || turn < 3) {
                // sample strategy 1, normal sample
                sample = getRandomPoint(dimensions, angleRange, tester);
                ASVConfig asv = cfgToASVConfig(sample);
                
                while(!cSpaceCheck1(asv, tester)) {
                    sample = getRandomPoint(dimensions, angleRange, tester);
                    asv = cfgToASVConfig(sample);
                }                
            } else if (turn < 6){
            	// sample strategy 2, sample in narrow passages
                sample = getRandomPoint2(dimensions, angleRange, tester);
                ASVConfig asv = cfgToASVConfig(sample);
                while(!cSpaceCheck1(asv, tester)) {
                    sample = getRandomPoint2(dimensions, angleRange, tester);
                    asv = cfgToASVConfig(sample);
                }
            } else {
                // sample strategy 2, sample in narrow passages
                sample = getRandomPoint3(dimensions, angleRange, tester);
                ASVConfig asv = cfgToASVConfig(sample);
                while(!cSpaceCheck1(asv, tester)) {
                    sample = getRandomPoint3(dimensions, angleRange, tester);
                    asv = cfgToASVConfig(sample);
                }
            }            
            // find nearest configurations in c-space from both sides and extend the trees
            if (turn%3 == 1) {
            	/*
            	 * extending strategy 1: both the two tree will grow towards the sample
            	 */
                nearest1 = findNearest(fromInit, sample, tester);
                initNext = findNext2(sample, nearest1, tester);
                nearest2 = findNearest(fromGoal, initNext, tester);
                goalNext = findNext2(initNext, nearest2, tester);
                fromInit.add(initNext);
                fromGoal.add(goalNext);
                
            } else if (turn%3 == 2) {
            	/*
            	 * extending strategy 2: the goal side grows towards the sample first, then 
            	 * the initial side grows towards the goal side
            	 */
                nearest2 = findNearest(fromGoal, sample, tester);
                goalNext = findNext2(sample, nearest2, tester);
                nearest1 = findNearest(fromInit, goalNext, tester);
                initNext = findNext2(goalNext, nearest1, tester);
                fromInit.add(initNext);
                fromGoal.add(goalNext);
            } else {
                /*
                 * extending strategy 3: the initial side grows towards the sample first, then 
                 * the goal side grows towards the initial side
                 */
                nearest1 = findNearest(fromInit, sample, tester);
                nearest2 = findNearest(fromGoal, sample, tester);
                initNext = findNext2(sample, nearest1, tester);
                goalNext = findNext2(sample, nearest2, tester);
                fromInit.add(initNext);
                fromGoal.add(goalNext);
            }
            if (total%500 == 0) {
                // this is the signal to change different strategies
                turn++;
                if (turn == 8) turn = 0;
                System.out.println("samples: " + total);
            }
        }
        // finished
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
    /*
     * this method is used to retrieve the path from the tree on the initial side
     */
    private static ArrayList<ASVConfig> getSol1(Config initNext, Test tester) {
        ArrayList<Config> solution = new ArrayList<Config>();
        ArrayList<ASVConfig> sol1 = new ArrayList<ASVConfig>();
        solution.add(initNext);
        Config pred = initNext.predecessor;
        
        while (pred != null) {
            solution.add(pred);
            pred = pred.predecessor;
        }
        for (int i = solution.size()-1; i > 0; i--) {
            Config result = solution.get(i-1);
            sol1.add(cfgToASVConfig(result));
        }
        return sol1;
     }
    /*
     * this method is used to retrieve the path from the tree on the goal side
     */
    private static ArrayList<ASVConfig> getSol2(Config goalNext, Test tester) {
        ArrayList<Config> solution = new ArrayList<Config>();
        ArrayList<ASVConfig> sol2 = new ArrayList<ASVConfig>();
        solution.add(goalNext);
        Config pred = goalNext.predecessor;
        while (pred != null) {
            solution.add(pred);
            pred = pred.predecessor;
        }
        for (int i = solution.size()-1; i > 0; i--) {
            Config result = solution.get(i-1);
            sol2.add(cfgToASVConfig(result));
        }
        ArrayList<ASVConfig> reverse = new ArrayList<ASVConfig>();
        int len = sol2.size();
        for (int i = 0; i < len; i++) {
            reverse.add(sol2.get(len-1-i));
        }
        return reverse;
    }
    /*
     * write an ASVConfig to a file
     */
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
     * the angle range will be used in sampling to improve the efficiency
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
     * convert a c-space state to an ASVs in workspace
     * @param C-state
     * @return ASVs in work space
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
    
    /**
     * the following are 3 different sample strategies used to get random C-state
     * @param dimensions = point.number+2
     * @param angleRange 
     * @return a random C-state
     */
    private static Config getRandomPoint(int dimensions, double[] angleRange, Test tester) {
        boolean flag = true;
        int times = 0;
        boolean uselimit = true;
        
        while (true) {
            double[] position = new double[(dimensions-1)*2];
            double pre = 0;
            flag = true;
            times++;
            
            // start position
            getStartPosition(position, tester);
            int i;
            for(i = 1; i < dimensions-1; i++) {
                if (uselimit) pre = getNextPoint(pre, angleRange[i-1], position, i, tester);
                else pre = getNextPoint(pre, 2*PI*clockwise, position, i, tester);
                if (pre == 10) {
                    flag = false;
                    break;
                }
            }
            if (flag == true) {
                return asvConfigToCfg(new ASVConfig(position), tester);
            }
            if (times%50 == 0) {
                uselimit = false;
                System.out.println("rand: " + times + " " + i + " "+ position[0] + " " + position[1]);
            }
        }
    }
    private static Config getRandomPoint2(int dimensions, double[] angleRange, Test tester) {
        boolean flag = true;
        int times = 0;
        boolean uselimit = true;
        
        double[] range = angleRange.clone();
        while (true) {
            double[] position = new double[(dimensions-1)*2];
            double pre = 0;
            flag = true;
            times++;
            
            getSinCor(position,tester);
            int i;
            for(i = 1; i < dimensions-1; i++) {
                if (uselimit) pre = getNextPoint(pre, range[i-1], position, i, tester);
                else pre = getNextPoint(pre, 2*PI*clockwise, position, i, tester);
                if (pre == 10) {
                    flag = false;
                    break;
                }
            }
            if (flag == true) {
                return asvConfigToCfg(new ASVConfig(position), tester);
            }
            if (times%50 == 0) {
                uselimit = false;
                System.out.println("rand: " + times + " " + i + " "+ position[0] + " " + position[1]);
            }
        }
    }
    private static Config getRandomPoint3(int dimensions, double[] angleRange, Test tester) {
        boolean flag = true;
        int times = 0;
        boolean uselimit = true;
        
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
                if (uselimit) pre = getNextPoint(pre, range[i-1], position, i, tester, -clockwise);
                else pre = getNextPoint(pre, 2*PI*clockwise, position, i, tester, -clockwise);
                if (pre == 10) {
                    flag = false;
                    break;
                }
            }
            if (flag == true) {
                reversePosition(position);
                return asvConfigToCfg(new ASVConfig(position), tester);
            }
            if (times%50 == 0) {
                uselimit = false;
                System.out.println("rand: " + times + " " + i + " "+ position[0] + " " + position[1]);
            }
        }
    }
    /*
     * generate the coordinates of a start point when sampling
     */
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
     * generate parameters of a c-space state one by one when sampling
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
                if (cSpaceCheck2(asv, tester)) {
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
                if (cSpaceCheck2(asv, tester)) {
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
    
    /*
     * reverse an array
     */
    private static void reversePosition(double[] position) {
        double[] temp = position.clone();
        int l = position.length/2;
        int m;
        for (int i = 0; i < l; i++) {
            m = l - 1 - i;
            position[2*i] = temp[2*m];
            position[2*i+1] = temp[2*m+1];
        }
    }
    
    /*
     * used to check the validity of different aspects
     */
    private static boolean cSpaceCheck(ASVConfig cfg, Test test) {
        if(test.hasEnoughArea(cfg) && test.isConvex(cfg) && test.fitsBounds(cfg) 
                && !test.hasCollision(cfg, test.ps.obstacles)) {
            return true;
        } else {
            return false;
        }
}
    private static boolean cSpaceCheck1(ASVConfig cfg, Test test) {
        if(test.hasEnoughArea(cfg) && test.isConvex(cfg)) {
            return true;
        } else {
            return false;
        }
    }
    private static boolean cSpaceCheck2(ASVConfig cfg, Test test) {
        if(test.fitsBounds(cfg) && !test.hasCollision(cfg, test.ps.obstacles)) {
            return true;
        } else {
            return false;
        }
    }
    
    /**
     * retrieve a configuration which is nearest to the sampled c-space state
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
        return -dist;
    }
    
    /**
     * find the next vertex to extend the tree towards to the sample
     * @param sample: the sampled configuration
     * @param near: nearest configuration to the sample
     * @return: expanded configuration towards the sample from nearest
     */
    private static Config findNext2(Config end, Config start, Test tester) {
        int same = 0;
        Config y_temp = start;
        Config result;
        
        while (true) {
            // move one step size on the direction of x and y respectively
            Config x = stepMove(y_temp, end, tester, 0);
            Config y = stepMove(x, end, tester, 1);
            
            if (x.equals(y_temp) && y.equals(x)) {  //  can't move any more on both x and y direction 
                result = new Config(y.coords, y.predecessor);
                return result;
            } else if (y.equals(end)) { // encounter the target point
                result = new Config(y.coords, y.predecessor);
                return result;
            }
            
            if (y.isSame(y_temp)) {
                same++;
                if (same == 1) result = y;
            } else {
                same = 0;
            }
            if (same == 10) {
                return new Config(y.coords, y.predecessor);
            }
            y_temp = y;
        }
    }
    
    /**
     * move horizontally or vertically within one step size
     * @param direction: 0 and 1 means move horizontally and vertically respectively
     * @throws IOException 
     */
    private static Config stepMove(Config start, Config goal, Test tester, int direction) {
        Config end = new Config(goal.coords);
        double step = maxDistance(start, end);
        
        // cut the distance until the step size is valid
        while (step > MAX_STEP) {
            end = cutDist2(step, start, end, direction);
            step = maxDistance(start, end);
        }
        // check whether the extended point is valid
        ASVConfig asv = cfgToASVConfig(end);
        if (cSpaceCheck(asv, tester)) {
            end.predecessor = start;
            return end;
        } else {
            // if not valid, try to enlarge the angles
            Config trans = increaseAngle(end, tester);
            if (trans != null) {
                recurrent++;    // this is used to avoid infinitive recursion
                Config result;
                if (recurrent < 2) {
                    // call findNext recursively
                    result = findNext2(trans, start, tester);
                } else {
                    result = start;
                }
                recurrent--;
                return result;
            }
        }
        return start;
    }

    private static Config increaseAngle(Config cfg, Test tester) {
        double[] coords = cfg.coords.clone();
        // increase the angles by by a small value at each step
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
            if (cSpaceCheck(cfgToASVConfig(cfg), tester)) {
                return cfg;
            }
        }
        // if no valid c-space state is found, return null
        return null;
    }

    /**
     * scale down the distance by a factor
     * @param start
     * @param end
     * @return
     */
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
    /*
     * the corresponding workspace max distance between two c-space states
     */
    private static double maxDistance(Config start, Config end) {
        ASVConfig asv1 = cfgToASVConfig(start);
        ASVConfig asv2 = cfgToASVConfig(end);
        return asv1.maxDistance(asv2);
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

