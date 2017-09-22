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
    /** The default value for maximum error */
    private static final double DEFAULT_MAX_ERROR = 1e-5;
    
    private static final double PI = Math.PI;
        
    private static int clockwise;
    
	public static void main(String[] args) throws IOException {
		// load problem from a file
	    String srcFile = args[0];
	    String outputName = args[1];
	    Test tester = new Test(srcFile);
	    System.out.println("obstacle number: " + tester.ps.obstacles.size());
	    
	    int asvCount = tester.ps.getASVCount();
	    int dimensions = asvCount + 1; // dimension degree of c space
	    
	    // HashSets used to store found configurations in cspace from init and goal sides
	    HashSet<Config> fromInit = new HashSet<Config>();
	    HashSet<Config> fromGoal = new HashSet<Config>();
	    
	    // get initial and goal coordinates in c space
	    Config initConfig = asvConfigToCfg(tester.ps.getInitialState(),tester);
	    Config goalConfig = asvConfigToCfg(tester.ps.getGoalState(),tester);
	    // clockwise or anti-clockwise
	    if (initConfig.coords.length > 3 && initConfig.coords[3] < 0) {
	        clockwise = -1;
	    } else {
	        clockwise = 1;
	    }
	    //System.out.println("clockwise: " + clockwise);
	    for (double d:initConfig.coords){
	    	System.out.println(d);
	    }
	    System.out.println("");
	    for (double d:goalConfig.coords){
	    	System.out.println(d);
	    }
	    
	    long time1 = 0;
	    long time2 = 0;
	    long start;
	    
	    // add initial and goal into hashsets
	    fromInit.add(initConfig);
	    fromGoal.add(goalConfig);
	    
	    double[] angleRange = getAngleRange(initConfig, goalConfig);
	    
	    // extend tree from both initial and goal point
	    Config initNext = initConfig;
	    Config goalNext = goalConfig;
	    
	    Config sample, nearest1, nearest2;
	    int total = 0;
	    int turn = 1;

	    FileWriter fw1 = new FileWriter("sample1.txt");
        FileWriter fw2 = new FileWriter("sample2.txt");
        FileWriter fw0 = new FileWriter("sample0.txt");
        fw0.write(19998 + " " + 10 + "\n");
        
	    while (!initNext.equals(goalNext)) {
	        total++;
	        if (turn == 1) {
	            sample = getRandomPoint(dimensions, angleRange, tester);
	            ASVConfig asv = cfgToASVConfig(sample);
	            while(!cSpaceCollisionCheck1(asv, tester)) {
	                sample = getRandomPoint(dimensions, angleRange, tester);
	                asv = cfgToASVConfig(sample);
	            }//System.out.println("got random");
	        } else if (turn ==2){
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
	        
	        if (total < 20000) printPosition(cfgToASVConfig(sample), fw0);
	        // find nearest configurations from both sides
	        //start = System.currentTimeMillis();
	        nearest1 = findNearest(fromInit, sample, tester);
	        nearest2 = findNearest(fromGoal, sample, tester);//System.out.println("found nearest");
	        //time1 = time1 + System.currentTimeMillis() - start;
	        // get the next configurations for both sides
	        //start = System.currentTimeMillis();
	        initNext = findNext(sample, nearest1, tester, fromInit);
	        goalNext = findNext(sample, nearest2, tester, fromGoal);//System.out.println("found next");
	        
	        /*double[] position = initNext.coords;
	        double[] position2 = sample.coords;
	        if (position[0] > 0.334 && position[0] < 0.665 && position[1] > 0.485 && position[1] < 0.515) {
	            System.out.println("sample: " + position2[0] + " " + position2[1]);
	            System.out.println("extend: " + position[0] + " " + position[1]);
	        }*/
	        
	        //time2 = time2 + System.currentTimeMillis() - start;
	        //if (initNext != nearest1 && goalNext != nearest2) total++;
	        if (total%1000 == 0) {
	            turn++;
	            if (turn == 4) turn = 1;
	            System.out.println(total + " size: " + fromInit.size() + " " + fromGoal.size());
	        }
	        
	        if (total == 20000) {
	            
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
	    System.out.println("finished, total configs: " + total);
	    
	    FileWriter fw = new FileWriter(outputName);
	    List<ASVConfig> solution = getSol1(initNext,tester);
	    solution.addAll(getSol2(goalNext,tester));
	    tester.ps.setPath(solution);
	    fw.write(solution.size()-1+" "+tester.ps.calculateTotalCost()+"\n"); //get sol cost
	    printPosition(tester.ps.getInitialState(),fw);
	    for (ASVConfig asv:solution){
	    	printPosition(asv,fw);
	    }
	    printPosition(tester.ps.getGoalState(),fw);
	    fw.close();
	}
	
	private static List<ASVConfig> getSol1(Config initNext, Test tester) 
			throws IOException {
        // TODO Auto-generated method stub
        ArrayList<Config> solution = new ArrayList<Config>();
        List<ASVConfig> sol1 = new ArrayList<ASVConfig>();
        Config pred = initNext.predecessor;
        while (pred != null) {
            solution.add(pred);
            pred = pred.predecessor;
        }
        Config config;
        for (int i = solution.size()-1; i > -1; i--) {
            config = solution.get(i);
            Config start = config;
            if(i!=0){
            	Config result = solution.get(i-1);
	            while(!validDistance(start,result)){
	            	while(!validDistance(start, result)){
		        		result = cutDist(start, result);
		        	}
	            	sol1.add(cfgToASVConfig(result));
	        		start=result;
	        		result=solution.get(i-1);
	            }
            }else{	        		
        		Config result = initNext;
            	while(!validDistance(start,result)){
	            	while(!validDistance(start, result)){
		        		result = cutDist(start, result);
		        	}
	            	sol1.add(cfgToASVConfig(result));
	        		start=result;
	        		result=initNext;
	            }
            }
        }
        return sol1;
        
     }

    private static List<ASVConfig> getSol2(Config goalNext, Test tester) 
    		throws IOException {
        // TODO Auto-generated method stub
    	List<ASVConfig> sol2 = new ArrayList<ASVConfig>();
    	sol2.add(cfgToASVConfig(goalNext));
               
        while(goalNext.predecessor!=null){
        	Config start = goalNext;
        	Config result = goalNext.predecessor;
        	while (!validDistance(start, result)){
	        	while(!validDistance(start, result)){
	        		result = cutDist(start, result);
	        	}
	        	sol2.add(cfgToASVConfig(result));
        		start=result;
        		result=goalNext.predecessor;
        	}
            goalNext=goalNext.predecessor;
            //printPosition(goalNext,fw);
        }
        return sol2;
        
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
	        angleRange[i] = (initCoords[i+2]<goalCoords[i+2]?initCoords[i+2]:goalCoords[i+2]);
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
	    int in = 0;    //debug
	    
	    while (true) {
	        double[] position = new double[(dimensions-1)*2];
	        double pre = 0;
	        flag = true;
	        times++;
	        
	        // start position
	        getStartPosition(position, tester);
	        /*if (position[0] > 0.334 && position[0] < 0.665 && position[1] > 0.485 && position[1] < 0.515) {
	            in++;
	            //System.out.println("is in");
	            //System.out.println(position[0] + " " + position[1]);
	        }*/
	        // generate following positions
	        int i;
	        for(i = 1; i < dimensions-1; i++) {
	            pre = getNextPoint(pre, angleRange[i-1], position, i, tester);
	            if (pre == 10) {
	                flag = false;
	                in = 0;
	                //System.out.println("fail: " + position[0] + " " + position[1]);
	                break;
	            }
	        }
	        if (flag == true) {
	            /*if (in == 1) {
	                System.out.println("successful");
	                System.out.println(position[0] + " " + position[1]);
	                in = 0;
	            }*/
	            return asvConfigToCfg(new ASVConfig(position), tester);
	        }
	        if (times%50 == 0) {
	            System.out.println("rand: " + times + " " + i + " "+ position[0] + " " + position[1]);
	        }
	    }
	    //return toConfig(new ASVConfig(position), tester);
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
                limit = limit*clockwise;//*0.75;
                double angle = (rand.nextDouble()*(PI-limit)+limit)*clockwise;
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
		return new ASVConfig(cfgArray);
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
	        newDist = cSpaceDist(c.coords, sample.coords, tester);
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
        
        /*for (int i = 2; i < n; i++) {
            double angleDiff = PI - Math.abs(tester.normaliseAngle(array1[2] - array2[2]));
            diff = diff + (angleDiff*angleDiff);
        }*/
        return diff - dist;
    }
    private static double cSpaceDist2(Config c, Config sample, Test tester) {
        double dist = 0;
        boolean flag = new Random().nextBoolean();
        if (flag) {
            double[] array1 = c.coords;
            double[] array2 = sample.coords;
            for (int i = 0; i < 2; i++) {
                dist += (array1[i] - array2[i]) * (array1[i] - array2[i]);
            }
        } else {
            double[] array1 = cfgToArray(c);
            double[] array2 = cfgToArray(sample);
            int m;
            for (int i = 0; i < 2; i++) {
                m = array1.length - 1 - i;
                dist += (array1[m] - array2[m]) * (array1[m] - array2[m]);
            }
        }
        
        /*
        ASVConfig asv1 = cfgToASVConfig(c);
        ASVConfig asv2 = cfgToASVConfig(sample);
        int num = asv1.getASVCount();
        Point2D pstart = asv1.getASVPositions().get(0);
        Point2D pend = asv1.getASVPositions().get(num-1);
        Point2D qstart = asv2.getASVPositions().get(0);
        Point2D qend = asv2.getASVPositions().get(num-1);
        
        double dist =  pstart.distance(qstart);
        dist += pend.distance(qend);
        */
        /*int n = 1 + c.length/2;
        for (int i = 0; i < 2; i++) {
            dist += (c[i] - sample[i]) * (c[i] - sample[i]);
        }*/
        /*for (int i = 2; i < n; i++) {
            double angleDiff = PI - Math.abs(tester.normaliseAngle(array1[2] - array2[2]));
            diff = diff + (angleDiff*angleDiff);
        }*/
        return -dist;
    }
    
    /**
     * find the next vertex to extend the tree towards to the sample
     * @param sample: the sampled configuration
     * @param near: nearest configuration to the sample
     * @return: expanded configuration towards the sample from nearest
     */
    private static Config findNext(Config sample, Config near, Test tester, HashSet<Config> cfgSet) {
        Config start = near;
        Config result = sample;
        Config previous = near;
        Config next;
        ASVConfig asv;
        int num = 1;
         
        // extend towards the sample as far as possible
        while (true) {
            double step = maxDistance(start, result);
            //int i = 0;
            /*if (!validDistance(start, result)) {
                result = normDist(start, result);
            }*/
            while (step > MAX_STEP) {
                // scale down, if the next configuration exceeds the step limitation
                //i++;
                result = cutDist(step, start, result);
                step = maxDistance(start, result);
                //if (i%1000 == 0) System.out.println(i);
            }
            //System.out.println("distance ok: " + i);
            // if the next configuration touches collision space, break loop
            asv = cfgToASVConfig(result);
            if (!cSpaceCollisionCheck(asv, tester)) {
                //if (start.equals(sample)) System.out.println("equal");
                //System.out.println("next ok");
                //System.out.println("next " + num);
                double[] coords = result.coords.clone();
                for (int j = 3; j < coords.length; j++) {
                    coords[j] *= clockwise;
                    if (coords[j] - PI/180 > 0) {
                        coords[j] -= PI/180;
                    }
                }
                result = new Config(coords);
                asv = cfgToASVConfig(result);
                if (!cSpaceCollisionCheck(asv, tester)) {
                    for (int j = 3; j < coords.length; j++) {
                        coords[j] *= clockwise;
                        if (coords[j] + PI/90 < PI) {
                            coords[j] += PI/90;
                        }
                    }
                    result = new Config(coords);
                    asv = cfgToASVConfig(result);
                    if (cSpaceCollisionCheck(asv, tester)) {
                        step = maxDistance(start, result);
                        while (step > MAX_STEP) {
                            result = cutDist(step, start, result);
                            step = maxDistance(start, result);
                        }
                        result.predecessor = previous;
                        cfgSet.add(result);
                        //System.out.println("success");
                        if (validDistance(result, sample)) {
                            //System.out.println("equal");
                            next = new Config(sample.coords, previous);
                            cfgSet.add(next);
                            //System.out.println("next " + num);
                            return next;
                        }
                        return result;
                    }
                } else {
                    step = maxDistance(start, result);
                    while (step > MAX_STEP) {
                        result = cutDist(step, start, result);
                        step = maxDistance(start, result);
                    }
                    result.predecessor = previous;
                    cfgSet.add(result);
                    //System.out.println("success");
                    if (validDistance(result, sample)) {
                        //System.out.println("equal");
                        next = new Config(sample.coords, previous);
                        cfgSet.add(next);
                        //System.out.println("next " + num);
                        return next;
                    }
                    return result;
                }
                //System.out.println("fail");
                return start; // if start is near
            } else if (validDistance(result, sample)) {
                //System.out.println("equal");
                next = new Config(sample.coords, previous);
                cfgSet.add(next);
                //System.out.println("next " + num);
                return next;
            } else {
                //System.out.println("next retry");
                num++;
                if (num%100 == 0) {
                    next = new Config(result.coords, previous);
                    cfgSet.add(next);
                    previous = next;
                }
                start = result;
                result = sample;
            }
        }
    }

    private static Config normDist(Config start, Config end) {
        double diffx = end.coords[0] - start.coords[0];
        double diffy = end.coords[1] - start.coords[1];
        Config result = new Config(end.coords.clone());
        double scalar = (diffx * diffx + diffy * diffy)/(MAX_STEP*MAX_STEP);
        
        scalar = Math.pow(scalar, 0.5);
        result.coords[0] = start.coords[0] + diffx/scalar;
        result.coords[1] = start.coords[1] + diffy/scalar;
        return result;
    }

    /**
     * scale down the distance by a fixed factor
     * @param start
     * @param end
     * @return
     */
    private static Config cutDist(Config start, Config end) {
        double[] coords1 = start.coords;
        double[] coords2 = end.coords;
        double[] result = new double[coords1.length];
        // scale down
        for (int i = 0; i < coords1.length; i++) {
            result[i] = coords1[i] + 0.4 * (coords2[i] - coords1[i]);
        }
        return new Config(result);
    }
    
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

    /**
     * test whether two configurations meet the step size restriction
     */
    private static boolean validDistance(Config start, Config end) {
        // TODO Auto-generated method stub  
        ASVConfig asv1 = cfgToASVConfig(start);
        ASVConfig asv2 = cfgToASVConfig(end);
        return asv1.maxDistance(asv2) <= MAX_STEP + DEFAULT_MAX_ERROR;
    }
    
    private static double maxDistance(Config start, Config end) {
        ASVConfig asv1 = cfgToASVConfig(start);
        ASVConfig asv2 = cfgToASVConfig(end);
        return asv1.maxDistance(asv2);
    }
    
    private static Config getRandomPoint2(int dimensions, double[] angleRange, Test tester) {
        boolean flag = true;
        int times = 0;
        int in = 0;    //debug
        
        double[] range = angleRange.clone();
        while (true) {
            double[] position = new double[(dimensions-1)*2];
            double pre = 0;
            flag = true;
            times++;
            
            // start position
            
            
            //getStartPosition(position, tester);
            
            
            getSinCor(position,tester);
            /*if (position[0] > 0.334 && position[0] < 0.665 && position[1] > 0.485 && position[1] < 0.515) {
                in++;
                //System.out.println("is in");
                //System.out.println(position[0] + " " + position[1]);
            }*/
            // generate following positions
            int i;
            for(i = 1; i < dimensions-1; i++) {
                pre = getNextPoint(pre, range[i-1], position, i, tester);
                if (pre == 10) {
                    flag = false;
                    in = 0;
                    //System.out.println("fail: " + position[0] + " " + position[1]);
                    break;
                }
            }
            if (flag == true) {
                /*if (in == 1) {
                    System.out.println("successful");
                    System.out.println(position[0] + " " + position[1]);
                    in = 0;
                }*/
                return asvConfigToCfg(new ASVConfig(position), tester);
            }
            if (times%50 == 0) {
                System.out.println("rand: " + times + " " + i + " "+ position[0] + " " + position[1]);
            }
        }
        //return toConfig(new ASVConfig(position), tester);
    }
    
    private static Config getRandomPoint3(int dimensions, double[] angleRange, Test tester) {
        boolean flag = true;
        int times = 0;
        int in = 0;    //debug
        
        double[] range = angleRange.clone();
        for (int j = 1; j < range.length; j++) {
            range[j] = -angleRange[range.length-j];
        }
        while (true) {
            double[] position = new double[(dimensions-1)*2];
            double pre = 0;
            flag = true;
            times++;
            
            // start position
            
            
            //getStartPosition(position, tester);
            
            
            getSinCor(position,tester);
            /*if (position[0] > 0.334 && position[0] < 0.665 && position[1] > 0.485 && position[1] < 0.515) {
                in++;
                //System.out.println("is in");
                //System.out.println(position[0] + " " + position[1]);
            }*/
            // generate following positions
            int i;
            for(i = 1; i < dimensions-1; i++) {
                pre = getNextPoint(pre, range[i-1], position, i, tester, -clockwise);
                if (pre == 10) {
                    flag = false;
                    in = 0;
                    //System.out.println("fail: " + position[0] + " " + position[1]);
                    break;
                }
            }
            if (flag == true) {
                /*if (in == 1) {
                    System.out.println("successful");
                    System.out.println(position[0] + " " + position[1]);
                    in = 0;
                }*/
                reversePosition(position);
                return asvConfigToCfg(new ASVConfig(position), tester);
            }
            if (times%50 == 0) {
                System.out.println("rand: " + times + " " + i + " "+ position[0] + " " + position[1]);
            }
        }
        //return toConfig(new ASVConfig(position), tester);
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
        /*for (Obstacle o: obs){
            System.out.println(o);
        }*/
        //System.out.println("ww");
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
                //Rectangle2D rect = new Rectangle2D.Double
                //      (x.get(i), y.get(i)+h.get(i), w.get(i), Math.abs(y.get(j)-y.get(i)-h.get(i)));
                Obstacle rect = new Obstacle
                        (x.get(i), y.get(i)+h.get(i), w.get(i), Math.abs(y.get(j)-y.get(i)-h.get(i)));
                rects.add(rect);
                //System.out.println(rect);
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

