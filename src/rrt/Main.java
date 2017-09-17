package rrt;
/**
 * motion planning using bidirectional rapidly exploring random tree
 */
import java.io.IOException;
import java.util.HashSet;
import java.util.List;
import java.util.Random;

import problem.ASVConfig;
import problem.ProblemSpec;
import tester.Tester;

public class Main {
    public final static double broomLength = 0.05;
    final double step = 0.001;
    
	public static void main(String[] args) throws IOException {
		// load problem from a file
	    String fileName = args[0];
	    ProblemSpec problem = new ProblemSpec();
	    problem.loadProblem(fileName);
	    
	    int asvCount = problem.getASVCount();
	    int dimensions = asvCount + 2; // dimension degree of c space
	    
	    for (int i = 0; i < 1000; i++) {
	        double[] pts = get_random_point(dimensions);
	        double[] coords=cfgToWSpace(pts);
	        ASVConfig cfg=  new ASVConfig(coords);
	        	        
	    }
	}
	/**
	 * Get random C-state with 2 start coords and n-1 angle;
	 * @param dimensions = point.number+2
	 * @return array of C-state
	 */
	public static double[] get_random_point(int dimensions)
	{
	    double[] pts = new double[dimensions];
	    Random randP = new Random();
	    //start position
	    pts[0]= randP.nextDouble();
	    pts[1]= randP.nextDouble();
	    pts[2]= randP.nextDouble()*2*Math.PI-Math.PI;
	    //angle
	    for(int i = 3; i < dimensions; i++) {
	        Double degree=randP.nextDouble()*Math.PI;
	        pts[i] = Math.toDegrees(degree);
	    }
	    return pts;
	}
	/**
	 * return array of coords in work space where i%2=>y,i+1%2=>x
	 * @param pts array of C-state
	 * @return array of coords in work space
	 */
	public static double[] cfgToWSpace(double[] pts) {
		double [] cfgArray= new double[pts.length];
		double currentX=pts[0];
		double currentY=pts[1];
		
		cfgArray[0]=pts[0];
		cfgArray[1]=pts[1];
		int j=2;
		for (int i=2; i<pts.length;i++){
			//need test
			double x = currentX+broomLength*Math.cos(pts[i]);
			double y = currentY+broomLength*Math.sin(pts[i]);
			cfgArray[j]=x;
			cfgArray[j+1]=y;
			j+=2;
			currentX=x;
			currentY=y;
		}
		return cfgArray;
	}
	
	public static boolean cSpaceCollisionCheck(ASVConfig coords){
		Tester test = new Tester();
        if(!test.hasValidBoomLengths(coords)||!test.hasEnoughArea(coords)){
        	//need other test
        	return false;
        }
     
		return true;
	}
	/**
	 * retrieve a configuration which is nearest to the sampled configuration
	 * @param allConfig: all found configuration
	 * @param target: the sampled configuration
	 * @return
	 */
	public static Config findNearest(HashSet<Config> allConfig, Config sample) {
	    Config result = null;
	    double dist = Double.POSITIVE_INFINITY;
	    double newDist;
	    
	    for (Config c: allConfig) {
	        newDist = getDist(c.coords, sample.coords);
	        if (newDist < dist) {
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
    private static double getDist(double[] array1, double[] array2) {
        double sum = 0;
        for (int i = 0; i < array1.length; i++) {
            sum += (array1[i] - array2[i]) * (array1[i] - array2[i]);
        }
        return sum;
    }
    
    /**
     * find the next vertex to extend the tree towards to the sample
     * @param sample: the sampled configuration
     * @param near: nearest configuration to the sample
     * @return: expanded configuration towards the sample from nearest
     */
    public static Config findNext(Config sample, Config near) {
        Config start = near;
        Config result = sample;
        
        // extend towards the sample as far as possible
        while (true) {
            while (distOverflow(start, result)) {
                // scale down, if the next configuration exceeds the step limitation
                result = cutDist(start, result);
            }
            // if the next configuration touches collision space, break loop
            if (!testConfig(result)) {
                return start;
            } else {
                start = result;
                result = sample;
            }
        }
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
            result[i] = 0.75 * coords1[i] + coords2[i];
        }
        return new Config(result);
    }

    /**
     * test whether two configurations meet the step size restriction
     */
    private static boolean distOverflow(Config start, Config end) {
        // TODO Auto-generated method stub
        return false;
    }

    /**
     * test whether a configuration is valid
     */
    private static boolean testConfig(Config result) {
        // TODO Auto-generated method stub
        return false;
    }
}

