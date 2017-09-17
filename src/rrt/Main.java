package rrt;
/**
 * motion planning using bidirectional rapidly exploring random tree
 */
import java.io.IOException;
import java.util.HashSet;
import java.util.Random;
import problem.ProblemSpec;

public class Main {
    final double broomLength = 0.05;
    final double step = 0.001;
    
	public static void main(String[] args) throws IOException {
		// load problem from a file
	    String fileName = args[0];
	    ProblemSpec problem = new ProblemSpec();
	    problem.loadProblem(fileName);
	    
	    int asvCount = problem.getASVCount();
	    int dimensions = asvCount + 2; // dimension degree of c space
	    
	    for (int i = 0; i < 1000; i++) {
	        double[] pt = get_random_point(dimensions);

	    }
		double[] pt = get_random_point(dimensions);
		for (double p:pt){
			System.out.println(p);
		}
		
		
	}
	
	public static double[] get_random_point(int dimensions)
	{
	    double[] pt = new double[dimensions];
	    Random randP = new Random();
	    //start position
	    pt[0]= randP.nextDouble();
	    pt[1]= randP.nextDouble();
	    pt[2]= randP.nextDouble()*2*Math.PI-Math.PI;
	    //angle
	    for(int i = 3; i < dimensions; i++) {
	        Double degree=randP.nextDouble()*Math.PI;
	        pt[i] = Math.toDegrees(degree);
	    }
	    return pt;
	}
	
	public static double[] cfgToWorkspace(double[] pt) {
		
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
     * 
     * @param sample: the sampled configuration
     * @param near: nearest configuration to the sample
     * @return: expanded configuration towards the sample from nearest
     */
    public static Config findNext(Config sample, Config near) {
        return null;
    }
}

