package rrt;
/**
 * motion planning using bidirectional rapidly exploring random tree
 */
import java.io.IOException;
import java.util.HashSet;
import java.util.Random;
import problem.ProblemSpec;

public class Main {

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

	public static Config findNearest(HashSet<Config> allConfig, Config target) {
	    Config result = null;
	    double dist = Double.POSITIVE_INFINITY;
	    double newDist;
	    
	    for (Config c: allConfig) {
	        newDist = getDist(c.coord)
	    }
	    return null;
	}
}

