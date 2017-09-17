package rrt;

import java.io.IOException;
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
	    //angle
	    for(int i = 2; i < dimensions; i++) {
	        Double degree=randP.nextDouble()*Math.PI;
	        pt[i] = Math.toDegrees(degree);
	    }

	    return pt;
	}


}

