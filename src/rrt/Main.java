package rrt;

import java.util.Random;

public class Main {

	public static void main(String[] args) {
		// TODO Auto-generated method stub
		int dimensions = Integer.parseInt(args[0]);
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


}

