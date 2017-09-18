package rrt;

import problem.ASVConfig;

/**
 * represent a configuration in c space
 */
public class Config {
    protected double[] coords;   // the coordinates of this configuration in c space
    protected Config predecessor;   // used to trace back to retrieve path
    
    public Config(double[] coords){
    	this.coords=coords;
    	this.predecessor = null;
    }
    
    public Config(double[] coords, Config predecessor) {
        this.coords = coords;
        this.predecessor = predecessor;
    }
    
    public double wSpaceTotalDistance(Config otherState) {
		if (this.getConfigCount() != otherState.getConfigCount()) {
			return -1;
		}
		double totalDistance = 0;
		for (int i = 0; i < this.getConfigCount(); i++) {
			double currentDist = Math.sqrt
					(Math.pow(this.coords[i]-otherState.coords[i], 2)+Math.pow
							(this.coords[i+1]-otherState.coords[i+1],2));
			totalDistance += currentDist;
			i+=2;
			}
		return totalDistance;
	}
    
    public int getConfigCount() {
		return coords.length;
	}
}
