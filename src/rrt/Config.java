package rrt;
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
}
