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
    
    public int getConfigCount() {
		return coords.length;
	}
    
    @Override
    public boolean equals(Object config) {
        if (config instanceof Config) {
            double[] coords2 = ((Config)config).coords;
            if (this.coords.length != coords2.length) return false;
            for (int i = 0; i < coords2.length; i++) {
                if (this.coords[i] != coords2[i]) return false;
            }
            return true;
        } else {
            return false;
        }
    }
    
}
