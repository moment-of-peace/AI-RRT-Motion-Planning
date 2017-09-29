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
                if (Math.abs(this.coords[i] - coords2[i]) > 0.00001) return false;
            }
            return true;
        } else {
            return false;
        }
    }

    public boolean isSame(Config cfg) {
        double[] coords2 = cfg.coords;
        double diff = 0;
        for (int i = 0; i < 2; i++) {
            diff += Math.abs(this.coords[i] - coords2[i]);
        }
        if (diff < 2 * 0.00001) {
            return true;
        }
        return false;
    }
    
}
