package rrt;
/**
 * represent a configuration in c space
 */
public class Config {
    protected double[] coord;   // the coordinates of this configuration in c space
    protected Config predecessor;   // used to trace back to retrieve path
}
