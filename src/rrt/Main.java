package rrt;
/**
 * motion planning using bidirectional rapidly exploring random tree
 */

import java.awt.geom.Line2D;
import java.awt.geom.Point2D;
import java.awt.geom.Rectangle2D;
import java.io.IOException;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Random;

import problem.ASVConfig;
import problem.Obstacle;
import problem.ProblemSpec;
import tester.Tester;

public class Main {
    /** The maximum distance any ASV can travel between two states */
    private static final double MAX_STEP = 0.001;
    /** The minimum allowable boom length */
    private static final double MIN_BOOM_LENGTH = 0.05;
    /** The maximum allowable boom length */
    private static final double MAX_BOOM_LENGTH = 0.05;
    /** The workspace bounds */
    private static final Rectangle2D BOUNDS = new Rectangle2D.Double(0, 0, 1, 1);
    /** The default value for maximum error */
    private static final double DEFAULT_MAX_ERROR = 1e-5;

    /**
     * Returns the minimum area required for the given number of ASVs.
     *
     * @param asvCount
     *            the number of ASVs
     * @return the minimum area required.
     */
    private static final double getMinimumArea(int asvCount) {
        double radius = 0.007 * (asvCount - 1);
        return Math.PI * radius * radius;
    }

    /**
     * Creates a new Rectangle2D that is grown by delta in each direction
     * compared to the given Rectangle2D.
     *
     * @param rect
     *            the Rectangle2D to expand.
     * @param delta
     *            the amount to expand by.
     * @return a Rectangle2D expanded by delta in each direction.
     */
    private static Rectangle2D grow(Rectangle2D rect, double delta) {
        return new Rectangle2D.Double(rect.getX() - delta, rect.getY() - delta,
                rect.getWidth() + delta * 2, rect.getHeight() + delta * 2);
    }

    /** Remembers the specifications of the problem. */
    private ProblemSpec ps = new ProblemSpec();
    /** The maximum error allowed by this Tester */
    private double maxError;
    /** The workspace bounds, with allowable error. */
    private Rectangle2D lenientBounds;

    /**
     * Constructor. Creates a Tester with the default value for maximum error.
     */
    public Main() {
        this(DEFAULT_MAX_ERROR);
    }

    /**
     * Constructor. Creates a Tester with the given maximum error.
     *
     * @param maxError
     *            the maximum allowable error.
     */
    public Main(double maxError) {
        this.maxError = maxError;
        lenientBounds = grow(BOUNDS, maxError);
    }

    /**
     * Returns a copy of list where each value is incremented by delta.
     *
     * @param list
     *            the list of integers to add to.
     * @return a copy of list where each value is incremented by delta.
     */
    private List<Integer> addToAll(List<Integer> list, int delta) {
        List<Integer> result = new ArrayList<Integer>();
        for (int i : list) {
            result.add(i + delta);
        }
        return result;
    }

    /**
     * Returns whether the step from s0 to s1 is a valid primitive step.
     *
     * @param cfg0
     *            A configuration.
     * @param cfg1
     *            Another configuration.
     * @return whether the step from s0 to s1 is a valid primitive step.
     */
    private boolean isValidStep(ASVConfig cfg0, ASVConfig cfg1) {
        return (cfg0.maxDistance(cfg1) <= MAX_STEP + maxError);
    }

    /**
     * Returns whether the booms in the given configuration have valid lengths.
     *
     * @param cfg
     *            the configuration to test.
     * @return whether the booms in the given configuration have valid lengths.
     */
    private boolean hasValidBoomLengths(ASVConfig cfg) {
        List<Point2D> points = cfg.getASVPositions();
        for (int i = 1; i < points.size(); i++) {
            Point2D p0 = points.get(i - 1);
            Point2D p1 = points.get(i);
            double boomLength = p0.distance(p1);
            if (boomLength < MIN_BOOM_LENGTH - maxError) {
                return false;
            } else if (boomLength > MAX_BOOM_LENGTH + maxError) {
                return false;
            }
        }
        return true;
    }

    /**
     * Normalises an angle to the range (-pi, pi]
     *
     * @param angle
     *            the angle to normalise.
     * @return the normalised angle.
     */
    private double normaliseAngle(double angle) {
        while (angle <= -Math.PI) {
            angle += 2 * Math.PI;
        }
        while (angle > Math.PI) {
            angle -= 2 * Math.PI;
        }
        return angle;
    }

    /**
     * Returns whether the given configuration is convex.
     *
     * @param cfg
     *            the configuration to test.
     * @return whether the given configuration is convex.
     */
    private boolean isConvex(ASVConfig cfg) {
        List<Point2D> points = cfg.getASVPositions();
        points.add(points.get(0));
        points.add(points.get(1));

        double requiredSign = 0;
        double totalTurned = 0;
        Point2D p0 = points.get(0);
        Point2D p1 = points.get(1);
        double angle = Math.atan2(p1.getY() - p0.getY(), p1.getX() - p0.getX());
        for (int i = 2; i < points.size(); i++) {
            Point2D p2 = points.get(i);
            double nextAngle = Math.atan2(p2.getY() - p1.getY(),
                    p2.getX() - p1.getX());
            double turningAngle = normaliseAngle(nextAngle - angle);

            if (turningAngle == Math.PI) {
                return false;
            }

            totalTurned += Math.abs(turningAngle);
            if (totalTurned > 3 * Math.PI) {
                return false;
            }

            double turnSign;
            if (turningAngle < -maxError) {
                turnSign = -1;
            } else if (turningAngle > maxError) {
                turnSign = 1;
            } else {
                turnSign = 0;
            }

            if (turnSign * requiredSign < 0) {
                return false;
            } else if (turnSign != 0) {
                requiredSign = turnSign;
            }

            p0 = p1;
            p1 = p2;
            angle = nextAngle;
        }
        return true;
    }

    /**
     * Returns whether the given configuration has sufficient area.
     *
     * @param cfg
     *            the configuration to test.
     * @return whether the given configuration has sufficient area.
     */
    private boolean hasEnoughArea(ASVConfig cfg) {
        double total = 0;
        List<Point2D> points = cfg.getASVPositions();
        points.add(points.get(0));
        points.add(points.get(1));
        for (int i = 1; i < points.size() - 1; i++) {
            total += points.get(i).getX()
                    * (points.get(i + 1).getY() - points.get(i - 1).getY());
        }
        double area = Math.abs(total) / 2;
        return (area >= getMinimumArea(cfg.getASVCount()) - maxError);
    }

    /**
     * Returns whether the given configuration fits wholly within the bounds.
     *
     * @param cfg
     *            the configuration to test.
     * @return whether the given configuration fits wholly within the bounds.
     */
    private boolean fitsBounds(ASVConfig cfg) {
        for (Point2D p : cfg.getASVPositions()) {
            if (!lenientBounds.contains(p)) {
                return false;
            }
        }
        return true;
    }

    /**
     * Returns whether the given config collides with any of the given
     * obstacles.
     *
     * @param cfg
     *            the configuration to test.
     * @param obstacles
     *            the obstacles to test against.
     * @return whether the given config collides with any of the given
     *         obstacles.
     */
    private boolean hasCollision(ASVConfig cfg, List<Obstacle> obstacles) {
        for (Obstacle o : obstacles) {
            if (hasCollision(cfg, o)) {
                return true;
            }
        }
        return false;
    }

    /**
     * Returns whether the given config collides with the given obstacle.
     *
     * @param cfg
     *            the configuration to test.
     * @param o
     *            the obstacle to test against.
     * @return whether the given config collides with the given obstacle.
     */
    private boolean hasCollision(ASVConfig cfg, Obstacle o) {
        Rectangle2D lenientRect = grow(o.getRect(), -maxError);
        List<Point2D> points = cfg.getASVPositions();
        for (int i = 1; i < points.size(); i++) {
            if (new Line2D.Double(points.get(i - 1), points.get(i))
                    .intersects(lenientRect)) {
                return true;
            }
        }
        return false;
    }
    
    final double broomLength = 0.05;
    final double step = 0.001;
    
	public static void main(String[] args) throws IOException {
		// load problem from a file
	    String fileName = args[0];
	    Main tester = new Main(DEFAULT_MAX_ERROR);
	    tester.ps.loadProblem(fileName);
	    
	    int asvCount = tester.ps.getASVCount();
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
	
	private static double[] cfgToWorkspace(double[] pt) {
		
	}

	/**
	 * retrieve a configuration which is nearest to the sampled configuration
	 * @param allConfig: all found configuration
	 * @param target: the sampled configuration
	 * @return
	 */
	private static Config findNearest(HashSet<Config> allConfig, Config sample) {
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
    private static Config findNext(Config sample, Config near) {
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

