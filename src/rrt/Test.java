package rrt;

import java.awt.geom.Line2D;
import java.awt.geom.Point2D;
import java.awt.geom.Rectangle2D;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

public class Test {
    /** The maximum distance any ASV can travel between two states */
    public static final double MAX_STEP = 0.001;
    /** The minimum allowable broom length */
    public static final double MIN_BROOM_LENGTH = 0.05;
    /** The maximum allowable broom length */
    public static final double MAX_BROOM_LENGTH = 0.05;
    /** The workspace bounds */
    public static final Rectangle2D BOUNDS = new Rectangle2D.Double(0, 0, 1, 1);
    /** The default value for maximum error */
    public static final double DEFAULT_MAX_ERROR = 1e-5;

    /**
     * Returns the minimum area required for the given number of ASVs.
     *
     * @param asvCount
     *            the number of ASVs
     * @return the minimum area required.
     */
    public static final double getMinimumArea(int asvCount) {
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
    public static Rectangle2D grow(Rectangle2D rect, double delta) {
        return new Rectangle2D.Double(rect.getX() - delta, rect.getY() - delta,
                rect.getWidth() + delta * 2, rect.getHeight() + delta * 2);
    }

    /** Remembers the specifications of the problem. */
    protected ProblemSpec ps = new ProblemSpec();
    /** The maximum error allowed by this Tester */
    private double maxError;
    /** The workspace bounds, with allowable error. */
    private Rectangle2D lenientBounds;

    /**
     * Constructor. Creates a Tester with the default value for maximum error.
     * @throws IOException 
     */
    public Test(String fileName) throws IOException {
        this(DEFAULT_MAX_ERROR);
        this.ps.loadProblem(fileName);
    }

    /**
     * Constructor. Creates a Tester with the given maximum error.
     *
     * @param maxError
     *            the maximum allowable error.
     */
    public Test(double maxError) {
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
    public List<Integer> addToAll(List<Integer> list, int delta) {
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
    public boolean isValidStep(ASVConfig cfg0, ASVConfig cfg1) {
        return (cfg0.maxDistance(cfg1) <= MAX_STEP + maxError);
    }

    /**
     * Returns whether the brooms in the given configuration have valid lengths.
     *
     * @param cfg
     *            the configuration to test.
     * @return whether the brooms in the given configuration have valid lengths.
     */
    public boolean hasValidbroomLengths(ASVConfig cfg) {
        List<Point2D> points = cfg.getASVPositions();
        for (int i = 1; i < points.size(); i++) {
            Point2D p0 = points.get(i - 1);
            Point2D p1 = points.get(i);
            double broomLength = p0.distance(p1);
            if (broomLength < MIN_BROOM_LENGTH - maxError) {
                return false;
            } else if (broomLength > MAX_BROOM_LENGTH + maxError) {
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
    public double normaliseAngle(double angle) {
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
    public boolean isConvex(ASVConfig cfg) {
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
    public boolean hasEnoughArea(ASVConfig cfg) {
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
    public boolean fitsBounds(ASVConfig cfg) {
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
    public boolean hasCollision(ASVConfig cfg, List<Obstacle> obstacles) {
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
    public boolean hasCollision(ASVConfig cfg, Obstacle o) {
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

    public boolean hasCollision(Point2D p, Obstacle o) {
        
        return false;
    }
    /**
     * Checks that the total cost of the solution is correctly calculated.
     */
    public boolean testTotalCost(int testNo, boolean verbose) {
        System.out.println(String.format("Test #%d: Solution cost", testNo));
        double cost = ps.getSolutionCost();
        double actualCost = ps.calculateTotalCost();
        if (Math.abs(cost - actualCost) > maxError) {
            System.out.println(String.format(
                    "FAILED: Incorrect solution cost; was %f but should be %f",
                    cost, actualCost));
            return false;
        } else {
            System.out.println("Passed.");
            return true;
        }
    }
}
