import java.awt.geom.Point2D;
import java.awt.geom.Rectangle2D;
import java.util.ArrayList;

public class VisualHelperTester {

	/**
	 * @param args
	 */
	public static void main(String[] args) {
		
		// Create list of points
		ArrayList<Point2D> points = new ArrayList<Point2D>();
		points.add(new Point2D.Double(0.3, 0.3));
		points.add(new Point2D.Double(0.5, 0.5));
		ArrayList<Point2D> points2 = new ArrayList<Point2D>();
		points2.add(new Point2D.Double(0.8, 0.5));
		points2.add(new Point2D.Double(0.9, 0.6));
		points2.add(new Point2D.Double(0.9, 0.5));
		points2.add(new Point2D.Double(0.8, 0.4));
		
		// Create list of rectangles
		ArrayList<Rectangle2D> rects = new ArrayList<Rectangle2D>();
		rects.add(new Rectangle2D.Double(0, 0, 0.2, 0.3));
		
		// Test
		VisualHelper visualHelper = new VisualHelper();
		visualHelper.addPoints(points);
		visualHelper.addRectangles(rects);
		visualHelper.repaint();
		
		// Wait for user key press
		visualHelper.waitKey();
		
		// Clear points, then draw linkedPoints
		visualHelper.clearPoints();
		visualHelper.addLinkedPoints(points);
		visualHelper.addLinkedPoints(points2);
		visualHelper.repaint();
	}

}
