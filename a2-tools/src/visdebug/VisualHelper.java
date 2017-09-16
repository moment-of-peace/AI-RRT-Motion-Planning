import javax.swing.JFrame;
import javax.swing.JPanel;

import java.awt.Graphics;
import java.awt.Color;
import java.awt.event.KeyEvent;
import java.awt.event.KeyListener;
import java.awt.geom.Point2D;
import java.awt.geom.Rectangle2D;
import java.awt.geom.Line2D;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;

/**
 * A simple visualiser for debugging.
 * @author Joshua Song
 *
 */
public class VisualHelper extends JPanel implements KeyListener {
	
	private int panelWidth = 500;
	private int panelHeight = 500;

	private List<Point2D> points = new ArrayList<Point2D>();
	private List<Rectangle2D> rectangles = new ArrayList<Rectangle2D>();
	private List<Line2D> lines = new ArrayList<Line2D>();
	private List<ArrayList<Point2D>> linkedPoints =
			new ArrayList<ArrayList<Point2D>>();
	
	private Random random = new Random();
	private JFrame frame;
	
	private char keyPressed = 0;

	/**
	 * Constructor
	 */
	public VisualHelper() {
		frame = new JFrame("COMP3702 A1");
		frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
		frame.setSize(panelWidth, panelHeight);
		frame.add(this);
		frame.addKeyListener(this);
		frame.setVisible(true);
	}
	
	/**
	 * Adds points to draw, coordinates between 0 and 1
	 * @param input List of Point2D
	 */
	public void addPoints(List<Point2D> input) {
		points.addAll(input);
	}
	
	/**
	 * Adds rectangles to draw, coordinates between 0 and 1
	 * @param input List of Rectangle2D
	 */
	public void addRectangles(List<Rectangle2D> input) {
		rectangles.addAll(input);
	}
	
	/**
	 * Adds lines to draw, coordinates between 0 and 1
	 * @param input List of Line2D
	 */
	public void addLines(List<Line2D> input) {
		lines.addAll(input);
	}
	
	/**
	 * Adds a set of lines
	 * @param input List of Point2D, the vertices 
	 */
	public void addLinkedPoints(List<Point2D> input) {
		linkedPoints.add(new ArrayList<Point2D>());
		linkedPoints.get(linkedPoints.size() - 1).addAll(input);
	}
	
	public void clearAll() {
		points.clear();
		rectangles.clear();
		lines.clear();
		linkedPoints.clear();
	}
	
	public void clearPoints() {
		points.clear();
	}
	
	public void clearRectangles() {
		rectangles.clear();
	}
	
	public void clearLines() {
		lines.clear();
	}
	
	public void clearLinkedPoints() {
		linkedPoints.clear();
	}
	
	/**
	 * Call this to refresh the window
	 */
	public void repaint() {
		if (frame != null) {
			frame.repaint();
		}
	}
	
	/**
	 * Waits until user presses a key, then returns the key pressed
	 * @return key pressed as char
	 */
	public char waitKey() {
		keyPressed = 0;
		while (keyPressed == 0) {
			
			// Wait
			try {
				Thread.sleep(25);
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
		}
		return keyPressed;		
	}
	
	/**
	 * Waits until user presses a key, then returns the key pressed
	 * Will return 0 if timeout occurs
	 * @param timeout Max time to wait in milliseconds
	 * @return key pressed as char
	 */
	public char waitKey(long timeout) {
		long currentTime = System.currentTimeMillis();
		long startTime = System.currentTimeMillis();
		keyPressed = 0;
		while (currentTime - startTime < timeout && keyPressed == 0) {
			
			// Wait
			try {
				Thread.sleep(25);
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
			currentTime = System.currentTimeMillis();
		}
		return keyPressed;
	}
	
	/**
	 * Overloaded JPanel method
	 */
	public void paint(Graphics g) {
		
		// Update size in case user has resized window
		panelHeight = this.getHeight();
		panelWidth = this.getWidth();
		
		// White background
		g.setColor(Color.white);
		g.fillRect(0, 0, panelWidth, panelHeight);

		// Draw points
		for (Point2D p : points) {
			int x1 = (int) (p.getX() * panelWidth);
			int y1 = (int) (p.getY() * panelHeight);
			g.setColor(Color.black);
			g.drawRect(x1-2, panelHeight - y1-2, 4, 4);
		}

		// Draw rectangles
		for (Rectangle2D r : rectangles) {
			int x = (int) (r.getX() * panelWidth);
			int y = (int) (r.getY() * panelHeight);
			int width = (int) (r.getWidth() * panelWidth);
			int height = (int) (r.getHeight() * panelHeight);
			g.setColor(Color.red);
			g.fillRect(x, panelHeight - y - height, width, height);
		}
		
		// Draw lines
		for (Line2D l : lines) {
			int x1 = (int) (l.getX1() * panelWidth);
			int y1 = (int) (l.getY1() * panelHeight);
			int x2 = (int) (l.getX2() * panelWidth);
			int y2 = (int) (l.getY2() * panelHeight);
			g.setColor(Color.blue);
			g.drawLine(x1, panelHeight - y1, x2, panelHeight - y2);
		}

		// Draw linked points
		for (int i = 0; i < linkedPoints.size(); i++) {
			
			// Randomise color for each set of linked points
			int color = random.nextInt(5);
			if (color == 0) {
				g.setColor(Color.blue);
			} else if (color == 1) {
				g.setColor(Color.green);
			} else if (color == 2) {
				g.setColor(Color.yellow);
			} else if (color == 3) {
				g.setColor(Color.pink);
			} else if (color == 4) {
				g.setColor(Color.cyan);
			}
			
			for (int j = 0; j < linkedPoints.get(i).size() - 1; j++) {
				Point2D p1 = linkedPoints.get(i).get(j);
				Point2D p2 = linkedPoints.get(i).get(j + 1);
				int x1 = (int) (p1.getX() * panelWidth);
				int y1 = (int) (p1.getY() * panelHeight);
				int x2 = (int) (p2.getX() * panelWidth);
				int y2 = (int) (p2.getY() * panelHeight);	
				g.drawLine(x1, panelHeight - y1, x2, panelHeight - y2);
			}
		}
	}

	@Override
	public void keyTyped(KeyEvent e) {
		// TODO Auto-generated method stub
		
	}

	@Override
	public void keyPressed(KeyEvent e) {
		keyPressed = e.getKeyChar();
	}

	@Override
	public void keyReleased(KeyEvent e) {
		// TODO Auto-generated method stub
		
	}
}