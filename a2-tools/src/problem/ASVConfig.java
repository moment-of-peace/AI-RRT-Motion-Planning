package problem;

import java.util.ArrayList;
import java.util.InputMismatchException;
import java.util.List;
import java.util.Scanner;
import java.awt.geom.Point2D;

/**
 * Represents a configuration of the ASVs. This class doesn't do any validity
 * checking - see the code in tester.Tester for this.
 *
 * @author lackofcheese
 */
public class ASVConfig {
	/** The position of each ASV */
	private List<Point2D> asvPositions = new ArrayList<Point2D>();

	/**
	 * Constructor. Takes an array of 2n x and y coordinates, where n is the
	 * number of ASVs
	 *
	 * @param coords
	 *            the x- and y-coordinates of the ASVs.
	 */
	public ASVConfig(double[] coords) {
		for (int i = 0; i < coords.length / 2; i++) {
			asvPositions.add(new Point2D.Double(coords[i * 2],
					coords[i * 2 + 1]));
		}
	}

	/**
	 * Constructs an ASVConfig from a space-separated string of x- and y-
	 * coordinates
	 *
	 * @param asvCount
	 *            the number of ASVs to read.
	 * @param str
	 *            the String containing the coordinates.
	 */
	public ASVConfig(int asvCount, String str) throws InputMismatchException {
		Scanner s = new Scanner(str);
		for (int i = 0; i < asvCount; i++) {
			asvPositions
					.add(new Point2D.Double(s.nextDouble(), s.nextDouble()));
		}
		s.close();
	}

	/**
	 * Copy constructor.
	 *
	 * @param cfg
	 *            the configuration to copy.
	 */
	public ASVConfig(ASVConfig cfg) {
		asvPositions = cfg.getASVPositions();
	}

	/**
	 * Returns a space-separated string of the ASV coordinates.
	 *
	 * @return a space-separated string of the ASV coordinates.
	 */
	public String toString() {
		StringBuilder sb = new StringBuilder();
		for (Point2D point : asvPositions) {
			if (sb.length() > 0) {
				sb.append(" ");
			}
			sb.append(point.getX());
			sb.append(" ");
			sb.append(point.getY());
		}
		return sb.toString();
	}

	/**
	 * Returns the maximum straight-line distance between the ASVs in this state
	 * vs. the other state, or -1 if the ASV counts don't match.
	 *
	 * @param otherState
	 *            the other state to compare.
	 * @return the maximum straight-line distance for any ASV.
	 */
	public double maxDistance(ASVConfig otherState) {
		if (this.getASVCount() != otherState.getASVCount()) {
			return -1;
		}
		double maxDistance = 0;
		for (int i = 0; i < this.getASVCount(); i++) {
			double distance = this.getPosition(i).distance(
					otherState.getPosition(i));
			if (distance > maxDistance) {
				maxDistance = distance;
			}
		}
		return maxDistance;
	}

	/**
	 * Returns the total straight-line distance over all the ASVs between this
	 * state and the other state, or -1 if the ASV counts don't match.
	 *
	 * @param otherState
	 *            the other state to compare.
	 * @return the total straight-line distance over all ASVs.
	 */
	public double totalDistance(ASVConfig otherState) {
		if (this.getASVCount() != otherState.getASVCount()) {
			return -1;
		}
		double totalDistance = 0;
		for (int i = 0; i < this.getASVCount(); i++) {
			totalDistance += this.getPosition(i).distance(
					otherState.getPosition(i));
		}
		return totalDistance;
	}

	/**
	 * Returns the position of the ASV with the given number.
	 *
	 * @param asvNo
	 *            the number of the ASV.
	 * @return the position of the ASV with the given number.
	 */
	public Point2D getPosition(int asvNo) {
		return asvPositions.get(asvNo);
	}

	/**
	 * Returns the number of ASVs in this configuration.
	 *
	 * @return the number of ASVs in this configuration.
	 */
	public int getASVCount() {
		return asvPositions.size();
	}

	/**
	 * Returns the positions of all the ASVs, in order.
	 *
	 * @return the positions of all the ASVs, in order.
	 */
	public List<Point2D> getASVPositions() {
		return new ArrayList<Point2D>(asvPositions);
	}
}
