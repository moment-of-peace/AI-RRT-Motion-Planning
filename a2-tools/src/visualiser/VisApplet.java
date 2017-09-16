package visualiser;

import java.io.File;

import javax.swing.JApplet;

public class VisApplet extends JApplet {
	/** UID, as required by Swing */
	private static final long serialVersionUID = 8479710856496756661L;

	private static final String DEFAULT_PATH = "C:\\Users\\lackofcheese\\Google Drive\\Coding\\Java\\comp3702-a1-tools";

	public void init() {
		try {
			javax.swing.SwingUtilities.invokeAndWait(new Runnable() {
				public void run() {
					new Visualiser(VisApplet.this, new File(DEFAULT_PATH));
				}
			});
		} catch (Exception e) {
			System.err.println("Could not create the visualizer.");
		}
		this.setSize(800, 600);
	}
}