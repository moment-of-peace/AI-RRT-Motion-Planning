package visualiser;

import java.awt.BorderLayout;
import java.awt.Component;
import java.awt.Container;
import java.awt.Dimension;
import java.awt.FlowLayout;
import java.awt.Font;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.event.ComponentEvent;
import java.awt.event.ComponentListener;
import java.awt.event.KeyEvent;
import java.awt.event.MouseEvent;
import java.awt.event.MouseListener;
import java.awt.event.WindowAdapter;
import java.awt.event.WindowEvent;
import java.io.File;
import java.io.IOException;

import javax.swing.BorderFactory;
import javax.swing.Box;
import javax.swing.BoxLayout;
import javax.swing.ImageIcon;
import javax.swing.JApplet;
import javax.swing.JButton;
import javax.swing.JFileChooser;
import javax.swing.JFrame;
import javax.swing.JLabel;
import javax.swing.JMenu;
import javax.swing.JMenuBar;
import javax.swing.JMenuItem;
import javax.swing.JOptionPane;
import javax.swing.JPanel;
import javax.swing.JSeparator;
import javax.swing.JSlider;
import javax.swing.JSpinner;
import javax.swing.SpinnerNumberModel;
import javax.swing.border.EtchedBorder;
import javax.swing.event.ChangeEvent;
import javax.swing.event.ChangeListener;

public class Visualiser {
	private Container container;

	private VisualisationPanel vp;

	private JPanel infoPanel;
	private JLabel infoLabel;

	private JMenuBar menuBar;
	private JMenu fileMenu;
	private JMenuItem loadProblemItem, loadSolutionItem, exitItem; // assumeDirectSolutionItem
	private JMenu animationMenu;
	private JMenuItem initialiseItem, playPauseItem, stopItem;
	private JMenu displayMenu;
	private JMenuItem problemItem, solutionItem;

	private JPanel animationControls;
	private JSlider manualSlider;
	private JSlider framerateSlider;

	private JSpinner samplingSpinner;

	protected ImageIcon createImageIcon(String path, String description) {
		java.net.URL imgURL = getClass().getResource(path);
		if (imgURL != null) {
			return new ImageIcon(imgURL, description);
		} else {
			return new ImageIcon(path, description);
		}
	}

	private JButton playPauseButton, stopButton;
	private ImageIcon playIcon = createImageIcon("play.gif", "Play");
	private ImageIcon pauseIcon = createImageIcon("pause.gif", "Pause");
	private ImageIcon stopIcon = createImageIcon("stop.gif", "Stop");

	private boolean animating;
	private boolean wasPlaying = false;
	private boolean playing;
	private boolean hasProblem;
	private boolean hasSolution;

	private static final int FRAMERATE_MIN = 1;
	private static final int FRAMERATE_MAX = 200;
	private static final int FRAMERATE_INIT = 50;

	private static final int SAMPLING_PERIOD_INIT = 100;

	private File defaultPath;

	private class MenuListener implements ActionListener {
		public void actionPerformed(ActionEvent e) {
			String cmd = e.getActionCommand();
			if (cmd.equals("Problem")) {
				setAnimating(false);
				vp.setDisplayingSolution(false);
				setInfoText();
				vp.repaint();
			} else if (cmd.equals("Solution")) {
				setAnimating(false);
				vp.setDisplayingSolution(true);
				setInfoText();
				vp.repaint();
			} else if (cmd.equals("Load problem")) {
				setAnimating(false);
				loadProblem();
			} else if (cmd.equals("Load solution")) {
				setAnimating(false);
				loadSolution();
			} else if (cmd.equals("Exit")) {
				container.setVisible(false);
				System.exit(0);
			} else if (cmd.equals("Initialise")) {
				setAnimating(true);
			} else if (cmd.equals("Play")) {
				playPause();
			} else if (cmd.equals("Pause")) {
				playPause();
			} else if (cmd.equals("Stop")) {
				setAnimating(false);
			}
		}
	}

	private class ResizeListener implements ComponentListener {
		@Override
		public void componentResized(ComponentEvent e) {
			updateTickSpacing();
		}

		@Override
		public void componentHidden(ComponentEvent e) {
		}

		@Override
		public void componentMoved(ComponentEvent e) {
		}

		@Override
		public void componentShown(ComponentEvent e) {
		}
	}

	private ResizeListener resizeListener = new ResizeListener();
	private MenuListener menuListener = new MenuListener();

	private ChangeListener manualSliderListener = new ChangeListener() {
		@Override
		public void stateChanged(ChangeEvent e) {
			if (!manualSlider.getValueIsAdjusting() && wasPlaying) {
				wasPlaying = false;
				if (manualSlider.getValue() < manualSlider.getMaximum()) {
					vp.playPauseAnimation();
				}
			}
			vp.gotoFrame(manualSlider.getValue());
		}
	};

	private MouseListener manualSliderClickListener = new MouseListener() {
		@Override
		public void mousePressed(MouseEvent e) {
			if (playing) {
				wasPlaying = true;
				vp.playPauseAnimation();
			}
		}

		@Override
		public void mouseReleased(MouseEvent e) {
		}

		@Override
		public void mouseClicked(MouseEvent e) {
		}

		@Override
		public void mouseEntered(MouseEvent e) {
		}

		@Override
		public void mouseExited(MouseEvent e) {
		}
	};

	private ChangeListener framerateListener = new ChangeListener() {
		@Override
		public void stateChanged(ChangeEvent e) {
			vp.setFramerate(framerateSlider.getValue());
		}
	};

	private ChangeListener samplingListener = new ChangeListener() {
		@Override
		public void stateChanged(ChangeEvent e) {
			vp.setSamplingPeriod((Integer) samplingSpinner.getValue());
		}
	};

	private ActionListener playPauseListener = new ActionListener() {
		@Override
		public void actionPerformed(ActionEvent arg0) {
			playPause();
		}
	};

	private ActionListener stopListener = new ActionListener() {
		@Override
		public void actionPerformed(ActionEvent arg0) {
			setAnimating(false);
		}
	};

	public Visualiser(Container container, File defaultPath) {
		this.container = container;
		this.defaultPath = defaultPath;
		createComponents();
	}

	public Visualiser(Container container) {
		this.container = container;
		try {
			this.defaultPath = new File(".").getCanonicalFile();
		} catch (IOException e) {
			this.defaultPath = null;
		}
		createComponents();
	}

	private void createComponents() {
		vp = new VisualisationPanel(this);
		JPanel wp = new JPanel(new BorderLayout());
		wp.add(vp, BorderLayout.CENTER);
		container.setLayout(new BorderLayout());
		wp.setBorder(BorderFactory.createCompoundBorder(
				BorderFactory.createEmptyBorder(5, 10, 10, 10),
				BorderFactory.createEtchedBorder(EtchedBorder.LOWERED)));
		container.add(wp, BorderLayout.CENTER);

		infoPanel = new JPanel();
		infoPanel.setLayout(new FlowLayout());

		infoLabel = new JLabel("No problem to display.");
		samplingSpinner = new JSpinner(new SpinnerNumberModel(
				SAMPLING_PERIOD_INIT, 1, null, 1));
		samplingSpinner.addChangeListener(samplingListener);
		samplingSpinner.setPreferredSize(new Dimension(50, 20));
		samplingSpinner.setVisible(false);
		vp.setSamplingPeriod(SAMPLING_PERIOD_INIT);
		infoPanel.add(infoLabel);
		infoPanel.add(samplingSpinner);

		container.add(infoPanel, BorderLayout.NORTH);

		createMenus();
		createAnimationControls();
	}

	private void createMenus() {
		menuBar = new JMenuBar();
		createFileMenu();
		createDisplayMenu();
		createAnimationMenu();
		if (container instanceof JFrame) {
			((JFrame) container).setJMenuBar(menuBar);
		} else if (container instanceof JApplet) {
			((JApplet) container).setJMenuBar(menuBar);
		}
	}

	private void createFileMenu() {
		fileMenu = new JMenu("File");
		fileMenu.setMnemonic(KeyEvent.VK_F);
		fileMenu.getAccessibleContext().setAccessibleDescription(
				"Load configs or close the app.");
		menuBar.add(fileMenu);

		loadProblemItem = new JMenuItem("Load problem");
		loadProblemItem.setMnemonic(KeyEvent.VK_P);
		loadProblemItem.addActionListener(menuListener);
		fileMenu.add(loadProblemItem);

		loadSolutionItem = new JMenuItem("Load solution");
		loadSolutionItem.setMnemonic(KeyEvent.VK_S);
		loadSolutionItem.addActionListener(menuListener);
		loadSolutionItem.setEnabled(false);
		fileMenu.add(loadSolutionItem);

		fileMenu.addSeparator();
		exitItem = new JMenuItem("Exit");
		exitItem.setMnemonic(KeyEvent.VK_X);
		exitItem.addActionListener(menuListener);
		fileMenu.add(exitItem);
	}

	private void createDisplayMenu() {
		displayMenu = new JMenu("Display");
		displayMenu.setMnemonic(KeyEvent.VK_D);
		fileMenu.getAccessibleContext().setAccessibleDescription(
				"Display the problem and solution.");
		menuBar.add(displayMenu);

		problemItem = new JMenuItem("Problem");
		problemItem.setMnemonic(KeyEvent.VK_P);
		problemItem.addActionListener(menuListener);
		problemItem.setEnabled(false);
		displayMenu.add(problemItem);

		solutionItem = new JMenuItem("Solution");
		solutionItem.setMnemonic(KeyEvent.VK_S);
		solutionItem.addActionListener(menuListener);
		solutionItem.setEnabled(false);
		displayMenu.add(solutionItem);
	}

	private void createAnimationMenu() {
		animationMenu = new JMenu("Animation");
		animationMenu.setMnemonic(KeyEvent.VK_A);
		fileMenu.getAccessibleContext().setAccessibleDescription(
				"Manage the animation.");
		menuBar.add(animationMenu);
		animationMenu.setEnabled(false);

		initialiseItem = new JMenuItem("Initialise");
		initialiseItem.setMnemonic(KeyEvent.VK_I);
		initialiseItem.addActionListener(menuListener);
		animationMenu.add(initialiseItem);

		playPauseItem = new JMenuItem("Play");
		playPauseItem.setMnemonic(KeyEvent.VK_P);
		playPauseItem.addActionListener(menuListener);
		animationMenu.add(playPauseItem);

		stopItem = new JMenuItem("Stop");
		stopItem.setMnemonic(KeyEvent.VK_T);
		stopItem.addActionListener(menuListener);
		stopItem.setEnabled(false);
		animationMenu.add(stopItem);
	}

	private void createAnimationControls() {
		Font sliderFont = new Font("Arial", Font.PLAIN, 12);

		animationControls = new JPanel();
		animationControls.setLayout(new BoxLayout(animationControls,
				BoxLayout.PAGE_AXIS));

		JLabel manualLabel = new JLabel("Frame #");
		manualLabel.setAlignmentX(Component.CENTER_ALIGNMENT);
		manualSlider = new JSlider(JSlider.HORIZONTAL);
		manualSlider.setPaintTicks(true);
		manualSlider.setPaintLabels(true);
		manualSlider.setFont(sliderFont);
		manualSlider.addChangeListener(manualSliderListener);
		manualSlider.addMouseListener(manualSliderClickListener);
		manualSlider.setMinorTickSpacing(1);
		manualSlider.addComponentListener(resizeListener);

		JLabel framerateLabel = new JLabel("Framerate");
		framerateLabel.setAlignmentX(Component.CENTER_ALIGNMENT);
		framerateSlider = new JSlider(JSlider.HORIZONTAL, FRAMERATE_MIN,
				FRAMERATE_MAX, FRAMERATE_INIT);
		framerateSlider.setMajorTickSpacing(10);
		framerateSlider.setMinorTickSpacing(1);
		framerateSlider.setPaintTicks(true);
		framerateSlider.setPaintLabels(true);
		framerateSlider.setLabelTable(framerateSlider.createStandardLabels(10,
				10));
		framerateSlider.setFont(sliderFont);
		framerateSlider.addChangeListener(framerateListener);
		JPanel frameratePanel = new JPanel();
		frameratePanel.setLayout(new BoxLayout(frameratePanel,
				BoxLayout.PAGE_AXIS));
		frameratePanel.add(framerateLabel);
		frameratePanel.add(Box.createRigidArea(new Dimension(0, 2)));
		frameratePanel.add(framerateSlider);

		playPauseButton = new JButton(playIcon);
		playPauseButton.addActionListener(playPauseListener);
		stopButton = new JButton(stopIcon);
		stopButton.addActionListener(stopListener);

		animationControls.add(new JSeparator(JSeparator.HORIZONTAL));
		animationControls.add(Box.createRigidArea(new Dimension(0, 2)));
		animationControls.add(manualLabel);
		animationControls.add(Box.createRigidArea(new Dimension(0, 2)));
		animationControls.add(manualSlider);
		animationControls.add(Box.createRigidArea(new Dimension(0, 5)));
		JPanel p2 = new JPanel();
		p2.setLayout(new BoxLayout(p2, BoxLayout.LINE_AXIS));
		p2.add(playPauseButton);
		p2.add(Box.createRigidArea(new Dimension(10, 0)));
		p2.add(stopButton);
		p2.add(frameratePanel);
		animationControls.add(p2);
		animationControls.setVisible(false);
		animationControls.setBorder(BorderFactory.createEmptyBorder(0, 10, 5,
				10));
		container.add(animationControls, BorderLayout.SOUTH);
	}

	private File askForFile() {
		JFileChooser fc = new JFileChooser(defaultPath);
		int returnVal = fc.showOpenDialog(container);
		if (returnVal != JFileChooser.APPROVE_OPTION) {
			return null;
		}
		return fc.getSelectedFile();
	}

	private void showFileError(File f) {
		JOptionPane.showMessageDialog(container,
				"Error loading " + f.getName(), "File I/O Error",
				JOptionPane.ERROR_MESSAGE);
	}

	private void loadProblem(File f) {
		try {
			vp.getProblemSetup().loadProblem(f.getPath());
			setHasProblem(true);
		} catch (IOException e1) {
			showFileError(f);
			setHasProblem(false);
		}
	}

	private void loadProblem() {
		File f = askForFile();
		if (f == null) {
			return;
		}
		loadProblem(f);
	}

	private void loadSolution(File f) {
		try {
			vp.getProblemSetup().loadSolution(f.getPath());
			setHasSolution(true);
		} catch (IOException e1) {
			showFileError(f);
			setHasSolution(false);
		}
	}

	private void loadSolution() {
		File f = askForFile();
		if (f == null) {
			return;
		}
		loadSolution(f);
	}

	private void playPause() {
		if (!animating) {
			setAnimating(true);
		}
		vp.playPauseAnimation();
	}

	private void setInfoText() {
		if (!hasProblem) {
			infoLabel.setText("No problem to display.");
			samplingSpinner.setVisible(false);
		} else if (animating) {
			infoLabel
					.setText("Play the animation, or use the slider to control it manually.");
			samplingSpinner.setVisible(false);
		} else if (vp.isDisplayingSolution()) {
			infoLabel.setText("Displaying the solution; sampling period:");
			samplingSpinner.setVisible(true);
		} else {
			infoLabel
					.setText("Displaying the problem: blue = initial, green = goal, red = obstacle. ASV-1 is circled.");
			samplingSpinner.setVisible(false);
		}
	}

	private void setHasProblem(boolean hasProblem) {
		this.hasProblem = hasProblem;
		loadSolutionItem.setEnabled(hasProblem);
		problemItem.setEnabled(hasProblem);
		setHasSolution(false);
		setInfoText();
		vp.repaint();
	}

	public boolean hasProblem() {
		return hasProblem;
	}

	private void setHasSolution(boolean hasSolution) {
		this.hasSolution = hasSolution;
		solutionItem.setEnabled(hasSolution);
		animationMenu.setEnabled(hasSolution);
		vp.setDisplayingSolution(hasSolution);
		setAnimating(hasSolution);
		setInfoText();
		vp.repaint();
	}

	public boolean hasSolution() {
		return hasSolution;
	}

	private void setAnimating(boolean animating) {
		if (animating) {
			vp.initAnimation();
		} else {
			vp.stopAnimation();
		}
		if (this.animating == animating) {
			return;
		}
		this.animating = animating;
		stopItem.setEnabled(animating);
		animationControls.setVisible(animating);
		container.validate();
		vp.calculateTransform();
		vp.repaint();
		setInfoText();
	}

	public void setPlaying(boolean playing) {
		if (this.playing == playing) {
			return;
		}
		this.playing = playing;
		if (playing) {
			playPauseItem.setText("Pause");
			playPauseButton.setIcon(pauseIcon);
		} else {
			playPauseItem.setText("Play");
			playPauseButton.setIcon(playIcon);
		}
		playPauseButton.repaint();
	}

	public void updateMaximum() {
		int maximum = vp.getProblemSetup().getPath().size() - 1;
		manualSlider.setMaximum(maximum);
		updateTickSpacing();
	}

	public void updateSliderSpacing(JSlider slider) {
		int width = slider.getBounds().width;
		int max = slider.getMaximum();
		int spacing = 1;
		int mode = 1;
		double pxPerLabel = (double) width * spacing / max;
		if (pxPerLabel <= 0) {
			return;
		}
		while (pxPerLabel <= 30) {
			if (mode == 1) {
				spacing *= 2;
				pxPerLabel *= 2;
				mode = 2;
			} else if (mode == 2) {
				spacing = spacing * 5 / 2;
				pxPerLabel *= 2.5;
				mode = 5;
			} else {
				spacing *= 2;
				pxPerLabel *= 2;
				mode = 1;
			}
		}
		slider.setMajorTickSpacing(spacing);
		int min = slider.getMinimum();
		if (min % spacing > 0) {
			min += (spacing - (min % spacing));
		}
		slider.setLabelTable(slider.createStandardLabels(spacing, min));
	}

	public void updateTickSpacing() {
		updateSliderSpacing(manualSlider);
		updateSliderSpacing(framerateSlider);
	}

	public void setFrameNumber(int frameNumber) {
		manualSlider.setValue(frameNumber);
	}

	public static void main(String[] args) {
		JFrame frame = new JFrame("Assignment 1 visualiser");
		Visualiser vis = new Visualiser(frame);
		if (args.length > 0) {
			vis.loadProblem(new File(args[0]));
			if (vis.hasProblem() && args.length >= 2) {
				vis.loadSolution(new File(args[1]));
			}
		}
		frame.setSize(700, 766);
		frame.setLocation(300, 100);
		frame.addWindowListener(new WindowAdapter() {
			public void windowClosing(WindowEvent e) {
				System.exit(0);
			}
		});
		frame.setVisible(true);
	}
}
