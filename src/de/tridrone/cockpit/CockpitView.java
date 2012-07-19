package de.tridrone.cockpit;

import java.awt.BorderLayout;
import java.awt.Container;
import java.awt.Dimension;
import java.awt.Toolkit;

import javax.swing.JFrame;
import javax.swing.JTextArea;

import de.tridrone.cockpit.CMenuBar;

public class CockpitView extends JFrame {

	CockpitModel model;

	CMenuBar menuBar;
  JTextArea terminal;

	public CockpitView(CockpitModel model) {
		super("TriDrone Cockpit");

		this.model = model;

		this.menuBar = new CMenuBar();
		this.terminal = new JTextArea();

		initFrame();
	}

	private void initFrame() {
		Container container = this.getContentPane();
		container.setLayout(new BorderLayout());
		container.add(this.terminal, BorderLayout.CENTER);

		Dimension dim = Toolkit.getDefaultToolkit().getScreenSize();

		setJMenuBar(this.menuBar);
		setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
		setSize(dim.width - 10 * dim.width / 100,
            dim.height - 10 * dim.height / 100);
		setLocationRelativeTo(null);
		setVisible(true);
	}
}
