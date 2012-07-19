package de.tridrone.cockpit;

import javax.swing.Box;
import javax.swing.JButton;
import javax.swing.JComboBox;
import javax.swing.JLabel;
import javax.swing.JMenuBar;

public class CMenuBar extends JMenuBar {

	public CMenuBar() {
		String[] baudrates = { "600", "1200", "2400", "4800", "9600", "19200",
				"38400", "57600", "115200", "576000" };

		JLabel baudLabel = new JLabel("Baudrate: ");
		JComboBox baudComboBox = new JComboBox(baudrates);

		JLabel portsLabel = new JLabel("Serial Ports: ");
		JComboBox portsComboBox = new JComboBox();

		JButton connectButton = new JButton("Connect");

		add(Box.createHorizontalStrut(20));
		add(baudLabel);
		add(baudComboBox);
		add(Box.createHorizontalStrut(20));
		add(portsLabel);
		add(portsComboBox);
		add(Box.createHorizontalGlue());
		add(connectButton);
	}
}
