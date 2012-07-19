package de.tridrone.cockpit;

import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;

public class CockpitController implements ActionListener {

	CockpitModel model;
	CockpitView view;

	public CockpitController(CockpitModel model, CockpitView view) {
		this.model = model;
		this.view = view;
	}

	@Override
	public void actionPerformed(ActionEvent e) {
		// TODO Auto-generated method stub

	}
}
