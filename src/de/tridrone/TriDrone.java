package de.tridrone;

import de.tridrone.cockpit.*;

public class TriDrone {

	public static void main(String[] args) {
		CockpitModel model = new CockpitModel();
		CockpitView view = new CockpitView(model);
		new CockpitController(model, view);
	}
}
