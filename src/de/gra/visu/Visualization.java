package de.gra.visu;

import java.awt.Color;
import java.awt.Graphics;
import java.util.ArrayList;

import javax.swing.JFrame;
import javax.swing.JPanel;

import org.orekit.errors.OrekitException;

import de.gra.propagation.PropagationErrorExtraction;
import de.gra.propagation.SpacePosition;

public class Visualization extends JFrame {

	public Visualization() {
		setVisible(true);
		setDefaultCloseOperation(EXIT_ON_CLOSE);
		setSize(800, 600);
		JPanel jpanel = new JPanel() {

			public void paintComponent(Graphics g) {
				super.paintComponent(g);
				g.drawRect(200, 200, 200, 200);
				PropagationErrorExtraction pex = new PropagationErrorExtraction();
				ArrayList<SpacePosition> pos = null;
				try {
					pos = pex.propagateSGP4();
				} catch (OrekitException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}

				double min_X = 0;
				double min_Y = 0;
				double max_X = 0;
				double max_Y = 0;

				for (int i = 0; i < pos.size(); i++) {
					SpacePosition p = pos.get(i);

					if (p.getX() > max_X)
						max_X = p.getX();
					if (p.getY() > max_Y)
						max_Y = p.getY();

				}

				double scaleX = getWidth() / max_X;
				double scaleY = getHeight() / max_Y;

				System.out.println(max_X + " SCALE X = " + scaleX);
				System.out.println(max_Y + " SCALE Y = " + scaleY);

				pos.forEach(p -> {
					int scaldedX = (int) (p.getX() * scaleX);
					System.out.println(scaldedX);
					g.drawOval((int) (p.getX() * scaleX), (int) (p.getY() * scaleY), 1, 1);

				});

			}
		};

		jpanel.setBackground(new Color(255, 255, 255));
		this.add(jpanel);
		repaint();

	}

	public static void main(String[] args) {
		new Visualization();
	}

}
