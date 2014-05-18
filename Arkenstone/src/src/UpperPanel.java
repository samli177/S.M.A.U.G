/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package src;

import java.awt.Color;
import java.awt.Font;
import java.awt.Graphics;
import java.util.LinkedList;
import javax.swing.JPanel;

/**
 *
 * @author Martin
 */
public class UpperPanel extends JPanel {

    int MAX_POINTS = 20;

    Sensor vertical;
    Sensor ultraSound;
    int x0 = 100;
    int y0 = 132;

    public UpperPanel() {
        super();
        vertical = new Sensor(MAX_POINTS);
        ultraSound = new Sensor(MAX_POINTS);
    }

    public void setParameters(int s, int m) {
        vertical.setMaxValues(s);
        vertical.setMedianCount(m);
        ultraSound.setMaxValues(s);
        ultraSound.setMedianCount(m);

        MAX_POINTS = s;
    }

    public synchronized void updatePoints(float length, MainWindow.SENSOR sensor) {
        switch (sensor) {
            case VERTICAL:
                vertical.add(length);
                break;
            case ULTRA_SOUND:
                ultraSound.add(length);
                break;
        }
    }

    protected synchronized void paintComponent(Graphics g) {
        super.paintComponent(g);

        g.setColor(Color.red);
        g.fillOval(x0, y0, 9, 9);
        g.setColor(Color.black);

        g.setFont(new Font("ARIAL", Font.BOLD, 16));
        g.drawString("Vertikala Sensorer", 10, 20);

        float a;
        float l;
        for (int i = 0; i < vertical.size(); i++) {
            l = (float) vertical.get(i);
            g.fillOval(x0 + 30 + (int) (l * 1.75 - Math.cos(Math.toRadians(70)) * i * 80 / MAX_POINTS),
                    y0 + 20 + (int) (Math.sin(Math.toRadians(70)) * i * 80 / MAX_POINTS),
                    5, 5);
        }
        for (int i = 0; i < ultraSound.size(); i++) {
            l = (float) ultraSound.get(i);
            g.fillOval(x0 + 30 + (int) (l * 1.75 - Math.cos(Math.toRadians(70)) * i * 80 / MAX_POINTS),
                    y0 - 20 - (int) (Math.sin(Math.toRadians(70)) * i * 80 / MAX_POINTS),
                    5, 5);
        }
    }
}
