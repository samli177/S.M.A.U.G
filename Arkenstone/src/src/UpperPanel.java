/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package src;

import java.awt.Color;
import java.awt.Font;
import java.awt.Graphics;
import java.awt.geom.Point2D;
import java.util.ArrayList;
import javax.swing.JPanel;

/**
 *
 * @author Martin
 */
public class UpperPanel extends JPanel {

    int MAX_POINTS = 20;

    ArrayList<Point2D> vertical;
    ArrayList<Float> ultraSound;
    int x0 = 100;
    int y0 = 132;

    public UpperPanel() {
        super();
        vertical = new ArrayList<>();
        ultraSound = new ArrayList<>();
    }

    public void updatePoints(float length, float angle, MainWindow.SENSOR sensor) {
        if (sensor == MainWindow.SENSOR.VERTICAL) {
            Point2D newPoint = new Point2D.Float(angle, length);
            vertical.add(newPoint);
            if (vertical.size() > MAX_POINTS) {
                ArrayList<Point2D> temp = new ArrayList<>();
                for (int i = 1; i < vertical.size(); i++) {
                    temp.add(vertical.get(i));
                }

                vertical = (ArrayList<Point2D>) temp.clone();
                temp.clear();
            }
        } else if (sensor == MainWindow.SENSOR.ULTRA_SOUND) {
            ultraSound.add(length);
                if(ultraSound.size() > MAX_POINTS){
                    ArrayList<Float> temp = new ArrayList<>();
                    for (int i = 1; i < ultraSound.size(); i++) {
                        temp.add(ultraSound.get(i));
                    }

                    ultraSound = (ArrayList<Float>) temp.clone();
                    temp.clear();
                }
        }
    }

    protected void paintComponent(Graphics g) {
        super.paintComponent(g);

        g.setColor(Color.red);
        g.fillOval(x0, y0, 10, 10);
        g.setColor(Color.black);

        g.setFont(new Font("ARIAL", Font.BOLD, 16));
        g.drawString("Vertikal Sensor", 10, 20);

        float a;
        float l;
        for (Point2D pnt : vertical) {
            a = (float) pnt.getX();
            l = (float) pnt.getY();
            g.fillOval(x0 + (int) (l * Math.cos(Math.toRadians(a))), y0 + (int) (l * Math.sin(Math.toRadians(a))), 5, 5);
        }
        for(int i = 0; i < ultraSound.size(); i++){
            l = (float) ultraSound.get(ultraSound.size() - 1 - i);
            g.fillOval(x0 + (int)(l - Math.cos(Math.toRadians(70))* i * 3), y0 - 40 - (int) (Math.sin(Math.toRadians(70))* i * 3), 5, 5);
        }
    }
}
