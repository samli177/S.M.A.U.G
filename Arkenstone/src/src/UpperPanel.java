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
    
    int MAX_POINTS = 200;
    
    ArrayList<Point2D> points;
    int x0 = 100;
    int y0 = 132;
    
    public UpperPanel(){
        super();
        points = new ArrayList<>();
    }
    
    public void updatePoints(float angle, float length, MainWindow.SENSOR sensor){
        Point2D newPoint = new Point2D.Float(angle, length);
        points.add(newPoint);
        if(points.size() > MAX_POINTS){
            ArrayList<Point2D> temp = new ArrayList<>();
            for(int i = 1; i < points.size(); i++){
                temp.add(points.get(i));
            }
            
            points = (ArrayList<Point2D>) temp.clone();
            temp.clear();
        }
    }

    protected void paintComponent(Graphics g) {
        super.paintComponent(g);
        
        g.setColor(Color.red);
        g.fillOval(x0, y0, 10, 10);
        g.setColor(Color.black);
        
        g.setFont(new Font("ARIAL", Font.BOLD, 16));
        g.drawString("Vertikal Sensor", 10, 20);
        
        float a = 0;
        float l = 0;
        for(int i = 0; i < points.size(); i++){
            a = (float) points.get(i).getX();
            l = (float) points.get(i).getY();
            g.fillOval(x0 + (int) (l * Math.cos(Math.toRadians(a))), y0 + (int) (l * Math.sin(Math.toRadians(a))), 5, 5);
        }
    }
}
