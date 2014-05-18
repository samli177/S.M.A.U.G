/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package src;

import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.Font;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.Stroke;
import java.util.LinkedList;
import javax.swing.JPanel;

/**
 *
 * @author Martin
 */
public class LowerPanel extends JPanel {

    int MAX_POINTS = 20;

    Sensor leftFront;
    Sensor rightFront;
    Sensor leftBack;
    Sensor rightBack;
    Sensor front;
    Sensor back;

    int xL = 310 - 30;
    int yU = 195 - 50;
    int xR = 310 + 30;
    int yD = 195 + 30;

    public LowerPanel() {
        super();
        leftFront = new Sensor(MAX_POINTS);
        rightFront = new Sensor(MAX_POINTS);
        leftBack = new Sensor(MAX_POINTS);
        rightBack = new Sensor(MAX_POINTS);
        front = new Sensor(MAX_POINTS);
        back = new Sensor(MAX_POINTS);
    }

    public void setParameters(int s, int m){
        leftFront.setMaxValues(s);
        leftFront.setMedianCount(m);
        rightFront.setMaxValues(s);
        rightFront.setMedianCount(m);
        leftBack.setMaxValues(s);
        leftBack.setMedianCount(m);
        rightBack.setMaxValues(s);
        rightBack.setMedianCount(m);
        front.setMaxValues(s);
        front.setMedianCount(m);
        back.setMaxValues(s);
        back.setMedianCount(m);
        
        MAX_POINTS = s;
    }
    
    public synchronized void updatePoints(float length, MainWindow.SENSOR sensor) {
        switch (sensor) {
            case LEFT_FRONT:
                leftFront.add(length);
                break;
            case RIGHT_FRONT:
                rightFront.add(length);
                break;
            case LEFT_BACK:
                leftBack.add(length);
                break;
            case RIGHT_BACK:
                rightBack.add(length);
                break;
            case FRONT:
                front.add(length);
                break;
            case BACK:
                back.add(length);
                break;
        }

    }

    protected synchronized void paintComponent(Graphics g) {
        super.paintComponent(g);
        
        g.setColor(Color.red);
        g.fillOval(xL, yU, 9, 9);
        g.fillOval(xL, yD, 9, 9);
        g.fillOval(xR, yU, 9, 9);
        g.fillOval(xR, yD, 9, 9);
        g.fillOval((int) ((xR + xL)/2.0), yU, 9, 9);
        g.fillOval((int) ((xR + xL)/2.0), yD, 9, 9);
        g.setColor(Color.black);
        
        g.setFont(new Font("ARIAL", Font.BOLD, 16));
        g.drawString("Horisontella Sensorer", 10, 20);
        
        Graphics2D g2 = (Graphics2D) g;
        g2.setStroke(new BasicStroke(3));
        g2.drawLine((xL + xR)/2 + 4, yU + 24, (xL + xR)/2 + 4, yD - 16);
        g2.drawLine((xL + xR)/2 + 4, yU + 24, (xL + xR)/2 - 10, yU + 38);
        g2.drawLine((xL + xR)/2 + 4, yU + 24, (xL + xR)/2 + 18, yU + 38);
        g2.setStroke(new BasicStroke(1));
        
        float l;
        for (int i = 0; i < leftFront.size(); i++) {
            l = (float) leftFront.get(i);
            g.fillOval(xL - (int) l * 2 - 10, yU + i * 140 / MAX_POINTS - 80, 5, 5);
        }
        for (int i = 0; i < rightFront.size(); i++) {
            l = (float) rightFront.get(i);
            g.fillOval(xR + (int) l * 2 + 10, yU + i * 140 / MAX_POINTS - 80, 5, 5);
        }
        for (int i = 0; i < leftBack.size(); i++) {
            l = (float) leftBack.get(i);
            g.fillOval(xL - (int) l * 2 - 10, yD + i * 140 / MAX_POINTS, 5, 5);
        }
        for (int i = 0; i < rightBack.size(); i++) {
            l = (float) rightBack.get(i);
            g.fillOval(xR + (int) l * 2 + 10, yD + i * 140 / MAX_POINTS, 5, 5);
        }
        for (int i = 0; i < front.size(); i++) {
            l = (float) front.get(i);
            g.fillOval((int) ((xR + xL)/2.0) + 70 - i * 140 / MAX_POINTS, yU - 10 - (int) l, 5, 5);
        }
        for (int i = 0; i < back.size(); i++) {
            l = (float) back.get(i);
            g.fillOval((int) ((xR + xL)/2.0) + 70 - i * 140 / MAX_POINTS, yD + 15 + (int) l, 5, 5);
        }
        
        if(hasFocus()){
            g.drawString("C", getWidth() - 20, 20);
        }
    }
}
