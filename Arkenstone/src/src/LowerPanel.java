/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package src;

import java.awt.Color;
import java.awt.Font;
import java.awt.Graphics;
import java.util.ArrayList;
import javax.swing.JPanel;

/**
 *
 * @author Martin
 */
public class LowerPanel extends JPanel {

    int MAX_POINTS = 20;

    ArrayList<Float> leftFront;
    ArrayList<Float> rightFront;
    ArrayList<Float> leftBack;
    ArrayList<Float> rightBack;
    ArrayList<Float> front;
    ArrayList<Float> back;

    int xL = 326 - 30;
    int yU = 214 - 40;
    int xR = 326 + 30;
    int yD = 214 + 40;

    public LowerPanel() {
        super();
        leftFront = new ArrayList<>();
        rightFront = new ArrayList<>();
        leftBack = new ArrayList<>();
        rightBack = new ArrayList<>();
        front = new ArrayList<>();
        back = new ArrayList<>();
    }

    public synchronized void updatePoints(float length, MainWindow.SENSOR sensor) {
        switch (sensor) {
            case LEFT_FRONT:
                leftFront.add(length);
                if(leftFront.size() > MAX_POINTS){
                    ArrayList<Float> temp = new ArrayList<>();
                    for (int i = 1; i < leftFront.size(); i++) {
                        temp.add(leftFront.get(i));
                    }

                    leftFront = (ArrayList<Float>) temp.clone();
                    temp.clear();
                }
                break;
            case RIGHT_FRONT:
                rightFront.add(length);
                if(rightFront.size() > MAX_POINTS){
                    ArrayList<Float> temp = new ArrayList<>();
                    for (int i = 1; i < rightFront.size(); i++) {
                        temp.add(rightFront.get(i));
                    }

                    rightFront = (ArrayList<Float>) temp.clone();
                    temp.clear();
                }
                break;
            case LEFT_BACK:
                leftBack.add(length);
                if(leftBack.size() > MAX_POINTS){
                    ArrayList<Float> temp = new ArrayList<>();
                    for (int i = 1; i < leftBack.size(); i++) {
                        temp.add(leftBack.get(i));
                    }

                    leftBack = (ArrayList<Float>) temp.clone();
                    temp.clear();
                }
                break;
            case RIGHT_BACK:
                rightBack.add(length);
                if(rightBack.size() > MAX_POINTS){
                    ArrayList<Float> temp = new ArrayList<>();
                    for (int i = 1; i < rightBack.size(); i++) {
                        temp.add(rightBack.get(i));
                    }

                    rightBack = (ArrayList<Float>) temp.clone();
                    temp.clear();
                }
                break;
            case FRONT:
                front.add(length);
                if(front.size() > MAX_POINTS){
                    ArrayList<Float> temp = new ArrayList<>();
                    for (int i = 1; i < front.size(); i++) {
                        temp.add(front.get(i));
                    }

                    front = (ArrayList<Float>) temp.clone();
                    temp.clear();
                }
                break;
            case BACK:
                back.add(length);
                if(back.size() > MAX_POINTS){
                    ArrayList<Float> temp = new ArrayList<>();
                    for (int i = 1; i < back.size(); i++) {
                        temp.add(back.get(i));
                    }

                    back = (ArrayList<Float>) temp.clone();
                    temp.clear();
                }
                break;
        }

    }

    protected synchronized void paintComponent(Graphics g) {
        super.paintComponent(g);

        g.setColor(Color.red);
        g.fillOval(xL, yU, 10, 10);
        g.fillOval(xL, yD, 10, 10);
        g.fillOval(xR, yU, 10, 10);
        g.fillOval(xR, yD, 10, 10);
        g.fillOval((int) ((xR + xL)/2.0), yU, 10, 10);
        g.setColor(Color.black);

        g.setFont(new Font("ARIAL", Font.BOLD, 16));
        g.drawString("Horisontella Sensorer", 10, 20);

        float l;
        for (int i = 0; i < leftFront.size(); i++) {
            l = (float) leftFront.get(leftFront.size() - 1 - i);
            g.fillOval(xL - (int) l * 2 - 10, yU + i * 7 - 80, 5, 5);
        }
        for (int i = 0; i < rightFront.size(); i++) {
            l = (float) rightFront.get(rightFront.size() - 1 - i);
            g.fillOval(xR + (int) l * 2 + 10, yU + i * 7 - 80, 5, 5);
        }
        for (int i = 0; i < leftBack.size(); i++) {
            l = (float) leftBack.get(leftBack.size() - 1 - i);
            g.fillOval(xL - (int) l * 2 - 10, yD + i * 7, 5, 5);
        }
        for (int i = 0; i < rightBack.size(); i++) {
            l = (float) rightBack.get(rightBack.size() - 1 - i);
            g.fillOval(xR + (int) l * 2 + 10, yD + i * 7, 5, 5);
        }
        for (int i = 0; i < front.size(); i++) {
            l = (float) front.get(front.size() - 1 - i);
            g.fillOval((int) ((xR + xL)/2.0) + (int) (MAX_POINTS * 3.5) - i * 7, yU - (int) l, 5, 5);
        }
        
        if(hasFocus()){
            g.drawString("C", getWidth() - 20, 20);
        }
    }
}
