/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package src;

import java.util.ArrayList;
import java.util.Collections;
import java.util.LinkedList;

/**
 *
 * @author Martin
 */
public class Sensor {

    private int maxValues = 0;
    private int medianCount = 1;
    private final LinkedList<Float> values;
    private final LinkedList<Float> medianNumbers;

    public Sensor(int maxValues) {
        this.maxValues = maxValues;
        values = new LinkedList<>();
        medianNumbers = new LinkedList<>();
    }

    /**
     * Sets max number of values stored.
     * @param max Max values stored.
     * @return True is valid number, false otherwise.
     */
    public boolean setMaxValues(int max) {
        if (max > 0) {
            maxValues = max;
            while (values.size() > maxValues) {
                values.removeLast();
            }
            return true;
        }
        return false;
    }

    /**
     * Sets the number of values used in the median.
     * @param medianCount Number of values.
     * @return True if valid number.
     */
    public boolean setMedianCount(int medianCount) {
        if (medianCount > 0) {
            this.medianCount = medianCount;
            while (medianNumbers.size() > medianCount) {
                medianNumbers.removeLast();
            }
            return true;
        }
        return false;
    }

    /**
     * Adds a new sensor value. Removes oldest if buffer is full.
     * @param sensorValue The length in mm.
     */
    public void add(float sensorValue) {
        if (medianCount == 1) {
            values.addFirst(sensorValue);
        } else {
            medianNumbers.addFirst(sensorValue);
            if (medianNumbers.size() > medianCount) {
                medianNumbers.removeLast();
            }
            ArrayList<Float> temp = new ArrayList<>();
            for (Float f : medianNumbers) {
                temp.add(f);
            }
            Collections.sort(temp);

            float median = 0;
            if (temp.size() % 2 == 0) {
                median = temp.get(temp.size() / 2 - 1) + temp.get(temp.size() / 2);
                median /= 2;
            } else {
                median = temp.get(temp.size() / 2);
            }

            values.addFirst(median);
        }

        if (values.size() > maxValues) {
            values.removeLast();
        }
    }

    /**
     * Returns the value at a position in the buffer.
     * @param index The index.
     * @return The length in mm.
     */
    public float get(int index) {
        return values.get(index);
    }
    
    /**
     * Number of values stored in the buffer.
     * @return The number of values.
     */
    public int size(){
        return values.size();
    }
}
