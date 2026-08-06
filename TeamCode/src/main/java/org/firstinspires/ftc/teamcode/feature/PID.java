package org.firstinspires.ftc.teamcode.feature;

import com.acmerobotics.dashboard.config.Config;

/**
 * PID controller class for controlling motors or other systems.
 * <p>
 * This class implements a PID controller with proportional, integral, and derivative components.
 * It allows for dynamic adjustment of PID coefficients and provides methods to reset the integral term.
 */
@Config
public class PID {
    private double p;
    private double i;
    private double d;

    /**
     * Constructor for PID controller with specified coefficients.
     * @param P the proportional coefficient.
     * @param I the integral coefficient.
     * @param D the derivative coefficient.
     */
    public PID(double P, double I, double D){
        p=P;
        i=I;
        d=D;
    }
    private double integral = 0;
    private long lastLoopTime = System.nanoTime();
    private double lastError = 0;
    private int counter = 0;
    private double lastDerivative = 0.0;

    /**
     *  Low-pass filter constant for derivative smoothing.
     *  <p>
     * In real-world scenarios, the input consists of discrete values,
     * so a low-pass filter is used to achieve smooth feedback.
     */
    public static double LOW_PASS_FILTER = 0.01; // The value to manipulate (w. FTC Dashboard)

    /**
     * Resets the integral term to zero.
     * <p>
     * The integral term should be reset when the target position changes.
     */
    public void resetIntegral() {
        integral = 0;
    }

    /**
     * Getters for integral error.
     * @return the current value of the integral term.
     */
    public double getIntegral() { return integral; }

    /**
     * Provides feedback based on the PID controller using the given error.
     * <p>
     * Note: Error = target - currentValue.
     * @param error the error value.
     * @param min the minimum output value.
     * @param max the maximum output value.
     * @return the calculated PID output value, constrained between min and max.
     */
    public double update(double error, double min, double max){
        if (counter == 0) {
            lastLoopTime = System.nanoTime();
            counter++;
            return 0;
        }

        long currentTime = System.nanoTime();
        double loopTime = (currentTime - lastLoopTime) / 1000000000.0;
        lastLoopTime = currentTime; // lastLoopTime's start time

        double proportion = p * error;
        integral += error * i * loopTime;

        double rawDerivative = (error - lastError) / loopTime;
        if (counter == 0) {
            lastDerivative = (error - lastError) / loopTime;
        } else {
            lastDerivative = (1.0-LOW_PASS_FILTER) * lastDerivative + LOW_PASS_FILTER * rawDerivative; // 1차 low-pass filter
        }
        double derivative = d * lastDerivative;

        lastError = error;
        counter ++;

        double value = proportion + integral + derivative;
        if (value < min) {
            value = min;
            resetIntegral();
        } else if (value > max) {
            value = max;
            resetIntegral();
        }
        return value;
    }

    /**
     * Updates the PID coefficients dynamically.
     * @param p the proportional coefficient.
     * @param i the integral coefficient.
     * @param d the derivative coefficient.
     */
    public void updatePID(double p, double i, double d) {
        this.p = p;
        this.i = i;
        this.d = d;
    }
}