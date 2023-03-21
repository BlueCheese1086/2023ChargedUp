package frc.robot.Util;

import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax;


/**
 * ChatGPT-4 just went crazy
 */
public class SparkMaxPIDTuner {
    private SparkMaxPIDController pidController;
    private double kp, ki, kd;
    private double ku, tu;
    private boolean tuningInProgress;
    private double stepSize;
    private double prevProcessValue;
    private double prevError;
    private double minError;
    private double maxError;
    private double errorSum;
    private double oscillationStartTime;
    private int pidSlot;

    public SparkMaxPIDTuner(CANSparkMax sparkMax, double stepSize, int pidSlot) {
        this.pidController = sparkMax.getPIDController();
        this.stepSize = stepSize;
        this.pidSlot = pidSlot;
        this.tuningInProgress = false;
        this.prevProcessValue = 0;
        this.prevError = Double.MAX_VALUE;
        this.minError = Double.MAX_VALUE;
        this.maxError = Double.MIN_VALUE;
        this.errorSum = 0;
        this.oscillationStartTime = -1;
    }

    public void startTuning(double initialKp) {
        this.kp = initialKp;
        this.pidController.setP(kp, pidSlot);
        this.pidController.setI(0, pidSlot);
        this.pidController.setD(0, pidSlot);
        this.tuningInProgress = true;
    }

    public void update(double targetPos, double processValue) {
        if (!tuningInProgress) return;

        double error = targetPos - processValue;
        double currentError = Math.abs(error);
        errorSum += currentError;

        if (prevError * error <= 0) { // Zero crossing
            if (oscillationStartTime > 0) {
                double oscillationPeriod = System.currentTimeMillis() - oscillationStartTime;
                if (tu == 0) {
                    tu = oscillationPeriod;
                } else {
                    tu = (tu + oscillationPeriod) / 2.0;
                }
            }
            oscillationStartTime = System.currentTimeMillis();
        }

        minError = Math.min(minError, currentError);
        maxError = Math.max(maxError, currentError);

        if (maxError - minError < stepSize) {
            ku = kp;
            kp = 0.6 * ku;
            ki = 1.2 * ku / tu;
            kd = 0.075 * ku * tu;

            pidController.setP(kp, pidSlot);
            pidController.setI(ki, pidSlot);
            pidController.setD(kd, pidSlot);
            tuningInProgress = false;
            return;
        }

        kp += stepSize;
        pidController.setP(kp, pidSlot);
        prevError = error;
    }
}
