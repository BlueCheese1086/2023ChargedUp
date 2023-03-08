package frc.robot.Util;

import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DebugPID extends SubsystemBase {

    SparkMaxPIDController controller;
    PIDController debug;
    PIDController lastDebug;

    ComplexWidget widget;

    public DebugPID(SparkMaxPIDController c, String name) {
        this.controller = c;
        debug = new PIDController(
            c.getP(), 
            c.getI(), 
            c.getD());
        this.lastDebug = new PIDController(
            c.getP(), 
            c.getI(), 
            c.getD());
        Shuffleboard.getTab("Debug").add(name, debug);
    }

    @Override
    public void periodic() {
        if (!equals(debug, lastDebug)) {
            controller.setP(debug.getP());
            controller.setI(debug.getI());
            controller.setD(debug.getD());
        }
        lastDebug.setP(debug.getP());
        lastDebug.setI(debug.getI());
        lastDebug.setD(debug.getD());
    }

    private boolean equals(PIDController one, PIDController two) {
        return 
        one.getP() == two.getP() &&
        one.getI() == two.getI() &&
        one.getD() == two.getD();
    }

    
}
