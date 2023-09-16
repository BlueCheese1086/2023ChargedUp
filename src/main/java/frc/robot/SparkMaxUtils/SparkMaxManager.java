package frc.robot.SparkMaxUtils;

import java.util.ArrayList;

import com.revrobotics.CANSparkMax.FaultID;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SparkMaxManager extends SubsystemBase {

    private static ArrayList<SparkMax> maxes = new ArrayList<>();

    public static void addMax(SparkMax m) {
        maxes.add(m);
    }

    public void periodic() {
        for (SparkMax m : maxes) {
            SmartDashboard.putBoolean(String.format("%s/IsConnected", m.getName()), m.isConnected());
            SmartDashboard.putNumber(String.format("%s/Position", m.getName()), m.getEncoder().getPosition());
            SmartDashboard.putNumber(String.format("%s/Velocity", m.getName()), m.getEncoder().getVelocity());
            SmartDashboard.putNumber(String.format("%s/Current", m.getName()), m.getOutputCurrent());
            SmartDashboard.putNumber(String.format("%s/Temperature(C)", m.getName()), m.getMotorTemperature());
            SmartDashboard.putBoolean(String.format("%s/Stall", m.getName()), m.getFault(FaultID.kStall));
        }
    }

}
