package frc.robot.SparkMaxUtils;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class SparkMax extends CANSparkMax {

    private final String name;
    
    private boolean disabled = false;

    public SparkMax(String name, int channel, MotorType motorType) {
        super(channel, motorType);
        this.name = name;

        SparkMaxManager.addMax(this);

        ShuffleboardTab motors = Shuffleboard.getTab(name);

        motors.addNumber(this.name + " Position", () -> super.getEncoder().getPosition());
        motors.addNumber(this.name + " Velocity", () -> super.getEncoder().getVelocity());
        motors.addNumber(this.name + " Current", () -> super.getOutputCurrent());
        motors.addNumber(this.name + " Temperature (C)", () -> super.getMotorTemperature());
        motors.addBoolean(this.name + " Stall", () -> super.getFault(FaultID.kStall));

    }

    public String getName() {
        return name;
    }

    public boolean getDisabled() {
        return disabled;
    }

    public void setDisabled(boolean d) {
        disabled = d;
    }

}
