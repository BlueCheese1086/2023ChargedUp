package frc.robot.SparkMaxUtils;

import com.revrobotics.CANSparkMax;

import frc.robot.Configuration.ControllableConfiguration;

public class SparkMax extends CANSparkMax {

    private final String name;
    
    private boolean disabled = false;

    private ControllableConfiguration disableOverride;

    public SparkMax(String name, int channel, MotorType motorType) {
        super(channel, motorType);
        this.name = name;

        SparkMaxManager.addMax(this);

        disableOverride = new ControllableConfiguration("Motors", name + " Enabled", true);
    }

    public String getName() {
        return name;
    }

    public void setDisabled(boolean d) {
        disabled = d;
    }

    public boolean getDisabled() {
        return !(Boolean) disableOverride.getValue() || disabled;
    }

    // public ControllableConfiguration getDisableOverride() {
    //     return disableOverride;
    // }

}
