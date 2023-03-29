package frc.robot.Configuration;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class ConfigurationEntry {

    private final String key;
    private Object defaultValue;
    
    protected ConfigurationEntry(String key, Object defaultValue) {
        this.key = key;
        this.defaultValue = defaultValue;


    }



}
