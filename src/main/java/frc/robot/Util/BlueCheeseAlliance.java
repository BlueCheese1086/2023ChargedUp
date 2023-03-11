package frc.robot.Util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Sensors.Field.PositionManager.Location;

public class BlueCheeseAlliance {

    private static BlueCheeseAlliance instance;

    public Location charger, scoring, pickup;

    public static BlueCheeseAlliance getAlliance() {
        instance = new BlueCheeseAlliance();
        return instance;
    }

    private BlueCheeseAlliance() {
        charger = getFriendlyZones()[0];
        pickup = getFriendlyZones()[1];
        scoring = getFriendlyZones()[2];
    }

    public Alliance getColor() {
        return DriverStation.getAlliance();
    }
    
    private Location[] getFriendlyZones() {
        if (getColor() == Alliance.Blue) return Location.blueLocations;
        return Location.redLocations;
    }
    
}
