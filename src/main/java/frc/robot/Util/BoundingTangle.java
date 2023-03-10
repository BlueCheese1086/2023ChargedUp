package frc.robot.Util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

public class BoundingTangle {

    private final Translation2d lowerRight, upperLeft;

    public BoundingTangle(Translation2d lowerRight, Translation2d upperLeft) {
        this.lowerRight = lowerRight;
        this.upperLeft = upperLeft;
    }

    public boolean isInside(Pose2d position) {
        Translation2d tranlation = position.getTranslation();

        return 
            (tranlation.getX() >= lowerRight.getX() && tranlation.getX() <= upperLeft.getX()) &&
            (tranlation.getY() >= lowerRight.getY() && tranlation.getY() <= upperLeft.getY());
    }
    
}
