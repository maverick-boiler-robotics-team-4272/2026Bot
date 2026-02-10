package frc.robot.constants;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

public class FieldConstants {
    public static final double HUB_HEIGHT_M = 1.83;
    public static final double FUEL_MAX_HEIGHT_M = 3;
    public static final double ROBOT_MASS_KG = 68.4;
    public static final Translation2d HUB_LOCATION = new Translation2d(isRedSide() ? 12.51 : 4.03, 4.035);
    
    public static final SendableChooser<String> SIDE_CHOOSER = new SendableChooser<>();
    
    public static boolean isRedSide() {
        if(SIDE_CHOOSER.getSelected() == "red"){
            return true;
        }
        return false;
    }
}
