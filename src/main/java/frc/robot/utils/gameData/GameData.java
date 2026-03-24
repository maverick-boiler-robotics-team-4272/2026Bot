package frc.robot.utils.gamedata;

import static frc.robot.constants.FieldConstants.isRedSide;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.utils.periodic.Periodical;
import static frc.robot.RobotContainer.joystick;
import static frc.robot.RobotContainer.operator;

public class GameData implements Periodical{
    private static String gameData;
    private static Trigger rumbleTrigger;
    private static double matchTime;

    public static void setGameData() {
        gameData = DriverStation.getGameSpecificMessage();
        rumbleTrigger = new Trigger(() -> buzz());
        rumbleTrigger.onTrue(Commands.runOnce(() -> joystick.setRumble(edu.wpi.first.wpilibj.GenericHID.RumbleType.kBothRumble, 0.5)));
    }
    public static boolean redWonAuto() {
        return gameData.toLowerCase().startsWith("r") ? true : false;
    }

    public static boolean redHubActive() {
        matchTime = DriverStation.getMatchTime();
        if(redWonAuto()) {
            if((matchTime > 20 && matchTime < 30) || (matchTime > 55 && matchTime < 80) || (matchTime > 105)) return true;
            return false;
        } else {
            if((matchTime > 20 && matchTime < 30) || (matchTime < 55 && matchTime > 30) || (matchTime < 105 && matchTime > 80) || (matchTime > 130)) return false;
            return true; 
        }
    }
    /**
     * Buzz 15 seconds before our shift starts, and when our shift ends.
     */
    private static boolean buzz() {
        matchTime = DriverStation.getMatchTime();
        //red wins and on red
        if(isRedSide() && redWonAuto() && (matchTime == 40 || matchTime == 80 || matchTime == 90) ) {
            return true;
        } 
        //red loses and on red
        else if(isRedSide() && !redWonAuto() && (matchTime == 55 || matchTime == 65 || matchTime == 105 || matchTime == 115)) {
            return true;
        } 
        //blue wins and on blue
        else if(!isRedSide() && !redWonAuto() && (matchTime == 40 || matchTime == 80 || matchTime == 90)) {
            return true;
        } 
        //blue loses and on blue
        else if(!isRedSide() && redWonAuto() && (matchTime == 55 || matchTime == 65 || matchTime == 105 || matchTime == 115)) {
            return true;
        }
        return false;
    }
    @Override
    public void periodic() {
        // DogLog.log("GameData/data", gameData);
        // DogLog.log("GameData/time", matchTime);
    }
}
