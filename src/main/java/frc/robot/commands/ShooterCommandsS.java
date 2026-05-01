package frc.robot.commands;

import static frc.robot.constants.FieldConstants.*;
import static frc.robot.constants.SubsystemConstants.HopperConstants.*;
import static frc.robot.constants.SubsystemConstants.IntakeConstants.EXTEND_DISTANCE;
import static frc.robot.constants.SubsystemConstants.IntakeConstants.INTAKE_SPEED;
import static frc.robot.constants.SubsystemConstants.ShooterConstants.*;

import java.util.Set;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveRequest;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.*;

public class ShooterCommandsS{
        private static Translation2d currentLocation;// = new Translation2d();
        private static Timer timer = new Timer();

        public static Command fire(Loader loader, Intake intake, Hopper hopper, Shooter shooter, CommandSwerveDrivetrain drive) {
            timer.start();
            return new SequentialCommandGroup(
                //run backwards to avoid lousy shots
                new ParallelCommandGroup(
                        loader.loadBoth(-50),
                        hopper.agitate(-HOPPER_LOWER_MAX_SPEED, -HOPPER_UPPER_SPEED)
                ).onlyWhile(() -> !shooter.isAtDesiredSpeed() && timer.get() < 0.15),

                //repeat this logic sequence for whether to run
                Commands.repeatingSequence(
                    //default is run 
                    new ParallelRaceGroup(
                        loader.loadBoth(40),
                        intake.agitateIntake(),
                        hopper.agitate(HOPPER_LOWER_SPEED, HOPPER_UPPER_SPEED)
                    ).until(
                        //run until you are behind the hub and you aren't in your zone
                        () -> (
                            (drive.isNotInAllianceZone().getAsBoolean() && 
                            drive.getState().Pose.getY() > Units.inchesToMeters(135) && 
                            drive.getState().Pose.getY() < FIELD_WIDTH_M - Units.inchesToMeters(135)) ||
                            drive.isInAllianceZone().getAsBoolean()
                        )
                    ).andThen(
                        //then don't do anything
                        new InstantCommand(()-> {})
                    ).until(
                        //stop doing nothing if you move out from behind the hub
                        () -> (
                            (drive.isNotInAllianceZone().getAsBoolean() && 
                            (drive.getState().Pose.getY() < Units.inchesToMeters(135)) && 
                            (drive.getState().Pose.getY() > FIELD_WIDTH_M - Units.inchesToMeters(135))) ||
                            drive.isInAllianceZone().getAsBoolean()
                        )  
                    )
                )
                //keep doing this sequence
                .repeatedly()).finallyDo(() -> {timer.stop();timer.reset();});
        }
        public static Command aim(Shooter shooter, CommandSwerveDrivetrain drive, DoubleSupplier x, DoubleSupplier y) {
            return new ParallelCommandGroup(
                    new ConditionalCommand(
                        new ConditionalCommand(
                            //run SOTF
                            new ParallelCommandGroup(
                                drive.pointTowardsPoint(nextLocation(drive, TufF_TABLE, x, y, getHubLocation()), () -> x.getAsDouble() / 3, () -> y.getAsDouble() / 3),
                                setDesiredShooterStates(shooter, drive, nextLocation(drive, TufF_TABLE, x, y, getHubLocation()))
                            ),
                            //run shuttle SOTF
                            new ParallelCommandGroup(
                                drive.pointTowardsPoint(nextLocation(drive, TufF_TABLE_SHUTTLE, x, y, drive.getState().Pose.nearest(getShuttlePoses())), () -> x.getAsDouble() / 3, () -> y.getAsDouble() / 3),
                                setDesiredShooterStates(shooter, drive, nextLocation(drive, TufF_TABLE_SHUTTLE, x, y, drive.getState().Pose.nearest(getShuttlePoses())))
                            ), 
                            //do the first one if we are in our alliance zone
                            drive.isInAllianceZone()), 
                            new ParallelCommandGroup(
                                // run far shuttling... no SOTF just max speed and angle
                                drive.pointTowardsPoint(() -> drive.getState().Pose.nearest(getShuttlePoses()).getTranslation(), () -> x.getAsDouble() / 3, () -> y.getAsDouble() / 3),
                                setDesiredShooterStates(shooter, drive, () -> drive.getState().Pose.nearest(getShuttlePoses()).getTranslation())
                            ), 
                            //do the first two if we aren't in the oposing alliance zone
                            () -> !drive.isInOpposingAllianceZone().getAsBoolean())
                            //run these until we are aiming the right direction
                            .until(drive.isAtDesiredAngleCheck())
                            .andThen(
                                //This is me realizing that this isn't going to work...
                                drive.applyRequest(() -> new SwerveRequest.SwerveDriveBrake()))
                            .until(() -> !drive.isAtDesiredAngle())
                ).repeatedly();
        }

    public static Command setDesiredShooterStates(Shooter shooter, CommandSwerveDrivetrain drive, Supplier<Translation2d> currentLocation) {
        return new ParallelCommandGroup(
            shooter.defer(
                () -> Commands.repeatingSequence(
                    new ConditionalCommand(
                        new ConditionalCommand(
                            shooter.setShooterState(
                                () -> SCORE_ANGLE_LOOKUP.get(drive.getState().Pose
                                        .getTranslation()
                                        .getDistance(currentLocation.get())),
                                () -> SHOOTER_VELOCITY_LOOKUP
                                        .get(drive.getState().Pose.getTranslation()
                                                .getDistance(currentLocation.get()))), 
                            shooter.setShooterState(
                                () -> 0.08,
                                () -> SHOOTER_VELOCITY_LOOKUP
                                        .get(drive.getState().Pose.getTranslation()
                                                .getDistance(currentLocation.get()))), 
                            drive.isInAllianceZone()), 
                            shooter.setShooterState(0.08, 100), 
                            () -> !drive.isInOpposingAllianceZone().getAsBoolean())
                    )
            )
        );
    }

    private static Supplier<Translation2d> nextLocation(CommandSwerveDrivetrain drive, InterpolatingDoubleTreeMap tuff, DoubleSupplier x, DoubleSupplier y, Pose2d shot) {
        return  () -> {
                Translation2d currentLocation = shot.getTranslation();
                Translation2d robotPos = drive.getState().Pose.getTranslation();
                double vx = drive.getVelocityX().getAsDouble();
                double vy = drive.getVelocityY().getAsDouble();
                for (int i = 0; i < 15; i++) {
                    double tof = tuff.get(robotPos.getDistance(currentLocation));
                    currentLocation = shot.getTranslation().minus(new Translation2d(vx * tof, vy * tof));
                    DogLog.log("Subsystems/Drive/Virtual Shot", new Pose2d(currentLocation, Rotation2d.kZero));
                }
                return currentLocation;
            };
    }
}
