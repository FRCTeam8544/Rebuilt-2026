package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Arm.Arm;
import frc.robot.subsystems.Feeder.Feeder;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.shooter.*;
import frc.robot.subsystems.vision.*;
import frc.robot.commands.DriveCommands;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import org.dyn4j.geometry.Rotation;
import org.littletonrobotics.junction.Logger;

/** Autonomous command factories and NamedCommand registration for PathPlanner. */
public class AutoCommands {

  // Close-range fixed RPM for shooting
  private static final double CLOSE_SHOOT_RPM = 3000.0;

  // Test auto path constraints
  private static final double TEST_AUTO_MAX_VELOCITY_MPS = 2.0;
  private static final double TEST_AUTO_MAX_ACCELERATION_MPSS = 2.0;
  private static final double TEST_AUTO_MAX_ANGULAR_VELOCITY_RAD_PER_SEC = 2.0 * Math.PI;
  private static final double TEST_AUTO_MAX_ANGULAR_ACCELERATION_RAD_PER_SEC_SQ = 4.0 * Math.PI;

  private AutoCommands() {}

  /**
   * Registers all NamedCommands for PathPlanner event markers and autos. MUST be called BEFORE
   * AutoBuilder.buildAutoChooser().
   *
   * @param arm the arm subsystem
   * @param intake the intake subsystem
   * @param shooter the shooter subsystem
   * @param feeder the feeder subsystem
   * @param autoFlywheelSpeed vision-based RPM supplier for distance-adjusted shots
   */
  public static void registerNamedCommands(
      Arm arm, Intake intake, Shooter shooter, Feeder feeder, 
      DoubleSupplier autoFlywheelSpeed,
      Supplier<Rotation2d> robotRotationSupplier,
      Supplier<Rotation2d> hubRotationSupplier) {
    // Shooting commands (spin up + feed + terminate)
    NamedCommands.registerCommand("AutoRpmShootFuel", new ShootFuelCommand(shooter, feeder, autoFlywheelSpeed));
    NamedCommands.registerCommand("CloseShootFuel", new ShootFuelCommand(shooter, feeder, CLOSE_SHOOT_RPM));

    // Intake command (terminates via timeout)
    NamedCommands.registerCommand("IntakeFuel", IntakeCommands.intakeFuel(intake));

    // Arm commands (terminate via .until(supplier) + timeout)
    NamedCommands.registerCommand("DeployHopper", ArmCommands.deployHopper(arm));
    NamedCommands.registerCommand("RetractHopper", ArmCommands.retractHopper(arm));

    // Stop commands (instant, fire-and-forget)
    NamedCommands.registerCommand("stopIntake", stopIntake(intake));
    NamedCommands.registerCommand("stopShooter", stopShooter(shooter));
    NamedCommands.registerCommand("stopFeeder", stopFeeder(feeder));

    // Lock the robot to aim at the hub until the unlock command is given or the command is interupted
    NamedCommands.registerCommand("lockRotationToHub", lockRotationToTarget(robotRotationSupplier, hubRotationSupplier));
    NamedCommands.registerCommand("unlockRotation", unlockRotation());

    // Turn the robot to face the hub, end when the robot has finished rotating
    NamedCommands.registerCommand("turnToHub", turnToHub(robotRotationSupplier, hubRotationSupplier));
  }

  /**
   * Stops the intake immediately.
   *
   * @param intake the intake subsystem
   * @return an instant command
   */
  public static Command stopIntake(Intake intake) {
    return Commands.runOnce(() -> intake.stopMotors(), intake).withName("AutoStopIntake");
  }

  /**
   * Stops the shooter flywheel and resets RPM adjustment.
   *
   * @param shooter the shooter subsystem
   * @return an instant command
   */
  public static Command stopShooter(Shooter shooter) {
    return Commands.runOnce(
            () -> {
              shooter.stopMotors();
              shooter.resetShooterDefaultRpm();
            },
            shooter)
        .withName("AutoStopShooter");
  }

  /**
   * Stops the feeder immediately.
   *
   * @param feeder the feeder subsystem
   * @return an instant command
   */
  public static Command stopFeeder(Feeder feeder) {
    return Commands.runOnce(() -> feeder.stopMotors(), feeder).withName("AutoStopFeeder");
  }

  public static Command lockRotationToTarget(
      Supplier<Rotation2d> robotSupplier,
      Supplier<Rotation2d> targetSupplier) {
    
    final double ANGLE_KP = 5.0;
    final double ANGLE_KD = 0.4;
    final double ANGLE_MAX_VELOCITY = 8.0;
    final double ANGLE_MAX_ACCELERATION = 20.0;

    // Create PID controller
    ProfiledPIDController angleController =
        new ProfiledPIDController(
            ANGLE_KP,
            0.0,
            ANGLE_KD,
            new TrapezoidProfile.Constraints(ANGLE_MAX_VELOCITY, ANGLE_MAX_ACCELERATION));
    angleController.enableContinuousInput(-Math.PI, Math.PI);

    return Commands.startRun(
        () -> { 
                // Reset PID controller when command starts
                angleController.reset(robotSupplier.get().getRadians());
        
                // Register angle feedback supplier to turn the robot to the target
                PPHolonomicDriveController.overrideRotationFeedback(
                  () -> {
                          // Calculate angular speed
                          double omega =
                          angleController.calculate(
                            robotSupplier.get().getRadians(), 
                            targetSupplier.get().getRadians());
                          return omega;
                  }
                );
              },
        // NOOP to keep the command going to give the robot time to find the target rotation
        () -> {   }
      ).handleInterrupt( () -> { PPHolonomicDriveController.clearRotationFeedbackOverride(); } );
  }

  // Stop following the target rotation - Instant command
  public static Command unlockRotation()
  {
    return Commands.runOnce( () -> { PPHolonomicDriveController.clearRotationFeedbackOverride(); } );
  }

  // This command runs until the robot is aimed at the hub
  public static Command turnToHub(Supplier<Rotation2d> robotRotation,
                                  Supplier<Rotation2d> targetRotation)

  {
    final double turnToleranceDegrees = 3.0;
    return AutoCommands.lockRotationToTarget(robotRotation, targetRotation)
          .until( () -> { 
            Rotation2d robot = robotRotation.get();
            Rotation2d target = targetRotation.get();
            double angleDiff = Math.abs( target.minus(robot).getDegrees() );
            return angleDiff < turnToleranceDegrees;
          } ).finallyDo(  () -> { PPHolonomicDriveController.clearRotationFeedbackOverride(); } );
  }

  /**
   * Creates a simple test auto that drives forward 2 meters in a straight line. This is a code-only
   * auto for verifying PathPlanner path following works without requiring any .path or .auto files.
   *
   * @return the test auto command
   */
  public static Command simpleTestAuto() {
    PathConstraints constraints =
        new PathConstraints(
            TEST_AUTO_MAX_VELOCITY_MPS,
            TEST_AUTO_MAX_ACCELERATION_MPSS,
            TEST_AUTO_MAX_ANGULAR_VELOCITY_RAD_PER_SEC,
            TEST_AUTO_MAX_ANGULAR_ACCELERATION_RAD_PER_SEC_SQ);

    PathPlannerPath path =
        new PathPlannerPath(
            PathPlannerPath.waypointsFromPoses(
                new Pose2d(1.0, 1.0, Rotation2d.fromDegrees(0)),
                new Pose2d(3.0, 1.0, Rotation2d.fromDegrees(0))),
            constraints,
            null, // No ideal starting state
            new GoalEndState(0.0, Rotation2d.fromDegrees(0)));

    path.preventFlipping = true;

    return Commands.sequence(
            Commands.runOnce(() -> Logger.recordOutput("Auto/TestAutoStarted", true)),
            AutoBuilder.followPath(path))
        .withName("SimpleTestAuto");
  }
}
