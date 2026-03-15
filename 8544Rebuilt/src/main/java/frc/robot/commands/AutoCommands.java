package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Arm.Arm;
import frc.robot.subsystems.Feeder.Feeder;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import org.littletonrobotics.junction.Logger;

/** Autonomous command factories and NamedCommand registration for PathPlanner. */
public class AutoCommands {

  // Arm positions (rotations)
  private static final double ARM_EXTEND_POSITION_ROTATIONS = 0.78;
  private static final double ARM_RETRACT_POSITION_ROTATIONS = 0.037;
  private static final double ARM_POSITION_TIMEOUT_SECONDS = 2.0;

  // Intake
  private static final double INTAKE_DUTY = 0.9;
  private static final double INTAKE_RUN_TIMEOUT_SECONDS = 2.0;

  // Shooter
  private static final double SHOOTER_SPINUP_RPM = 3000.0;
  private static final double SHOOTER_SPINUP_TIMEOUT_SECONDS = 3.0;

  // Feeder
  private static final double FEEDER_NOMINAL_RPM = 300.0;
  private static final double FEED_SCORE_TIMEOUT_SECONDS = 1.0;

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
   */
  public static void registerNamedCommands(
      Arm arm, Intake intake, Shooter shooter, Feeder feeder) {
    NamedCommands.registerCommand("extendArm", extendArm(arm));
    NamedCommands.registerCommand("retractArm", retractArm(arm));
    NamedCommands.registerCommand("runIntake", runIntake(intake));
    NamedCommands.registerCommand("stopIntake", stopIntake(intake));
    NamedCommands.registerCommand("spinUpShooter", spinUpShooter(shooter));
    NamedCommands.registerCommand("stopShooter", stopShooter(shooter));
    NamedCommands.registerCommand("feedToShooter", feedToShooter(feeder));
    NamedCommands.registerCommand("stopFeeder", stopFeeder(feeder));
  }

  /**
   * Moves the arm to the extend position. The motor controller's onboard PID holds the position
   * after the command ends.
   *
   * @param arm the arm subsystem
   * @return a terminating command
   */
  public static Command extendArm(Arm arm) {
    return Commands.runOnce(() -> arm.runToPosition(ARM_EXTEND_POSITION_ROTATIONS), arm)
        .andThen(Commands.waitSeconds(ARM_POSITION_TIMEOUT_SECONDS))
        .withName("AutoExtendArm");
  }

  /**
   * Moves the arm to the retract position. The motor controller's onboard PID holds the position
   * after the command ends.
   *
   * @param arm the arm subsystem
   * @return a terminating command
   */
  public static Command retractArm(Arm arm) {
    return Commands.runOnce(() -> arm.runToPosition(ARM_RETRACT_POSITION_ROTATIONS), arm)
        .andThen(Commands.waitSeconds(ARM_POSITION_TIMEOUT_SECONDS))
        .withName("AutoRetractArm");
  }

  /**
   * Runs the intake at duty cycle for a fixed duration, stopping when complete.
   *
   * @param intake the intake subsystem
   * @return a terminating command
   */
  public static Command runIntake(Intake intake) {
    return Commands.run(() -> intake.runOpenLoop(INTAKE_DUTY), intake)
        .withTimeout(INTAKE_RUN_TIMEOUT_SECONDS)
        .finallyDo(() -> intake.stopMotors())
        .withName("AutoRunIntake");
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
   * Spins up the shooter flywheel. Terminates when the flywheel reaches its RPM target or after
   * timeout. Does NOT stop the shooter on completion so it remains spinning for a subsequent feed
   * command.
   *
   * @param shooter the shooter subsystem
   * @return a terminating command
   */
  public static Command spinUpShooter(Shooter shooter) {
    return Commands.run(() -> shooter.runAtRpm(SHOOTER_SPINUP_RPM), shooter)
        .until(shooter.flywheelAtRpmTarget)
        .withTimeout(SHOOTER_SPINUP_TIMEOUT_SECONDS)
        .withName("AutoSpinUpShooter");
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
   * Feeds a game piece into the shooter for a fixed duration, stopping when complete.
   *
   * @param feeder the feeder subsystem
   * @return a terminating command
   */
  public static Command feedToShooter(Feeder feeder) {
    return Commands.run(() -> feeder.runAtRpm(FEEDER_NOMINAL_RPM), feeder)
        .withTimeout(FEED_SCORE_TIMEOUT_SECONDS)
        .finallyDo(() -> feeder.stopMotors())
        .withName("AutoFeedToShooter");
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
