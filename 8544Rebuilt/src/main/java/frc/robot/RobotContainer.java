package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.*;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Arm.*;
import frc.robot.subsystems.Feeder.*;
import frc.robot.subsystems.Intake.*;
import frc.robot.subsystems.climber.*;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.shooter.*;
import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.Arena2026Rebuilt;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  // Subsystems
  private final Drive drive;
  private final Arm arm;
  private final Intake intake;
  private final Feeder feeder;
  private final Shooter shooter;
  private final Climber climber;

  // Simulation
  private SwerveDriveSimulation driveSimulation = null;
  private IntakeSimulation intakeSimulation = null;

  // Projectile launch parameters
  private static final double SHOOTER_LAUNCH_THRESHOLD_RPM = 500.0;
  private static final double LAUNCH_COOLDOWN_SECONDS = 0.3;
  private static final double MAX_LAUNCH_SPEED_MPS = 12.0;
  private double lastLaunchTimestamp = 0.0;

  // Controller
  private final CommandXboxController maverick = new CommandXboxController(0);
  private final CommandXboxController goose = new CommandXboxController(1);

  private final Trigger aButtonGoose = new Trigger(goose.a());
  private final Trigger bButtonGoose = new Trigger(goose.b());
  private final Trigger yButtonGoose = new Trigger(goose.y());
  private final Trigger xButtonGoose = new Trigger(goose.x());
  private final Trigger rightBackGoose = new Trigger(goose.rightBumper());
  private final Trigger leftBackGoose = new Trigger(goose.leftBumper());
  private final Trigger leftTriggerGoose = new Trigger(goose.leftTrigger());
  private final Trigger rightTriggerGoose = new Trigger(goose.rightTrigger());
  private final Trigger dpadUpTriggerGoose = new Trigger(goose.povUp());
  private final Trigger dpadDownTriggerGoose = new Trigger(goose.povDown());
  private final Trigger dpadLeftTriggerGoose = new Trigger(goose.povLeft());
  private final Trigger dpadRightTriggerGoose = new Trigger(goose.povRight());
  private final Trigger startButtonGoose = new Trigger(goose.start());
  private final Trigger backButtonGoose = new Trigger(goose.back());

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    arm = new Arm();
    feeder = new Feeder();
    climber = new Climber();

    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        intake = new Intake(new IntakeIOMax(Intake.intakeCanId));
        shooter = new Shooter(new ShooterIOTalonFX(Shooter.leftMotorCanID, Shooter.rightMotorCanID));
        drive =
            new Drive(
                new GyroIOPigeon(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));
        break;

      case SIM:
        // Sim robot, instantiate maple-sim physics simulation
        // Disable ramp colliders so the robot can drive through ramp areas
        SimulatedArena.overrideInstance(new Arena2026Rebuilt(false));

        driveSimulation =
            new SwerveDriveSimulation(
                Drive.getMapleSimConfig(), new Pose2d(3, 3, new Rotation2d()));
        SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation);

        // Intake simulation: over-the-bumper, front-mounted, ~24in wide, capacity 5
        intakeSimulation =
            IntakeSimulation.OverTheBumperIntake(
                "Fuel",
                driveSimulation,
                Meters.of(0.6),
                Meters.of(0.15),
                IntakeSimulation.IntakeSide.FRONT,
                5);

        intake = new Intake(new IntakeIOSim(intakeSimulation));
        shooter = new Shooter(new ShooterIOSim());

        // Populate fuel on the field at startup
        SimulatedArena.getInstance().resetFieldForAuto();

        drive =
            new Drive(
                new GyroIOSim(driveSimulation.getGyroSimulation()),
                new ModuleIOSim(driveSimulation.getModules()[0]),
                new ModuleIOSim(driveSimulation.getModules()[1]),
                new ModuleIOSim(driveSimulation.getModules()[2]),
                new ModuleIOSim(driveSimulation.getModules()[3]));
        break;

      default:
        // Replayed robot, disable IO implementations
        intake = new Intake(new IntakeIO() {});
        shooter = new Shooter(new ShooterIO() {});
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        break;
    }

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    // ----- Driver Controls ------

    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -maverick.getLeftY(),
            () -> -maverick.getLeftX(),
            () -> -maverick.getRightX()));

    // Lock to 0° when A button is held
    maverick
        .a()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -maverick.getLeftY(),
                () -> -maverick.getLeftX(),
                () -> Rotation2d.kZero));

    // Switch to X pattern when X button is pressed
    maverick.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Reset gyro to 0° when B button is pressed
    maverick
        .b()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)),
                    drive)
                .ignoringDisable(true));


    // ----- Operator Controls -------
    
    arm.setDefaultCommand(
        ArmCommands.openLoopControl(
            arm,
            leftBackGoose, // retract arm
            rightBackGoose // extend arm
    ));
    
    intake.setDefaultCommand(
        IntakeCommands.openLoopControl(
            intake,
            aButtonGoose, // intake fuel
            yButtonGoose // expel fuel
    ));

    feeder.setDefaultCommand(
        FeederCommands.buttonFeed(
            feeder,
            rightTriggerGoose, // Fuel feed roller to shooter flywheel
            bButtonGoose,      // Reverse feed 
            dpadLeftTriggerGoose,   // Decrease feed speed
            dpadRightTriggerGoose   // Increase feed speed
          )
    );



    // Calibration only, replace the default shooter command to use
//    goose.start().whileTrue(ShooterCommands.feedforwardCharacterization(shooter));
 //   goose.start().whileFalse(ShooterCommands.stopMotors(shooter));

    shooter.setDefaultCommand(
        ShooterCommands.buttonShoot(shooter,
                                    leftTriggerGoose, // Run Shooter flywheel
                                    dpadDownTriggerGoose, // Decrease flywheel speed
                                    dpadUpTriggerGoose    // Increase flywheel speed
                                  )
    );
   
    climber.setDefaultCommand(
        ClimberCommands.openVoltageControl(climber,
                                           backButtonGoose, startButtonGoose));

  }


  /** Updates the maple-sim physics simulation. Call from Robot.simulationPeriodic(). */
  public void updateSimulation() {
    if (driveSimulation == null) return;
    SimulatedArena.getInstance().simulationPeriodic();
    Logger.recordOutput(
        "FieldSimulation/RobotPosition", driveSimulation.getSimulatedDriveTrainPose());
    Logger.recordOutput(
        "FieldSimulation/Fuel",
        SimulatedArena.getInstance().getGamePiecesArrayByType("Fuel"));

    // Launch fuel projectile when shooter is spinning and intake has fuel
    if (intakeSimulation != null
        && intakeSimulation.getGamePiecesAmount() > 0
        && shooter.flywheelRpmSupplier.getAsDouble() > SHOOTER_LAUNCH_THRESHOLD_RPM
        && Timer.getFPGATimestamp() - lastLaunchTimestamp > LAUNCH_COOLDOWN_SECONDS) {

      if (intakeSimulation.obtainGamePieceFromIntake()) {
        double launchSpeedMps =
            shooter.flywheelRpmSupplier.getAsDouble()
                / Shooter.Flywheel.kMaxShooterRPM
                * MAX_LAUNCH_SPEED_MPS;

        SimulatedArena.getInstance()
            .addGamePieceProjectile(
                new RebuiltFuelOnFly(
                    driveSimulation.getSimulatedDriveTrainPose().getTranslation(),
                    new Translation2d(-0.2, 0), // rear-mounted shooter offset
                    driveSimulation.getDriveTrainSimulatedChassisSpeedsFieldRelative(),
                    driveSimulation
                        .getSimulatedDriveTrainPose()
                        .getRotation()
                        .plus(Rotation2d.k180deg), // rear-facing
                    Meters.of(0.4),
                    MetersPerSecond.of(launchSpeedMps),
                    Degrees.of(45)));
        lastLaunchTimestamp = Timer.getFPGATimestamp();
      }
    }
  }

  /** Resets the simulated field game pieces for autonomous mode. */
  public void resetSimulationField() {
    if (driveSimulation == null) return;
    SimulatedArena.getInstance().resetFieldForAuto();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
