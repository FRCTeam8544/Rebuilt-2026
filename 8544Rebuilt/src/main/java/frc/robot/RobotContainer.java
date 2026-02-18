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
import frc.robot.commands.DriveCommands;
import frc.robot.commands.ShooterCommands;
import frc.robot.commands.ClimberCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.climber.ClimberIOFlex;
import frc.robot.subsystems.climber.ClimberIOSim;
import frc.robot.subsystems.climber.ClimberIO;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;

import java.util.function.BooleanSupplier;

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
  private final Shooter shooter;
  private final Climber climber;

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

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    shooter = new Shooter();
    climber = new Climber();

    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        // ModuleIOTalonFX is intended for modules with TalonFX drive, TalonFX turn, and
        // a CANcoder
        drive =
            new Drive(
                new GyroIOPigeon(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));

        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));

        break;

      default:
        // Replayed robot, disable IO implementations
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

     // Default shooter controls
 /*   shooter.setDefaultCommand(ShooterCommands.joystickVoltsShoot(
        shooter,
        () -> -goose.getLeftX(),  
        () -> -goose.getLeftY(), // Feed voltage
        () -> -goose.getRightX(),
        () -> -goose.getRightY())); // Shooter voltage
*/

    shooter.setDefaultCommand(
        ShooterCommands.buttonShoot(shooter, leftTriggerGoose,
                                             rightTriggerGoose, 
                                             dpadDownTriggerGoose, dpadUpTriggerGoose,
                                             dpadLeftTriggerGoose, dpadRightTriggerGoose,
                                             startButtonGoose)
    );


        climber.setDefaultCommand(
        ClimberCommands.closedPositionControl(climber, xButtonGoose,
                                             bButtonGoose
                                            )
    );
    // Raw feed and shooter voltage tuning
    /*goose.leftTrigger().whileTrue(
        ShooterCommands.openVoltageControl(shooter, 
                                            dpadUpTriggerGoose, // Feed trigger
                                            yButtonGoose, aButtonGoose, 
                                            xButtonGoose, bButtonGoose));

    goose.leftTrigger().whileFalse(ShooterCommands.stopMotors(shooter));*/

   /* shooter.setDefaultCommand(ShooterCommands.buttonShoot(
              shooter,
              leftBackGoose, 
              aButtonGoose));*/
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
