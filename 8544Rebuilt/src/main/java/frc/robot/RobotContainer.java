package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;

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
import frc.robot.subsystems.game.*;
import frc.robot.subsystems.leds.*;
import frc.robot.subsystems.shooter.*;
import frc.robot.subsystems.vision.*;

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
  private final Leds leds;
private final Game game;
  private final Vision vision;

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
  private final Trigger rightBackGoosePID = new Trigger(goose.rightBumper());
  private final Trigger leftBackGoosePID = new Trigger(goose.leftBumper());

  private final Trigger isRobotIntaking;
  private final Trigger isRobotShooting;
  private final Trigger isRobotClimbing;

  private final Trigger manualArmOverrideTrigger;

  private final Trigger shakeWhenTimeToShoot;

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
        game = new Game(new GameIOFMS());
        intake = new Intake(new IntakeIOMax(Intake.intakeCanId));
        shooter = new Shooter(new ShooterIOTalonFX(Shooter.leftMotorCanID, Shooter.rightMotorCanID));
        drive =
            new Drive(
                new GyroIOPigeon(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));
        leds = new Leds(new LedIOCANdle());

        vision =
            new Vision(
                drive.robotPoseSupplier,
                drive::addVisionMeasurement,
                new VisionIOPhotonVision(
                    VisionConstants.FrontRightCam, VisionConstants.robotToFrontAprilCam),
                new VisionIOPhotonVision(
                    VisionConstants.DriverCam, VisionConstants.robotToDriverCam),
                new VisionIOPhotonVision(
                    VisionConstants.RearModuleA, VisionConstants.robotToRearModuleA)//,
            //    new VisionIOPhotonVision(
              //      VisionConstants.RearModuleB, VisionConstants.robotToRearModuleB)
              );

        break;

      case SIM:
        game = new Game(new GameIOSim());

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
        leds = new Leds(new LedIOSim());

        // PhotonVisionSim with 3 cameras exceeds 20ms loop budget; use noop for now
        vision =
            new Vision(drive.robotPoseSupplier, drive::addVisionMeasurement, new VisionIO() {});
        break;

      default:
        // Replayed robot, disable IO implementations
        game = new Game(new GameIOSim()); // TODO Fix should be default ctor
        intake = new Intake(new IntakeIO() {});
        shooter = new Shooter(new ShooterIO() {});
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        leds = new Leds(new LedIO() {});

        vision =
            new Vision(drive.robotPoseSupplier, drive::addVisionMeasurement, new VisionIO() {});
        break;
    }


    // Bind robot specific triggers, now that all subsystems have been created
    isRobotShooting = new Trigger(feeder.isFeeding); // Fuel in the air!!
    isRobotIntaking = new Trigger(intake.isIntaking); // Feed me seamore!
    isRobotClimbing = new Trigger(climber.isClimbing); // Going up!
    shakeWhenTimeToShoot = new Trigger(game.isShiftChangeSupplier);
    // User configuration triggers
    manualArmOverrideTrigger = new Trigger(arm.manualControlBooleanSupplier);

    // Register all NamedCommands BEFORE building auto chooser
    AutoCommands.registerNamedCommands(arm, intake, shooter, feeder, vision.AutoFlywheelSpeed);

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Code-only test auto for verifying path following
    autoChooser.addOption("Simple Test Auto (Drive Forward 2m)", AutoCommands.simpleTestAuto());

    // Configure the button bindings
    configureButtonBindings();
  }

  private void setupCharacterizationAutos()
  {
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
                vision.getHubRotation()));

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
    
/*     startButtonGoose.toggleOnTrue(
        
ShooterCommands.buttonShoot(shooter,
                                    vision.AutoFlywheelSpeed,
                                    shooter.flywheelAutoToggleBooleanSupplier,
                                    dpadDownTriggerGoose, // Decrease flywheel speed
                                    dpadUpTriggerGoose    // Increase flywheel speed
                                  )//.repeatedly()//.unless(aButtonGoose)

    ); */

    xButtonGoose.toggleOnTrue(
        ShooterCommands.gentleStopFlywheel(shooter)
    ); 





  
    dpadLeftTriggerGoose.onTrue(Commands.parallel(//ArmCommands.oneButtonControl(arm, true),
     (IntakeCommands.oneButtonControl(intake, true)))
     //.repeatedly()
     
     .unless(
            ()-> { return leftBackGoose.getAsBoolean() || rightBackGoose.getAsBoolean(); } //|| manualArmOverrideTrigger.getAsBoolean() ;
         )

     );

         dpadRightTriggerGoose.toggleOnTrue(Commands.parallel(//ArmCommands.oneButtonControl(arm, false),
     (IntakeCommands.oneButtonControl(intake, false)))
     //.repeatedly()
     
     .unless(  
            ()-> { return leftBackGoose.getAsBoolean() || rightBackGoose.getAsBoolean(); }
         )

     );

/*  //PID randomly turns on in Manual Mode

rightBackGoosePID.whileTrue(ArmCommands.runToPosition(arm, 0.037)
.unless(
    () -> !manualArmOverrideTrigger.getAsBoolean() == false  //was true
)
.finallyDo(
   () -> {arm.holdPosition();} 

));


leftBackGoosePID.whileTrue(ArmCommands.runToPosition(arm, 0.78)
.unless(
    () -> !manualArmOverrideTrigger.getAsBoolean() == false //was true
)
.finallyDo(
   () -> {arm.holdPosition();} 
));
*/

// Hold arm position using PID as default command. On button press this command will
// be interupted and the Arm will move with voltage control. Once button is released
// the default command will start again.
arm.setDefaultCommand(ArmCommands.holdPosition(arm));

leftBackGoose.whileTrue(ArmCommands.runToVoltage(arm, 0.9)// max speed
.unless(
    () -> !manualArmOverrideTrigger.getAsBoolean() == true //was false
)
.finallyDo(
   () -> {arm.holdPosition();} 
));

rightBackGoose.whileTrue(ArmCommands.runToVoltage(arm, -0.9) //max speed
.unless(
    () -> !manualArmOverrideTrigger.getAsBoolean() == true //was false
)
.finallyDo(
   () -> {arm.holdPosition();} 
));



/* 
aButtonGoose.whileTrue(IntakeCommands.runAtDuty(intake, 0.9)
.finallyDo(
   () -> {intake.stopMotors();} 
));
yButtonGoose.whileTrue(IntakeCommands.runAtDuty(intake, -0.9)
.finallyDo(
   () -> {intake.stopMotors();} 
)); */

intake.setDefaultCommand(
    IntakeCommands.closeLoopControl(
        intake, aButtonGoose, yButtonGoose).finallyDo(
            () ->
        intake.stopMotors()));


    feeder.setDefaultCommand(
        FeederCommands.buttonFeed(
            feeder,
            rightTriggerGoose, // Fuel feed roller to shooter flywheel
            bButtonGoose,
            shooter.flywheelAtRpmTarget,
            shooter.flywheelAutoToggleBooleanSupplier      // Reverse feed
         //   dpadLeftTriggerGoose,   // Decrease feed speed
           // dpadRightTriggerGoose   // Increase feed speed
          )
    
    );
    

    // Shooter buttons
    shooter.setDefaultCommand(ShooterCommands.idleFlywheel(shooter));
   // shooter.setDefaultCommand(ShooterCommands.autoIdleFlywheel(shooter, 
   //                                                 () -> { return game.isHubActive(); }, 
      //                                              vision.getZoneSupplier(),
     //                                               game.isFMSAvailableSupplier)
       //                                             );

    leftTriggerGoose.onTrue(
        ShooterCommands.buttonShoot(shooter,
                                    vision.AutoFlywheelSpeed,
                                    shooter.flywheelAutoToggleBooleanSupplier,
                                    dpadDownTriggerGoose, // Decrease flywheel speed
                                    dpadUpTriggerGoose    // Increase flywheel speed
                                  )
    );//.toggleOnFalse(
       // ShooterCommands.gentleStopFlywheel(shooter)
    ///);



  //  climber.setDefaultCommand(
  //      ClimberCommands.openVoltageControl(climber,
   //                                        backButtonGoose, startButtonGoose));

    // Status
  //  shakeWhenTimeToShoot.whileTrue(
    //Commands.run( () -> {goose.setRumble(RumbleType.kBothRumble,0.3);} )
   // );
    isRobotIntaking.whileTrue(
       Commands.run( () -> { 
            leds.setMechanicalState(Leds.MechanicalState.INTAKING); }, leds).
                finallyDo( () -> { leds.setMechanicalState(Leds.MechanicalState.NONE); } ) );

    isRobotShooting.whileTrue(
       Commands.run( () -> {
            leds.setMechanicalState(Leds.MechanicalState.SHOOTING); }, leds).
                finallyDo( () -> { leds.setMechanicalState(Leds.MechanicalState.NONE); } ) );

    isRobotClimbing.whileTrue(
        Commands.run( () -> {
            leds.setMechanicalState(Leds.MechanicalState.CLIMBING); }, leds).
                finallyDo( () -> { leds.setMechanicalState(Leds.MechanicalState.NONE); } ) );

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
