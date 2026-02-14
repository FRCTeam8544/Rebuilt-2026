package frc.robot.subsystems.Intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import static edu.wpi.first.units.Units.Rotations;

import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {

  // Neo vortex can do over 5000 RPM, but flywheel is quite a chonker... so limit to be safe for now
  public static final double kMaxIntakeRPM = 2000;
  // public static final double kMaxFeedRPM = 6000; // Attached to 20 to 1 gearbox

  public static final int armCanId = 30;
  //  public static final int rightMotorCanID = 25;
  // public static final int feedMotorCanID = 26;
  //public static final int rotations = 0;
  public final IntakeIO IntakeIO;
  public final IntakeIOInputsAutoLogged IntakeInputs = new IntakeIOInputsAutoLogged();

  // private final IntakeFeedIO IntakeFeedIO;
  // private final IntakeFeedIOInputsAutoLogged IntakeFeedInputs = new
  // IntakeFeedIOInputsAutoLogged();

  // private double tuneFeedVoltage = 3.0;
  private double tuneShootVoltage = 0.0;
  //   private final double tuneFeedVoltStep = 1.0 / 50.0; // 1 volt per second
  private final double tuneShootVoltStep = 0.25 / 50; // 1/4 volt per second

  public Intake() {
    this.IntakeIO = new IntakeIOMax(armCanId);
    // this.IntakeFeedIO = new IntakeFeedIOMax(feedMotorCanID);
  }

  public void tuneIncreaseVoltage() {
    tuneShootVoltage += tuneShootVoltStep;
    if (tuneShootVoltage > 12.0) {
      tuneShootVoltage = 12.0;
    }
  }

  public void tuneDecreaseVoltage() {
    tuneShootVoltage -= tuneShootVoltStep;
    if (tuneShootVoltage < 0.0) {
      tuneShootVoltage = 0.0;
    }
  }
  /*
  public void tuneIncreaseFeedVoltage() {
    tuneFeedVoltage += tuneFeedVoltStep;
    if (tuneFeedVoltage > 12.0) {
      tuneFeedVoltage = 12.0;
    }
  }

  public void tuneDecreaseFeedVoltage() {
    tuneFeedVoltage -= tuneFeedVoltStep;
    if (tuneFeedVoltage < 0.0) {
      tuneFeedVoltage = 0.0;
    }
  }
     */



  //   public void runFeedOpenLoop()
  ///  {
  //   runFeedOpenLoop( tuneFeedVoltage / 12.0 );
  // }

  public void runIntakeOpenLoop(boolean extend) {
 double sign = 1.0;
 if ( extend) {sign = 1.0;}
  else{ sign = -1.0;}

    IntakeInputs.voltageSetPoint = tuneShootVoltage * sign;
    IntakeInputs.positionSetPoint = 0.0;
    
    IntakeIO.setVoltage(IntakeInputs.voltageSetPoint);
  }

  //    public void runFeedOpenLoop(double duty)
  // {
  // Prevent duty beyond 1 to -1
  //    if ( (duty > 1.0) || (duty < -1.0) )
  //   {
  //    duty = Math.copySign(1.0, duty);
  //   }

  //    double scaledVolts = duty * Constants.NeoVortex.nominalVoltage;
  // IntakeFeedIO.setVoltage(scaledVolts);

  //  IntakeFeedInputs.voltageSetPoint = scaledVolts;
  // IntakeFeedInputs.velocitySetPoint = 0.0;
  //  }

  public void runIntake(double rotations) {

    // Prevent out of spec RPM

    IntakeInputs.positionSetPoint = rotations;
   // IntakeInputs.feedForward = tuneShootVoltage;
    IntakeIO.setPosition(IntakeInputs.positionSetPoint);
  } // */
  /*
      public void runFeed(double rpm)
      {
         // Prevent out of spec RPM
        if ( (rpm > kMaxFeedRPM) || (rpm < -kMaxFeedRPM) )
        {
          rpm = Math.copySign(kMaxIntakeRPM, rpm);
        }

        IntakeFeedInputs.voltageSetPoint = 0.0;
        IntakeFeedInputs.velocitySetPoint = rpm;

        IntakeFeedIO.setVelocity(IntakeFeedInputs.velocitySetPoint);
      }
  */
  public void stopOpenLoop() {
    // runFeedOpenLoop(0.0);
   IntakeInputs.voltageSetPoint = 0;
   IntakeIO.setVoltage(0);
  }

  @Override
  public void periodic() {
    IntakeIO.updateInputs(IntakeInputs);
    //  IntakeFeedIO.updateInputs(IntakeFeedInputs);
    Logger.processInputs("Intake/Motors", IntakeInputs);
    //  Logger.processInputs("Intake/Feed", IntakeFeedInputs);
  }
}
