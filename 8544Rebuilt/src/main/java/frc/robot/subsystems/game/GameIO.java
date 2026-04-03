package frc.robot.subsystems.game;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.wpilibj.DriverStation;

public interface GameIO {

  public static final double shiftLength = 25;

  @AutoLog
  public static class GameIOInputs {
    // Inputs
    public boolean connected = false;

    public String autoWinner = "UNKNOWN";
    public double matchTime = 0.0;
    public double shiftTimeRemaining = 0.0;

    public boolean fmsAvailable = false;
    public DriverStation.Alliance ownAlliance = DriverStation.Alliance.Blue;
    
    // Outputs
    public boolean isHubActive = false;
    public boolean shakeWhenTimeToShoot = false;
  }

  public default void updateInputs(GameIOInputs inOutData) {}

}
