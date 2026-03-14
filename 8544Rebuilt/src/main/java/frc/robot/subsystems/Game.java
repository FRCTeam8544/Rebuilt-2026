package frc.robot.subsystems;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Game extends SubsystemBase {

private static final String Blue = null;
private static final String Red = null;

public Game () {

}


private Alliance autoWinner() {
String gameData = DriverStation.getGameSpecificMessage();
if (!gameData.isEmpty());
{
  switch (gameData.charAt(0))
  {
    case 'B' :
      //Alliance.Blue=autoWinner(); 
      break;
    case 'R' :
     //Alliance.Red=autoWinner();
      break;
    default :
      //This is corrupt data
      break;
  }


}

@Override
public void periodic() {

    SmartDashboard.putBoolean("Auto Enabled", DriverStation.isAutonomousEnabled());
    SmartDashboard.putBoolean("Teleop Enabled", DriverStation.isTeleopEnabled());
    SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());
    
    SmartDashboard.putString("Game Specific Message ", DriverStation.getGameSpecificMessage());
    SmartDashboard.putBoolean("Hub Active", isHubActive());

    SmartDashboard.putString("Auto winner", autoWinner());

    }
  
  //private void setupDefaultDashboard()
  {
    
    //set the default setup of the dashboard 
    
    //SmartDashboard.setDefaultNumber("", shooterInputs.flywheelVelocity);
    //SmartDashboard.setDefaultNumber("Shooter RPM", shooterInputs.motorVelocity);
    //SmartDashboard.setDefaultNumber("Shooter RPM Setpoint", shooterInputs.velocitySetPoint);
    //SmartDashboard.setDefaultNumber("Shooter Leader Temp", shooterInputs.leaderMotorTemperature);
    //SmartDashboard.setDefaultNumber("Shooter Follow Temp", shooterInputs.followMotorTemperature);
  }
public boolean isHubActive() {
  Optional<Alliance> alliance = DriverStation.getAlliance();
  // If we have no alliance, we cannot be enabled, therefore no hub.
  if (alliance.isEmpty()) {
    return false;
  }
  // Hub is always enabled in autonomous.
  if (DriverStation.isAutonomousEnabled()) {
    return true;
  }
  // At this point, if we're not teleop enabled, there is no hub.
  if (!DriverStation.isTeleopEnabled()) {
    return false;
  }

  // We're teleop enabled, compute.
  double matchTime = DriverStation.getMatchTime();
  String gameData = DriverStation.getGameSpecificMessage();
  // If we have no game data, we cannot compute, assume hub is active, as its likely early in teleop.
  if (gameData.isEmpty()) {
    return true;
  }
  boolean redInactiveFirst = false;
  switch (gameData.charAt(0)) {
    case 'R' -> redInactiveFirst = true;
    case 'B' -> redInactiveFirst = false;
    default -> {
      // If we have invalid game data, assume hub is active.
      return true;
    }
  }

  // Shift was is active for blue if red won auto, or red if blue won auto.
  boolean shift1Active = switch (alliance.get()) {
    case Red -> !redInactiveFirst;
    case Blue -> redInactiveFirst;
  };

  if (matchTime > 130) {
    // Transition shift, hub is active.
    return true;
  } else if (matchTime > 105) {
    // Shift 1
    return shift1Active;
  } else if (matchTime > 80) {
    // Shift 2
    return !shift1Active;
  } else if (matchTime > 55) {
    // Shift 3
    return shift1Active;
  } else if (matchTime > 30) {
    // Shift 4
    return !shift1Active;
  } else {
    // End game, hub always active.
    return true;
  }
}

}
}  

