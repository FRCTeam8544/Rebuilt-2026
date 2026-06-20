package frc.robot.commands;

import org.littletonrobotics.junction.AutoLog;

@AutoLog
public class DriveCommandsIOInputs {

public double xPID = 0.0f;

public double xPIDOutput = 0.0f;
public double yPIDOutput = 0.0f;

public double xPIDGoalPosition = 0.0f;
public double xPIDGoalVelocity = 0.0f;

public double xPIDSetpointPosition = 0.0f;
public double xPIDSetpointVelocity = 0.0f;

public double xPIDPositionTolerance = 0.0f;
public double xPIDVelocityTolerance = 0.0f;
    
}