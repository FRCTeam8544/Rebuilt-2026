package frc.robot.subsystems.shooter;

public class ShooterIOSim implements ShooterIO{
 
  
  public ShooterIOSim() {
  }

  public void updateInputs(ShooterIOInputs inOutData)
  {
    inOutData.connected = false;
    inOutData.velocity = 0;
    inOutData.leaderMotorTemperature = 0;
    inOutData.followMotorTemperature = 0;

    // Fault codes
    inOutData.faultSensor = false;
    inOutData.faultCan = false;
    inOutData.faultTemperature = false;
    inOutData.faultGateDriver = false;
    inOutData.faultEscEeprom = false;
    inOutData.faultFirmware = false;
    
    // Outputs
    inOutData.busVoltage = 0.0f;
    inOutData.outputDuty = 0.0f; // -1 to 1 percent applied of bus voltage
    inOutData.outputCurrent = 0.0f;
    inOutData.outputVoltage = 0.0f;
    inOutData.velocitySetPoint = 0.0f; // Percent of max motor speed (0...1)
    inOutData.voltageSetPoint = 0.0f; // Motor voltage, usually not directly controlled
    
  }

  
  @Override
  public void setVelocity(double rpm) {
  }

  @Override
  public void setVoltage(double volts) {
  }
}
