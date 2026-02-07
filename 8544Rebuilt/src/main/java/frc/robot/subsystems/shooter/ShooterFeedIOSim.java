package frc.robot.subsystems.shooter;


public class ShooterFeedIOSim implements ShooterFeedIO {
  


  public ShooterFeedIOSim() {
  }

  @Override
  public void updateInputs(ShooterFeedIOInputs inOutData)
  {
    inOutData.connected = true;
    inOutData.velocity = 0;// TODO need encoder;
    inOutData.motorTemperature = 42;

    // Fault codes
    inOutData.faultCan = false;
    inOutData.faultTemperature = false;
    inOutData.faultSensor = false;
    inOutData.faultGateDriver = false;
    inOutData.faultEscEeprom = false;
    inOutData.faultFirmware = false;

    // Outputs
    inOutData.busVoltage = 0;
    inOutData.outputDuty = 0; // -1 to 1 percent applied of bus voltage
    inOutData.outputCurrent = 0;

    inOutData.velocitySetPoint = 0.0f; // Percent of max motor speed (0...1)
    inOutData.voltageSetPoint = 0.0f; // Motor voltage, usually not directly controlled
  }

  @Override
  public void setVelocity(double rpm) {
    // TODO, using closed loop controller
  }

  @Override
  public void setVoltage(double volts) {
    
  }

}