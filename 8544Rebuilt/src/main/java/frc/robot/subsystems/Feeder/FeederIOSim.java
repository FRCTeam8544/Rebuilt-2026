package frc.robot.subsystems.Feeder;

import frc.robot.subsystems.Feeder.FeederIO.FeedIOInputs;

public class FeederIOSim implements FeederIO {
  


  public FeederIOSim() {
  }

  @Override
  public void updateInputs(FeedIOInputs inOutData)
  {
    inOutData.connected = true;
    inOutData.wheelVelocity = 0;
    inOutData.motorVelocity = 0;
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