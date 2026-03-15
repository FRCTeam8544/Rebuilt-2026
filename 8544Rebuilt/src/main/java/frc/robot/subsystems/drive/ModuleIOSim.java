package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.generated.TunerConstants;
import frc.robot.util.PhoenixUtil;
import java.util.Arrays;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.motorsims.SimulatedMotorController;

/**
 * Physics sim implementation of module IO using maple-sim. Provides realistic swerve module
 * simulation with wheel-floor friction, skidding, and collision physics. Simulation is always based
 * on voltage control.
 */
public class ModuleIOSim implements ModuleIO {
  /** Last valid steer angle, used as fallback when maple-sim returns an invalid Rotation2d. */
  private Rotation2d lastValidAngle = Rotation2d.kZero;
  // TunerConstants doesn't support separate sim constants, so they are declared locally
  private static final double DRIVE_KS = 0.03;
  private static final double DRIVE_KV_ROT =
      0.91035; // Same units as TunerConstants: (volt * secs) / rotation
  private static final double DRIVE_KV = 1.0 / Units.rotationsToRadians(1.0 / DRIVE_KV_ROT);

  private final SwerveModuleSimulation moduleSimulation;
  private final SimulatedMotorController.GenericMotorController driveMotor;
  private final SimulatedMotorController.GenericMotorController turnMotor;

  private boolean driveClosedLoop = false;
  private boolean turnClosedLoop = false;
  private final PIDController driveController;
  private final PIDController turnController;
  private double driveFFVolts = 0.0;
  private double driveAppliedVolts = 0.0;
  private double turnAppliedVolts = 0.0;

  public ModuleIOSim(SwerveModuleSimulation moduleSimulation) {
    this.moduleSimulation = moduleSimulation;
    this.driveMotor =
        moduleSimulation
            .useGenericMotorControllerForDrive()
            .withCurrentLimit(Amps.of(TunerConstants.FrontLeft.SlipCurrent));
    this.turnMotor =
        moduleSimulation.useGenericControllerForSteer().withCurrentLimit(Amps.of(20));

    this.driveController = new PIDController(0.1, 0.0, 0.0);
    this.turnController = new PIDController(8.0, 0.0, 0.0);

    // Enable wrapping for turn PID
    turnController.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    // Read steer angle with safety guard (maple-sim 0.4.0-beta can produce invalid Rotation2d)
    Rotation2d steerAngle = safeGetSteerAngle();

    // Run closed-loop control
    if (driveClosedLoop) {
      driveAppliedVolts =
          driveFFVolts
              + driveController.calculate(
                  moduleSimulation.getDriveWheelFinalSpeed().in(RadiansPerSecond));
    } else {
      driveController.reset();
    }
    if (turnClosedLoop) {
      turnAppliedVolts = turnController.calculate(steerAngle.getRadians());
    } else {
      turnController.reset();
    }

    // Update simulation state
    driveMotor.requestVoltage(Volts.of(driveAppliedVolts));
    turnMotor.requestVoltage(Volts.of(turnAppliedVolts));

    // Update drive inputs
    inputs.driveConnected = true;
    inputs.drivePositionRad = moduleSimulation.getDriveWheelFinalPosition().in(Radians);
    inputs.driveVelocityRadPerSec =
        moduleSimulation.getDriveWheelFinalSpeed().in(RadiansPerSecond);
    inputs.driveAppliedVolts = driveAppliedVolts;
    inputs.driveCurrentAmps = Math.abs(moduleSimulation.getDriveMotorStatorCurrent().in(Amps));

    // Update turn inputs
    inputs.turnConnected = true;
    inputs.turnEncoderConnected = true;
    inputs.turnAbsolutePosition = steerAngle;
    inputs.turnPosition = steerAngle;
    inputs.turnVelocityRadPerSec =
        moduleSimulation.getSteerAbsoluteEncoderSpeed().in(RadiansPerSecond);
    inputs.turnAppliedVolts = turnAppliedVolts;
    inputs.turnCurrentAmps = Math.abs(moduleSimulation.getSteerMotorStatorCurrent().in(Amps));

    // Update odometry inputs (high-frequency from maple-sim sub-ticks)
    inputs.odometryTimestamps = PhoenixUtil.getSimulationOdometryTimeStamps();
    inputs.odometryDrivePositionsRad =
        Arrays.stream(moduleSimulation.getCachedDriveWheelFinalPositions())
            .mapToDouble(angle -> angle.in(Radians))
            .toArray();
    inputs.odometryTurnPositions = safeGetCachedSteerPositions();
  }

  /**
   * Reads steer angle from maple-sim with a fallback for invalid Rotation2d values. Maple-sim
   * 0.4.0-beta can produce zero-component Rotation2d during collisions (see maple-sim #86).
   */
  private Rotation2d safeGetSteerAngle() {
    try {
      Rotation2d angle = moduleSimulation.getSteerAbsoluteFacing();
      lastValidAngle = angle;
      return angle;
    } catch (Exception e) {
      return lastValidAngle;
    }
  }

  /**
   * Reads cached steer positions with safety fallback for invalid Rotation2d values in the array.
   */
  private Rotation2d[] safeGetCachedSteerPositions() {
    try {
      Rotation2d[] positions = moduleSimulation.getCachedSteerAbsolutePositions();
      // Validate each position
      for (int i = 0; i < positions.length; i++) {
        if (positions[i] == null) {
          positions[i] = lastValidAngle;
        }
      }
      return positions;
    } catch (Exception e) {
      // Return array filled with last valid angle
      int count = PhoenixUtil.getSimulationOdometryTimeStamps().length;
      Rotation2d[] fallback = new Rotation2d[count];
      Arrays.fill(fallback, lastValidAngle);
      return fallback;
    }
  }

  @Override
  public void setDriveOpenLoop(double output) {
    driveClosedLoop = false;
    driveAppliedVolts = output;
  }

  @Override
  public void setTurnOpenLoop(double output) {
    turnClosedLoop = false;
    turnAppliedVolts = output;
  }

  @Override
  public void setDriveVelocity(double velocityRadPerSec) {
    driveClosedLoop = true;
    driveFFVolts = DRIVE_KS * Math.signum(velocityRadPerSec) + DRIVE_KV * velocityRadPerSec;
    driveController.setSetpoint(velocityRadPerSec);
  }

  @Override
  public void setTurnPosition(Rotation2d rotation) {
    turnClosedLoop = true;
    turnController.setSetpoint(rotation.getRadians());
  }
}
