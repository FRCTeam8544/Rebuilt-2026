package frc.robot.subsystems.drive;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.generated.TunerConstants;
import java.util.Queue;

/** IO implementation for NavX. */
public class GyroIOPigeon implements GyroIO {
  // private final AHRS navX = new AHRS(NavXComType.kMXP_SPI, (byte) Drive.ODOMETRY_FREQUENCY);
  private final Queue<Double> yawPositionQueue;
  private final Queue<Double> yawTimestampQueue;
  private final Pigeon2 pigeon2 = new Pigeon2(TunerConstants.kPigeonId, TunerConstants.kCANBus);

  public GyroIOPigeon() {
    yawTimestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();
    yawPositionQueue = PhoenixOdometryThread.getInstance().registerSignal(pigeon2.getYaw());
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected = pigeon2.isConnected();
    inputs.yawPosition = pigeon2.getRotation2d();
    //  Rotation2d.fromDegrees(
    // pigeon2.getYaw().getValueAsDouble()); // may need to be inverted (line below too)

    inputs.yawVelocityRadPerSec =
        Units.degreesToRadians(pigeon2.getAngularVelocityZWorld().getValueAsDouble());

    inputs.odometryYawTimestamps =
        yawTimestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryYawPositions =
        yawPositionQueue.stream()
            .map((Double value) -> Rotation2d.fromDegrees(value)) // may need to be -
            .toArray(Rotation2d[]::new);
    yawTimestampQueue.clear();
    yawPositionQueue.clear();
  }
}
