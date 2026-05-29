package frc.robot.subsystems.shooter;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;

public class Ballistics {
    

   private final double gravity = 9.8;
   private final double launchAngle = Units.degreesToRadians(50);
  // private final double hubHeight = 2; // meters
   private final double shooterHeight = Units.inchesToMeters(22);
   private final double shooterXoffset = -Units.inchesToMeters(28 - 3.5); // Robot relative X

   private final double sinOfLaunch = Math.sin(launchAngle);
   private final double sinOfLaunchSq = sinOfLaunch * sinOfLaunch;

   private Translation3d targetPoint = Translation3d.kZero;
   private double currentTimeOfFlight = 0.0;
   private double currentLaunchSpeed = 0.0;

   public Ballistics() {
        currentLaunchSpeed = 0.0;
        currentTimeOfFlight = 0.0;
   }
    
  // Given the inital velociy and distance, assuming robot hub, compute flight time.
  public double determineTimeOfFlight(Translation2d robotCenterPoint, Translation3d targetPoint) {
    
    double launchSpeed = determineLaunchSpeed(robotCenterPoint, targetPoint);
    Translation2d launchVector = targetPoint.toTranslation2d().minus(robotCenterPoint);

    // Assume that the robot is facing the hub for the p
    final double x = launchVector.getNorm() + Math.abs(shooterXoffset);

    return  x / (launchSpeed * Math.cos(launchAngle));
  }

  // Given the robot center point in field coordinates
  // determine the inital launch speed that must be used to reach the target location.
  // It is assumed that the target location is the top of the hub.
  // Shooter launch offset from robot center point is also accounted for here.
  public double determineLaunchSpeed(Translation2d robotCenterPoint, Translation3d targetPoint)
  {
    Translation2d launchVector = targetPoint.toTranslation2d().minus(robotCenterPoint);

    // Assume that the robot is facing the target for the shot
    final double x = launchVector.getNorm() + Math.abs(shooterXoffset);
    final double z = targetPoint.getZ() - shooterHeight;

    // initialV = sqrt( (x^2 * g ) / x sin theta - 2 * z cos^2(theta)
    
    return Math.sqrt( (x*x*gravity) / (x * sinOfLaunch - 2 * z * sinOfLaunchSq) );
  }

  public void resetTarget(Translation3d targetPoint) {
    this.targetPoint = targetPoint;
  }

  public Rotation2d calculate( Pose2d robotPose, ChassisSpeeds speeds) {
    Translation2d robotCenterPoint = robotPose.getTranslation();

    currentLaunchSpeed = determineLaunchSpeed(robotCenterPoint, targetPoint);
    currentTimeOfFlight = determineTimeOfFlight(robotCenterPoint, targetPoint);

    Translation2d launchVector = targetPoint.toTranslation2d().minus(robotCenterPoint);
    Translation2d normalizedLaunchVector = launchVector.div( launchVector.getNorm() );

    
  }

}
