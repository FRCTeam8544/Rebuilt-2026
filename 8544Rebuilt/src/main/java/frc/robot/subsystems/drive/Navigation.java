// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

// import frc.robot.util.LogUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.subsystems.vision.VisionConstants;
import java.util.ArrayList;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class Navigation {

  // GameConsts
  // BLUE Alliance
  public static final int[] blueHubAprilTagIDs = {27, 18, 19, 20, 26, 25, 21, 24};
  // RED Alliance
  public static final int[] redHubAprilTagsIDs = {5, 2, 8, 9, 10, 11, 4, 3};
  public static final int numHubfaceTags = 8;
  // End GameConsts

  private Alliance alliance;
  private int currentTargetTag = -1; // No target
  private double bestApproachAngle = 0.0;

  private Supplier<Pose2d> robotPoseSupplier;

  // Reef data
  private ArrayList<Pose2d> HubfacePoses = new ArrayList<Pose2d>();
  private ArrayList<Rotation2d> HubAngles = new ArrayList<Rotation2d>();
  private ArrayList<Integer> HubTags = new ArrayList<Integer>();
  private Translation2d HubCentroid;
  Translation2d blueHubCentroid;
  Translation2d redHubCentroid;
  public DoubleSupplier approachAngleSupplier =
      () -> {
        return bestApproachAngle;
      };

  public Navigation(Supplier<Pose2d> robotPoseSupplier) {
    this.robotPoseSupplier = robotPoseSupplier;
    blueHubCentroid = initializeHubpose(blueHubAprilTagIDs);
    redHubCentroid = initializeHubpose(redHubAprilTagsIDs);
  }

  public Rotation2d getAnglefromHub(DriverStation.Alliance alliance) {
    // This method will be called once per scheduler run
    // navigationIO.updateInputs(climberInOutData);
    Translation2d HubTargetpose;
    final Translation2d currentPose = robotPoseSupplier.get().getTranslation();
    if (alliance == Alliance.Blue) {
      HubTargetpose = blueHubCentroid;

    } else {
      HubTargetpose = redHubCentroid;
    }
    Translation2d clockVector = redHubCentroid.minus(currentPose);
    return clockVector.getAngle();

    // Log summary data
    // log current pose, selected tag, approach angle
    // LogUtil.logData("Climber", climberInOutData);
  }

  // Get the Pose2d for the given tag number if it exists
  private Optional<Pose2d> getTagPose(int tagNumber) {
    Optional<Pose3d> tagOption = VisionConstants.aprilTagLayout.getTagPose(tagNumber);
    if (tagOption.isPresent()) {
      Pose2d thePose =
          new Pose2d(
              tagOption.get().getX(),
              tagOption.get().getY(),
              tagOption.get().getRotation().toRotation2d());
      return Optional.of(thePose);
    } else {
      return Optional.empty();
    }
  }

  // Return a field relative rotation that the robot should use to "aim" at a target
  private Rotation2d getBestApproachAngle(Pose2d targetPose) {
    // Add filtering on position to avoid tag flipping... TODO

    // Assume that tag pose "facing out" from field element, so flip it to be as the robot sees it.
    return targetPose.getRotation().unaryMinus();
  }

  // TODO This is game specific and may belong elsewhere
  private Translation2d initializeHubpose(int[] HubtagArray) {
    // TODO add other areas of interest...

    // Determine the centroid (field reference frame, blue origin) and tag poses of the provided
    // alliance's reef
    Double xCentroid = 0.0;
    double yCentroid = 0.0;
    double tagCount = 0.0;

    for (int tagIdx = 0; tagIdx < numHubfaceTags; ++tagIdx) {
      final int lookupTag = HubtagArray[tagIdx];
      Optional<Pose3d> tagOption = VisionConstants.aprilTagLayout.getTagPose(lookupTag);
      if (tagOption.isPresent()) {
        xCentroid += tagOption.get().getX();
        yCentroid += tagOption.get().getY();
        tagCount += 1.0;

        HubfacePoses.add(getTagPose(lookupTag).get());
      }
      // tagOption.ifPresent(thePose -> { xCentroid += thePose.getX();
      //                                 yCentroid += thePose.getY();
      //                                tagCount += 1.0; });
    }

    if (tagCount > 0.0) {
      xCentroid = xCentroid / tagCount;
      yCentroid = yCentroid / tagCount;
    }
    return HubCentroid = new Translation2d(xCentroid, yCentroid);
  }
}
