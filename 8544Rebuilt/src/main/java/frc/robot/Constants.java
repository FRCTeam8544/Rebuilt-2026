package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

   public static final class Neo550 {
    public static final double nominalVoltage = 12;
    public static final double motorKV = 917;
    public static final double nominalFF = 1.0 / motorKV;
  }

  public static final class NeoVortex {
    public static final double nominalVoltage = 12;
    public static final double motorKV = 565;
    public static final double nominalFF = 1.0 / motorKV;
  }

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }
  // public static final int FrontLeftTurnCanid = 1;

}
