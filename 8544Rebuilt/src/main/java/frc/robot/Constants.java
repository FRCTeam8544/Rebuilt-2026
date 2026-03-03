package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

public final class Constants {

  public static final int tickUpdatesPerSecond = 1000 / 20;
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;
  public static final double kNominalVoltage = 12.0;

  public static final class Neo {
    public static final double nominalVoltage = kNominalVoltage;;
    public static final double motorKv = 473;
    public static final double nominalFF = 1.0 / Neo.motorKv;
    public static final double freeSpeedRPM = 5676;
  }

  public static final class Neo550 {
    public static final double nominalVoltage = kNominalVoltage;
    public static final double motorKV = 917;
    public static final double nominalFF = 1.0 / motorKV;
    public static final double freeSpeedRPM = 11000;
  }

  public static final class NeoVortex {
    public static final double nominalVoltage = kNominalVoltage;
    public static final double motorKV = 565;
    public static final double nominalFF = 1.0 / motorKV;
    public static final double maxFreeRpm = 6784;
    public static final double freeSpeedRPM = 6784;
  }

  public static final class KrakenX60 {
    public static final double nominalVoltage = kNominalVoltage;
    public static final double motorKV = 505;
    public static final double nominalFF = 1.0 / motorKV;
    public static final double freeSpeedRPM = 5640; // With FOC enabled
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
