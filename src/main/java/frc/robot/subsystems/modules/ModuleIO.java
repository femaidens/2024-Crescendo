import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import monologue.Logged;


public interface ModuleIO extends Logged{
 // @AutoLog
 class ModuleIOInputs {
  public boolean driveMotorConnected = true;
  public boolean turnMotorConnected = true;
  public boolean hasCurrentControl = false;

  public double drivePositionRads = 0.0;
  public double driveVelocityRadsPerSec = 0.0;
  public double driveAppliedVolts = 0.0;
  public double driveSupplyCurrentAmps = 0.0;
  public double driveTorqueCurrentAmps = 0.0;

  public Rotation2d turnAbsolutePosition = new Rotation2d();
  public Rotation2d turnPosition = new Rotation2d();
  public double turnVelocityRadsPerSec = 0.0;
  public double turnAppliedVolts = 0.0;
  public double turnSupplyCurrentAmps = 0.0;
  public double turnTorqueCurrentAmps = 0.0;

  public double[] odometryDrivePositionsMeters = new double[] {};
  public Rotation2d[] odometryTurnPositions = new Rotation2d[] {};
 }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ModuleIOInputs inputs) {}

  /** Run the drive motor at the specified voltage. */
  public default void setDriveVoltage(double volts) {}

  /** Run the turn motor at the specified voltage. */
  public default void setTurnVoltage(double volts) {}

 
}