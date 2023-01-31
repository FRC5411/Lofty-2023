//Root Package
package frc.robot.subsystems;

//Libraries
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Limelight Class
 */
public class Limelight extends SubsystemBase 
{
  public Limelight() {}

  /**
   * Return Yaw as Double
   */
  public double getyaw() {return NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);}
  /**
   * Return Pitch as Double
   */
  public double getpitch() {return NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);}

  @Override
  public void periodic() {}

  @Override
  public void simulationPeriodic() {}
}
