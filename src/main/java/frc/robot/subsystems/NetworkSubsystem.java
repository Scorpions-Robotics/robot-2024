
package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.ConnectionInfo;
// import edu.wpi.first.networktables.
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class NetworkSubsystem extends SubsystemBase {

  double apriltag_x_value;
  double apriltag_y_value;

  public NetworkTableInstance inst = NetworkTableInstance.getDefault();
  //table
  public NetworkTable table = inst.getTable("vision");

  //subtable
  public NetworkTable networkTable = table.getSubTable("apriltag");

  //entry
  public NetworkTableEntry apriltag_x = networkTable.getEntry("apriltag_x");
  public NetworkTableEntry apriltag_y = networkTable.getEntry("apriltag_y");

  public NetworkSubsystem() {
   // inst.setServerTeam(7672);
    //inst.startServer("10.76.72.2");
    inst.setServerTeam(7672);
  }

  public double getX(){
    return apriltag_x_value;
  }

  public double getY(){
      return apriltag_y_value;
   }

  @Override
  public void periodic() {
    apriltag_x_value = apriltag_x.getDouble(0.0);
    apriltag_y_value = apriltag_y.getDouble(0.0);
    SmartDashboard.putNumber("apriltag x value", apriltag_x_value);
    SmartDashboard.putNumber("apriltag y value", apriltag_y_value);
    SmartDashboard.putBoolean("connection",inst.isConnected());
  }


}
