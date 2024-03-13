
package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class NetworkSubsystem extends SubsystemBase {

  String getstr;

  public NetworkTableInstance inst = NetworkTableInstance.create();
  //table
  public NetworkTable table = inst.getTable("table");

  //subtable
  public NetworkTable networkTable = table.getSubTable("Subtable");

  //entry
  public NetworkTableEntry veri = networkTable.getEntry("veri");

  public NetworkSubsystem() {
    inst.startClient4("10.0.0.0");
  }

  public String return_value(){
    return getstr;
    }

  @Override
  public void periodic() {
    getstr = veri.getString("0.0");
    System.out.println(getstr);  
  }


}
