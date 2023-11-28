package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.SwerveSubsystem;

public class Shuffleboard {

    private ShuffleboardTab tab = Shuffleboard.getTab("Vision");
    private NetworkTableEntry distanceEntry =
        tab.add("Distance to target", 0)
           .getEntry();
           
    public Shuffleboard(){
        SmartDashboard.putNumber("Heading", SwerveSubsystem.getHeading());
        SmartDashboard.putString("Robot Location", SwerveSubsystem.getPose().getTranslation().toString());
    }

}
