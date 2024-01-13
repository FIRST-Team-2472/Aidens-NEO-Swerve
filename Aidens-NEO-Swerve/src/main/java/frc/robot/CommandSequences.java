package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.SwerveSubsystem;

public class CommandSequences {
    public Command robot1Command(SwerveSubsystem swerveSubsystem) {
        swerveSubsystem.resetOdometry(null);

        return new SequentialCommandGroup(
            
        );
    }

    public Command robot2Command(SwerveSubsystem swerveSubsystem) {
        swerveSubsystem.resetOdometry(null);

        return new SequentialCommandGroup(
            
        );
    }

    public Command robot3Command(SwerveSubsystem swerveSubsystem) {
        swerveSubsystem.resetOdometry(null);

        return new SequentialCommandGroup(
            
        );
    }

}
