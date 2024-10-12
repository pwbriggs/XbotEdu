package competition.subsystems;

import competition.subsystems.drive.DriveSubsystem;
import competition.subsystems.drive.SwerveDriveSubsystem;
import competition.subsystems.drive.commands.SwerveDriveWithJoysticksCommand;
import competition.subsystems.drive.commands.TankDriveWithJoysticksCommand;

import javax.inject.Inject;
import javax.inject.Singleton;

@Singleton
public class SubsystemDefaultCommandMap {
    // For setting the default commands on subsystems

    @Inject
    public SubsystemDefaultCommandMap() {}

    @Inject
    public void setupDriveSubsystem(DriveSubsystem driveSubsystem, TankDriveWithJoysticksCommand command) {
        driveSubsystem.setDefaultCommand(command);
    }

    @Inject
    public void setupSwerveDriveSubsystem(SwerveDriveSubsystem swerveDriveSubsystem, SwerveDriveWithJoysticksCommand command) {
        swerveDriveSubsystem.setDefaultCommand(command);
    }
}
