package competition.subsystems.drive.commands;

import javax.inject.Inject;

import xbot.common.math.PIDManager.PIDManagerFactory;
import xbot.common.properties.DoubleProperty;
import xbot.common.properties.PropertyFactory;
import xbot.common.subsystems.drive.control_logic.HeadingModule.HeadingModuleFactory;
import competition.subsystems.drive.DriveSubsystem;
import competition.subsystems.pose.PoseSubsystem;

public class TurnLeft90DegreesCommand extends DriveToOrientationCommand {

    @Inject
    public TurnLeft90DegreesCommand(DriveSubsystem driveSubsystem, PoseSubsystem pose, HeadingModuleFactory headingModuleFactory, PIDManagerFactory pidManagerFactory) {
        super(driveSubsystem, pose, headingModuleFactory, pidManagerFactory);
    }

    @Override
    public void initialize() {
        setTargetHeading(pose.getCurrentHeading().getDegrees() + 90);
    }
}
