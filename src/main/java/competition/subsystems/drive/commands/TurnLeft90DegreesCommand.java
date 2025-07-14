package competition.subsystems.drive.commands;

import javax.inject.Inject;

import xbot.common.properties.DoubleProperty;
import xbot.common.properties.PropertyFactory;
import competition.subsystems.drive.DriveSubsystem;
import competition.subsystems.pose.PoseSubsystem;

public class TurnLeft90DegreesCommand extends DriveToOrientationCommand {

    @Inject
    public TurnLeft90DegreesCommand(DriveSubsystem driveSubsystem, PoseSubsystem pose, PropertyFactory propertyFactory) {
        super(driveSubsystem, pose, propertyFactory);
    }

    @Override
    public void initialize() {
        setTargetHeading(pose.getCurrentHeading().getDegrees() + 90);
    }
}
