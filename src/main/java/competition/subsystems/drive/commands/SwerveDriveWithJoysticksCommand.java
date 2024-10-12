package competition.subsystems.drive.commands;

import competition.operator_interface.OperatorInterface;
import competition.subsystems.drive.SwerveDriveSubsystem;
import xbot.common.command.BaseCommand;

import javax.inject.Inject;

public class SwerveDriveWithJoysticksCommand extends BaseCommand {

    SwerveDriveSubsystem swerveDrive;
    OperatorInterface oi;

    @Inject
    public SwerveDriveWithJoysticksCommand(SwerveDriveSubsystem driveSubsystem, OperatorInterface oi) {
        this.swerveDrive = driveSubsystem;
        this.addRequirements(driveSubsystem);
        this.oi = oi;
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        // Get some kind of X, Y, and rotation values from the joysticks.
        // Remember the robot standard coordinates/frame of reference:
        // +X: forward. -X: backward.
        // +Y: left. -Y: right.
        // +rotation: to the left (counter-clockwise). -rotation: to the right (clockwise).

        // Scale these values from "percentages" to "velocity in meters per second".
        // The drive has a maximum speed, you can get it via SwerveDriveSubsystem.maxVelocity.
        // For rotation, scale the values from "percentages" to "velocity in radians per second".

        swerveDrive.move(0,0, 0);
    }
}
