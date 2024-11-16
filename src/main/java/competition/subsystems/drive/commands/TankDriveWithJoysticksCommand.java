package competition.subsystems.drive.commands;

import javax.inject.Inject;

import xbot.common.command.BaseCommand;
import competition.operator_interface.OperatorInterface;
import competition.subsystems.drive.DriveSubsystem;

//This first exercise is mostly completed for you. Take your time to read through it,
// review all the comments, and then complete it by adding in a bit more code of your own.

public class TankDriveWithJoysticksCommand extends BaseCommand {

    DriveSubsystem drive;
    OperatorInterface operatorInterface;

    @Inject
    public TankDriveWithJoysticksCommand(DriveSubsystem driveSubsystem, OperatorInterface oi) {
        drive = driveSubsystem;
        operatorInterface = oi;
        this.addRequirements(drive);
    }

    @Override
    public void initialize() {
        // This code is run one time, right when the command is started.
        // You don't need to write any code here for this exercise.
    }

    @Override
    public void execute() {
        // You need to get values from the joysticks and pass them into the motors.

        // Get values from the joysticks:
        // Here's how to get how far the left joystick's Y-axis is pushed:
        double leftValue = operatorInterface.gamepad.getLeftVector().getY();
        // TODO: get how far the RIGHT joystick's Y-axis is pushed as well

        // Pass values into the DriveSubsystem so it can control motors:
        // right now, this just sends the left power to the left part of the drive.
        // You'll need to give it a right power value as well.
        drive.tankDrive(leftValue, 0);
    }

}
