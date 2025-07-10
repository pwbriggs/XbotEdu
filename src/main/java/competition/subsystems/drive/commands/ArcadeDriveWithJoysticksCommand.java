package competition.subsystems.drive.commands;

import javax.inject.Inject;

import competition.operator_interface.OperatorInterface;
import xbot.common.command.BaseCommand;
import competition.subsystems.drive.DriveSubsystem;

public class ArcadeDriveWithJoysticksCommand extends BaseCommand {

    OperatorInterface operatorInterface;
    DriveSubsystem drive;

    @Inject
    public ArcadeDriveWithJoysticksCommand(DriveSubsystem driveSubsystem, OperatorInterface oi) {
        this.operatorInterface = oi;
        this.drive = driveSubsystem;
        this.addRequirements(drive);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        double longitudinalJoystick = operatorInterface.gamepad.getLeftVector().getY();
        double lateralJoystick = operatorInterface.gamepad.getLeftVector().getX();
        double rTriggerValue = operatorInterface.gamepad.getRightTrigger();
        
        // log.info("lat: {} / lon: {}", lateralJoystick, longitudinalJoystick);

        drive.tankDrive(
            longitudinalJoystick - lateralJoystick,
            longitudinalJoystick + lateralJoystick
        ); // We might pass this values greater than 1, but it seems like it can handle this.

        // Variable precision control
        drive.setMotorMultiplier(1 - 0.9 * rTriggerValue);
    }

}
