package competition.injection.components;

import competition.operator_interface.OperatorCommandMap;
import competition.operator_interface.OperatorInterface;
import competition.simulation.EduSimulator;
import competition.subsystems.SubsystemDefaultCommandMap;
import competition.subsystems.drive.SwerveDriveSubsystem;
import competition.subsystems.drive.commands.ArcadeDriveWithJoysticksCommand;
import competition.subsystems.drive.commands.DriveToOrientationCommand;
import competition.subsystems.drive.commands.DriveToPositionCommand;
import competition.subsystems.drive.commands.TankDriveWithJoysticksCommand;
import competition.subsystems.drive.commands.TogglePrecisionDriveCommand;
import competition.subsystems.drive.commands.TurnLeft90DegreesCommand;
import xbot.common.injection.components.BaseComponent;

public abstract class BaseRobotComponent extends BaseComponent {
    public abstract SubsystemDefaultCommandMap subsystemDefaultCommandMap();

    public abstract OperatorCommandMap operatorCommandMap();

    public abstract OperatorInterface operatorInterface();

    public abstract ArcadeDriveWithJoysticksCommand arcadeDriveWithJoysticksCommand();
    public abstract TankDriveWithJoysticksCommand tankDriveWithJoysticksCommand();
    public abstract TogglePrecisionDriveCommand togglePrecisionDriveCommand();
    public abstract DriveToPositionCommand driveToPositionCommand();
    public abstract TurnLeft90DegreesCommand turnLeft90DegreesCommand();
    public abstract DriveToOrientationCommand driveToOrientationCommand();
    public abstract EduSimulator eduSimulator();
    public abstract SwerveDriveSubsystem swerve();
}
