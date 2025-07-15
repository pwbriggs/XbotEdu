package competition.subsystems.drive.commands;

import javax.inject.Inject;

import xbot.common.command.BaseCommand;
import xbot.common.math.PIDManager;
import xbot.common.math.PIDManager.PIDManagerFactory;
import xbot.common.properties.DoubleProperty;
import xbot.common.properties.PropertyFactory;
import competition.subsystems.drive.DriveSubsystem;
import competition.subsystems.pose.PoseSubsystem;

public class DriveToPositionCommand extends BaseCommand {

    // Subsystems
    final DriveSubsystem drive;
    final PoseSubsystem pose;
    final PIDManager pid;

    // Physics computation numbers
    double targetPosition;
    double previousPosition;

    double error;
    double velocity;

    @Inject
    public DriveToPositionCommand(DriveSubsystem driveSubsystem, PoseSubsystem pose, PIDManagerFactory pidManagerFactory) {
        this.drive = driveSubsystem;
        this.pose = pose;
        this.pid = pidManagerFactory.create("DriveToPoint");

        pid.setEnableErrorThreshold(true); // Turn on distance checking
        pid.setErrorThreshold(0.1);
        pid.setEnableDerivativeThreshold(true); // Turn on speed checking
        pid.setDerivativeThreshold(0.1);

        pid.setP(3);
        pid.setD(100);
    }

    public void setTargetPosition(double newTarget) {
        // This method will be called by the test, and will give you a goal distance.
        // You'll need to remember this target position and use it in your calculations.
        targetPosition = newTarget;
    }

    @Override
    public void initialize() {
        // If you have some one-time setup, do it here.
        pid.reset();
    }

    @Override
    public void execute() {
        double position = pose.getPosition();
        double power = pid.calculate(targetPosition, position);

        drive.tankDrive(power, power);
    }

    @Override
    public boolean isFinished() {
        return pid.isOnTarget();
    }

    public void end() {
        // Cut the motors when we're close enough or the command gets canceled
        drive.tankDrive(0, 0);
    }

}
