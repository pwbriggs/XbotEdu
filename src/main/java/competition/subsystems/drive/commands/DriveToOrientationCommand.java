package competition.subsystems.drive.commands;

import javax.inject.Inject;

import xbot.common.command.BaseCommand;
import xbot.common.properties.DoubleProperty;
import xbot.common.properties.PropertyFactory;
import competition.subsystems.drive.DriveSubsystem;
import competition.subsystems.pose.PoseSubsystem;

public class DriveToOrientationCommand extends BaseCommand {

    DriveSubsystem drive;
    PoseSubsystem pose;

    double targetPosition;

    double error;
    double previousError;

    double velocity;

    DoubleProperty proportionalConstant;
    DoubleProperty derivativeConstant;

    @Inject
    public DriveToOrientationCommand(DriveSubsystem driveSubsystem, PoseSubsystem pose, PropertyFactory propertyFactory) {
        this.drive = driveSubsystem;
        this.pose = pose;
        propertyFactory.setPrefix(this);
        proportionalConstant = propertyFactory.createPersistentProperty("Proportional constant (angular error)", 1);
        derivativeConstant = propertyFactory.createPersistentProperty("Derivative constant (angular velocity)", -10);
        this.addRequirements(drive);
    }

    @Override
    public void initialize() {
    }

    public void setTargetHeading(double target) {
        targetPosition = target % 360; // Handle multi-revolution numbers
        // Wrap around to between -180 and 180, the FoR this class uses.
        if (targetPosition <= -180) {
            targetPosition += 360;
        } else if (targetPosition > 180) {
            targetPosition -= 360;
        }
    }

    @Override
    public void execute() {
        double position = pose.getCurrentHeading().getDegrees();

        // Assume for now we don't need to turn across -180; we'll update this value in a moment.
        error = targetPosition - position;

        if (error > 180) { // It wants us to turn too far to the left,
            error -= 360;  // so let's turn right to the same spot instead
        } else if (error < -180) { // And vice-versa
            error += 360;
        }

        velocity = error - previousError;

        double power = error * proportionalConstant.get() + velocity * derivativeConstant.get();

        log.info("Err: {}; Vel: {}; Pos: {}; Tar: {}", error, velocity, position, targetPosition);

        drive.tankDrive(-power, power);

        previousError = error;
    }

    @Override
    public boolean isFinished() {
        // log.info("Err: {}; Vel: {}", error, velocity);
        return ((Math.abs(error) < 0.5) && (Math.abs(velocity) < 0.1));
    }
}
