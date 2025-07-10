package competition.subsystems.drive.commands;

import javax.inject.Inject;

import xbot.common.command.BaseCommand;
import xbot.common.properties.DoubleProperty;
import xbot.common.properties.PropertyFactory;
import competition.subsystems.drive.DriveSubsystem;
import competition.subsystems.pose.PoseSubsystem;

public class TurnLeft90DegreesCommand extends BaseCommand {

    DriveSubsystem drive;
    PoseSubsystem pose;

    double targetPosition;
    double previousPosition;

    double error;
    double velocity;

    DoubleProperty proportionalConstant;
    DoubleProperty derivativeConstant;

    @Inject
    public TurnLeft90DegreesCommand(DriveSubsystem driveSubsystem, PoseSubsystem pose, PropertyFactory propertyFactory) {
        this.drive = driveSubsystem;
        this.pose = pose;
        propertyFactory.setPrefix(this);
        proportionalConstant = propertyFactory.createPersistentProperty("Proportional constant (angular error)", 1);
        derivativeConstant = propertyFactory.createPersistentProperty("Derivative constant (angular velocity)", 1);
    }

    @Override
    public void initialize() {
        targetPosition = pose.getCurrentHeading().getDegrees() + 90;
        // Wrap around. According to .getDegrees() docs, range is exclusive on the negative end.
        if (targetPosition <= -180) { // We shouldn't need this one if we're only ever turning left
            targetPosition += 360;
        } else if (targetPosition > 180) {
            targetPosition -= 360;
        }
    }

    @Override
    public void execute() {
        double position = pose.getCurrentHeading().getDegrees();

        error = targetPosition - position; // Assume we don't need to turn across -180; we'll update this value in a moment.

        if (error > 180) { // It wants us to turn too far to the left,
            error -= 360;  // so let's turn right to the same spot instead
        } else if (error < -180) { // And vice-versa
            error += 360;
        }

        velocity = position - previousPosition;

        if (velocity > 180) { // If our velocity reads super high, we probably just turned over -180,
            velocity -= 360;  // so lets wrap around our reading as well.
        } else if (velocity < -180) { // And vice-versa
            velocity += 360;
        }

        double power = error * proportionalConstant.get() + velocity * derivativeConstant.get();

        log.info("Err: {}; Vel: {}; Pos: {}; Tar: {}", error, velocity, position, targetPosition);

        drive.tankDrive(-power, power);

        previousPosition = position;
    }

    @Override
    public boolean isFinished() {
        // log.info("Err: {}; Vel: {}", error, velocity);
        return ((Math.abs(error) < 0.5) && (Math.abs(velocity) < 0.1));
    }
}
