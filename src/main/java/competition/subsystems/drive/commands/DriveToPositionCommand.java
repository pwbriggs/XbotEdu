package competition.subsystems.drive.commands;

import javax.inject.Inject;

import xbot.common.command.BaseCommand;
import xbot.common.properties.DoubleProperty;
import xbot.common.properties.PropertyFactory;
import competition.subsystems.drive.DriveSubsystem;
import competition.subsystems.pose.PoseSubsystem;
import edu.wpi.first.units.measure.Velocity;

public class DriveToPositionCommand extends BaseCommand {

    DriveSubsystem drive;
    PoseSubsystem pose;

    double targetPosition;
    double previousPosition;

    double error;
    double velocity;

    DoubleProperty proportionalConstant;
    DoubleProperty derivativeConstant;

    @Inject
    public DriveToPositionCommand(DriveSubsystem driveSubsystem, PoseSubsystem pose, PropertyFactory propertyFactory) {
        this.drive = driveSubsystem;
        this.pose = pose;
        propertyFactory.setPrefix(this);
        proportionalConstant = propertyFactory.createPersistentProperty("Proportional constant (error)", 1);
        derivativeConstant = propertyFactory.createPersistentProperty("Derivative constant (velocity)", 1);
    }

    public void setTargetPosition(double position) {
        // This method will be called by the test, and will give you a goal distance.
        // You'll need to remember this target position and use it in your calculations.
        targetPosition = position;
        previousPosition = position; // Don't spike the velocity
    }

    @Override
    public void initialize() {
        // If you have some one-time setup, do it here.
    }

    @Override
    public void execute() {
        // Here you'll need to figure out a technique that:
        // - Gets the robot to move to the target position
        // - Hint: use pose.getPosition() to find out where you are
        // - Gets the robot stop (or at least be moving really really slowly) at the
        // target position

        // How you do this is up to you. If you get stuck, ask a mentor or student for
        // some hints!
        double position = pose.getPosition();
        error = targetPosition - position;
        velocity = position - previousPosition;

        double power = error * proportionalConstant.get() + velocity * derivativeConstant.get();

        log.info("Err: {}; Vel: {}; Pos: {}; Tar: {}", error, velocity, position, targetPosition);

        drive.tankDrive(power, power);

        previousPosition = position;
    }

    @Override
    public boolean isFinished() {
        // double position = pose.getPosition();
        // double error = targetPosition - position;
        // double velocity = position - previousPosition;
        // log.info("Err: {}; Vel: {}", error, velocity);
        return ((Math.abs(error) < 0.005) && (Math.abs(velocity) < 0.0001));
    }

    public void end() {
        // Cut the motors when we're close enough or the command gets canceled
        drive.tankDrive(0, 0);
    }

}
