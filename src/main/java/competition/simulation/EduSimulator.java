package competition.simulation;

import competition.subsystems.drive.DriveSubsystem;
import competition.subsystems.pose.PoseSubsystem;
import edu.wpi.first.wpilibj.MockDigitalInput;
import xbot.common.controls.actuators.mock_adapters.MockCANSparkMax;

import javax.inject.Inject;

public class EduSimulator {

    PoseSubsystem pose;
    DriveSubsystem drive;
    LinearEngine linearEngine;
    RotationEngine rotationEngine;

    @Inject
    public EduSimulator(PoseSubsystem pose, DriveSubsystem drive) {
        this.pose = pose;
        this.drive = drive;

        linearEngine = new LinearEngine();
        rotationEngine = new RotationEngine();
    }

    public void update() {
        simulateDrive();
    }

    private void simulateDrive() {
        // Translation
        double chassisTotalLinearDistanceTravelled = linearEngine.step(getForwardPower());
        addDistanceToEncoders(chassisTotalLinearDistanceTravelled);

        //Rotation
        rotationEngine.setPower(getRotationalPower());
        rotationEngine.step();

        pose.setCurrentHeading(rotationEngine.getOrientation());
    }

    private double getForwardPower() {
        return (drive.frontLeft.get() + drive.frontRight.get())  / 2.0;
    }

    private double getRotationalPower() {
        return (drive.frontRight.get() - drive.frontLeft.get()) / 2.0;
    }

    private void addDistanceToEncoders(double distance) {
        ((MockCANSparkMax)drive.frontLeft).setPosition(distance);
        ((MockCANSparkMax)drive.frontRight).setPosition(distance);
    }

}
