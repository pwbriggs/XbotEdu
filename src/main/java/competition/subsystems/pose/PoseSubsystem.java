package competition.subsystems.pose;

import javax.inject.Inject;
import javax.inject.Singleton;

import competition.subsystems.drive.DriveSubsystem;
import xbot.common.controls.sensors.XGyro.XGyroFactory;
import xbot.common.properties.PropertyFactory;
import xbot.common.subsystems.pose.BasePoseSubsystem;

import static edu.wpi.first.units.Units.Rotations;

@Singleton
public class PoseSubsystem extends BasePoseSubsystem {

    private final DriveSubsystem drive;

    public double scalingFactorFromRotationsToMeters = 0.5;
    
    @Inject
    public PoseSubsystem(XGyroFactory gyroFactory, PropertyFactory propManager, DriveSubsystem drive) {
        super(gyroFactory, propManager);
        this.drive = drive;
    }

    public double getPosition() {
        return (getLeftDriveDistance() + getRightDriveDistance()) / 2.0; 
    }

    @Override
    protected double getLeftDriveDistance() {
        return drive.frontLeft.getPosition().in(Rotations) * scalingFactorFromRotationsToMeters;
    }

    @Override
    protected double getRightDriveDistance() {
        return drive.frontRight.getPosition().in(Rotations) * scalingFactorFromRotationsToMeters;
    }

    @Override
    public void periodic() {
        super.periodic();
        aKitLog.record("Location", this.getCurrentPose2d());
    }
}