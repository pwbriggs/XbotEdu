package competition.simulation;

import competition.subsystems.drive.DriveSubsystem;
import competition.subsystems.drive.SwerveDriveSubsystem;
import competition.subsystems.pose.PoseSubsystem;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.MockDigitalInput;
import xbot.common.controls.actuators.mock_adapters.MockCANSparkMax;

import javax.inject.Inject;

public class EduSimulator {

    PoseSubsystem pose;
    DriveSubsystem drive;
    LinearEngine linearEngine;
    RotationEngine rotationEngine;


    SwerveDriveSubsystem swerve;
    SimSwerveModule frontLeftSimModule;
    SimSwerveModule frontRightSimModule;
    SimSwerveModule rearLeftSimModule;
    SimSwerveModule rearRightSimModule;

    boolean forceDriverStationEnabledForUnitTests = false;

    @Inject
    public EduSimulator(PoseSubsystem pose, DriveSubsystem drive, SwerveDriveSubsystem swerve) {
        this.pose = pose;
        this.drive = drive;
        this.swerve = swerve;

        linearEngine = new LinearEngine();
        rotationEngine = new RotationEngine();

        frontLeftSimModule = new SimSwerveModule();
        frontRightSimModule = new SimSwerveModule();
        rearLeftSimModule = new SimSwerveModule();
        rearRightSimModule = new SimSwerveModule();
    }

    public void setForceDriverStationEnabledForUnitTests(boolean forceDriverStationEnabledForUnitTests) {
        this.forceDriverStationEnabledForUnitTests = forceDriverStationEnabledForUnitTests;
    }

    public void update() {
        simulateDrive();
        simulateSwerve();
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


    public void simulateSwerve() {
        // Get voltages from swerve subsystem and apply them to the simulated object, then
        // feed the outputs of the simulated object into the mock motors

        frontLeftSimModule.update();
        frontRightSimModule.update();
        rearLeftSimModule.update();
        rearRightSimModule.update();

        // Set new inputs to the simulated motors for next loop
        frontLeftSimModule.setDrivePower(swerve.frontLeftDrive.get());
        frontRightSimModule.setDrivePower(swerve.frontRightDrive.get());
        rearLeftSimModule.setDrivePower(swerve.rearLeftDrive.get());
        rearRightSimModule.setDrivePower(swerve.rearRightDrive.get());

        frontLeftSimModule.setSteeringPower(swerve.frontLeftSteering.get());
        frontRightSimModule.setSteeringPower(swerve.frontRightSteering.get());
        rearLeftSimModule.setSteeringPower(swerve.rearLeftSteering.get());
        rearRightSimModule.setSteeringPower(swerve.rearRightSteering.get());

        // If the robot is disabled, the real robot would have all its outputs locked out by firmware.
        // We emulate this by setting the simulated motor outputs to 0.
        // (However, unit tests don't enable the robot, so we also listen to the flag "forceDriverStationEnabledForUnitTests"
        // to allow the unit tests to run)
        if (!DriverStation.isEnabled() && !forceDriverStationEnabledForUnitTests) {
            // if disabled, set voltages to 0.
            frontLeftSimModule.setDrivePower(0);
            frontRightSimModule.setDrivePower(0);
            rearLeftSimModule.setDrivePower(0);
            rearRightSimModule.setDrivePower(0);

            frontLeftSimModule.setSteeringPower(0);
            frontRightSimModule.setSteeringPower(0);
            rearLeftSimModule.setSteeringPower(0);
            rearRightSimModule.setSteeringPower(0);
        }

        double radiansPerMeter = 30;

        // Read outputs from the simulated motors
        ((MockCANSparkMax)swerve.frontLeftDrive).setVelocity(frontLeftSimModule.getDriveVelocityRad() / radiansPerMeter);
        ((MockCANSparkMax)swerve.frontRightDrive).setVelocity(frontRightSimModule.getDriveVelocityRad() / radiansPerMeter);
        ((MockCANSparkMax)swerve.rearLeftDrive).setVelocity(rearLeftSimModule.getDriveVelocityRad() / radiansPerMeter);
        ((MockCANSparkMax)swerve.rearRightDrive).setVelocity(rearRightSimModule.getDriveVelocityRad() / radiansPerMeter);

        ((MockCANSparkMax)swerve.frontLeftDrive).setPosition(frontLeftSimModule.getDrivePositionRad() / radiansPerMeter);
        ((MockCANSparkMax)swerve.frontRightDrive).setPosition(frontRightSimModule.getDrivePositionRad() / radiansPerMeter);
        ((MockCANSparkMax)swerve.rearLeftDrive).setPosition(rearLeftSimModule.getDrivePositionRad() / radiansPerMeter);
        ((MockCANSparkMax)swerve.rearRightDrive).setPosition(rearRightSimModule.getDrivePositionRad() / radiansPerMeter);


        ((MockCANSparkMax)swerve.frontLeftSteering).setPosition(frontLeftSimModule.getTurnAbsolutePositionRad());
        ((MockCANSparkMax)swerve.frontRightSteering).setPosition(frontRightSimModule.getTurnAbsolutePositionRad());
        ((MockCANSparkMax)swerve.rearLeftSteering).setPosition(rearLeftSimModule.getTurnAbsolutePositionRad());
        ((MockCANSparkMax)swerve.rearRightSteering).setPosition(rearRightSimModule.getTurnAbsolutePositionRad());
    }

}
