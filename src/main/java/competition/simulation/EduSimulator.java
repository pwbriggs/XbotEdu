package competition.simulation;

import competition.subsystems.drive.DriveSubsystem;
import competition.subsystems.drive.SwerveDriveSubsystem;
import competition.subsystems.pose.PoseSubsystem;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import xbot.common.controls.actuators.mock_adapters.MockCANMotorController;

import javax.inject.Inject;
import javax.inject.Singleton;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;

@Singleton
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
        return (MathUtil.clamp(drive.frontLeft.getPower(), -1, 1) + MathUtil.clamp(drive.frontRight.getPower(), -1, 1))  / 2.0;
    }

    private double getRotationalPower() {
        return (MathUtil.clamp(drive.frontRight.getPower(), -1, 1) - MathUtil.clamp(drive.frontLeft.getPower(), -1, 1)) / 2.0;
    }

    private void addDistanceToEncoders(double distance) {
        ((MockCANMotorController)drive.frontLeft).setPosition(Rotations.of(distance));
        ((MockCANMotorController)drive.frontRight).setPosition(Rotations.of(distance));
    }

    public void reset() {
        linearEngine.reset();
    }


    public void simulateSwerve() {
        // Get voltages from swerve subsystem and apply them to the simulated object, then
        // feed the outputs of the simulated object into the mock motors

        frontLeftSimModule.update();
        frontRightSimModule.update();
        rearLeftSimModule.update();
        rearRightSimModule.update();

        // Set new inputs to the simulated motors for next loop
        frontLeftSimModule.setDrivePower(swerve.frontLeftDrive.getPower());
        frontRightSimModule.setDrivePower(swerve.frontRightDrive.getPower());
        rearLeftSimModule.setDrivePower(swerve.rearLeftDrive.getPower());
        rearRightSimModule.setDrivePower(swerve.rearRightDrive.getPower());

        frontLeftSimModule.setSteeringPower(swerve.frontLeftSteering.getPower());
        frontRightSimModule.setSteeringPower(swerve.frontRightSteering.getPower());
        rearLeftSimModule.setSteeringPower(swerve.rearLeftSteering.getPower());
        rearRightSimModule.setSteeringPower(swerve.rearRightSteering.getPower());

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
        ((MockCANMotorController)swerve.frontLeftDrive).setVelocity(RadiansPerSecond.of(frontLeftSimModule.getDriveVelocityRad() / radiansPerMeter));
        ((MockCANMotorController)swerve.frontRightDrive).setVelocity(RadiansPerSecond.of(frontRightSimModule.getDriveVelocityRad() / radiansPerMeter));
        ((MockCANMotorController)swerve.rearLeftDrive).setVelocity(RadiansPerSecond.of(rearLeftSimModule.getDriveVelocityRad() / radiansPerMeter));
        ((MockCANMotorController)swerve.rearRightDrive).setVelocity(RadiansPerSecond.of(rearRightSimModule.getDriveVelocityRad() / radiansPerMeter));

        ((MockCANMotorController)swerve.frontLeftDrive).setPosition(Radians.of(frontLeftSimModule.getDrivePositionRad() / radiansPerMeter));
        ((MockCANMotorController)swerve.frontRightDrive).setPosition(Radians.of(frontRightSimModule.getDrivePositionRad() / radiansPerMeter));
        ((MockCANMotorController)swerve.rearLeftDrive).setPosition(Radians.of(rearLeftSimModule.getDrivePositionRad() / radiansPerMeter));
        ((MockCANMotorController)swerve.rearRightDrive).setPosition(Radians.of(rearRightSimModule.getDrivePositionRad() / radiansPerMeter));


        ((MockCANMotorController)swerve.frontLeftSteering).setPosition(Radians.of(frontLeftSimModule.getTurnAbsolutePositionRad()));
        ((MockCANMotorController)swerve.frontRightSteering).setPosition(Radians.of(frontRightSimModule.getTurnAbsolutePositionRad()));
        ((MockCANMotorController)swerve.rearLeftSteering).setPosition(Radians.of(rearLeftSimModule.getTurnAbsolutePositionRad()));
        ((MockCANMotorController)swerve.rearRightSteering).setPosition(Radians.of(rearRightSimModule.getTurnAbsolutePositionRad()));
    }

}
