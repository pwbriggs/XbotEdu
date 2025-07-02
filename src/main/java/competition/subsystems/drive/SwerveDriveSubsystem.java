package competition.subsystems.drive;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import xbot.common.advantage.DataFrameRefreshable;
import xbot.common.command.BaseSubsystem;
import xbot.common.controls.actuators.XCANMotorController;
import xbot.common.injection.electrical_contract.CANMotorControllerInfo;
import xbot.common.injection.electrical_contract.DeviceInfo;

import javax.inject.Inject;
import javax.inject.Singleton;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotation;
import static edu.wpi.first.units.Units.RotationsPerSecond;

@Singleton
public class SwerveDriveSubsystem extends BaseSubsystem implements DataFrameRefreshable {

    public XCANMotorController frontLeftDrive;
    public XCANMotorController frontRightDrive;
    public XCANMotorController rearLeftDrive;
    public XCANMotorController rearRightDrive;

    public XCANMotorController frontLeftSteering;
    public XCANMotorController frontRightSteering;
    public XCANMotorController rearLeftSteering;
    public XCANMotorController rearRightSteering;

    SwerveDriveKinematics kinematics;
    public static double metersPerRotation = 0.5;
    public static double maxVelocityMetersPerSecond = 3;
    SwerveDrivePoseEstimator swerveDrivePoseEstimator;

    Rotation2d swerveDriveHeading = Rotation2d.fromDegrees(0);
    Pose2d swerveDrivePose = new Pose2d();

    @Inject
    public SwerveDriveSubsystem(XCANMotorController.XCANMotorControllerFactory motorControllerFactory) {
        frontLeftDrive = motorControllerFactory.create(new CANMotorControllerInfo("FrontLeftDrive", 11), this.getPrefix(), "FrontLeftDrive");
        frontRightDrive = motorControllerFactory.create(new CANMotorControllerInfo("FrontRightDrive", 22), this.getPrefix(), "FrontRightDrive");
        rearLeftDrive = motorControllerFactory.create(new CANMotorControllerInfo("RearLeftDrive", 3), this.getPrefix(), "RearLeftDrive");
        rearRightDrive = motorControllerFactory.create(new CANMotorControllerInfo("RearRightDrive", 4), this.getPrefix(), "RearRightDrive");

        frontLeftDrive.setDistancePerMotorRotationsScaleFactor(Meters.per(Rotation).of(metersPerRotation));
        frontRightDrive.setDistancePerMotorRotationsScaleFactor(Meters.per(Rotation).of(metersPerRotation));
        rearLeftDrive.setDistancePerMotorRotationsScaleFactor(Meters.per(Rotation).of(metersPerRotation));
        rearRightDrive.setDistancePerMotorRotationsScaleFactor(Meters.per(Rotation).of(metersPerRotation));

        frontLeftSteering = motorControllerFactory.create(new CANMotorControllerInfo("FrontLeftSteering", 5), this.getPrefix(), "FrontLeftSteering");
        frontRightSteering = motorControllerFactory.create(new CANMotorControllerInfo("FrontRightSteering", 6), this.getPrefix(), "FrontRightSteering");
        rearLeftSteering = motorControllerFactory.create(new CANMotorControllerInfo("RearLeftSteering", 7), this.getPrefix(), "RearLeftSteering");
        rearRightSteering = motorControllerFactory.create(new CANMotorControllerInfo("RearRightSteering", 8), this.getPrefix(), "RearRightSteering");

        kinematics = new SwerveDriveKinematics(
                new Translation2d(0.5, 0.5),
                new Translation2d(0.5, -0.5),
                new Translation2d(-0.5, 0.5),
                new Translation2d(-0.5, -0.5)
        );

        swerveDrivePoseEstimator = new SwerveDrivePoseEstimator(
                kinematics,
                Rotation2d.fromDegrees(0),
                getSwerveModulePositions(),
                new Pose2d());
    }

    public void move(double xVelocity, double yVelocity, double radiansPerSecond) {
        // This standard WPI library converts velocity/rotation goals
        // into individual wheel angles and speeds. Don't change this code!
        var desiredSwerveModuleStates = kinematics.toSwerveModuleStates(
                new ChassisSpeeds(xVelocity, yVelocity, radiansPerSecond)
        );
        aKitLog.record("DesiredSwerveStates", desiredSwerveModuleStates);

        // From this point on, you will need to:
        // - get each swerve module pointing in the correct direction, according to the desiredSwerveModuleStates
        // - gte each swerve module translating at the correct speed, according to the desiredSwerveModuleStates

        // Note for mentors - all the code below this line in this method
        // should be deleted before releasing to students, as it is the partial implementation
        // of the swerve drive algorithm that students will be implementing.
    }

    // ----------------------------------------------------------
    // Code below here should not be changed; the simulator needs these functions to update the robot's position,
    // or the code is required for other robot infrastructure reasons (e.g. refreshDataFrame).
    // It's worth looking at these functions to understand some swerve aspects, but you don't need to modify them.
    // ----------------------------------------------------------

    public SwerveModuleState[] getSwerveModuleStates() {
        return new SwerveModuleState[]{
                buildSwerveModuleStateFromMotors(frontLeftDrive, frontLeftSteering),
                buildSwerveModuleStateFromMotors(frontRightDrive, frontRightSteering),
                buildSwerveModuleStateFromMotors(rearLeftDrive, rearLeftSteering),
                buildSwerveModuleStateFromMotors(rearRightDrive, rearRightSteering)
        };
    }

    public SwerveModulePosition[] getSwerveModulePositions() {
        return new SwerveModulePosition[]{
                new SwerveModulePosition(frontLeftDrive.getPositionAsDistance(), new Rotation2d(frontLeftSteering.getPosition())),
                new SwerveModulePosition(frontRightDrive.getPositionAsDistance(), new Rotation2d(frontRightSteering.getPosition())),
                new SwerveModulePosition(rearLeftDrive.getPositionAsDistance(), new Rotation2d(rearLeftSteering.getPosition())),
                new SwerveModulePosition(rearRightDrive.getPositionAsDistance(), new Rotation2d(rearRightSteering.getPosition()))
        };
    }

    public SwerveModuleState buildSwerveModuleStateFromMotors(XCANMotorController driveMotor, XCANMotorController steeringMotor) {
        return new SwerveModuleState(
                driveMotor.getVelocity().in(RotationsPerSecond) * metersPerRotation,
                Rotation2d.fromRadians(steeringMotor.getPosition().in(Radians))
        );
    }

    public Pose2d getSwervePose() {
        return swerveDrivePose;
    }

    @Override
    public void periodic() {
        swerveDrivePose = swerveDrivePoseEstimator.update(
                swerveDriveHeading,
                getSwerveModulePositions()
        );

        var speeds = kinematics.toChassisSpeeds(getSwerveModuleStates());
        swerveDriveHeading = swerveDrivePose.getRotation().plus(Rotation2d.fromRadians(speeds.omegaRadiansPerSecond * 0.02));

        aKitLog.record("CurrentPose", swerveDrivePose);
    }

    @Override
    public void refreshDataFrame() {
        frontLeftDrive.refreshDataFrame();
        frontRightDrive.refreshDataFrame();
        rearLeftDrive.refreshDataFrame();
        rearRightDrive.refreshDataFrame();

        frontLeftSteering.refreshDataFrame();
        frontRightSteering.refreshDataFrame();
        rearLeftSteering.refreshDataFrame();
        rearRightSteering.refreshDataFrame();

        aKitLog.record("CurrentSwerveState", getSwerveModuleStates());
    }
}
