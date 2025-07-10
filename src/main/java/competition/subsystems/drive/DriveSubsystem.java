package competition.subsystems.drive;

import javax.inject.Inject;
import javax.inject.Singleton;

import competition.electrical_contract.ElectricalContract;
import edu.wpi.first.math.MathUtil;
import xbot.common.advantage.AKitLogger;
import xbot.common.advantage.DataFrameRefreshable;
import xbot.common.controls.actuators.XCANMotorController;
import xbot.common.injection.electrical_contract.CANMotorControllerInfo;
import xbot.common.math.PIDManager;
import xbot.common.math.XYPair;
import xbot.common.properties.DoubleProperty;
import xbot.common.properties.PropertyFactory;
import xbot.common.subsystems.drive.BaseDriveSubsystem;

@Singleton
public class DriveSubsystem extends BaseDriveSubsystem implements DataFrameRefreshable {

    public final XCANMotorController frontLeft;
    public final XCANMotorController frontRight;
    double motorMultiplier = 1.0; // default to full power

    DoubleProperty dp;

    @Inject
    public DriveSubsystem(XCANMotorController.XCANMotorControllerFactory motorControllerFactory, ElectricalContract electricalContract, PropertyFactory pf) {
        log.info("Creating DriveSubsystem");
        // instantiate speed controllers and sensors here, save them as class members

        this.frontLeft = motorControllerFactory
                .create(new CANMotorControllerInfo("FrontLeft", 1), this.getPrefix(), "FrontLeft");
        this.frontRight = motorControllerFactory
                .create(new CANMotorControllerInfo("FrontRight", 2), this.getPrefix(), "FrontRight");

        this.motorMultiplier = 1.0; // init with no precision mode

        pf.setPrefix(this);
        dp = pf.createPersistentProperty("DriveSubsystem", 1.5);
    }

    public void tankDrive(double leftInput, double rightInput) {
        // You'll need to take these power values and assign them to all of the motors.
        // As an example, here is some code that has the frontLeft motor to spin
        // according to the value of leftPower:
        frontLeft.setPower(leftInput * motorMultiplier);
        frontRight.setPower(rightInput * motorMultiplier);
    }

    public double getMotorMultiplier() {
        return motorMultiplier;
    }

    public void setMotorMultiplier(double multiplier) {
        motorMultiplier = MathUtil.clamp(multiplier, -1.0 1.0);
    }


    // You can ignore all of the code undearneat this comment -->
    // You won't have to touch it.

    @Override
    public PIDManager getPositionalPid() {
        // TODO: Auto-generated method stub
        return null;
    }

    @Override
    public PIDManager getRotateToHeadingPid() {
        // TODO: Auto-generated method stub
        return null;
    }

    @Override
    public PIDManager getRotateDecayPid() {
        // TODO: Auto-generated method stub
        return null;
    }

    @Override
    public void move(XYPair translate, double rotate) {
       throw new RuntimeException("Not yet implemented");
    }

    @Override
    public double getLeftTotalDistance() {
        // TODO: Auto-generated method stub
        return 0;
    }

    @Override
    public double getRightTotalDistance() {
        // TODO: Auto-generated method stub
        return 0;
    }

    @Override
    public double getTransverseDistance() {
        // TODO: Auto-generated method stub
        return 0;
    }

    public void periodic() {
        aKitLog.setLogLevel(AKitLogger.LogLevel.DEBUG);
    }

    public void refreshDataFrame() {
        frontLeft.refreshDataFrame();
        frontRight.refreshDataFrame();
    }
}
