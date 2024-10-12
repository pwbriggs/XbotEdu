package competition.simulation;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import xbot.common.math.MathUtils;

public class SimSwerveModule {

    private FlywheelSim driveMotor;
    private FlywheelSim steeringMotor;

    private double loopPeriodSecs = 0.02;

    private double turnRelativePositionRad = 0.0;
    private double turnAbsolutePositionRad = 0.0;
    private double driveVelocityRad = 0.0;
    private double drivePositionRad = 0.0;

    public SimSwerveModule() {
        driveMotor = new FlywheelSim(DCMotor.getNEO(1), 6.75, 0.025);
        steeringMotor = new FlywheelSim(DCMotor.getNEO(1), 150.0 / 7.0, 0.004);
    }

    public void setDrivePower(double power) {
        driveMotor.setInputVoltage(MathUtils.constrainDouble(power, -1, 1) * 12.0);
    }

    public void setSteeringPower(double power) {
        steeringMotor.setInputVoltage(MathUtils.constrainDouble(power, -1, 1) * 12.0);
    }

    public void update() {
        // Let the simulated objects update based on the passage of time since last update
        driveMotor.update(0.02);
        steeringMotor.update(0.02);

        // Figure out how much the steering module has turned
        double angleDiffRad = steeringMotor.getAngularVelocityRadPerSec() * loopPeriodSecs;
        turnRelativePositionRad += angleDiffRad;
        turnAbsolutePositionRad += angleDiffRad;
        // Keep the absolute position within 0 to 2pi, as this mimics how the motors report absolute position
        while (turnAbsolutePositionRad < 0) {
            turnAbsolutePositionRad += 2.0 * Math.PI;
        }
        while (turnAbsolutePositionRad > 2.0 * Math.PI) {
            turnAbsolutePositionRad -= 2.0 * Math.PI;
        }

        // Calculate velocity
        driveVelocityRad = driveMotor.getAngularVelocityRadPerSec();

        // Calculate accumulated position
        drivePositionRad += driveVelocityRad * loopPeriodSecs;
    }

    public double getDriveVelocityRad() {
        return driveVelocityRad;
    }

    public double getDrivePositionRad() {
        return drivePositionRad;
    }

    public double getTurnAbsolutePositionRad() {
        return turnAbsolutePositionRad;
    }

    public double getTurnRelativePositionRad() {
        return turnRelativePositionRad;
    }

}
