package competition.subsystems.drive.commands;

import javax.inject.Inject;

import xbot.common.command.BaseCommand;
import xbot.common.math.PIDManager;
import xbot.common.math.PIDManager.PIDManagerFactory;
import xbot.common.subsystems.drive.control_logic.HeadingModule;
import xbot.common.subsystems.drive.control_logic.HeadingModule.HeadingModuleFactory;
import competition.subsystems.drive.DriveSubsystem;
import competition.subsystems.pose.PoseSubsystem;

public class DriveToOrientationCommand extends BaseCommand {

    final DriveSubsystem drive;
    final PoseSubsystem pose;
    final HeadingModule headingModule;
    final PIDManager pid;
    
    double targetPosition;

    @Inject
    public DriveToOrientationCommand(DriveSubsystem driveSubsystem, PoseSubsystem pose, HeadingModuleFactory headingModuleFactory, PIDManagerFactory pidManagerFactory) {
        this.drive = driveSubsystem;
        this.pose = pose;
        this.pid = pidManagerFactory.create("DriveToOrientation");
        pid.setP(1);
        pid.setD(5);
        pid.setEnableDerivativeThreshold(true);
        pid.setEnableErrorThreshold(true);
        pid.setErrorThreshold(0.01);
        pid.setDerivativeThreshold(0.01);
        this.headingModule = headingModuleFactory.create(pid);

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
        double power = headingModule.calculateHeadingPower(targetPosition);
        // log.info(power);
        drive.tankDrive(-power, power);
    }

    @Override
    public boolean isFinished() {
        return headingModule.isOnTarget();
    }
}
