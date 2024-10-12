package competition.subsystems.drive;

import competition.BaseCompetitionTest;
import competition.operator_interface.OperatorInterface;
import competition.simulation.EduSimulator;
import competition.subsystems.drive.commands.SwerveDriveWithJoysticksCommand;
import competition.subsystems.pose.PoseSubsystem;
import edu.wpi.first.wpilibj.MockXboxControllerAdapter;
import org.junit.Before;
import org.junit.Test;

public class SwerveDriveWithJoysticksCommandTest extends BaseCompetitionTest {

    SwerveDriveSubsystem swerveDrive;
    OperatorInterface oi;
    MockXboxControllerAdapter gamepad;

    SwerveDriveWithJoysticksCommand command;
    EduSimulator simulator;

    @Before
    public void setUp() {
        super.setUp();
        swerveDrive = this.getInjectorComponent().swerveDriveSubsystem();
        command = this.getInjectorComponent().swerveDriveWithJoysticksCommand();
        simulator = this.getInjectorComponent().eduSimulator();
        oi = this.getInjectorComponent().operatorInterface();
        gamepad = (MockXboxControllerAdapter) oi.gamepad;

        simulator.setForceDriverStationEnabledForUnitTests(true);
    }

    private void driveForShortTime() {
        command.initialize();
        for (int i = 0; i < 100; i++) {
            swerveDrive.refreshDataFrame();
            swerveDrive.periodic();
            simulator.simulateSwerve();
            command.execute();
        }
    }

    @Test
    public void testGoingForward() {
        gamepad.setLeftStick(0, 1);

        driveForShortTime();

        var robotPose = swerveDrive.getSwervePose();
        // Check that the robot has moved mostly in the +X direction, with very little +Y.
        assert(robotPose.getTranslation().getX() > 1.0);
        assert(Math.abs(robotPose.getTranslation().getY()) < 0.1);
    }

    @Test
    public void testGoingBackward() {
        gamepad.setLeftStick(0, -1);

        driveForShortTime();

        var robotPose = swerveDrive.getSwervePose();
        // Check that the robot has moved mostly in the -X direction, with very little +Y.
        assert(robotPose.getTranslation().getX() < -1.0);
        assert(Math.abs(robotPose.getTranslation().getY()) < 0.1);
    }

    @Test
    public void testGoingLeft() {
        gamepad.setLeftStick(-1, 0);

        driveForShortTime();

        var robotPose = swerveDrive.getSwervePose();
        // Check that the robot has moved mostly in the -Y direction, with very little +X.
        assert(robotPose.getTranslation().getY() > 1.0);
        assert(Math.abs(robotPose.getTranslation().getX()) < 0.1);
    }

    @Test
    public void testGoingRight() {
        gamepad.setLeftStick(1, 0);

        driveForShortTime();

        var robotPose = swerveDrive.getSwervePose();
        // Check that the robot has moved mostly in the +Y direction, with very little +X.
        assert(robotPose.getTranslation().getY() < -1.0);
        assert(Math.abs(robotPose.getTranslation().getX()) < 0.1);
    }
}
