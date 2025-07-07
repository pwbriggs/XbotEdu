package xbot.edubot.rotation;

import org.junit.Test;

import competition.subsystems.drive.commands.DriveToOrientationCommand;
import xbot.edubot.rotation.RotationTestVisualizer.OrientationTest;
import xbot.edubot.rotation.RotationTestVisualizer.SelectableOrientationTest;


public class DriveToOrientationTest extends BaseOrientationEngineTest implements SelectableOrientationTest {

    @Test
    public void testGoToOrientation0to150() {
        DriveToOrientationCommand command = this.getInjectorComponent().driveToOrientationCommand();
        command.setTargetHeading(150);

        setUpTestEnvironment(command, 0, 150);
        runTestEnv();
    }

    @Test
    public void testGoToOrientation0toNeg150() {
        DriveToOrientationCommand command = this.getInjectorComponent().driveToOrientationCommand();
        command.setTargetHeading(-150);

        setUpTestEnvironment(command, 0, -150);
        runTestEnv();
    }

    @Test
    public void testGoToOrientationNeg90to150() {
        DriveToOrientationCommand command = this.getInjectorComponent().driveToOrientationCommand();
        command.setTargetHeading(150);

        setUpTestEnvironment(command, -90, 150);
        runTestEnv();
    }

    @Override
    public void invokeOrientationTest(OrientationTest test) {
        switch (test) {
            case ROTATE_TO_ORIENTATION_0_150:
                this.testGoToOrientation0to150();
                break;
            case ROTATE_TO_ORIENTATION_0_NEG_150:
                this.testGoToOrientation0toNeg150();
                break;
            case ROTATE_TO_ORIENTATION_NEG_90_150:
                this.testGoToOrientationNeg90to150();
                break;
            default:
                throw new RuntimeException("The requested orientation test is not available in this test class.");
        }
    }
}
