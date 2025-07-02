package xbot.edubot.rotation;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.util.Timer;
import java.util.TimerTask;

import competition.subsystems.drive.BaseDriveTest;
import xbot.common.command.BaseCommand;
import competition.simulation.RotationEngine;

public class BaseOrientationEngineTest extends BaseDriveTest {

    public static final double POSITION_ERROR_THRESHOLD = 3;
    public static final double VELOCITY_ERROR_THRESHOLD = 0.1;
    public static final int MAX_ROTATION_CYCLES = 501;
    public static final double BASE_TIME_STEP = 0.1;

    protected double targetYaw;
    protected BaseCommand currentRotationCommand;
    protected RotationEngine engine;

    protected Timer asyncTimer;
    protected AsyncRotationIntervalJob asyncIntervalJob;
    protected boolean isAsync = false;
    protected double periodMultiplier = 1;

    public void setAsAsync(AsyncRotationIntervalJob asyncIntervalJob) {
        this.asyncIntervalJob = asyncIntervalJob;
        isAsync = true;
    }

    protected void setUpTestEnvironment(BaseCommand command, double initialYaw, double targetYaw) {
        engine = new RotationEngine(BASE_TIME_STEP, initialYaw, 0);
        this.pose.setCurrentHeading(initialYaw);
        this.targetYaw = targetYaw;
        this.currentRotationCommand = command;
    }

    protected void setUpTestEnvironmentFor90(double initialYaw) {
        double targetYaw = initialYaw + 90;
        if (targetYaw > 180) {
            targetYaw -= 360;
        }
        else if (targetYaw < -180) {
            targetYaw += 360;
        }
        setUpTestEnvironment(this.getInjectorComponent().turnLeft90DegreesCommand(), initialYaw, targetYaw);
    }

    protected void runTestEnv() {
        currentRotationCommand.initialize();
        if (isAsync) {
            startAsyncTimer();
            return;
        }

        for (int i = 0; i < MAX_ROTATION_CYCLES; i++) {
            runRotationStep(engine);

            System.out.printf("Time:%.1f sec, TurningPower:%.2f, Velocity:%.2f, Yaw:%.2f \n", (double) i * BASE_TIME_STEP,
                    getRotationPower(), engine.getVelocity(), engine.getOrientation());

            if (currentRotationCommand.isFinished()) {
                break;
            }

            currentRotationCommand.execute();
        }

        assertTrue("Verify command reports successfully finished", currentRotationCommand.isFinished());
        currentRotationCommand.end(false);

        assertRotationTarget(engine, targetYaw);
    }

    private void startAsyncTimer() {
        asyncTimer = new Timer();
        asyncTimer.schedule(new TimerTask() {
            @Override
            public void run() {
                runRotationStep(engine);
                currentRotationCommand.execute();

                asyncIntervalJob.onNewStep(new RotationEnvironmentState(targetYaw, engine.getOrientation(),
                        engine.getVelocity(), getRotationPower(), currentRotationCommand.isFinished()));
            }
        }, 0, (int) (BASE_TIME_STEP * 1000 * periodMultiplier));
    }
    
    public void setAsyncPeriodMultiplier(double newMultiplier) {
        if(Math.abs(this.periodMultiplier - newMultiplier) > 0.01) {
            this.periodMultiplier = newMultiplier;
            
            asyncTimer.cancel();
            startAsyncTimer();
        }
    }
    
    public void stopTestEnv() {
        asyncTimer.cancel();
    }

    protected void runRotationStep(RotationEngine engine) {
        engine.setPower(getRotationPower());
        engine.step();
        this.pose.setCurrentHeading(engine.getOrientation());
    }

    protected void assertRotationTarget(RotationEngine engine, double targetHeading) {
        assertEquals("Make sure robot is close to target position within " + POSITION_ERROR_THRESHOLD, targetHeading,
                pose.getCurrentHeading().getDegrees(), POSITION_ERROR_THRESHOLD);
        assertEquals("Make sure robot has come to a stop, not just flying past the target position.", 0.0,
                engine.getVelocity(), VELOCITY_ERROR_THRESHOLD);
    }

    private double getRotationPower() {
        // read from drive wheels, make a turning function
        // left pair
        double l1 = drive.frontLeft.getPower();

        // right pair
        double r1 = drive.frontRight.getPower();

        // left turns are positive. So right power is positive, left power negative.
        return (r1 - l1) / 2;
    }

    public static interface AsyncRotationIntervalJob {
        void onNewStep(RotationEnvironmentState envState);
    }

    public static class RotationEnvironmentState {
        public final double targetOrientation;
        public final double currentOrientation;
        public final double orientationThreshold;
        public final double currentVelocity;
        public final double velocityThreshold;
        public final double currentRotationalPower;
        public final boolean isCommandFinished;
        
        public final boolean isAtOrientationTarget;
        public final boolean isAtVelocityTarget;

        public RotationEnvironmentState(double target, double yaw, double velocity, double rotationalPower,
                boolean isFinished) {
            this.targetOrientation = target;
            this.currentOrientation = yaw;
            this.currentVelocity = velocity;
            this.currentRotationalPower = rotationalPower;
            this.isCommandFinished = isFinished;
            
            this.orientationThreshold = POSITION_ERROR_THRESHOLD;
            this.velocityThreshold = VELOCITY_ERROR_THRESHOLD;
            
            this.isAtOrientationTarget = Math.abs(currentOrientation - targetOrientation) <= orientationThreshold;
            this.isAtVelocityTarget = Math.abs(currentVelocity) <= velocityThreshold;
        }
        
        public RotationEnvironmentState() {
            this(0, 0, 0, 0, true);
        }
    }
}
