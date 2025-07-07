
package competition;

import competition.injection.components.BaseRobotComponent;
import competition.injection.components.DaggerRobotComponent;
import competition.injection.components.DaggerSimulationComponent;
import competition.operator_interface.OperatorCommandMap;
import competition.simulation.EduSimulator;
import competition.subsystems.drive.DriveSubsystem;
import competition.subsystems.drive.SwerveDriveSubsystem;
import competition.subsystems.pose.PoseSubsystem;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import xbot.common.command.BaseRobot;
import xbot.common.injection.components.BaseComponent;
import xbot.common.math.FieldPose;
import xbot.common.subsystems.pose.BasePoseSubsystem;

public class Robot extends BaseRobot {

    EduSimulator simulator;

    @Override
    protected void initializeSystems() {
        super.initializeSystems();
        getInjectorComponent().subsystemDefaultCommandMap();
        getInjectorComponent().operatorCommandMap();

        simulator = getInjectorComponent().eduSimulator();

        dataFrameRefreshables.add((DriveSubsystem)getInjectorComponent().driveSubsystem());
        dataFrameRefreshables.add(getInjectorComponent().swerve());
    }

    protected BaseRobotComponent createDaggerComponent() {
        if (BaseRobot.isReal()) {
            return DaggerRobotComponent.create();
        } else {
            return DaggerSimulationComponent.create();
        }
    }

    public BaseRobotComponent getInjectorComponent() {
        return (BaseRobotComponent)super.getInjectorComponent();
    }

    @Override
    public void simulationInit() {
        // teleport the robot to the field, bottom left ish pointing towards the middle
        getInjectorComponent().poseSubsystem().setCurrentPosition(2, 2);
        getInjectorComponent().poseSubsystem().setCurrentHeading(0);
        super.simulationInit();
        // Automatically enables the robot; remove this line of code if you want the robot
        // to start in a disabled state (as it would on the field). However, this does save you the 
        // hassle of navigating to the DS window and re-enabling the simulated robot.
        DriverStationSim.setEnabled(true);
    }

    @Override
    public void simulationPeriodic() {
        super.simulationPeriodic();

        if (simulator != null) {
            simulator.update();
        }
    }
}
