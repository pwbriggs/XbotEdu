package competition.electrical_contract;

import javax.inject.Inject;

import competition.subsystems.pose.PoseSubsystem;
import xbot.common.injection.electrical_contract.CANBusId;
import xbot.common.injection.electrical_contract.CANMotorControllerInfo;
import xbot.common.injection.electrical_contract.MotorControllerType;

public class CompetitionContract extends ElectricalContract {
    
    @Inject
    public CompetitionContract() {}

    @Override
    public CANMotorControllerInfo getLeftLeader() {
        return new CANMotorControllerInfo("Left", MotorControllerType.SparkMax, CANBusId.RIO, 1);
    }

    @Override
    public CANMotorControllerInfo getRightLeader() {
        return new CANMotorControllerInfo("Left", MotorControllerType.SparkMax, CANBusId.RIO, 2);
    }
}
