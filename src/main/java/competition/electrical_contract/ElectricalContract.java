package competition.electrical_contract;

import xbot.common.injection.electrical_contract.CANMotorControllerInfo;

public abstract class ElectricalContract {
    public abstract CANMotorControllerInfo getLeftLeader();
    public abstract CANMotorControllerInfo getRightLeader();
}
