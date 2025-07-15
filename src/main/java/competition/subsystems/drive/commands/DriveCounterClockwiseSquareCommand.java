package competition.subsystems.drive.commands;

import javax.inject.Inject;
import javax.inject.Provider;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class DriveCounterClockwiseSquareCommand extends SequentialCommandGroup {
    @Inject
    public DriveCounterClockwiseSquareCommand(
            Provider<DriveToDistanceCommand> straightProvider,
            Provider<TurnLeft90DegreesCommand> turnLeftProvider) {
        int squareSize = 4; // Arbitrary, could be made configurable.
        this.addCommands(
                straightProvider.get().withTargetDistance(squareSize),
                turnLeftProvider.get(),
                straightProvider.get().withTargetDistance(squareSize),
                turnLeftProvider.get(),
                straightProvider.get().withTargetDistance(squareSize),
                turnLeftProvider.get(),
                straightProvider.get().withTargetDistance(squareSize),
                turnLeftProvider.get()
        );
    }
}
