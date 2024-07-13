package competition.simulation;

import org.littletonrobotics.junction.Logger;

public class LinearEngine {

    double velocity = 0;
    double friction = 0.02;
    double maxAccelerationMPSS = 4;
    double totalDistance = 0;
    double stepsPerSecond = 50;
    double maxMetersPerSecond = 3.0;
    
    public LinearEngine() {
        
    }

    public double step(double forwardPower) {

        // Calculate acceleration based on power and max acceleration
        double acceleration = forwardPower * maxAccelerationMPSS;

        // Apply friction proportional to current velocity
        velocity *= 1-friction;

        // Calculate new velocity
        velocity += acceleration / stepsPerSecond;

        // Limit velocity to max velocity
        if (velocity > maxMetersPerSecond) {
            velocity = maxMetersPerSecond;
        }

        if (velocity < -maxMetersPerSecond) {
            velocity = -maxMetersPerSecond;
        }

        // Calculate new position
        totalDistance += velocity / stepsPerSecond;

        Logger.recordOutput("Simulator/LinearEngine/Velocity", velocity);
        Logger.recordOutput("Simulator/LinearEngine/Acceleration", acceleration);

        // Return new position
        return totalDistance;
    }
    
    public double getVelocity() {
        return velocity;
    }
    
    public double getDistance() {
        return totalDistance;
    }
}
