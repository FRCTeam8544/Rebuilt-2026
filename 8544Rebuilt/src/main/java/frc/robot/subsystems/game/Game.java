package frc.robot.subsystems.game;

import java.util.Optional;
import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Subsystem for managing FRC REBUILT 2026 game state and Hub status.
 */
public class Game extends SubsystemBase {

    private final GameIO gameIO;
    private final GameIOInputsAutoLogged gameInputOutputs = new GameIOInputsAutoLogged();

    private boolean shakeWhenTimeToShoot = false;
   // private boolean isShiftChange = false; 

    public BooleanSupplier isHubActive = 
        () -> {
            return gameInputOutputs.isHubActive;
        };

    public BooleanSupplier isShiftChangeSupplier = 
        () -> {
            return gameInputOutputs.shakeWhenTimeToShoot;
        };

    public BooleanSupplier isFMSAvailableSupplier = 
        () -> {
            return gameInputOutputs.fmsAvailable;
        };

    public Game(GameIO gameIO) {
        this.gameIO = gameIO;
        shakeWhenTimeToShoot = false;
    }

    /**
     * Gets the color of the alliance whose Hub goes INACTIVE first.
     * Per 2026 rules, this is the alliance that "won" or performed better in Auto.
     */
    private Optional<Alliance> getFirstInactiveAlliance() {
        String gameData = DriverStation.getGameSpecificMessage();
        if (gameData != null && !gameData.isEmpty()) {
            switch (gameData.toUpperCase().charAt(0)) {
                case 'R': return Optional.of(Alliance.Red);
                case 'B': return Optional.of(Alliance.Blue);
            }
        }
        return Optional.empty();
    }

    /**
     * Determines if the current robot's alliance Hub is active.
     */
    public boolean isHubActive() {
        Optional<Alliance> ourAlliance = DriverStation.getAlliance();
        if (ourAlliance.isEmpty()) return false;

        // Both alliance hubs are active in Auto and Endgame
        if (DriverStation.isAutonomous()) return true;
        
        double matchTime = DriverStation.getMatchTime();
        if (matchTime <= 30) return true; // Endgame (last 30s)

        // For the first 10s of Teleop (130s-140s remaining), both are active
        if (matchTime > 130) return true;

        Optional<Alliance> firstInactive = getFirstInactiveAlliance();
        // If data isn't available yet, assume active to allow scoring
        if (firstInactive.isEmpty()) return true;

        // Shift logic: 25-second cycles
        // Shift 1: 130-105s | Shift 2: 105-80s | Shift 3: 80-55s | Shift 4: 55-30s
        boolean isFirstInactiveShift = (matchTime > 105 && matchTime <= 130) || (matchTime > 55 && matchTime <= 80);
        
        boolean weAreTheInactiveAlliance = ourAlliance.get() == firstInactive.get();

        if (isFirstInactiveShift) {
            return !weAreTheInactiveAlliance;
        } else {
            // It's the other shift (2 or 4), so our Hub is active if we were the inactive one in shift 1/3
            return weAreTheInactiveAlliance;
        }
    }
        
    private boolean determineShakeOnLoss() {
        double matchTime = DriverStation.getMatchTime();

        if ( 80> matchTime && matchTime>77){
            return true;
        }
        else if (55> matchTime && matchTime>52) {
            return true;
        }
        else if(33> matchTime && matchTime > 30) {
            return true;
        }
        else {
            return false;
        }
    }

    private boolean determineShakeOnWin() {
        double matchTime = DriverStation.getMatchTime();

        if (105>matchTime && matchTime>102) {
            return true;
        }
        else if (55>matchTime && matchTime>52){
            return true;
        }
        else {
            return false;
        }
    }

    @Override
    public void periodic() {

        gameInputOutputs.fmsAvailable = DriverStation.isFMSAttached();
        gameInputOutputs.ownAlliance = DriverStation.getAlliance().orElse(Alliance.Blue);

        boolean hubActive = isHubActive();
        double matchTime = DriverStation.getMatchTime();
        Alliance autowinner = getFirstInactiveAlliance().orElse(Alliance.Blue);
        
        if (autowinner == gameInputOutputs.ownAlliance) {
            shakeWhenTimeToShoot = determineShakeOnWin();
        }
        else {
            shakeWhenTimeToShoot = determineShakeOnLoss();
        }

        // Match state for Elastic Dashboard
        SmartDashboard.putBoolean("Hub Active", hubActive);
        SmartDashboard.putString("Hub Status Label", hubActive ? "HUB ACTIVE" : "HUB INACTIVE");
        SmartDashboard.putNumber("Match Time", matchTime);

        // Auto Winner Logic and Display
        Optional<Alliance> winner = getFirstInactiveAlliance();
        String winnerStr = winner.isPresent() ? winner.get().toString() : "Waiting for FMS...";
        SmartDashboard.putString("Auto Winner (Inactive First)", winnerStr);

        // Feedback for the drive team on remaining shift time
        double shiftTimeRemaining = 0;
        if (matchTime > 30 && matchTime <= 130) {
            shiftTimeRemaining = (matchTime - 30) % 25;
        }

        SmartDashboard.putNumber("Current Shift Remaining", shiftTimeRemaining);

        // Populate the logging structure
        // TODO restructure this later
        {
            String gameData = DriverStation.getGameSpecificMessage();
            if (gameData != null && !gameData.isEmpty()) {
                gameInputOutputs.connected = true;
            }
            else
            {
                gameInputOutputs.connected = false;
            }

            gameInputOutputs.matchTime = matchTime;
            gameInputOutputs.shiftTimeRemaining = shiftTimeRemaining;

            gameInputOutputs.autoWinner = winnerStr;

            gameInputOutputs.isHubActive = hubActive;
            gameInputOutputs.shakeWhenTimeToShoot = shakeWhenTimeToShoot;
        }
        
        Logger.processInputs("Game", gameInputOutputs);
    }
}
