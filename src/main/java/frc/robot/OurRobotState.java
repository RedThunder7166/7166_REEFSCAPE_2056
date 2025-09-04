package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.ArrayList;
import java.util.List;
import java.util.Set;

public class OurRobotState {
    private static boolean isIdle = true;

    public static boolean getIsIdle() {
        return isIdle;
    }

    public static void setIsIdle(boolean value) {
        isIdle = value;
    }

    public static enum ScoreMechanismState {
        HOME,
        CORAL_INTAKE,
        // CORAL_LOADED,
        CORAL_SCORE_L1,
        CORAL_SCORING_L1,
        CORAL_SCORE_L2,
        CORAL_SCORING_L2,
        CORAL_SCORE_L3,
        CORAL_SCORING_L3,
        CORAL_SCORE_L4,
        CORAL_SCORING_L4,
        ALGAE_PICKUP_FLOOR,
        ALGAE_PICKUP_L2,
        ALGAE_PICKUP_L3,
        ALGAE_HOME,
        ALGAE_SCORE_NET,
        ALGAE_SCORING_NET,
        ALGAE_SCORE_PROCESSOR,
        ALGAE_SCORING_PROCESSOR;

        private Set<ScoreMechanismState> allowedNext;

        static {
            ScoreMechanismState.HOME.allowedNext = Set.of(HOME, CORAL_INTAKE, ALGAE_PICKUP_FLOOR, ALGAE_PICKUP_L2,
                    ALGAE_PICKUP_L3);

            ScoreMechanismState.CORAL_INTAKE.allowedNext = Set.of(CORAL_INTAKE, HOME);
            // ScoreMechanismState.CORAL_LOADED.allowedNext = Set.of(CORAL_LOADED, CORAL_SCORE_L1, CORAL_SCORE_L2,
            //         CORAL_SCORE_L3, CORAL_SCORE_L4);
            ScoreMechanismState.CORAL_SCORE_L1.allowedNext = Set.of(
                    // CORAL_LOADED,
                    HOME,
                    CORAL_SCORE_L1,
                    CORAL_SCORE_L2,
                    CORAL_SCORE_L3,
                    CORAL_SCORE_L4,
                    CORAL_SCORING_L1);
            ScoreMechanismState.CORAL_SCORE_L2.allowedNext = Set.of(
                    // CORAL_LOADED,
                    HOME,
                    CORAL_SCORE_L1,
                    CORAL_SCORE_L2,
                    CORAL_SCORE_L3,
                    CORAL_SCORE_L4,
                    CORAL_SCORING_L2);
            ScoreMechanismState.CORAL_SCORE_L3.allowedNext = Set.of(
                    // CORAL_LOADED,
                    HOME,
                    CORAL_SCORE_L1,
                    CORAL_SCORE_L2,
                    CORAL_SCORE_L3,
                    CORAL_SCORE_L4,
                    CORAL_SCORING_L3);
            ScoreMechanismState.CORAL_SCORE_L4.allowedNext = Set.of(
                    // CORAL_LOADED,
                    HOME,
                    CORAL_SCORE_L1,
                    CORAL_SCORE_L2,
                    CORAL_SCORE_L3,
                    CORAL_SCORE_L4,
                    CORAL_SCORING_L4);
            ScoreMechanismState.CORAL_SCORING_L1.allowedNext = Set.of(CORAL_SCORING_L1, HOME);
            ScoreMechanismState.CORAL_SCORING_L2.allowedNext = Set.of(CORAL_SCORING_L2, HOME);
            ScoreMechanismState.CORAL_SCORING_L3.allowedNext = Set.of(CORAL_SCORING_L3, HOME);
            ScoreMechanismState.CORAL_SCORING_L4.allowedNext = Set.of(CORAL_SCORING_L4, HOME);

            ScoreMechanismState.ALGAE_PICKUP_FLOOR.allowedNext = Set.of(ALGAE_PICKUP_FLOOR, ALGAE_HOME);
            ScoreMechanismState.ALGAE_PICKUP_L2.allowedNext = Set.of(ALGAE_PICKUP_L2, ALGAE_HOME);
            ScoreMechanismState.ALGAE_PICKUP_L3.allowedNext = Set.of(ALGAE_PICKUP_L3, ALGAE_HOME);
            ScoreMechanismState.ALGAE_HOME.allowedNext = Set.of(ALGAE_HOME, ALGAE_SCORE_PROCESSOR, ALGAE_SCORE_NET);
            ScoreMechanismState.ALGAE_SCORE_PROCESSOR.allowedNext = Set.of(ALGAE_SCORE_PROCESSOR, ALGAE_HOME,
                    ALGAE_SCORING_PROCESSOR);
            ScoreMechanismState.ALGAE_SCORE_NET.allowedNext = Set.of(ALGAE_SCORE_NET, ALGAE_HOME, ALGAE_SCORING_NET);
            ScoreMechanismState.ALGAE_SCORING_PROCESSOR.allowedNext = Set.of(ALGAE_SCORING_PROCESSOR, HOME);
            ScoreMechanismState.ALGAE_SCORING_NET.allowedNext = Set.of(ALGAE_SCORING_NET, HOME);
        }

        public boolean canTransitionTo(ScoreMechanismState possibleNext) {
            return allowedNext.contains(possibleNext);
        }
    };

    private static List<Runnable> scoreMechanismStateChangeCallbackList = new ArrayList<>();

    public static void addScoreMechanismStateChangeCallback(Runnable callback) {
        scoreMechanismStateChangeCallbackList.add(callback);
    }

    private static ScoreMechanismState scoreMechanismState = ScoreMechanismState.HOME;

    public static ScoreMechanismState getScoreMechanismState() {
        return scoreMechanismState;
    }

    public static void setScoreMechanismState(ScoreMechanismState targetState) {
        if (!scoreMechanismState.canTransitionTo(targetState))
            return;
        scoreMechanismState = targetState;

        scoreMechanismStateChangeCallbackList.forEach(Runnable::run);
    }

    public static Command setScoreMechanismStateCommand(ScoreMechanismState state) {
        return Commands.runOnce(() -> setScoreMechanismState(state));
    }

    public static Command scoreCommand = Commands.runOnce(
            () -> {
                var targetState = scoreMechanismState;

                switch (scoreMechanismState) {
                    case CORAL_SCORE_L1:
                        targetState = ScoreMechanismState.CORAL_SCORING_L1;
                        break;
                    case CORAL_SCORE_L2:
                        targetState = ScoreMechanismState.CORAL_SCORING_L2;
                        break;
                    case CORAL_SCORE_L3:
                        targetState = ScoreMechanismState.CORAL_SCORING_L3;
                        break;
                    case CORAL_SCORE_L4:
                        targetState = ScoreMechanismState.CORAL_SCORING_L4;
                        break;
                    case ALGAE_SCORE_NET:
                        targetState = ScoreMechanismState.ALGAE_SCORING_NET;
                    case ALGAE_SCORE_PROCESSOR:
                        targetState = ScoreMechanismState.ALGAE_SCORING_PROCESSOR;
                    default:
                        break;
                }

                setScoreMechanismState(targetState);
            });

    private static boolean isCoralInGripper = false;

    public static boolean getIsCoralInGripper() {
        return isCoralInGripper;
    }

    public static void setIsCoralInGripper(boolean value) {
        isCoralInGripper = value;
    }

    public static final Trigger isCoralInGripperTrigger = new Trigger(OurRobotState::getIsCoralInGripper);

    private static boolean isClimbOut = false;

    public static boolean getIsClimbOut() {
        return isClimbOut;
    }

    public static void setIsClimbOut(boolean value) {
        isClimbOut = value;
    }

    public static final Trigger isClimbOutTrigger = new Trigger(OurRobotState::getIsClimbOut);

    private static boolean isDeployingClimb = false;

    public static boolean getIsDeployingClimb() {
        return isDeployingClimb;
    }

    private static void setIsDeployingClimb(boolean value) {
        isDeployingClimb = value;
    }

    public static void deployClimb() {
        setIsDeployingClimb(true);
    }

    public static final Trigger isDeployingClimbTrigger = new Trigger(OurRobotState::getIsDeployingClimb);

    private static boolean isDeployingIntake = false;

    public static boolean getIsDeployingIntake() {
        return isDeployingIntake;
    }

    private static void setIsDeployingIntake(boolean value) {
        isDeployingIntake = value;
    }

    public static void deployIntake() {
        setIsDeployingIntake(true);
    }

    public static final Trigger isDeployingIntakeTrigger = new Trigger(OurRobotState::getIsDeployingIntake);


    private static boolean isElevatorAboveArmClearance = false;

    public static boolean getIsElevatorAboveArmClearance() {
        return isElevatorAboveArmClearance;
    }

    public static void setIsElevatorAboveArmClearance(boolean value) {
        isElevatorAboveArmClearance = value;
    }


    private static boolean isArmPastFarNetClearance = false;

    public static boolean getIsArmPastFarNetClearance() {
        return isArmPastFarNetClearance;
    }

    public static void setIsArmPastFarNetClearance(boolean value) {
        isArmPastFarNetClearance = value;
    }


    private static boolean isCoralHolderFirstSensorTripped = false;

    public static boolean getIsCoralHolderFirstSensorTripped() {
        return isCoralHolderFirstSensorTripped;
    }

    public static void setIsCoralHolderFirstSensorTripped(boolean value) {
        isCoralHolderFirstSensorTripped = value;
    }


    private static boolean isCoralHolderSecondSensorTripped = false;

    public static boolean getIsCoralHolderSecondSensorTripped() {
        return isCoralHolderSecondSensorTripped;
    }

    public static void setIsCoralHolderSecondSensorTripped(boolean value) {
        isCoralHolderSecondSensorTripped = value;
    }


    public static enum IntakeState {
        IDLE,
        OUT,
        IN
    }

    private static IntakeState intakeState = IntakeState.IDLE;

    public static synchronized IntakeState getIntakeState() {
        return intakeState;
    }

    private static synchronized void setIntakeState(IntakeState desiredState) {
        intakeState = desiredState;
    }

    public static synchronized void startIntake(IntakeState intakeState) {
        setIntakeState(intakeState);
    }

    public static synchronized void stopIntake() {
        setIntakeState(IntakeState.IDLE);
    }
}
