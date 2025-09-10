package frc.robot;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.ArrayList;
import java.util.List;
import java.util.Set;

public class OurRobotState {
    public static final NetworkTableInstance NETWORK_TABLE_INSTANCE = NetworkTableInstance.getDefault();
    public static final NetworkTable robotStateTable = NETWORK_TABLE_INSTANCE.getTable("RobotState");

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

            ScoreMechanismState.ALGAE_PICKUP_FLOOR.allowedNext = Set.of(ALGAE_PICKUP_FLOOR, HOME);
            ScoreMechanismState.ALGAE_PICKUP_L2.allowedNext = Set.of(ALGAE_PICKUP_L2, ALGAE_PICKUP_L3, HOME);
            ScoreMechanismState.ALGAE_PICKUP_L3.allowedNext = Set.of(ALGAE_PICKUP_L3, ALGAE_PICKUP_L2, HOME);
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
    private static final StringPublisher scoreMechanismStatePublisher = robotStateTable.getStringTopic("ScoreMechanismState").publish();

    public static ScoreMechanismState getScoreMechanismState() {
        return scoreMechanismState;
    }

    public static boolean setScoreMechanismState(ScoreMechanismState targetState) {
        if (!scoreMechanismState.canTransitionTo(targetState))
            return false;
        if (targetState != scoreMechanismState)
            scoreMechanismStatePublisher.set(targetState.toString());

        scoreMechanismState = targetState;
        scoreMechanismStateChangeCallbackList.forEach(Runnable::run);

        return true;
    }

    static { scoreMechanismStatePublisher.set(scoreMechanismState.toString()); }

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
                    break;
                case ALGAE_SCORE_PROCESSOR:
                    targetState = ScoreMechanismState.ALGAE_SCORING_PROCESSOR;
                default:
                    break;
            }

            setScoreMechanismState(targetState);
        });

    public static boolean isTargetingAlgae = false;
    private static final BooleanPublisher isTargetingAlgaePublisher = robotStateTable.getBooleanTopic("IsTargetingAlgae").publish();
    public static final Trigger isTargetingAlgaeTrigger = new Trigger(OurRobotState::getIsTargetingAlgae);

    public static boolean getIsTargetingAlgae() {
        return isTargetingAlgae;
    }

    private static void setIsTargetingAlgae(boolean value) {
        if (value != isTargetingAlgae)
            isTargetingAlgaePublisher.set(value);
        isTargetingAlgae = value;
    }

    static { isTargetingAlgaePublisher.set(isTargetingAlgae); }

    static {
        scoreMechanismStateChangeCallbackList.add(() -> {
            switch (OurRobotState.getScoreMechanismState()) {
                case ALGAE_HOME:
                case ALGAE_SCORE_PROCESSOR:
                case ALGAE_SCORE_NET:
                // case ALGAE_PICKUP_FLOOR:
                // case ALGAE_PICKUP_L2:
                // case ALGAE_PICKUP_L3:
                    setIsTargetingAlgae(true);
                    break;

                default:
                    setIsTargetingAlgae(false);
            }
        });
    }

    private static boolean isCoralInGripper = false;
    private static final BooleanPublisher isCoralInGripperPublisher = robotStateTable.getBooleanTopic("IsCoralInGripper").publish();
    public static final Trigger isCoralInGripperTrigger = new Trigger(OurRobotState::getIsCoralInGripper);

    public static boolean getIsCoralInGripper() {
        return isCoralInGripper;
    }

    public static void setIsCoralInGripper(boolean value) {
        if (value != isCoralInGripper)
            isCoralInGripperPublisher.set(value);
        isCoralInGripper = value;
    }

    static { isCoralInGripperPublisher.set(isCoralInGripper); }


    private static boolean isCoralInHolder = false;
    private static final BooleanPublisher isCoralInHolderPublisher = robotStateTable.getBooleanTopic("IsCoralInHolder").publish();
    public static final Trigger isCoralInBotHolder = new Trigger(OurRobotState::getIsCoralInHolder);

    public static boolean getIsCoralInHolder() {
        return isCoralInHolder;
    }

    public static void setIsCoralInHolder(boolean value) {
        if (value != isCoralInHolder)
            isCoralInHolderPublisher.set(value);
        isCoralInHolder = value;
    }

    static { isCoralInHolderPublisher.set(isCoralInHolder); }


    private static boolean isClimbOut = false;
    private static final BooleanPublisher isClimbOutPublisher = robotStateTable.getBooleanTopic("IsClimbOut").publish();
    public static final Trigger isClimbOutTrigger = new Trigger(OurRobotState::getIsClimbOut);

    public static boolean getIsClimbOut() {
        return isClimbOut;
    }

    public static void setIsClimbOut(boolean value) {
        if (value != isClimbOut)
            isClimbOutPublisher.set(value);
        isClimbOut = value;
    }

    static { isClimbOutPublisher.set(isClimbOut); }


    private static boolean isDeployingClimb = false;
    private static final BooleanPublisher isDeployingClimbPublisher = robotStateTable.getBooleanTopic("IsDeployingClimb").publish();
    public static final Trigger isDeployingClimbTrigger = new Trigger(OurRobotState::getIsDeployingClimb);

    public static boolean getIsDeployingClimb() {
        return isDeployingClimb;
    }

    private static void setIsDeployingClimb(boolean value) {
        if (value != isDeployingClimb)
            isDeployingClimbPublisher.set(value);
        isDeployingClimb = value;
    }

    public static void deployClimb() {
        setIsDeployingClimb(true);
    }

    static { isDeployingClimbPublisher.set(isDeployingClimb); }


    // private static boolean isDeployingIntake = false;
    // private static final BooleanPublisher isDeployingIntakePublisher = robotStateTable.getBooleanTopic("IsDeployingIntake").publish();
    // public static final Trigger isDeployingIntakeTrigger = new Trigger(OurRobotState::getIsDeployingIntake);

    // public static boolean getIsDeployingIntake() {
    //     return isDeployingIntake;
    // }

    // public static void setIsDeployingIntake(boolean value) {
    //     if (value != isDeployingIntake)
    //         isDeployingIntakePublisher.set(value);
    //     isDeployingIntake = value;
    // }

    // public static Command deployIntake() {
    //     return Commands.runOnce(() -> {
    //         if (setScoreMechanismState(ScoreMechanismState.CORAL_INTAKE))
    //             setIsDeployingIntake(true);
    //     });
    // }

    // static { isDeployingIntakePublisher.set(isDeployingIntake); }


    private static boolean isElevatorAboveArmClearance = false;
    private static final BooleanPublisher isElevatorAboveArmClearancePublisher = robotStateTable.getBooleanTopic("IsElevatorAboveArmClearance").publish();

    public static boolean getIsElevatorAboveArmClearance() {
        return isElevatorAboveArmClearance;
    }

    public static void setIsElevatorAboveArmClearance(boolean value) {
        if (isElevatorAboveArmClearance != value)
            isElevatorAboveArmClearancePublisher.set(value);
        isElevatorAboveArmClearance = value;
    }

    static { isElevatorAboveArmClearancePublisher.set(isElevatorAboveArmClearance); }


    private static boolean isArmPastFarNetClearance = false;
    private static final BooleanPublisher isArmPastFarNetClearancePublisher = robotStateTable.getBooleanTopic("IsArmPastFarNetClearance").publish();

    public static boolean getIsArmPastFarNetClearance() {
        return isArmPastFarNetClearance;
    }

    public static void setIsArmPastFarNetClearance(boolean value) {
        if (value != isArmPastFarNetClearance)
            isArmPastFarNetClearancePublisher.set(value);
        isArmPastFarNetClearance = value;
    }

    static { isArmPastFarNetClearancePublisher.set(isArmPastFarNetClearance); }


    private static boolean isCoralHolderFirstSensorTripped = false;
    private static final BooleanPublisher isCoralHolderFirstSensorTrippedPublisher = robotStateTable.getBooleanTopic("IsCoralHolderFirstSensorTripped").publish();

    public static boolean getIsCoralHolderFirstSensorTripped() {
        return isCoralHolderFirstSensorTripped;
    }

    public static void setIsCoralHolderFirstSensorTripped(boolean value) {
        if (value != isCoralHolderFirstSensorTripped)
            isCoralHolderFirstSensorTrippedPublisher.set(value);
        isCoralHolderFirstSensorTripped = value;
    }

    static { isCoralHolderFirstSensorTrippedPublisher.set(isCoralHolderFirstSensorTripped); }


    private static boolean isCoralHolderSecondSensorTripped = false;
    private static final BooleanPublisher isCoralHolderSecondSensorTrippedPublisher = robotStateTable.getBooleanTopic("IsCoralHolderSecondSensorTripped").publish();

    public static boolean getIsCoralHolderSecondSensorTripped() {
        return isCoralHolderSecondSensorTripped;
    }

    public static void setIsCoralHolderSecondSensorTripped(boolean value) {
        if (value != isCoralHolderSecondSensorTripped)
            isCoralHolderSecondSensorTrippedPublisher.set(value);
        isCoralHolderSecondSensorTripped = value;
    }

    static { isCoralHolderSecondSensorTrippedPublisher.set(isCoralHolderSecondSensorTripped); }


    public static enum IntakeState {
        IDLE,
        OUT,
        IN
    }

    private static IntakeState intakeState = IntakeState.IDLE;
    private static final StringPublisher intakeStatePublisher = robotStateTable.getStringTopic("IntakeState").publish();

    public static synchronized IntakeState getIntakeState() {
        return intakeState;
    }

    private static synchronized void setIntakeState(IntakeState value) {
        if (value != intakeState)
            intakeStatePublisher.set(value.toString());
        intakeState = value;
    }

    static { intakeStatePublisher.set(intakeState.toString()); }

    public static synchronized void startIntake(IntakeState intakeState) {
        setIntakeState(intakeState);
    }

    public static synchronized void stopIntake() {
        setIntakeState(IntakeState.IDLE);
    }
}
