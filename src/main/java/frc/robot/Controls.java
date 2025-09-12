package frc.robot;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Controls {
    public static final CommandXboxController controller = new CommandXboxController(Constants.CONTROLLER_PORT);

    private static final Trigger l1Button = controller.a();
    private static final Trigger l2Button = controller.x();
    private static final Trigger l3Button = controller.b();
    private static final Trigger l4Button = controller.y();

    private static final Trigger autoHomeButton = controller.back();
    private static final Trigger resetGyroButtonOne = autoHomeButton;
    private static final Trigger resetGyroButtonTwo = controller.start();

    private static final Trigger climbButton = controller.povUp(); // bottom right paddle
    private static final Trigger intakeDeployButton = controller.rightBumper();

    private static final Trigger homeButton = controller.povRight(); // top right paddle

    // TODO: this should probably instead be if we're in a coral state (CORAL_LOADED, CORAL_SCORE_*, or CORAL_SCORING_*)
    public static final Trigger l1Coral = l1Button.and(OurRobotState.isCoralInGripperTrigger);
    public static final Trigger l2Coral = l2Button.and(OurRobotState.isCoralInGripperTrigger);
    public static final Trigger l3Coral = l3Button.and(OurRobotState.isCoralInGripperTrigger);
    public static final Trigger l4Coral = l4Button.and(OurRobotState.isCoralInGripperTrigger);

    public static final Trigger score = controller.rightTrigger();
    public static final Trigger algaePickupFloor = controller.leftTrigger();

    public static final Trigger switchTargetPole = score.and(() -> Math.abs(controller.getLeftX()) > 0.5d).debounce(0.5);

    public static final Trigger processor = l1Button.and(OurRobotState.isCoralInGripperTrigger.negate());
    public static final Trigger l2Algae = l2Button.and(OurRobotState.isCoralInGripperTrigger.negate());
    public static final Trigger l3Algae = l3Button.and(OurRobotState.isCoralInGripperTrigger.negate());
    public static final Trigger net = l4Button.and(OurRobotState.isCoralInGripperTrigger.negate());

    public static final Trigger invertedNet = controller.povLeft(); // bottom left paddle

    public static final Trigger home = homeButton.and(OurRobotState.isTargetingAlgaeTrigger.negate());
    public static final Trigger algaeHome = homeButton.and(OurRobotState.isTargetingAlgaeTrigger);

    public static final Trigger autoHome = autoHomeButton.and(resetGyroButtonTwo.negate());
    public static final Trigger resetGyro = resetGyroButtonOne.and(resetGyroButtonTwo);

    private static final Trigger coralIntakeTrigger = intakeDeployButton.and(OurRobotState.isClimbOutTrigger.negate());

    public static final Trigger deployIntake = coralIntakeTrigger.and(OurRobotState.isCoralInBotHolder.negate());
    public static final Trigger retractIntake = controller.leftBumper();

    public static final Trigger intakePurge = controller.povDown(); // top left paddle

    public static final Trigger grabCoral = coralIntakeTrigger.and(OurRobotState.isCoralInBotHolder);

    public static final Trigger deployClimb = intakeDeployButton.and(climbButton);
}
