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

    private static final Trigger climbButton = controller.povRight(); // bottom right paddle
    private static final Trigger deployButton = controller.rightBumper();

    // TODO: this should probably instead be if we're in a coral state (CORAL_LOADED, CORAL_SCORE_*, or CORAL_SCORING_*)
    public static final Trigger l1Coral = l1Button.and(OurRobotState.isCoralInGripperTrigger);
    public static final Trigger l2Coral = l2Button.and(OurRobotState.isCoralInGripperTrigger);
    public static final Trigger l3Coral = l3Button.and(OurRobotState.isCoralInGripperTrigger);
    public static final Trigger l4Coral = l4Button.and(OurRobotState.isCoralInGripperTrigger);

    public static final Trigger score = controller.rightTrigger();
    public static final Trigger algaePickupFloor = controller.leftTrigger();

    public static final Trigger processor = l1Button.and(OurRobotState.isCoralInGripperTrigger.negate());
    public static final Trigger l2Algae = l2Button.and(OurRobotState.isCoralInGripperTrigger.negate());
    public static final Trigger l3Algae = l3Button.and(OurRobotState.isCoralInGripperTrigger.negate());
    public static final Trigger net = l4Button.and(OurRobotState.isCoralInGripperTrigger.negate());

    public static final Trigger homeButton = controller.povUp(); // top right paddle

    public static final Trigger autoHome = autoHomeButton.and(resetGyroButtonTwo.negate());
    public static final Trigger resetGyro = resetGyroButtonOne.and(resetGyroButtonTwo);

    private static final Trigger coralIntakeTrigger = deployButton.and(OurRobotState.isClimbOutTrigger.negate());

    public static final Trigger deployIntake = coralIntakeTrigger.and(OurRobotState.isCoralInGripperTrigger.negate());
    public static final Trigger retractIntake = controller.leftBumper();

    public static final Trigger grabCoral = coralIntakeTrigger.and(OurRobotState.isCoralInGripperTrigger);

    public static final Trigger deployClimb = deployButton.and(climbButton);
}
