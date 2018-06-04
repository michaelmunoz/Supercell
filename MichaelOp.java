package org.firstinspires.ftc.teamcode.vv7797.opmode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.vv7797.opmode.control.AutonomousSuite.NestedInstruction;
import org.firstinspires.ftc.teamcode.vv7797.opmode.control.AutonomousSuite.ConcurrentInstruction;
import org.firstinspires.ftc.teamcode.vv7797.opmode.util.exception.HeartbeatException;
import org.firstinspires.ftc.teamcode.vv7797.opmode.util.exception.AbortException;

import java.util.ArrayList;


//import org.firstinspires.ftc.teamcode.vv7797.opmode.control.VictorianGamepad;

@TeleOp(name = "Supercell Michael")
// @Disabled
public class MichaelOp extends OpMode {
    private DcMotorEx dtFrontLeft, dtFrontRight, dtBackLeft, dtBackRight;
    private DcMotorEx drvIntakeLeft, drvIntakeRight, drvLinearSlide, drvVacuum;
    private Servo srvRamp, srvPhone, srvRelicArm, srvRelicClaw, srvArmSwivel, srvArmShoulder, srvLatch;
    private ElapsedTime runtime = new ElapsedTime();
    private boolean intake, intakeReverse, ramp, limiter, arm, claw, latch;
    private boolean cTIntake, cTIntakeReverse, cTRamp, cTLimiter, cTArm, cTClaw, cTCorrector, cTLatch;
    private final double INTAKE_CAP = 0.25, DRIVE_CAP = 0.6;
    //private final double DRIVE_TO_STRAFE_RATIO = 1.447375;
    private final double RELIC_ARM_POSITION_DOWN = 0;
    private final double RELIC_ARM_POSITION_UP = 1;
    private final double RELIC_CLAW_POSITION_CLOSED = 1;
    private final double RELIC_CLAW_POSITION_OPEN = 0.33;
    private double LFP, LRP, RFP, RRP, driveAngle, intakePower, INTAKE_MOD = INTAKE_CAP, DRIVE_MOD = DRIVE_CAP;
    private ArrayList<ConcurrentInstruction> schedule = new ArrayList<>();
    private DcMotorEx[] drivetrain;

    @Override
    public void init() {
        dtFrontLeft = (DcMotorEx)hardwareMap.dcMotor.get("drvFrontLeft");
        dtFrontRight = (DcMotorEx)hardwareMap.dcMotor.get("drvFrontRight");
        dtBackLeft = (DcMotorEx)hardwareMap.dcMotor.get("drvBackLeft");
        dtBackRight = (DcMotorEx)hardwareMap.dcMotor.get("drvBackRight");
        drvIntakeLeft = (DcMotorEx)hardwareMap.dcMotor.get("drvIntakeLeft");
        drvIntakeRight = (DcMotorEx)hardwareMap.dcMotor.get("drvIntakeRight");
        drvLinearSlide = (DcMotorEx)hardwareMap.dcMotor.get("drvLinearSlide");
        drvVacuum = (DcMotorEx)hardwareMap.dcMotor.get("drvVacuum");

        dtFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        dtFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        dtBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        dtBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        drvIntakeLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        drvIntakeRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        dtFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        dtFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        dtBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        dtBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        limiter = true;
        cTIntake = true;
        cTIntakeReverse = true;
        cTRamp = true;
        cTLimiter = true;
        cTArm = true;
        cTClaw = true;
        cTCorrector = true;
        cTLatch = true;

        dtFrontLeft.setDirection(DcMotor.Direction.REVERSE);
        dtBackLeft.setDirection(DcMotor.Direction.REVERSE);
        drvIntakeRight.setDirection(DcMotor.Direction.REVERSE);
        drvLinearSlide.setDirection(DcMotor.Direction.REVERSE);

        srvRamp = hardwareMap.servo.get("srvRamp");
        srvRamp.setPosition(1);
        srvPhone = hardwareMap.servo.get("srvPhone");
        srvPhone.setPosition(0.5);
        srvRelicArm = hardwareMap.servo.get("srvRelicArm");
        srvRelicArm.setPosition(RELIC_ARM_POSITION_DOWN);
        srvRelicClaw = hardwareMap.servo.get("srvRelicClaw");
        srvRelicClaw.setPosition(RELIC_CLAW_POSITION_CLOSED);
        srvArmSwivel = hardwareMap.servo.get("srvArmSwivel");
        srvArmSwivel.setPosition(0.5);
        srvArmShoulder = hardwareMap.servo.get("srvArmShoulder");
        srvArmShoulder.setPosition(1);
        srvLatch = hardwareMap.servo.get("srvLatch");
        srvLatch.setPosition(0);

        driveAngle = Math.PI/4;
        /*
        DcMotorEx[] drivetrain = new DcMotorEx[] { dtFrontLeft, dtFrontRight, dtBackLeft, dtBackRight };

        for (DcMotorEx motor : drivetrain) {
            //PIDCoefficients cfs = motor.getPIDCoefficients(DcMotor.RunMode.RUN_TO_POSITION);
            motor.setPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDCoefficients(10,0,0));
        }
        */

        telemetry.addData("Status", "Hardware initialized");
    }

    @Override
    public void start() { runtime.reset(); }

    @Override
    public void loop() {
        victorianDrive();
        setVacuumPower();
        setIntakePower();
        setRampPosition();
        setLinearSlide();
        setLinearServos();
        setJewelArm();
        setLimiter();
        runConcurrents();
        sendTelemetry();
    }

    @Override
    public void stop() {
        dtFrontLeft.setPower(0);
        dtFrontRight.setPower(0);
        dtBackLeft.setPower(0);
        dtBackRight.setPower(0);
    }

    // Analog Controls for setting correct drive train power
    private void victorianDrive() {
        // Collect controller inputs
        double lsx = gamepad1.left_stick_x, lsy = gamepad1.left_stick_y;
        double rsx = gamepad1.right_stick_x;
        double epsilon = Math.sqrt(2);
        double r = Math.hypot(lsx, lsy);
        double stickAngle = Math.atan2(lsy, -lsx);
        // Set "front" of the robot
        if(DPadPressed())
            setDriveAngle();
        // Calculate power angle derived from analog stick angle
        double powerAngle = stickAngle - driveAngle;
        // Calculate dual zone analog turn power
        double turn = getDualZonePower(rsx);
        // Calculate sin and cos power values
        double cos = r*Math.cos(powerAngle);
        double sin = r*Math.sin(powerAngle);
        // Ramp up power values
        double cosPow = epsilon*cos;
        double sinPow = epsilon*sin;
        // Calculate individual motor power values
        LFP = Range.clip(cosPow-turn, -1.0, 1.0);
        LRP = Range.clip(sinPow-turn, -1.0, 1.0);
        RFP = Range.clip(sinPow+turn, -1.0, 1.0);
        RRP = Range.clip(cosPow+turn, -1.0, 1.0);
        // Set power limiter
        LFP*=DRIVE_MOD;
        LRP*=DRIVE_MOD;
        RFP*=DRIVE_MOD;
        RRP*=DRIVE_MOD;

        dtFrontLeft.setPower(LFP);
        dtFrontRight.setPower(RFP);
        dtBackLeft.setPower(LRP);
        dtBackRight.setPower(RRP);
    }
    //  Sets the vacuum motor
    private void setVacuumPower() {
        drvVacuum.setPower(gamepad2.right_trigger - gamepad2.left_trigger);
    }
    // Returns proper turn power using dual zone
    private double getDualZonePower(double t) {
        if(Math.abs(t) < .5)
            return (.8*t*DRIVE_CAP);
        else
            return Math.signum(t)*(1.2*Math.abs(t)-0.2)*DRIVE_CAP;
    }
    // Returns Boolean (determine if D-Pad is being pressed)
    private boolean DPadPressed() {
        return (gamepad1.dpad_up || gamepad1.dpad_down || gamepad1.dpad_left || gamepad1.dpad_right);
    }
    // Sets the "front" of the robot
    private void setDriveAngle() {
        if(gamepad1.dpad_up)
            driveAngle = Math.PI/4;
        else if(gamepad1.dpad_left)
            driveAngle = 3*(Math.PI/4);
        else if(gamepad1.dpad_down)
            driveAngle = -3*(Math.PI/4);
        else if(gamepad1.dpad_right)
            driveAngle = -(Math.PI/4);
    }
    // Toggles Intake Motor Power
    private void setIntakePower() {
        boolean x = (gamepad1.x || gamepad2.x), y = (gamepad1.y || gamepad2.y);
        boolean c = (gamepad1.left_trigger > 0.1 || gamepad1.right_trigger > 0.1);
        if(x && cTIntake) {
            if(intakeReverse) {
                intakeReverse = false;
                cTIntakeReverse = true;
            }
            intake = !intake;
            cTIntake = false;
        }
        else if(!x)
            cTIntake = true;

        if(y && cTIntakeReverse) {
            if(intake) {
                intake = false;
                cTIntake = true;
            }
            intakeReverse = !intakeReverse;
            cTIntakeReverse = false;
        }
        else if(!y)
            cTIntakeReverse = true;

        if(intake)
            intakePower = 1*INTAKE_MOD;
        else if(intakeReverse)
            intakePower = -1*INTAKE_MOD;
        else
            intakePower = 0;

        if (c && cTCorrector) {
            intakePower = -1;
            intakeReverse = true;
            intake = false;
            cTCorrector = false;

            scheduleConcurrent(new NestedInstruction() {
                @Override public String run() {
                    intakePower = 1;
                    intakeReverse = false;
                    intake = true;
                    return "";
                }
            }, 0.5);
        } else if (!c)
            cTCorrector = true;

        drvIntakeLeft.setPower(intakePower);
        drvIntakeRight.setPower(intakePower);
    }
    // Toggles ramp position
    private void setRampPosition() {
        boolean lb = gamepad1.left_bumper, rb = gamepad1.right_bumper;
        // Both bumpers raise the ramp
        if((lb || rb) && cTRamp) {
            cTRamp = false;
            ramp = !ramp;
            srvRamp.setPosition(ramp ? 0 : 1);
        }
        else if(!lb && !rb)
            cTRamp = true;
    }
    // Controls Linear Slide Power
    private void setLinearSlide() {
        drvLinearSlide.setPower(gamepad2.left_stick_y);
    }
    // Toggles Relic Arm & Relic Claw Positions
    private void setLinearServos() {
        boolean lb = gamepad2.left_bumper, rb = gamepad2.right_bumper;
        // Toggles Relic Arm Position
        if(lb && cTArm) {
            arm = !arm;
            cTArm = false;
            // Set Relic Arm Position
            srvRelicArm.setPosition(arm ? RELIC_ARM_POSITION_UP : RELIC_ARM_POSITION_DOWN);
        }
        else if(!lb)
            cTArm = true;
        // Toggles Relic Claw Position
        if(rb && cTClaw) {
            claw = !claw;
            cTClaw = false;
            // Set Relic Claw Position
            srvRelicClaw.setPosition(claw ? RELIC_CLAW_POSITION_OPEN: RELIC_CLAW_POSITION_CLOSED);
        }
        else if(!rb)
            cTClaw = true;
        // Toggles the latch
        boolean a = gamepad2.a;
        if (a && cTLatch) {
            latch = !latch;
            cTLatch = false;
            srvLatch.setPosition(latch ? 1 : 0);
        } else if (!a)
            cTLatch = true;
    }
    // Controls Linear Slide Power
    private void setJewelArm() {
        /*double xAngle = srvArmSwivel.getPosition() + (0.1 * gamepad2.right_stick_x);
        double yAngle = srvArmShoulder.getPosition() + (0.1 * gamepad2.right_stick_y);

        srvArmSwivel.setPosition((gamepad2.right_stick_x + 1) / 2);
        srvArmShoulder.setPosition((gamepad2.right_stick_y + 1) / 2);*/

        final double speed = 0.001;
        final double deadzone = 0.05;
        final double xSpeed = (Math.abs(gamepad2.right_stick_x) < deadzone ? 0 : speed * Math.signum(gamepad2.right_stick_x));
        final double ySpeed = (Math.abs(gamepad2.right_stick_y) < deadzone ? 0 : speed * Math.signum(gamepad2.right_stick_y));

        srvArmSwivel.setPosition(srvArmSwivel.getPosition() + xSpeed);
        srvArmShoulder.setPosition(srvArmShoulder.getPosition() + ySpeed);


    }
    // Sets thee motor restrictions of the robot's motors
    private void setLimiter() {
        boolean ls = gamepad1.left_stick_button, rs = gamepad1.right_stick_button;
        // Both stick buttons toggle intake mods
        if((ls || rs) && cTLimiter) {
            limiter = !limiter;
            cTLimiter = false;
            // Set Restrictions
            INTAKE_MOD = limiter ? INTAKE_CAP : 1;
            DRIVE_MOD = limiter ? DRIVE_CAP : 1;

        }
        else if(!ls && !rs)
            cTLimiter = true;
    }
    // Run concurrent isntructions from the schedule
    private void runConcurrents() {
        for (int i = schedule.size() - 1; i >= 0; i--) {
            ConcurrentInstruction inst = schedule.get(i);

            try {
                if (runtime.time() >= inst.TIME)
                    schedule.remove(i).NESTED.run();
            } catch (HeartbeatException e) {
            } catch (AbortException e) {}
        }
    }
    // Send Telemetry
    private void sendTelemetry() {
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Front Motors", "Left (%.2f) | Right (%.2f)", LFP, RFP);
        telemetry.addData("Rear Motors", "Left (%.2f) | Right (%.2f)", LRP, RRP);
        telemetry.addData("Intake Power","%.2f",intakePower);
        telemetry.addData("Limiter",limiter);
        telemetry.addData("Ramp Engaged",ramp);
        telemetry.update();
    }
    // Schedule an instruction to be executed some time from now
    private void scheduleConcurrent(NestedInstruction nested, double inTime) {
        schedule.add(new ConcurrentInstruction(nested, runtime.time() + inTime));
    }
}
