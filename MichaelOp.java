package org.firstinspires.ftc.teamcode.vv7797.opmode;
/**
 * TeleOp written by FTC Team 7797 for the 2017-2018 Relic Recovery Game
 *
 * This code was designed and written for the most convenient and enjoyable driver experience.
 *
 * The TeleOp allows drivers to effortlessly move in any direction with the help of polar
 * mathematics, turn with ease thanks to "dual zone" analog stick calculations, toggle intake
 * motors and ramp positions with the push of a button, extend and retract linear slide, and
 * control servos and other ammenities with ease.
 *
 * This file is intenitonally commented to explain even the minorest of details and controls...
 * If it seems as though some aspects of the code have been overexplained, keep this in mind.
 *
 * Michael Munoz, 2017/18 7797 Software Co-Lead
 * michaelmunoz1101@gmail.com
 */

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

@TeleOp(name = "Supercell Michael")
public class MichaelOp extends OpMode {
    // Declare Hardware (Motors and servos to be used during TeleOp)
    private DcMotorEx dtFrontLeft, dtFrontRight, dtBackLeft, dtBackRight;
    private DcMotorEx drvIntakeLeft, drvIntakeRight, drvLinearSlide, drvVacuum;
    private Servo srvRamp, srvPhone, srvRelicArm, srvRelicClaw, srvArmSwivel, srvArmShoulder, srvLatch;

    // Declare Servo Positions And Motor Speed Limiters
    private final double INTAKE_CAP = 0.25;
    private final double DRIVE_CAP = 0.6;
    private final double EPSILON = Math.sqrt(2);
    private final double RELIC_ARM_POSITION_DOWN = 0;
    private final double RELIC_ARM_POSITION_UP = 1;
    private final double RELIC_CLAW_POSITION_CLOSED = 1;
    private final double RELIC_CLAW_POSITION_OPEN = 0.33;

    // Declare Boolean Variables
    private boolean intake, intakeReverse, ramp, limiter, arm, claw, latch;
    private boolean cTIntake, cTIntakeReverse, cTRamp, cTLimiter, cTArm, cTClaw, cTCorrector, cTLatch;

    // Declare Motor Power Variables
    private double LFP, LRP, RFP, RRP, driveAngle, intakePower;
    private double INTAKE_MOD = INTAKE_CAP, DRIVE_MOD = DRIVE_CAP;

    // Declare Data Structures
    private ArrayList<ConcurrentInstruction> schedule = new ArrayList<>();
    private DcMotorEx[] drivetrain;
    private ElapsedTime runtime = new ElapsedTime();


    /**
     * Initialize all hardware on the robot and any variables used to control the robot
     * Executes during TeleOp initialization
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Hardware Initialization Started");

        // Instantiate motors
        dtFrontLeft = (DcMotorEx)hardwareMap.dcMotor.get("drvFrontLeft");
        dtFrontRight = (DcMotorEx)hardwareMap.dcMotor.get("drvFrontRight");
        dtBackLeft = (DcMotorEx)hardwareMap.dcMotor.get("drvBackLeft");
        dtBackRight = (DcMotorEx)hardwareMap.dcMotor.get("drvBackRight");
        drvIntakeLeft = (DcMotorEx)hardwareMap.dcMotor.get("drvIntakeLeft");
        drvIntakeRight = (DcMotorEx)hardwareMap.dcMotor.get("drvIntakeRight");
        drvLinearSlide = (DcMotorEx)hardwareMap.dcMotor.get("drvLinearSlide");
        drvVacuum = (DcMotorEx)hardwareMap.dcMotor.get("drvVacuum");

        // Set motor run modes
        dtFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        dtFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        dtBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        dtBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        drvIntakeLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        drvIntakeRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Set drivetrain motor braking behavior (without this the robot will 'glide' after no power is given)
        dtFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        dtFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        dtBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        dtBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set motor directions
        dtFrontLeft.setDirection(DcMotor.Direction.REVERSE);
        dtBackLeft.setDirection(DcMotor.Direction.REVERSE);
        drvIntakeRight.setDirection(DcMotor.Direction.REVERSE);
        drvLinearSlide.setDirection(DcMotor.Direction.REVERSE);

        // Instantiate servos
        srvRamp = hardwareMap.servo.get("srvRamp");
        srvPhone = hardwareMap.servo.get("srvPhone");
        srvRelicArm = hardwareMap.servo.get("srvRelicArm");
        srvRelicClaw = hardwareMap.servo.get("srvRelicClaw");
        srvArmSwivel = hardwareMap.servo.get("srvArmSwivel");
        srvArmShoulder = hardwareMap.servo.get("srvArmShoulder");
        srvLatch = hardwareMap.servo.get("srvLatch");

        // Set initial servo positions
        srvRamp.setPosition(1);
        srvPhone.setPosition(0.5);
        srvRelicArm.setPosition(RELIC_ARM_POSITION_DOWN);
        srvRelicClaw.setPosition(RELIC_CLAW_POSITION_CLOSED);
        srvArmSwivel.setPosition(0.5);
        srvArmShoulder.setPosition(1);
        srvLatch.setPosition(0);

        // Set drive train to go forward
        driveAngle = Math.PI/4;

        // Instantiate boolean variables (used for toggle controls)
        limiter = true;
        cTIntake = true;
        cTIntakeReverse = true;
        cTRamp = true;
        cTLimiter = true;
        cTArm = true;
        cTClaw = true;
        cTCorrector = true;
        cTLatch = true;

        telemetry.addData("Status", "Hardware Initialized");
    }


    /**
     * Reset runtime
     * Executes upon start of TeleOp
     */
    @Override
    public void start() { runtime.reset(); }


    /**
     * Translate driver controller inputs into onboard robot hardware commands
     * Continuosly loops until completion of TeleOp
     */
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


    /**
     * Kill all drivetrain motors
     * Executes upon completion of TeleOp
     */
    @Override
    public void stop() {
        dtFrontLeft.setPower(0);
        dtFrontRight.setPower(0);
        dtBackLeft.setPower(0);
        dtBackRight.setPower(0);
    }


    /**
     * Calculate and set drive train motor power
     * Ability for driver to modify drivetrain power limiter
     */
    private void victorianDrive() {
        // Collect controller inputs
        double lsx = gamepad1.left_stick_x;
        double lsy = gamepad1.left_stick_y;
        double rsx = gamepad1.right_stick_x;

        // Set variables used for polar calculations
        double r = Math.hypot(lsx, lsy);
        double stickAngle = Math.atan2(lsy, -lsx);

        // Set "front" of the robot
        if(padPressed())
            setDriveAngle();

        // Calculate power angle derived from analog stick angle
        double powerAngle = stickAngle - driveAngle;

        // Calculate dual zone analog turn power
        double turn = getDualZonePower(rsx);

        // Calculate sin and cos power values
        double cos = r*Math.cos(powerAngle);
        double sin = r*Math.sin(powerAngle);

        // Ramp up power values
        double cosPow = EPSILON*cos;
        double sinPow = EPSILON*sin;

        // Calculate individual motor power values
        LFP = Range.clip(cosPow-turn, -1.0, 1.0);
        LRP = Range.clip(sinPow-turn, -1.0, 1.0);
        RFP = Range.clip(sinPow+turn, -1.0, 1.0);
        RRP = Range.clip(cosPow+turn, -1.0, 1.0);

        // Set drive train power limiter
        LFP*=DRIVE_MOD;
        LRP*=DRIVE_MOD;
        RFP*=DRIVE_MOD;
        RRP*=DRIVE_MOD;

        // Set drivetrain motor power
        dtFrontLeft.setPower(LFP);
        dtFrontRight.setPower(RFP);
        dtBackLeft.setPower(LRP);
        dtBackRight.setPower(RRP);
    }


    /**
     * Set jewel collection motor power
     */
    private void setVacuumPower() {
        drvVacuum.setPower(gamepad2.right_trigger - gamepad2.left_trigger);
    }


    /**
     * Calculate turn power using "dual zone" piece-wise math
     *
     * @param t right stick x position
     * @return drivetrain turn power
     */
    private double getDualZonePower(double t) {
        if(Math.abs(t) < .5)
            return (.8*t*DRIVE_CAP);
        else
            return Math.signum(t)*(1.2*Math.abs(t)-0.2)*DRIVE_CAP;
    }


    /**
     * Determine if D-Pad is being pressed
     *
     * @return dpad being pressed
     */
    private boolean padPressed() {
        return (gamepad1.dpad_up || gamepad1.dpad_down || gamepad1.dpad_left || gamepad1.dpad_right);
    }


    /**
     * Set the "front" of the robot
     */
    private void setDriveAngle() {
        if(gamepad1.dpad_up)
            driveAngle = Math.PI/4;
        else if(gamepad1.dpad_left)
            driveAngle = -(Math.PI/4);
        else if(gamepad1.dpad_down)
            driveAngle = -3*(Math.PI/4);
        else if(gamepad1.dpad_right)
            driveAngle = 3*(Math.PI/4);
    }


    /**
     * Toggle intake motor power
     */
    private void setIntakePower() {
        boolean x = (gamepad1.x || gamepad2.x);
        boolean y = (gamepad1.y || gamepad2.y);
        boolean c = (gamepad1.left_trigger > 0.1 || gamepad1.right_trigger > 0.1);

        // Intake toggle
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

        // Reverse intake toggle
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

        // Set intake motor power limiter
        if(intake)
            intakePower = 1*INTAKE_MOD;
        else if(intakeReverse)
            intakePower = -1*INTAKE_MOD;
        else
            intakePower = 0;

        // Intake corrector toggle
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

        // Set intake motor power
        drvIntakeLeft.setPower(intakePower);
        drvIntakeRight.setPower(intakePower);
    }


    /**
     * Toggles ramp position
     */
    private void setRampPosition() {
        boolean lb = gamepad1.left_bumper;
        boolean rb = gamepad1.right_bumper;

        // Both bumpers raise the ramp
        if((lb || rb) && cTRamp) {
            cTRamp = false;
            ramp = !ramp;
            srvRamp.setPosition(ramp ? 0 : 1);
        }
        else if(!lb && !rb)
            cTRamp = true;
    }


    /**
     * Set linear slide motor power
     */
    private void setLinearSlide() {
        drvLinearSlide.setPower(gamepad2.left_stick_y);
    }


    /**
     * Toggle relic arm & relic claw servo positions
     */
    private void setLinearServos() {
        boolean lb = gamepad2.left_bumper;
        boolean rb = gamepad2.right_bumper;

        // Toggles relic arm
        if(lb && cTArm) {
            arm = !arm;
            cTArm = false;
            // Set Relic Arm Position
            srvRelicArm.setPosition(arm ? RELIC_ARM_POSITION_UP : RELIC_ARM_POSITION_DOWN);
        }
        else if(!lb)
            cTArm = true;

        // Toggles relic claw
        if(rb && cTClaw) {
            claw = !claw;
            cTClaw = false;
            // Set Relic Claw Position
            srvRelicClaw.setPosition(claw ? RELIC_CLAW_POSITION_OPEN: RELIC_CLAW_POSITION_CLOSED);
        }
        else if(!rb)
            cTClaw = true;

        // Toggles latch
        boolean a = gamepad2.a;
        if (a && cTLatch) {
            latch = !latch;
            cTLatch = false;
            srvLatch.setPosition(latch ? 1 : 0);
        } else if (!a)
            cTLatch = true;
    }


    /**
     * Set jewel arm servo positions
     */
    private void setJewelArm() {
        final double speed = 0.002;
        final double deadzone = 0.05;
        final double xSpeed = (Math.abs(gamepad2.right_stick_x) < deadzone ? 0 : speed * Math.signum(gamepad2.right_stick_x));
        final double ySpeed = (Math.abs(gamepad2.right_stick_y) < deadzone ? 0 : speed * Math.signum(gamepad2.right_stick_y));

        srvArmSwivel.setPosition(srvArmSwivel.getPosition() + xSpeed);
        srvArmShoulder.setPosition(srvArmShoulder.getPosition() + ySpeed);
    }


    /**
     * Set intake and drivetrain motor power limiter
     */
    private void setLimiter() {
        boolean ls = gamepad1.left_stick_button;
        boolean rs = gamepad1.right_stick_button;

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


    /**
     * Run concurrent isntructions from the schedule
     */
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


    /**
     * Send onboard robot telemetry to driver station
     */
    private void sendTelemetry() {
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Front Motors", "Left (%.2f) | Right (%.2f)", LFP, RFP);
        telemetry.addData("Rear Motors", "Left (%.2f) | Right (%.2f)", LRP, RRP);
        telemetry.addData("Intake Power","%.2f",intakePower);
        telemetry.addData("Limiter",limiter);
        telemetry.addData("Ramp Engaged",ramp);
        telemetry.update();
    }


    /**
     * Schedule an instruction to be executed some time from now
     */
    private void scheduleConcurrent(NestedInstruction nested, double inTime) {
        schedule.add(new ConcurrentInstruction(nested, runtime.time() + inTime));
    }
}
