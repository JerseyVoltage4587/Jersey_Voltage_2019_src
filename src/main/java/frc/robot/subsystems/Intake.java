package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.*;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

import frc.robot.util.Gyro;
import frc.robot.util.VisionMath;
import frc.robot.util.AsyncStructuredLogger;
import frc.robot.util.CalcPathToTarget;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Trajectory.Segment;

import java.io.File;
import java.io.FileWriter;
import java.util.TimerTask;

import frc.robot.paths.*;

import frc.robot.Constants;
import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.loops.Loop;
import frc.robot.loops.Looper;
import frc.robot.util.ArcMath;
import frc.robot.util.AsyncAdHocLogger;
import frc.robot.util.AsyncStructuredLogger;
import frc.robot.util.DriveSignal;

public class Intake extends Subsystem {
	
	private long startTime;

    private static Intake mInstance = null;
    public static Intake getInstance() {
    	if ( mInstance == null ) {
    		synchronized ( Drive.class ) {
    			mInstance = new Intake();
    		}
    	}
    	return mInstance;
    }

    // The robot drivetrain's various states.
    public enum IntakeControlState {
        OFF, // open loop voltage control
		INTAKE_BALL, // used for autonomous driving
		HOLD_BALL,
		INTAKE_HATCH,
		HOLD_HATCH,
    }
    public IntakeControlState getState(){
    	return mIntakeControlState;
    }
    // Control states
    private IntakeControlState mIntakeControlState = IntakeControlState.OFF;

    // Hardware
    private final WPI_TalonSRX mIntakeTalon;

    // Logging
    private DebugOutput mDebugOutput;
	private final AsyncStructuredLogger<DebugOutput> mCSVWriter;
	
    public void startIntakeBall() {
    	synchronized (Intake.this) {
    		mIntakeControlState = IntakeControlState.INTAKE_BALL;
    	}
	}
	public void startHoldBall() {
    	synchronized (Intake.this) {
    		mIntakeControlState = IntakeControlState.HOLD_BALL;
    	}
	}
	public void startOff() {
    	synchronized (Intake.this) {
    		mIntakeControlState = IntakeControlState.OFF;
    	}
	}
	public void startIntakeHatch() {
    	synchronized (Intake.this) {
    		mIntakeControlState = IntakeControlState.INTAKE_HATCH;
    	}
	}
	public void startHoldHatch() {
    	synchronized (Intake.this) {
    		mIntakeControlState = IntakeControlState.HOLD_HATCH;
    	}
	}
	public void setState(IntakeControlState state){
		synchronized (Intake.this) {
    		mIntakeControlState = state;
    	}
	}

	
	AsyncAdHocLogger asyncAdHocLogger;
    int iCall = 0;
    private final Loop mLoop = new Loop() {

        @Override
        public void onStart(double timestamp) {
            synchronized (Intake.this) {
            	startTime = System.nanoTime();
				asyncAdHocLogger = new AsyncAdHocLogger("");
            }
        }

        @Override
        public void onLoop(double timestamp) {
			iCall++;
			if(iCall % 1000 == 0){
        		//System.out.println("onLoop " + iCall + " " + mDriveControlState + " " + mLeftMaster.getControlMode());
			}
			//asyncAdHocLogger.q("intakeState: ").q(mIntakeControlState.name()).go();
            synchronized (Intake.this) {
                switch (mIntakeControlState) {
                case OFF:
					//do nothing
					mIntakeTalon.set(0.0);
					pokeIn();
					openFingers();
                    break;
				case INTAKE_BALL:
					mIntakeTalon.set(1.0);
					pokeIn();
					openFingers();
					if(mIntakeTalon.getOutputCurrent() > Constants.kIntakeStallCurrent){
						count++;
					}else{
						count = 0;
					}
					if(count >= 10){
						//have ball
						setState(IntakeControlState.HOLD_BALL);
						flashLEDs();
					}
					break;
				case HOLD_BALL:
					mIntakeTalon.set(0.3);
					pokeIn();
					openFingers();
					break;
				case INTAKE_HATCH:
					mIntakeTalon.set(0.0);
					pokeOut();
					closeFingers();
					break;
				case HOLD_HATCH:
					mIntakeTalon.set(0.0);
					pokeOut();
					openFingers();
					break;
                default:
                    System.out.println("Unexpected climb control state: " + mIntakeControlState);
                    break;
                }
			}
        	logValues();
        }

        @Override
        public void onStop(double timestamp) {
            stop();
            mCSVWriter.flush();
        }
	};

	public class FlashTimer extends TimerTask{
		public void run(){
			NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable("limelight-front");
			limelightTable.getEntry("ledMode").forceSetNumber(0);
			
			NetworkTable limelightTable1 = NetworkTableInstance.getDefault().getTable("limelight-back");
			limelightTable1.getEntry("ledMode").forceSetNumber(0);
		}
	}

	int count = 0;

	private void flashLEDs(){
		NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable("limelight-front");
		limelightTable.getEntry("ledMode").forceSetNumber(2);
		NetworkTable limelightTable1 = NetworkTableInstance.getDefault().getTable("limelight-back");
		limelightTable1.getEntry("ledMode").forceSetNumber(2);

		m_timer.schedule(new FlashTimer(),2000);
	}

	java.util.Timer m_timer = null;
	
	private final Solenoid poke, fingers;
	private boolean pokeState = false;
	private boolean fingerState = false;

	private void openFingers(){
		if(fingerState != RobotMap.kFingersOpen){
			fingers.set(RobotMap.kFingersOpen);
			fingerState = RobotMap.kFingersOpen;
		}
	}
	private void closeFingers(){
		if(fingerState != RobotMap.kFingersClosed){
			fingers.set(RobotMap.kFingersClosed);
			fingerState = RobotMap.kFingersClosed;
		}
	}
	private void pokeOut(){
		if(pokeState != RobotMap.kPokeOut){
			poke.set(RobotMap.kPokeOut);
			pokeState = RobotMap.kPokeOut;
		}
	}
	private void pokeIn(){
		if(pokeState != RobotMap.kPokeIn){
			poke.set(RobotMap.kPokeIn);
			pokeState = RobotMap.kPokeIn;
		}
	}

	private Intake() {
		// Start all Talons in open loop mode.
        mIntakeTalon = new WPI_TalonSRX(RobotMap.INTAKE_TALON);
        mIntakeTalon.setNeutralMode(NeutralMode.Brake);
		mIntakeTalon.configNeutralDeadband(0.01, 10);

		poke = new Solenoid(1,RobotMap.POKE);
		fingers = new Solenoid(1,RobotMap.FINGERS);

        mDebugOutput = new DebugOutput();
        mCSVWriter = new AsyncStructuredLogger<DebugOutput>("IntakeLog" ,DebugOutput.class);
		m_timer = new java.util.Timer();
	}

    @Override
    public void registerEnabledLoops(Looper in) {
        in.register(mLoop);
    }

    @Override
    public synchronized void stop() {
        
    }

    @Override
    public void outputToSmartDashboard() {
		SmartDashboard.putNumber("intake percent output", mIntakeTalon.getMotorOutputPercent());
		SmartDashboard.putNumber("intake motor current", mIntakeTalon.getOutputCurrent());
    }
    
    public static class DebugOutput{
    	public long sysTime;
    	public String intakeMode;
		public double intakeMotorPercent;
		public double intakeMotorCurrent;
    }
    
    private void logValues(){
    	mDebugOutput.sysTime = System.nanoTime()-startTime;
		mDebugOutput.intakeMode = mIntakeControlState.name();
		mDebugOutput.intakeMotorPercent = mIntakeTalon.getMotorOutputPercent();
		mDebugOutput.intakeMotorCurrent = mIntakeTalon.getOutputCurrent();
		mCSVWriter.queueData(mDebugOutput);
    }

    public synchronized void resetEncoders() {
    }

    @Override
    public void zeroSensors() {
        resetEncoders();
    }

       @Override
    public void writeToLog() {
        //mCSVWriter.write();
    }

@Override
protected void initDefaultCommand() {
	// TODO Auto-generated method stub
	
}       
}