import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.motor.Motor;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.NXTUltrasonicSensor;
import lejos.robotics.Color;
import lejos.robotics.chassis.Wheel;
import lejos.robotics.chassis.WheeledChassis;
import lejos.robotics.navigation.MovePilot;
import lejos.utility.Delay;

public class Main { 
	// 	Variables for calculating speeds of left/right motors
	public static int power;
	public static int factor;
	public static float avgLight, aSpeed, bSpeed, f;
	public static boolean capturedBall;
	
	public static void main(String[] args) {	
		Wheel left = WheeledChassis.modelWheel(Motor.A, 42).offset(-56);
		Wheel right = WheeledChassis.modelWheel(Motor.B, 42).offset(56);
		Wheel claw = WheeledChassis.modelWheel(Motor.C, 50);
		WheeledChassis chassis = new WheeledChassis(new Wheel[] {left, right}, WheeledChassis.TYPE_DIFFERENTIAL);	  
    	MovePilot pilot = new MovePilot(chassis);
		
		EV3ColorSensor lineColorSensor = new EV3ColorSensor(SensorPort.S4);
		NXTUltrasonicSensor ultra = new NXTUltrasonicSensor(SensorPort.S2);
    	
    	pilot.setAngularAcceleration(150);
    	pilot.setAngularSpeed(45);
    	pilot.setLinearAcceleration(250);
    	pilot.setLinearSpeed(45);

		claw.getMotor().setAcceleration(250);
    	claw.getMotor().setSpeed(80);  	
    	
    	//	Variables for samples
		float[] color = new float[1];
		float[] distance = new float[1];
		//	Get the SURFACE light level sample. Use contrasting colors
		float[] sampleSurface = new float[1];	
		//	Get the LINE light level sample. Use contrasting colors
		float[] sampleLine = new float[1];
		
		// tracks whether we've captured the ball or not
		capturedBall = false;
		
		//	Startup sequence
    	Sound.beepSequenceUp();
		Delay.msDelay(500);
		Sound.beep();
		
		//	Press Escape when you are hovered over general SURFACE you want to sample
		Button.ESCAPE.waitForPressAndRelease();
		lineColorSensor.getRedMode().fetchSample(sampleSurface, 0);
		
		//	Wait 0.5 sec
		Delay.msDelay(500);
		Sound.beep();
		
		//	Press Escape when you are hovered over LINE you want to sample
		Button.ESCAPE.waitForPressAndRelease();
		lineColorSensor.getRedMode().fetchSample(sampleLine, 0);

		//	Wait 5 sec to position robot
		Delay.msDelay(5000);
		
		//	Variables for calculating speeds of right/left motors
		power = 30;
		factor = 100;
		//	the light level averaged between the LINE and the SURFACE levels
		avgLight = (sampleLine[0] + sampleSurface[0]) / 2;
		//	speed = power +/- factor * (avgLight - color[0]) / f);
		bSpeed = 0;
		aSpeed = 0;
		f = sampleSurface[0] - sampleLine[0];
		
		//	runs until we capture the ball, and make our way to exit
		//	when it reaches the exit it will drop the ball and break from the loop
		while (!Button.ESCAPE.isDown()) {
			//	Refresh the color sample
			lineColorSensor.getRedMode().fetchSample(color, 0);
			followLine(pilot, left, right, color);
			
			//	Keep sampling distances until we have caught the ball
			if(!capturedBall) {
				ultra.getDistanceMode().fetchSample(distance, 0);
//				Testing the samples to see if the ball is inrange
				detectBall(pilot, claw, ultra, distance);
			}
			
			//	Refresh color sample for color ID
			lineColorSensor.getColorIDMode().fetchSample(color, 0);
			//	Test the color sample for green or yellow
			detectColors(pilot, claw, left, right, color);
			
			if (color[0] == Color.GREEN && capturedBall)
				break;
		}
		
		lineColorSensor.close();
	}
	
	public static void followLine(MovePilot pilot, Wheel left, Wheel right, float[] color) {
//		if 0.04 away from the average (off course)
		if(color[0] < avgLight - 0.04) {
			pilot.rotate(-5);
		} else {
			//	Right motor speed
			aSpeed = power - factor * (avgLight - color[0]) / f;
			//	Left motor speed
			bSpeed = power + factor * (avgLight - color[0]) / f;

			right.getMotor().setSpeed((int) aSpeed * 3);
			right.getMotor().forward();	

			left.getMotor().setSpeed((int) bSpeed * 3);
			left.getMotor().forward();
		}
	}
	
	public static void detectBall(MovePilot pilot, Wheel claw, NXTUltrasonicSensor ultra, float[] distance) {
		if(distance[0] < 0.08 && !capturedBall) {
			pilot.travel(-100);
			claw.getMotor().rotate(-360);
			pilot.rotate(-5);
			pilot.travel(100);
			claw.getMotor().rotate(320);
			capturedBall = true;
			System.out.println("Captured ball");
			ultra.close();
		}
	}
	
	public static void detectColors(MovePilot pilot, Wheel claw, Wheel left, Wheel right, float[] color) {
		if(color[0] == Color.BROWN || (color[0] == Color.GREEN && !capturedBall)) {
			pilot.rotate(-150);
		} else if (color[0] == Color.GREEN && capturedBall) {
			right.getMotor().stop();
			left.getMotor().stop();
			
			pilot.travel(100);
			
			claw.getMotor().rotate(-260);
			pilot.travel(-50);
			claw.getMotor().rotate(320);
		}
	}
}
