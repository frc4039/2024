package frc.robot.utils;

import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class MultiButtonTrigger extends Trigger {

	public MultiButtonTrigger(JoystickButton... buttons) {
		super(() -> {
			for(JoystickButton b : buttons){
				if(!b.getAsBoolean()){
					return false;
				}
			}
			return true;
		});
	}
}
