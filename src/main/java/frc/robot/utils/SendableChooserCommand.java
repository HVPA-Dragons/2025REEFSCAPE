package frc.robot.utils;

import java.util.HashMap;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SelectCommand;

/**
 * A command that allows you to choose between a number of commands to run with
 * a {@link SendableChooser}.
 * 
 * <p>
 * 
 * SendableChooser options are populated using {@link Command#getName()}, which
 * uses the class name by default. Names can be changed by calling
 * {@link Command#setName(String)}.
 */
public class SendableChooserCommand extends SelectCommand<String> {
    /**
     * Creates a new {@link SendableChooserCommand} with the given name.
     * 
     * @param name     the name of the {@link SendableChooser}
     * @param commands the commands to choose from
     */
    public SendableChooserCommand(String name, Command... commands) {
        this(name, createMap(commands), createChooser(name, commands));
    }

    private SendableChooserCommand(String name, HashMap<String, Command> map, SendableChooser<String> chooser) {
        super(map, chooser::getSelected);
        SmartDashboard.putData(name, chooser);
    }

    private static HashMap<String, Command> createMap(Command... commands) {
        HashMap<String, Command> map = new HashMap<>();
        for (Command c : commands) {
            map.put(c.getName(), c);
        }
        return map;
    }

    private static SendableChooser<String> createChooser(String name, Command... commands) {
        SendableChooser<String> chooser = new SendableChooser<>();
        chooser.setDefaultOption(commands[0].getName(), commands[0].getName());
        for (int i = 1; i < commands.length; i++) {
            chooser.addOption(commands[i].getName(), commands[i].getName());
        }
        return chooser;
    }
}