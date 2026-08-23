package org.firstinspires.ftc.teamcode.opModes.auto.base;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.ivy.Command;
import com.pedropathing.paths.PathChain;
import org.firstinspires.ftc.teamcode.base.AutoOpMode;

import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.groups.Groups.sequential;

@Configurable
public abstract class Goal extends AutoOpMode {
    protected PathChain Path1;

    @Override
    public void loop() {
        super.loop();

    }

    @Override
    protected Command autoRoutine() {
        return sequential(

        );
    }
}
