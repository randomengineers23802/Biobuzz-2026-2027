package org.firstinspires.ftc.teamcode.opModes.auto.base;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.ivy.Command;
import com.pedropathing.paths.Path;
import org.firstinspires.ftc.teamcode.base.AutoOpMode;
import static com.pedropathing.ivy.groups.Groups.sequential;

@Configurable
public abstract class Far extends AutoOpMode {
    protected abstract Path path1();
    protected abstract Path path2();
    protected abstract Path path3();


    @Override
    public void loop() {
        super.loop();

    }

    @Override
    protected Command autoRoutine() {
        return sequential(
                follow(path1()),
                fastFollow(path2()),
                follow(path3())
        );
    }
}
