package org.firstinspires.ftc.teamcode.lib;

import androidx.annotation.Nullable;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.function.BooleanSupplier;


/**
 * Represents an operation that can temporarily suspend itself
 */
public abstract class Coroutine{
    @Nullable
    protected abstract Object step();

    private Object latest = null;

    private static class Termination {

    }

    private static class SuspendCondition{
        private final BooleanSupplier condition;

        public SuspendCondition(BooleanSupplier condition){
            this.condition = condition;
        }

        public boolean done(){
            return condition.getAsBoolean();
        }
    }

    private static class SuspendDelay{
        @Nullable
        public Double startTime = null;

        public final double delay;

        public SuspendDelay(double seconds){
            delay = seconds;
        }

        public boolean done(double currentTime){
            if (startTime == null) return false;
            return currentTime - startTime > delay;
        }
    }

    public static class Manager{
        private final HashSet<Coroutine> routines;

        public Manager(){
            routines = new HashSet<>();
        }

        public void step(double time){
            //iterate over copy so we can modify the set as we go
            for(Coroutine routine : new ArrayList<>(routines)){
                Object o = routine.latest;

                //noinspection StatementWithEmptyBody
                if(o == null){

                } else if (o instanceof Termination){
                    end(routine);
                    return;
                } else if (o instanceof SuspendDelay){
                    SuspendDelay d = (SuspendDelay) o;
                    if (d.startTime == null){
                        d.startTime = time;
                        return; //no matter what this will cause a 1 frame delay
                    }
                    if (!d.done(time)){
                        return;
                    }
                } else if (o instanceof SuspendCondition){
                    SuspendCondition d = (SuspendCondition) o;
                    if (!d.done()) return;
                }

                routine.latest = routine.step();
            }
        }

        public Coroutine start(Coroutine routine){
            routines.add(routine);
            return routine;
        }

        public boolean end(Coroutine routine){
            return routines.remove(routine);
        }
    }
}
