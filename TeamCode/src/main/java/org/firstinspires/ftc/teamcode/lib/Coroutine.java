package org.firstinspires.ftc.teamcode.lib;

import androidx.annotation.Nullable;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashSet;
import java.util.function.BooleanSupplier;


/**
 * Represents an operation that can temporarily suspend itself
 */
public abstract class Coroutine{
    @Nullable
    protected abstract Object loop();

    private Object latest = null;

    public static class Yield{
        public static Object termination(){
            return new Termination();
        }
        public static Object wait(BooleanSupplier condition){
            return new SuspendCondition(condition);
        }
        public static Object delay(double seconds){
            return new SuspendDelay(seconds);
        }
    }

    public final boolean isTerminated(){
        return latest instanceof Termination;
    }

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

        private final HashSet<Coroutine> disabled;

        public Manager(){
            routines = new HashSet<>();
            disabled = new HashSet<>();
        }

        public void loop(double time){
            //iterate over copy so we can modify the set as we go
            for(Coroutine routine : new ArrayList<>(routines)){
                Object o = routine.latest;

                //noinspection StatementWithEmptyBody
                if(o == null){

                } else if (o instanceof Termination){
                    end(routine);
                    continue;
                } else if (o instanceof SuspendDelay){
                    SuspendDelay d = (SuspendDelay) o;
                    if (d.startTime == null){
                        d.startTime = time;
                        continue; //no matter what this will cause a 1 frame delay
                    }
                    if (!d.done(time)){
                        continue;
                    }
                } else if (o instanceof SuspendCondition){
                    SuspendCondition d = (SuspendCondition) o;
                    if (!d.done()) continue;
                } else{

                }

                if (disabled.contains(routine)) continue;

                routine.latest = routine.loop();
            }
        }

        public Coroutine start(Coroutine routine){
            routines.add(routine);
            return routine;
        }

        public Coroutine[] startAll(Coroutine... routines){
            this.routines.addAll(Arrays.asList(routines));
            return routines;
        }

        public boolean end(Coroutine routine){
            return routines.remove(routine);
        }

        public boolean disable(Coroutine routine){
            return disabled.add(routine);
        }

        public boolean enable(Coroutine routine){
            return disabled.remove(routine);
        }

        public void disableAll(){
            disabled.addAll(routines);
        }

        public void enableAll(){
            disabled.clear();
        }

        public void disableAllExcept(Coroutine routine){
            disableAll();
            enable(routine);
        }
    }
}
