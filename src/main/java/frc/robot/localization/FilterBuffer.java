package frc.robot.localization;

import edu.wpi.first.math.Pair;

import java.util.Arrays;
import java.util.Optional;

public class FilterBuffer {

    private FilterSnapshot[] snapshots;
    private int index;
    private double period;

    public FilterBuffer(int length, double period) {
        snapshots = new FilterSnapshot[length];
        this.period = period;
    }

    public void addSnapshot(FilterSnapshot snapshot) {
        snapshots[index++ % snapshots.length] = snapshot;
    }

    public void addSnapshot(FilterSnapshot snapshot, int index) {
        snapshots[(this.index - index) % snapshots.length] = snapshot;
    }

    public Optional<Pair<FilterSnapshot, Integer>> getSnapshot(double time) {

        System.out.println("camera time: " + time);
        System.out.println("snapshot time: " + snapshots[index % snapshots.length].time());

        int guess = (int) Math.ceil((snapshots[index % snapshots.length].time() - time) / period);

        if (guess > snapshots.length) {
            return Optional.empty();
        }

        while (guess++ != index) {
            if (snapshots[(index - guess) % snapshots.length].time() < time) {
                return Optional.of(Pair.of(snapshots[guess % snapshots.length], index - guess));
            }
        }

        return Optional.empty();
    }

    public FilterSnapshot[] getReplay(int index) {
        FilterSnapshot[] replay = new FilterSnapshot[index-1];

        for (int i = 0; i+1 < index; i++) {
            replay[i] = snapshots[(this.index - index + i+1) % snapshots.length];
        }

        return replay;
    }


}
