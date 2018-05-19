package roadgraph;

public class MapNodeAndDistance implements Comparable<MapNodeAndDistance> {
    public final MapNode point;
    public final double distance;

    public MapNodeAndDistance(MapNode point, double distance) {
        this.point = point;
        this.distance = distance;
    }

    @Override
    public int compareTo(MapNodeAndDistance o) {

        if(this.distance > o.distance){
            return 1;
        }else if(this.distance < o.distance){
            return -1;
        }else{
            return 0;
        }

    }
}
