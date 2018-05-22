package roadgraph;

public class MapNodeAndTotalDistance implements Comparable<MapNodeAndTotalDistance> {
    public final MapNode point;
    public final double distance;
    public final double estDistance;

    public MapNodeAndTotalDistance(MapNode point, double distance, double estDistance) {
        this.point = point;
        this.distance = distance;
        this.estDistance = estDistance;
    }

    @Override
    public int compareTo(MapNodeAndTotalDistance o) {

        if(this.distance + this.estDistance > o.distance + o.estDistance){
            return 1;
        }else if(this.distance + this.estDistance < o.distance + o.estDistance){
            return -1;
        }else{
            return 0;
        }

    }
}