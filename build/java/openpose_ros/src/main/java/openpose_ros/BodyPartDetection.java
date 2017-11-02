package openpose_ros;

public interface BodyPartDetection extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "openpose_ros/BodyPartDetection";
  static final java.lang.String _DEFINITION = "geometry_msgs/Point pos\nfloat32 confidence\n";
  geometry_msgs.Point getPos();
  void setPos(geometry_msgs.Point value);
  float getConfidence();
  void setConfidence(float value);
}
