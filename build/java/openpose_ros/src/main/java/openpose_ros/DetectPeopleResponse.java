package openpose_ros;

public interface DetectPeopleResponse extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "openpose_ros/DetectPeopleResponse";
  static final java.lang.String _DEFINITION = "PersonDetection[] people_list";
  java.util.List<openpose_ros.PersonDetection> getPeopleList();
  void setPeopleList(java.util.List<openpose_ros.PersonDetection> value);
}
