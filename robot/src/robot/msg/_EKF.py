"""autogenerated by genpy from robot/EKF.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class EKF(genpy.Message):
  _md5sum = "55db70fe710cb462a76f2e14eff90744"
  _type = "robot/EKF"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """float32 x
float32 y
float32 theta
float32 y_wall
bool wall
bool right_sensor
"""
  __slots__ = ['x','y','theta','y_wall','wall','right_sensor']
  _slot_types = ['float32','float32','float32','float32','bool','bool']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       x,y,theta,y_wall,wall,right_sensor

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(EKF, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.x is None:
        self.x = 0.
      if self.y is None:
        self.y = 0.
      if self.theta is None:
        self.theta = 0.
      if self.y_wall is None:
        self.y_wall = 0.
      if self.wall is None:
        self.wall = False
      if self.right_sensor is None:
        self.right_sensor = False
    else:
      self.x = 0.
      self.y = 0.
      self.theta = 0.
      self.y_wall = 0.
      self.wall = False
      self.right_sensor = False

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def serialize(self, buff):
    """
    serialize message into buffer
    :param buff: buffer, ``StringIO``
    """
    try:
      _x = self
      buff.write(_struct_4f2B.pack(_x.x, _x.y, _x.theta, _x.y_wall, _x.wall, _x.right_sensor))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      end = 0
      _x = self
      start = end
      end += 18
      (_x.x, _x.y, _x.theta, _x.y_wall, _x.wall, _x.right_sensor,) = _struct_4f2B.unpack(str[start:end])
      self.wall = bool(self.wall)
      self.right_sensor = bool(self.right_sensor)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      _x = self
      buff.write(_struct_4f2B.pack(_x.x, _x.y, _x.theta, _x.y_wall, _x.wall, _x.right_sensor))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      end = 0
      _x = self
      start = end
      end += 18
      (_x.x, _x.y, _x.theta, _x.y_wall, _x.wall, _x.right_sensor,) = _struct_4f2B.unpack(str[start:end])
      self.wall = bool(self.wall)
      self.right_sensor = bool(self.right_sensor)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
_struct_4f2B = struct.Struct("<4f2B")
