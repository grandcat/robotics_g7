"""autogenerated by genpy from color_filter/Objects.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import color_filter.msg

class Objects(genpy.Message):
  _md5sum = "c0faa35c817b768e1343340247270ab6"
  _type = "color_filter/Objects"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """Rect2D_[] ROI
int32 ROI_id

================================================================================
MSG: color_filter/Rect2D_
int32 x
int32 y
int32 width
int32 height
"""
  __slots__ = ['ROI','ROI_id']
  _slot_types = ['color_filter/Rect2D_[]','int32']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       ROI,ROI_id

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(Objects, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.ROI is None:
        self.ROI = []
      if self.ROI_id is None:
        self.ROI_id = 0
    else:
      self.ROI = []
      self.ROI_id = 0

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
      length = len(self.ROI)
      buff.write(_struct_I.pack(length))
      for val1 in self.ROI:
        _x = val1
        buff.write(_struct_4i.pack(_x.x, _x.y, _x.width, _x.height))
      buff.write(_struct_i.pack(self.ROI_id))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      if self.ROI is None:
        self.ROI = None
      end = 0
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.ROI = []
      for i in range(0, length):
        val1 = color_filter.msg.Rect2D_()
        _x = val1
        start = end
        end += 16
        (_x.x, _x.y, _x.width, _x.height,) = _struct_4i.unpack(str[start:end])
        self.ROI.append(val1)
      start = end
      end += 4
      (self.ROI_id,) = _struct_i.unpack(str[start:end])
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
      length = len(self.ROI)
      buff.write(_struct_I.pack(length))
      for val1 in self.ROI:
        _x = val1
        buff.write(_struct_4i.pack(_x.x, _x.y, _x.width, _x.height))
      buff.write(_struct_i.pack(self.ROI_id))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      if self.ROI is None:
        self.ROI = None
      end = 0
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.ROI = []
      for i in range(0, length):
        val1 = color_filter.msg.Rect2D_()
        _x = val1
        start = end
        end += 16
        (_x.x, _x.y, _x.width, _x.height,) = _struct_4i.unpack(str[start:end])
        self.ROI.append(val1)
      start = end
      end += 4
      (self.ROI_id,) = _struct_i.unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
_struct_i = struct.Struct("<i")
_struct_4i = struct.Struct("<4i")
