"""autogenerated by genpy from fmMsgs/row.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import std_msgs.msg

class row(genpy.Message):
  _md5sum = "d0e0791535cb7eae30450b9fc21cff87"
  _type = "fmMsgs/row"
  _has_header = True #flag to mark the presence of a Header object
  _full_text = """Header header
int8 rightvalid
float64 rightdistance
float64 rightangle
float64 rightvar	
int8 leftvalid
float64 leftdistance
float64 leftangle
float64 leftvar		
bool headland
float64 error_angle
float64 error_distance
float64 var
 

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.secs: seconds (stamp_secs) since epoch
# * stamp.nsecs: nanoseconds since stamp_secs
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
# 0: no frame
# 1: global frame
string frame_id

"""
  __slots__ = ['header','rightvalid','rightdistance','rightangle','rightvar','leftvalid','leftdistance','leftangle','leftvar','headland','error_angle','error_distance','var']
  _slot_types = ['std_msgs/Header','int8','float64','float64','float64','int8','float64','float64','float64','bool','float64','float64','float64']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       header,rightvalid,rightdistance,rightangle,rightvar,leftvalid,leftdistance,leftangle,leftvar,headland,error_angle,error_distance,var

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(row, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.header is None:
        self.header = std_msgs.msg.Header()
      if self.rightvalid is None:
        self.rightvalid = 0
      if self.rightdistance is None:
        self.rightdistance = 0.
      if self.rightangle is None:
        self.rightangle = 0.
      if self.rightvar is None:
        self.rightvar = 0.
      if self.leftvalid is None:
        self.leftvalid = 0
      if self.leftdistance is None:
        self.leftdistance = 0.
      if self.leftangle is None:
        self.leftangle = 0.
      if self.leftvar is None:
        self.leftvar = 0.
      if self.headland is None:
        self.headland = False
      if self.error_angle is None:
        self.error_angle = 0.
      if self.error_distance is None:
        self.error_distance = 0.
      if self.var is None:
        self.var = 0.
    else:
      self.header = std_msgs.msg.Header()
      self.rightvalid = 0
      self.rightdistance = 0.
      self.rightangle = 0.
      self.rightvar = 0.
      self.leftvalid = 0
      self.leftdistance = 0.
      self.leftangle = 0.
      self.leftvar = 0.
      self.headland = False
      self.error_angle = 0.
      self.error_distance = 0.
      self.var = 0.

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
      buff.write(_struct_3I.pack(_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs))
      _x = self.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_b3db3dB3d.pack(_x.rightvalid, _x.rightdistance, _x.rightangle, _x.rightvar, _x.leftvalid, _x.leftdistance, _x.leftangle, _x.leftvar, _x.headland, _x.error_angle, _x.error_distance, _x.var))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      if self.header is None:
        self.header = std_msgs.msg.Header()
      end = 0
      _x = self
      start = end
      end += 12
      (_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs,) = _struct_3I.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.header.frame_id = str[start:end].decode('utf-8')
      else:
        self.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 75
      (_x.rightvalid, _x.rightdistance, _x.rightangle, _x.rightvar, _x.leftvalid, _x.leftdistance, _x.leftangle, _x.leftvar, _x.headland, _x.error_angle, _x.error_distance, _x.var,) = _struct_b3db3dB3d.unpack(str[start:end])
      self.headland = bool(self.headland)
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
      buff.write(_struct_3I.pack(_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs))
      _x = self.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_b3db3dB3d.pack(_x.rightvalid, _x.rightdistance, _x.rightangle, _x.rightvar, _x.leftvalid, _x.leftdistance, _x.leftangle, _x.leftvar, _x.headland, _x.error_angle, _x.error_distance, _x.var))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      if self.header is None:
        self.header = std_msgs.msg.Header()
      end = 0
      _x = self
      start = end
      end += 12
      (_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs,) = _struct_3I.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.header.frame_id = str[start:end].decode('utf-8')
      else:
        self.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 75
      (_x.rightvalid, _x.rightdistance, _x.rightangle, _x.rightvar, _x.leftvalid, _x.leftdistance, _x.leftangle, _x.leftvar, _x.headland, _x.error_angle, _x.error_distance, _x.var,) = _struct_b3db3dB3d.unpack(str[start:end])
      self.headland = bool(self.headland)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
_struct_b3db3dB3d = struct.Struct("<b3db3dB3d")
_struct_3I = struct.Struct("<3I")
