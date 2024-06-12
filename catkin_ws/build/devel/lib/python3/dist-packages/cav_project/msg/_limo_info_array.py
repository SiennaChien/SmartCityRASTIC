# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from cav_project/limo_info_array.msg. Do not edit."""
import codecs
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import cav_project.msg
import std_msgs.msg

class limo_info_array(genpy.Message):
  _md5sum = "cc8c33c215547bc4fe7561cb7d91633e"
  _type = "cav_project/limo_info_array"
  _has_header = False  # flag to mark the presence of a Header object
  _full_text = """cav_project/limo_info[] limo_infos

================================================================================
MSG: cav_project/limo_info
std_msgs/Int32 ID
std_msgs/Float64 x
std_msgs/Float64 y
std_msgs/Float64 vel
std_msgs/String path
std_msgs/Float64 d1
std_msgs/Float64 d2
std_msgs/Float64 origin_dist

================================================================================
MSG: std_msgs/Int32
int32 data
================================================================================
MSG: std_msgs/Float64
float64 data
================================================================================
MSG: std_msgs/String
string data
"""
  __slots__ = ['limo_infos']
  _slot_types = ['cav_project/limo_info[]']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       limo_infos

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(limo_info_array, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.limo_infos is None:
        self.limo_infos = []
    else:
      self.limo_infos = []

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
      length = len(self.limo_infos)
      buff.write(_struct_I.pack(length))
      for val1 in self.limo_infos:
        _v1 = val1.ID
        _x = _v1.data
        buff.write(_get_struct_i().pack(_x))
        _v2 = val1.x
        _x = _v2.data
        buff.write(_get_struct_d().pack(_x))
        _v3 = val1.y
        _x = _v3.data
        buff.write(_get_struct_d().pack(_x))
        _v4 = val1.vel
        _x = _v4.data
        buff.write(_get_struct_d().pack(_x))
        _v5 = val1.path
        _x = _v5.data
        length = len(_x)
        if python3 or type(_x) == unicode:
          _x = _x.encode('utf-8')
          length = len(_x)
        buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
        _v6 = val1.d1
        _x = _v6.data
        buff.write(_get_struct_d().pack(_x))
        _v7 = val1.d2
        _x = _v7.data
        buff.write(_get_struct_d().pack(_x))
        _v8 = val1.origin_dist
        _x = _v8.data
        buff.write(_get_struct_d().pack(_x))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    if python3:
      codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      if self.limo_infos is None:
        self.limo_infos = None
      end = 0
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.limo_infos = []
      for i in range(0, length):
        val1 = cav_project.msg.limo_info()
        _v9 = val1.ID
        start = end
        end += 4
        (_v9.data,) = _get_struct_i().unpack(str[start:end])
        _v10 = val1.x
        start = end
        end += 8
        (_v10.data,) = _get_struct_d().unpack(str[start:end])
        _v11 = val1.y
        start = end
        end += 8
        (_v11.data,) = _get_struct_d().unpack(str[start:end])
        _v12 = val1.vel
        start = end
        end += 8
        (_v12.data,) = _get_struct_d().unpack(str[start:end])
        _v13 = val1.path
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        if python3:
          _v13.data = str[start:end].decode('utf-8', 'rosmsg')
        else:
          _v13.data = str[start:end]
        _v14 = val1.d1
        start = end
        end += 8
        (_v14.data,) = _get_struct_d().unpack(str[start:end])
        _v15 = val1.d2
        start = end
        end += 8
        (_v15.data,) = _get_struct_d().unpack(str[start:end])
        _v16 = val1.origin_dist
        start = end
        end += 8
        (_v16.data,) = _get_struct_d().unpack(str[start:end])
        self.limo_infos.append(val1)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      length = len(self.limo_infos)
      buff.write(_struct_I.pack(length))
      for val1 in self.limo_infos:
        _v17 = val1.ID
        _x = _v17.data
        buff.write(_get_struct_i().pack(_x))
        _v18 = val1.x
        _x = _v18.data
        buff.write(_get_struct_d().pack(_x))
        _v19 = val1.y
        _x = _v19.data
        buff.write(_get_struct_d().pack(_x))
        _v20 = val1.vel
        _x = _v20.data
        buff.write(_get_struct_d().pack(_x))
        _v21 = val1.path
        _x = _v21.data
        length = len(_x)
        if python3 or type(_x) == unicode:
          _x = _x.encode('utf-8')
          length = len(_x)
        buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
        _v22 = val1.d1
        _x = _v22.data
        buff.write(_get_struct_d().pack(_x))
        _v23 = val1.d2
        _x = _v23.data
        buff.write(_get_struct_d().pack(_x))
        _v24 = val1.origin_dist
        _x = _v24.data
        buff.write(_get_struct_d().pack(_x))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    if python3:
      codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      if self.limo_infos is None:
        self.limo_infos = None
      end = 0
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.limo_infos = []
      for i in range(0, length):
        val1 = cav_project.msg.limo_info()
        _v25 = val1.ID
        start = end
        end += 4
        (_v25.data,) = _get_struct_i().unpack(str[start:end])
        _v26 = val1.x
        start = end
        end += 8
        (_v26.data,) = _get_struct_d().unpack(str[start:end])
        _v27 = val1.y
        start = end
        end += 8
        (_v27.data,) = _get_struct_d().unpack(str[start:end])
        _v28 = val1.vel
        start = end
        end += 8
        (_v28.data,) = _get_struct_d().unpack(str[start:end])
        _v29 = val1.path
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        if python3:
          _v29.data = str[start:end].decode('utf-8', 'rosmsg')
        else:
          _v29.data = str[start:end]
        _v30 = val1.d1
        start = end
        end += 8
        (_v30.data,) = _get_struct_d().unpack(str[start:end])
        _v31 = val1.d2
        start = end
        end += 8
        (_v31.data,) = _get_struct_d().unpack(str[start:end])
        _v32 = val1.origin_dist
        start = end
        end += 8
        (_v32.data,) = _get_struct_d().unpack(str[start:end])
        self.limo_infos.append(val1)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_d = None
def _get_struct_d():
    global _struct_d
    if _struct_d is None:
        _struct_d = struct.Struct("<d")
    return _struct_d
_struct_i = None
def _get_struct_i():
    global _struct_i
    if _struct_i is None:
        _struct_i = struct.Struct("<i")
    return _struct_i