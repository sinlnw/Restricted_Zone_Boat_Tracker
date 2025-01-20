import ctypes

PKT_TYPE_ADVERTISE = 0x01
PKT_TYPE_REPORT    = 0x02
PKT_TYPE_ACK       = 0x03
PKT_TYPE_BOOT      = 0x04


class BaseStruct(ctypes.Structure):

    _pack_ = 1

    @classmethod
    def size(cls):
        return ctypes.sizeof(cls)

    def unpack(self, blob):
        if not isinstance(blob, bytes):
            raise Exception('Byte array expected for blob')
        if ctypes.sizeof(self) != len(blob):
            raise Exception('Size mismatched')
        ctypes.memmove(ctypes.addressof(self), blob, self.size())

    def pack(self):
        blob = ctypes.create_string_buffer(ctypes.sizeof(self))
        ctypes.memmove(blob, ctypes.addressof(self), ctypes.sizeof(self))
        return blob.raw


class StructConfig(BaseStruct):
    _fields_ = [
        ('radio_device_address', ctypes.c_uint8),
        ('radio_gateway_address', ctypes.c_uint8),
        ('radio_freq', ctypes.c_float),
        ('radio_tx_power', ctypes.c_uint8),
        ('collect_interval_day', ctypes.c_uint16),
        ('collect_interval_night', ctypes.c_uint16),
        ('day_start_hour', ctypes.c_uint8),
        ('day_end_hour', ctypes.c_uint8),
        ('time_zone', ctypes.c_byte),
        ('advertise_interval', ctypes.c_uint16),
        ('use_ack', ctypes.c_byte),
        ('ack_timeout', ctypes.c_uint16),
        ('long_range', ctypes.c_uint8),
        ('tx_repeat', ctypes.c_uint8),
        ('gps_max_wait_for_fix', ctypes.c_uint16),
        ('next_collect_no_fix', ctypes.c_uint16),
        ('total_slots', ctypes.c_uint16),
        ('slot_interval', ctypes.c_uint16),
        ('prog_file_name', ctypes.c_char*10),
    ]


class StructReport(BaseStruct):
    _fields_ = [
        ('year', ctypes.c_uint8),
        ('month', ctypes.c_uint8),
        ('day', ctypes.c_uint8),
        ('hour', ctypes.c_uint8),
        ('minute', ctypes.c_uint8),
        ('second', ctypes.c_uint8),
        ('vbat', ctypes.c_uint16),    # unit of mV
        ('latitude', ctypes.c_int32),
        ('longitude', ctypes.c_int32), # unit of 1/100000 degrees
        ('quality', ctypes.c_uint8),
        ('satellites', ctypes.c_uint8),
        ('temperature', ctypes.c_uint16),
        ('ttf', ctypes.c_uint32),
    ]


class BasePacket(BaseStruct):

    TYPE_ID = None

    @property
    def bytes(self):
        #if self.TYPE_ID is None:
        #    raise AttributeError('TYPE_ID attribute is not defined')
        #if not hasattr(self, 'type'):
        #    raise AttributeError("'type' field is not defined")
        self.type = self.TYPE_ID
        return self.pack()

    @classmethod
    def probe(cls, bytes):
        return bytes[0] == cls.TYPE_ID and len(bytes) == cls.size()


class PacketBoot(BasePacket):
    TYPE_ID = PKT_TYPE_BOOT
    _fields_ = [
        ('type', ctypes.c_uint8),
        ('firmware', ctypes.c_uint16),
        ('device_model', ctypes.c_uint8),
        ('reset_flags', ctypes.c_uint8),
        ('config', StructConfig),
    ]


class PacketReport(BasePacket):
    TYPE_ID = PKT_TYPE_REPORT
    _fields_ = [
        ('type', ctypes.c_uint8),
        ('seq', ctypes.c_uint8),
        ('report', StructReport),
    ]


class PacketAdvertise(BasePacket):
    TYPE_ID = PKT_TYPE_ADVERTISE
    _fields_ = [
        ('type', ctypes.c_uint8),
        ('seq', ctypes.c_uint8),
    ]


class PacketAck(BasePacket):
    TYPE_ID = PKT_TYPE_ACK
    _fields_ = [
        ('type', ctypes.c_uint8),
        ('seq', ctypes.c_uint8),
    ]

