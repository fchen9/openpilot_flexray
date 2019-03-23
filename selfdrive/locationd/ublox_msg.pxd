from libc.stdint cimport uint8_t, uint16_t

DEF MsgHeaderSize = 8
DEF MaxMsgSize = 65536
DEF MsgBufSize = MsgHeaderSize + MaxMsgSize

cdef:
    enum:
        UN_USED = 1

    packed struct MsgHeader:
        uint8_t preamble1
        uint8_t preamble2
        uint8_t cls
        uint8_t id
        uint16_t len
        uint8_t checksum[2]


    void append_data(uint8_t *buf, uint8_t *data, uint16_t len)
