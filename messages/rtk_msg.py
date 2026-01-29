import pymavlink.dialects.v20.all as dialect
from mavcore.mav_message import MAVMessage, thread_safe


class RTKData(MAVMessage):
    """
    Sends RTCM message.
    """

    def __init__(self, sequence_num: int, payload: bytes):
        super().__init__("RTK_DATA")
        self.sequence_num = sequence_num
        self.data = list(payload)
        self.data_length = len(payload)

    def encode(self, system_id, component_id):
        return dialect.MAVLink_gps_rtcm_data_message(
            flags=self.sequence_num << 3,
            len=self.data_length,
            data=self.data + [0 for _ in range(180 - (self.data_length))],
        )

    @thread_safe
    def __repr__(self):
        return f"""(GPS_RTCM_DATA)\n
            \tflags: {bin(self.sequence_num << 3)}\n
            \tlength: {self.data_length}\n
            \tdata: {self.data}"""


# class RTKData(MAVMessage):
#     """
#     Sends RTCM message.
#     """

#     def __init__(self, is_fragmented: bool, fragment_id: int, sequence_num: int, payload: list[int]):
#         super().__init__("RTK_DATA")
#         self.flags = (sequence_num << 3) | (fragment_id << 1) | is_fragmented
#         self.data = payload
#         self.data_length = len(payload)

#     def encode(self, system_id, component_id):
#         return dialect.MAVLink_gps_rtcm_data_message(
#             flags=self.flags,
#             len=self.data_length,
#             data=self.data + [0 for _ in range(180 - (self.data_length))]
#         )

#     @thread_safe
#     def __repr__(self):
#         return f"""(GPS_RTCM_DATA)\n
#             \tflags: {bin(self.flags)}\n
#             \tlength: {self.data_length}\n
#             \tdata: {self.data}"""
