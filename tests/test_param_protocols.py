from types import SimpleNamespace

import pytest

from mavcore.mavtypes import MAVParamError, MAVParamType
from mavcore.messages import ParamRequestList, ParamRequestRead
from mavcore.protocols import ParamListProtocol, ParamReadProtocol, ParamWriteProtocol


class ParamTransport:
    """Deliver replies synchronously so tests do not depend on real time."""

    def __init__(self, respond):
        self.respond = respond
        self.connection = SimpleNamespace(portdead=False)
        self.listeners = {}

    def add_listener(self, listener):
        self.listeners[listener.name] = listener

    def remove_listener(self, listener):
        del self.listeners[listener.name]

    def send_msg(self, request):
        self.respond(self, request)

    def reply(self, message_name, *, component=1, **fields):
        message = SimpleNamespace(
            get_srcSystem=lambda: 1, get_srcComponent=lambda: component, **fields
        )
        listener = self.listeners[message_name]
        listener.decode(message)
        listener.callback_func(listener)

    def value(self, param_id, *, index=0, count=1, component=1, value=42.0):
        self.reply(
            "PARAM_VALUE",
            component=component,
            param_id=param_id,
            param_type=MAVParamType.REAL32,
            param_value=value,
            param_index=index,
            param_count=count,
        )


@pytest.mark.parametrize("param_id", [b"GAIN\x00\x00", "GAIN", "GAIN\x00\x00"])
@pytest.mark.parametrize(
    "protocol_class, args", [(ParamReadProtocol, ()), (ParamWriteProtocol, (43.0,))]
)
def test_matches_parameter_replies_by_name(param_id, protocol_class, args):
    def respond(transport, request):
        value = 42.0 if isinstance(request, ParamRequestRead) else request.value
        transport.value(param_id, value=value)

    transport = ParamTransport(respond)
    protocol = protocol_class("GAIN", *args, retries=0, timeout_s=0)
    protocol.run(transport, transport)

    assert protocol.success
    assert protocol.param.name == "GAIN"
    assert protocol.param.value == (args[0] if args else 42.0)


def test_matches_parameter_error_by_name():
    def respond(transport, request):
        transport.reply(
            "PARAM_ERROR",
            param_id=b"GAIN\x00\x00",
            param_index=-1,
            error=MAVParamError.DOES_NOT_EXIST,
        )

    transport = ParamTransport(respond)
    protocol = ParamReadProtocol("GAIN", retries=0, timeout_s=0)
    protocol.run(transport, transport)

    assert not protocol.success
    assert protocol.error == MAVParamError.DOES_NOT_EXIST


def test_parameter_list_requires_specific_component():
    with pytest.raises(ValueError, match="nonzero target_component"):
        ParamListProtocol(target_component=0)


@pytest.mark.parametrize("retries", [0, 2])
def test_parameter_list_recovers_missing_indices(retries):
    attempts = []

    def respond(transport, request):
        assert request.target_component == 7
        if isinstance(request, ParamRequestList):
            transport.value(b"OTHER", index=1, count=2, component=1)
            transport.value(b"FIRST\x00\x00", count=2, component=7)
        else:
            attempts.append(request.param_index)
            if len(attempts) == retries + 1:
                transport.value(b"SIXTEEN_CHAR_ID_", index=1, count=2, component=7)

    transport = ParamTransport(respond)
    protocol = ParamListProtocol(
        target_component=7, retries=retries, timeout_s=0, idle_timeout_s=0
    )
    protocol.run(transport, transport)

    assert attempts == [1] * (retries + 1)
    assert protocol.success
    assert protocol.param_count == 2
    assert protocol.missing == []
    assert list(protocol.params) == ["FIRST", "SIXTEEN_CHAR_ID_"]
