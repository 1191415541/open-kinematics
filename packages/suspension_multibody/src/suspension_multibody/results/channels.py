"""Read-only registry for the frozen Adams axle channel contract."""

from __future__ import annotations

from dataclasses import dataclass
from types import MappingProxyType
from typing import TYPE_CHECKING, Any, Mapping

if TYPE_CHECKING:
    from ..adams.axle_contract import AxleChannelBindings


def _freeze(value: Any) -> Any:
    if isinstance(value, dict):
        return MappingProxyType({key: _freeze(item) for key, item in value.items()})
    if isinstance(value, list):
        return tuple(_freeze(item) for item in value)
    return value


@dataclass(frozen=True)
class ChannelRegistry:
    """Minimal read-only view over the single frozen axle channel table."""

    _contract: Mapping[str, Any]

    @classmethod
    def load(cls) -> "ChannelRegistry":
        """Load the packaged ``axle_channels.yaml`` contract."""
        from ..adams.axle_contract import load_axle_channel_contract

        return cls(_contract=_freeze(load_axle_channel_contract()))

    @property
    def contract(self) -> Mapping[str, Any]:
        """The frozen contract payload."""
        return self._contract

    @property
    def schema_version(self) -> int:
        return int(self._contract["schema_version"])

    @property
    def world_axes(self) -> Mapping[str, str]:
        return self._contract["world_axes"]

    @property
    def rotation_sign(self) -> str:
        return str(self._contract["rotation_sign"])

    @property
    def required_role_bindings(self) -> tuple[str, ...]:
        return tuple(self._contract["required_role_bindings"])

    @property
    def channels(self) -> Mapping[str, Mapping[str, str]]:
        return self._contract["channels"]

    @property
    def channel_names(self) -> tuple[str, ...]:
        """Return frozen channel names in YAML declaration order."""
        return tuple(self.channels)

    def channel(self, name: str) -> Mapping[str, str]:
        """Return one channel definition without copying it."""
        try:
            return self.channels[name]
        except KeyError as error:
            raise KeyError(f"unknown axle channel {name!r}") from error

    def validate_bindings(self, model, bindings: "AxleChannelBindings") -> None:
        """Delegate semantic role validation to the existing contract owner."""
        from ..adams.axle_contract import validate_axle_channel_bindings

        validate_axle_channel_bindings(model, bindings)
