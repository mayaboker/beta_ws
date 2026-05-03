import yaml
from dataclasses import dataclass
from typing import Any

@dataclass
class Parameter:
    name: str
    type: str
    value: Any
    default: Any
    min: float | None = None
    max: float | None = None
    values: list[Any] | None = None

    def validate(self, new_value: Any):
        if self.type == "int":
            if not isinstance(new_value, int):
                raise ValueError(f"{self.name} must be int")
            if self.min is not None and new_value < self.min:
                raise ValueError(f"{self.name} below min {self.min}")
            if self.max is not None and new_value > self.max:
                raise ValueError(f"{self.name} above max {self.max}")

        elif self.type == "float":
            if not isinstance(new_value, (int, float)):
                raise ValueError(f"{self.name} must be float")
            new_value = float(new_value)
            if self.min is not None and new_value < self.min:
                raise ValueError(f"{self.name} below min {self.min}")
            if self.max is not None and new_value > self.max:
                raise ValueError(f"{self.name} above max {self.max}")

        elif self.type == "enum":
            if new_value not in self.values:
                raise ValueError(f"{self.name} must be one of {self.values}")

        elif self.type == "bool":
            if not isinstance(new_value, bool):
                raise ValueError(f"{self.name} must be bool")

        elif self.type == "str":
            if not isinstance(new_value, str):
                raise ValueError(f"{self.name} must be str")

        else:
            raise ValueError(f"Unsupported type {self.type}")

        return new_value
    

class ConfigRegistry:
    def __init__(self, yaml_path: str):
        self.yaml_path = yaml_path
        self.params: dict[str, Parameter] = {}
        self.load()

    def load(self):
        with open(self.yaml_path, "r") as f:
            data = yaml.safe_load(f)

        for group, params in data.items():
            for key, spec in params.items():
                name = f"{group}.{key}"
                self.params[name] = Parameter(
                    name=name,
                    type=spec["type"],
                    value=spec.get("value", spec["default"]),
                    default=spec["default"],
                    min=spec.get("min"),
                    max=spec.get("max"),
                    values=spec.get("values"),
                )

    def get(self, name: str):
        return self.params[name].value

    def set(self, name: str, value):
        if name not in self.params:
            raise KeyError(f"Unknown parameter {name}")

        param = self.params[name]
        value = param.validate(value)
        param.value = value
        return value

    def dump(self) -> dict[str, Any]:
        return {name: param.value for name, param in self.params.items()}

    def dump_yaml(self) -> str:
        return yaml.safe_dump(self.dump(), sort_keys=True)

    def save(self):
        data = {}

        for name, param in self.params.items():
            group, key = name.split(".", 1)
            data.setdefault(group, {})
            data[group][key] = {
                "type": param.type,
                "default": param.default,
                "value": param.value,
            }

            if param.min is not None:
                data[group][key]["min"] = param.min
            if param.max is not None:
                data[group][key]["max"] = param.max
            if param.values is not None:
                data[group][key]["values"] = param.values

        with open(self.yaml_path, "w") as f:
            yaml.safe_dump(data, f, sort_keys=False)

if __name__ == "__main__":
    config = ConfigRegistry("/home/user/workspaces/beta_ws/src/bt_bringup/config/app.yaml")
    print(config.dump())
