from enum import IntEnum


class RenderMode(IntEnum):
    HUMAN = 0
    RGB_ARRAY = 1
    RGB_ARRAY_REF = 2


def rendermode_from_string(mode):
    assert mode in ["human", "rgb_array", "rgb_array_ref"]
    return {"human": RenderMode.HUMAN, "rgb_array": RenderMode.RGB_ARRAY, "rgb_array_ref": RenderMode.RGB_ARRAY_REF}[
        mode
    ]
